"""Mission orchestrator v2 — top-level lifecycle node.

Hierarchical state machine via the ``transitions`` library:

    BOOT ──► READY ──► PREPARE ──► INSPECTING ──► RETURNING ──► DONE
                            │                                     │
                            └──────────► (FAULT) ◄────────────────┘
                                                                  │
                                                                  └──► READY
                                                                       (next mission)

PREPARE wraps a single LOCALIZING child this MR; MappingMission slots
in here later. INSPECTING wraps NAVIGATING / SCANNING / PUBLISHING and
walks the tag sequence once per mission.

Pause and E-stop are *flags*, not states — they freeze progress without
altering the lifecycle. /mission/resume is the only thing that unblocks
the orchestrator after either fires.
"""

from __future__ import annotations

import math
import time
import uuid
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Quaternion
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time as RclpyTime
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)
from transitions.extensions import HierarchicalGraphMachine

from lupin_msgs.msg import DiscoveredTags, MissionState, Observation
from lupin_msgs.srv import ConfirmTag, GetTagReading, SetArmPreset, StartMission

from .estop_monitor import EStopMonitor
from .battery_monitor import BatteryMonitor
from .inspection_mission import InspectionMission
from .exploration_mission import ExplorationMission, MonitoringMission
from .frontier import select_frontier_goal
from .observations import make_tag_observation
from .approach import (
    TagApproach,
    compute_approach,
    compute_discovered_approach,
    load_approach_overrides,
)
from .tag_locations import (
    load_default_tables,
    load_default_tag_locations,
    numeric_string_sort_key,
)

# ─── HSM topology ────────────────────────────────────────────────────────
#
# Kept as a free function so the docs exporter can build the diagram
# without standing up a ROS node. Uses the default ``_`` separator from
# transitions, so nested states are referenced as e.g.
# ``INSPECTING_NAVIGATING``.

LIFECYCLE_STATES = [
    'BOOT', 'READY', 'PREPARE', 'EXPLORING', 'INSPECTING', 'MONITORING',
    'RETURNING', 'DONE', 'FAULT',
]
INSPECTING_SUBSTATES = ['NAVIGATING', 'SCANNING', 'PUBLISHING']
# MONITORING reuses the same NAVIGATING→SCANNING→PUBLISHING sub-machine as
# INSPECTING; the only difference is PUBLISHING loops back forever instead of
# completing (MonitoringMission.is_complete() stays False).
MONITORING_SUBSTATES = ['NAVIGATING', 'SCANNING', 'PUBLISHING']
PREPARE_SUBSTATES = ['LOCALIZING']


def build_hsm_spec() -> dict:
    """States + transitions for the orchestrator. Pure data."""
    states = [
        'BOOT',
        'READY',
        {'name': 'PREPARE', 'children': PREPARE_SUBSTATES, 'initial': 'LOCALIZING'},
        'EXPLORING',
        {'name': 'INSPECTING', 'children': INSPECTING_SUBSTATES, 'initial': 'NAVIGATING'},
        {'name': 'MONITORING', 'children': MONITORING_SUBSTATES, 'initial': 'NAVIGATING'},
        'RETURNING',
        'DONE',
        'FAULT',
    ]
    transitions = [
        # BOOT
        {"trigger": "deps_up", "source": "BOOT", "dest": "READY"},
        # READY → PREPARE (via /mission/start)
        {'trigger': 'start_mission', 'source': 'READY', 'dest': 'PREPARE'},
        # PREPARE.LOCALIZING → EXPLORING (ExplorationMission) or INSPECTING.
        # transitions evaluates these in order; the first whose condition
        # passes wins, so the conditioned one must come first.
        {
            'trigger': 'localized',
            'source': 'PREPARE_LOCALIZING',
            'dest': 'EXPLORING',
            'conditions': '_is_exploration_mission',
        },
        {'trigger': 'localized', 'source': 'PREPARE_LOCALIZING', 'dest': 'INSPECTING'},
        # EXPLORING → MONITORING once N tags are discovered, or when frontiers
        # run out: to MONITORING if any were found, else home.
        {'trigger': 'tags_discovered', 'source': 'EXPLORING', 'dest': 'MONITORING'},
        {
            'trigger': 'no_frontiers',
            'source': 'EXPLORING',
            'dest': 'MONITORING',
            'conditions': '_has_discovered_any',
        },
        {'trigger': 'no_frontiers', 'source': 'EXPLORING', 'dest': 'RETURNING'},
        # INSPECTING sub-machine
        {
            "trigger": "nav_succeeded",
            "source": "INSPECTING_NAVIGATING",
            "dest": "INSPECTING_SCANNING",
        },
        {
            "trigger": "nav_unreachable",
            "source": "INSPECTING_NAVIGATING",
            "dest": "INSPECTING_PUBLISHING",
        },
        {
            "trigger": "scan_done",
            "source": "INSPECTING_SCANNING",
            "dest": "INSPECTING_PUBLISHING",
        },
        {
            "trigger": "next_tag",
            "source": "INSPECTING_PUBLISHING",
            "dest": "INSPECTING_NAVIGATING",
        },
        {
            "trigger": "inspection_complete",
            "source": ["INSPECTING_PUBLISHING", "MONITORING_PUBLISHING"],
            "dest": "RETURNING",
        },
        # MONITORING sub-machine — same triggers, different source states, and
        # next_tag always loops (no inspection_complete).
        {
            'trigger': 'nav_succeeded',
            'source': 'MONITORING_NAVIGATING',
            'dest': 'MONITORING_SCANNING',
        },
        {
            'trigger': 'nav_unreachable',
            'source': 'MONITORING_NAVIGATING',
            'dest': 'MONITORING_PUBLISHING',
        },
        {
            'trigger': 'scan_done',
            'source': 'MONITORING_SCANNING',
            'dest': 'MONITORING_PUBLISHING',
        },
        {
            'trigger': 'next_tag',
            'source': 'MONITORING_PUBLISHING',
            'dest': 'MONITORING_NAVIGATING',
        },
        # /mission/abort jumps any active inspecting/monitoring/exploring
        # state to RETURNING.
        {
            'trigger': 'abort_to_return',
            'source': [
                'EXPLORING',
                'INSPECTING_NAVIGATING',
                'INSPECTING_SCANNING',
                'INSPECTING_PUBLISHING',
                'MONITORING_NAVIGATING',
                'MONITORING_SCANNING',
                'MONITORING_PUBLISHING',
            ],
            "dest": "RETURNING",
        },
        # RETURNING → DONE
        {"trigger": "returned", "source": "RETURNING", "dest": "DONE"},
        # Battery/dock resume: go back to the mission family we left, without
        # passing through DONE. The trigger is chosen from _return_origin in
        # _handle_resume so a MONITORING/EXPLORING run doesn't fall into the
        # INSPECTING sub-machine (which would end a monitoring loop early).
        {
            "trigger": "resume_inspection",
            "source": "RETURNING",
            "dest": "INSPECTING",
        },
        {
            "trigger": "resume_monitoring",
            "source": "RETURNING",
            "dest": "MONITORING",
        },
        {
            "trigger": "resume_exploration",
            "source": "RETURNING",
            "dest": "EXPLORING",
        },
        # DONE → READY for next mission
        {"trigger": "reset_for_next", "source": "DONE", "dest": "READY"},
        # FAULT — any non-terminal transitions to FAULT on catastrophic error.
        {
            'trigger': 'fault',
            'source': [
                'BOOT',
                'READY',
                'PREPARE',
                'PREPARE_LOCALIZING',
                'EXPLORING',
                'INSPECTING',
                'INSPECTING_NAVIGATING',
                'INSPECTING_SCANNING',
                'INSPECTING_PUBLISHING',
                'MONITORING',
                'MONITORING_NAVIGATING',
                'MONITORING_SCANNING',
                'MONITORING_PUBLISHING',
                'RETURNING',
            ],
            "dest": "FAULT",
        },
    ]
    return {"states": states, "transitions": transitions, "initial": "BOOT"}


# Lifecycle-state strings that indicate "a mission is currently running",
# i.e. /mission/start should be rejected.
_BUSY_PREFIXES = ('PREPARE', 'EXPLORING', 'INSPECTING', 'MONITORING', 'RETURNING')

_TERMINAL_PREFIXES = ("DONE", "FAULT")

# on_enter_RETURNING defers the dock goal until an in-flight nav cancel acks.
# If that ack is lost (DDS hiccup, or rapid goal churn when aborting mid-loop)
# the dock goal would never send and the robot would sit in RETURNING forever.
# After this grace period the orchestrator forces the dock goal anyway — a fresh
# NavigateToPose preempts any lingering goal server-side.
_RETURN_CANCEL_GRACE_S = 3.0


def _is_state_busy(state: str) -> bool:
    return state.startswith(_BUSY_PREFIXES)


def _is_state_inspecting(state: str) -> bool:
    return state == "INSPECTING" or state.startswith("INSPECTING_")


def _is_state_monitoring(state: str) -> bool:
    return state == 'MONITORING' or state.startswith('MONITORING_')


def _is_state_scanning_phase(state: str) -> bool:
    """True for either sub-machine's NAVIGATING/SCANNING/PUBLISHING states."""
    return _is_state_inspecting(state) or _is_state_monitoring(state)


def _resume_origin_for(state: str) -> str:
    """Which mission family a battery/dock resume should return to.

    Captured at the instant we divert to RETURNING so /mission/resume re-enters
    the matching sub-machine. Without this every resume fell into INSPECTING,
    which terminates a MONITORING loop early (INSPECTING_PUBLISHING exits to
    RETURNING when the tag list ends, whereas MONITORING_PUBLISHING loops).
    """
    if _is_state_monitoring(state):
        return 'MONITORING'
    if state == 'EXPLORING':
        return 'EXPLORING'
    return 'INSPECTING'


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ─── Node ────────────────────────────────────────────────────────────────


class MissionOrchestratorNode(Node):
    """Top-level orchestrator node.

    Owns the HSM, the observation/state publishers, the operator services,
    the E-stop monitor, and the action/service clients to Nav2 and the
    greenhouse bridge. Drives the lifecycle in response to ROS events.
    """

    def __init__(self, node_name: str = "mission_orchestrator", **node_kwargs):
        super().__init__(node_name, **node_kwargs)

        # ─── parameters ────────────────────────────────────────────────
        self.declare_parameter("nav_action_name", "navigate_to_pose")
        self.declare_parameter(
            "bridge_service_name", "/greenhouse_bridge/get_tag_reading"
        )
        self.declare_parameter("estop_topic", "/e_stop_state")
        self.declare_parameter("dependency_timeout_s", 30.0)

        self.declare_parameter("map_yaml_path", "")
        self.declare_parameter("localization_timeout_s", 15.0)
        self.declare_parameter("localization_covariance_threshold", 0.25)
        self.declare_parameter("amcl_pose_topic", "/amcl_pose")

        # Empty → use the upstream mdp-greenhouse package JSON. Set this
        # when the world generator was run with --aisle-expand-y != 1 so
        # nav goals match the shifted tables.
        self.declare_parameter("tag_locations_file", "")

        # tag_sequence: type-only declaration so an empty default doesn't
        # infer as BYTE_ARRAY and reject string overrides.
        self.declare_parameter("tag_sequence", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("approach_yaw", 0.0)
        self.declare_parameter("approach_standoff_m", 0.5)
        self.declare_parameter("approach_overrides_file", "")
        self.declare_parameter("nav_timeout_s", 60.0)
        self.declare_parameter("nav_max_attempts", 2)
        self.declare_parameter("scan_timeout_s", 5.0)
        # Minimum time to dwell in SCANNING after a successful reading, so the
        # patrol arm reaches the inspect pose and the flower detector +
        # aggregator (which only attribute a bloom while mission_phase==SCANNING)
        # can read the colour before we advance. 0 = no dwell (advance as soon
        # as the bridge replies). sim sets this when arm patrol is on.
        self.declare_parameter("flower_scan_dwell_s", 0.0)
        # Visual confirmation gate. False (default) = bridge oracle path,
        # which is what sim uses. True = call /perception/confirm_tag before
        # the bridge — for hardware where the camera must actually see the
        # AprilTag before we trust the reading. The perception node ships
        # in a follow-up MR; this MR only defines the interface.
        self.declare_parameter("require_visual_confirmation", False)
        self.declare_parameter(
            "visual_confirmation_service",
            "/perception/confirm_tag",
        )
        self.declare_parameter("visual_confirmation_timeout_s", 3.0)

        # Per-leg AMCL drift gate. Re-uses the start-of-mission covariance
        # check (max diagonal of x/y/yaw) before *each* NavigateToPose so
        # we don't drive to a goal pose computed from a stale localisation.
        # Default = same value as the initial localization gate, so existing
        # tunings carry over; hardware can loosen it via launch arg if AMCL
        # is borderline-stable in low-feature aisles.
        self.declare_parameter(
            "nav_localization_cov_threshold",
            0.25,
        )

        self.declare_parameter("dock_pose", [0.0, 0.0, 0.0])
        self.declare_parameter("dock_timeout_s", 60.0)
        self.declare_parameter(
            "battery_topic", "/io/power/power_watcher"
        )  # real MIRTE topic; sim publisher mirrors it
        self.declare_parameter(
            "battery_low_threshold", 0.20
        )  # fraction 0–1; triggers docking below this

        # ─── exploration / monitoring (ExplorationMission) ──────────────
        # Default number of distinct AprilTags to discover before switching to
        # the monitoring loop. /mission/start can override per-request.
        self.declare_parameter('discovery_goal', 5)
        self.declare_parameter('discovered_tags_topic', '/perception/discovered_tags')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('base_frame', 'base_link')
        # Frontier exploration tuning (see frontier.select_frontier_goal).
        self.declare_parameter('frontier_free_thresh', 20)
        self.declare_parameter('frontier_occupied_thresh', 65)
        self.declare_parameter('frontier_min_cluster_cells', 6)
        self.declare_parameter('frontier_robot_radius_cells', 4)
        # Give up exploring after this long (whatever was found switches to
        # monitoring, or RETURNING if nothing). Also paces the no-frontier
        # bootstrap wait while SLAM fills the first scans.
        self.declare_parameter('exploration_timeout_s', 180.0)

        # ─── per-pot arm patrol (optional; sim flower-scan demo) ────────
        # When enabled, the orchestrator strikes a named arm pose at each pot
        # so the gripper/wrist camera frames the bloom for the flower
        # detector, and returns to a travel pose between pots. Default OFF so
        # hardware behaviour is unchanged unless explicitly enabled (the arm
        # otherwise stays parked at home for the whole mission). sim_full
        # turns it on. Fire-and-forget — a missing /lupin/arm/preset service
        # never stalls a scan.
        self.declare_parameter('arm_patrol_enabled', False)
        self.declare_parameter('arm_preset_service', '/lupin/arm/preset')
        self.declare_parameter('arm_inspect_preset', 'inspect')
        self.declare_parameter('arm_travel_preset', 'home')
        # Seconds to let the travel preset fold the arm in before driving off
        # (0 = drive immediately, the hardware default). Sim patrol sets ~3.2 so
        # the arm doesn't sweep through the pots mid-trajectory.
        self.declare_parameter('arm_travel_settle_s', 0.0)
        self.declare_parameter('monitoring_sweeps', 1)

        self.declare_parameter("state_publish_rate_hz", 5.0)
        self.declare_parameter("mission_id_prefix", "lupin")
        self.declare_parameter("frame_id", "map")

        self._nav_action_name = str(self.get_parameter("nav_action_name").value)
        self._bridge_service_name = str(self.get_parameter("bridge_service_name").value)
        self._estop_topic = str(self.get_parameter("estop_topic").value)
        self._dependency_timeout = float(
            self.get_parameter("dependency_timeout_s").value
        )

        self._map_yaml_path = str(self.get_parameter("map_yaml_path").value)
        self._localization_timeout = float(
            self.get_parameter("localization_timeout_s").value
        )
        self._localization_cov_thresh = float(
            self.get_parameter("localization_covariance_threshold").value
        )
        self._amcl_pose_topic = str(self.get_parameter("amcl_pose_topic").value)

        self._approach_yaw = float(self.get_parameter("approach_yaw").value)
        self._approach_standoff = float(self.get_parameter("approach_standoff_m").value)
        self._approach_overrides_file = str(
            self.get_parameter("approach_overrides_file").value or ""
        )
        self._nav_timeout = float(self.get_parameter("nav_timeout_s").value)
        self._nav_max_attempts = int(self.get_parameter("nav_max_attempts").value)
        self._scan_timeout = float(self.get_parameter("scan_timeout_s").value)
        self._flower_scan_dwell = float(self.get_parameter("flower_scan_dwell_s").value)
        self._scan_dwell_timer = None
        self._require_visual_confirmation = bool(
            self.get_parameter("require_visual_confirmation").value
        )
        self._visual_confirmation_service = str(
            self.get_parameter("visual_confirmation_service").value
        )
        self._visual_confirmation_timeout = float(
            self.get_parameter("visual_confirmation_timeout_s").value
        )
        self._nav_localization_cov_thresh = float(
            self.get_parameter("nav_localization_cov_threshold").value
        )

        self._dock_pose = list(self.get_parameter("dock_pose").value or [0.0, 0.0, 0.0])
        self._dock_timeout = float(self.get_parameter("dock_timeout_s").value)

        self._discovery_goal_param = int(self.get_parameter('discovery_goal').value)
        self._discovered_tags_topic = str(
            self.get_parameter('discovered_tags_topic').value
        )
        self._map_topic = str(self.get_parameter('map_topic').value)
        self._base_frame = str(self.get_parameter('base_frame').value)
        self._frontier_free_thresh = int(self.get_parameter('frontier_free_thresh').value)
        self._frontier_occupied_thresh = int(
            self.get_parameter('frontier_occupied_thresh').value
        )
        self._frontier_min_cluster = int(
            self.get_parameter('frontier_min_cluster_cells').value
        )
        self._frontier_robot_radius = int(
            self.get_parameter('frontier_robot_radius_cells').value
        )
        self._exploration_timeout = float(
            self.get_parameter('exploration_timeout_s').value
        )

        state_rate = float(self.get_parameter('state_publish_rate_hz').value)
        self._state_publish_period = 1.0 / max(state_rate, 0.1)
        self._mission_id_prefix = str(self.get_parameter("mission_id_prefix").value)
        self._frame_id = str(self.get_parameter("frame_id").value)

        # ─── tag locations ─────────────────────────────────────────────
        # Loaded once on startup; the bridge uses string IDs.
        tag_file = str(self.get_parameter("tag_locations_file").value or "")
        self._tag_locations: dict = load_default_tag_locations(tag_file or None)
        # Tables drive per-tag approach-pose geometry (the robot parks on the
        # outside of the nearest table edge). Loaded from the same JSON.
        self._table_locations: dict = load_default_tables(tag_file or None)
        if tag_file:
            self.get_logger().info(
                f"Loaded tag locations from {tag_file} "
                f"(tags={len(self._tag_locations)}, tables={len(self._table_locations)})"
            )

        # Per-tag approach-pose overrides — operator-tunable, optional. Empty
        # path = no overrides, sim leaves it that way; hardware populates the
        # YAML when geometry doesn't match the physical layout.
        try:
            self._approach_overrides = load_approach_overrides(
                self._approach_overrides_file
            )
        except (FileNotFoundError, ValueError) as exc:
            # Misconfigured override file is operator-fixable — log loudly
            # and continue with no overrides rather than blocking startup.
            self.get_logger().error(
                f"Failed to load approach overrides ({self._approach_overrides_file}): {exc}"
            )
            self._approach_overrides = {}
        if self._approach_overrides:
            self.get_logger().info(
                f"Loaded {len(self._approach_overrides)} approach override(s) from "
                f"{self._approach_overrides_file}"
            )

        # Optional preset tag_sequence parameter — if non-empty, used as
        # the default when /mission/start passes an empty tag_sequence.
        # Keeps callers' YAML param files useful without forcing them to
        # repeat the list in every service request.
        try:
            preset = self.get_parameter("tag_sequence").value or []
        except rclpy.exceptions.ParameterUninitializedException:
            preset = []
        self._default_tag_sequence: list[str] = [str(t) for t in preset]

        # ─── runtime state ─────────────────────────────────────────────
        # Active mission model — InspectionMission, ExplorationMission, or
        # MonitoringMission (the latter swapped in when exploration hits N).
        self._mission = None
        # User-facing mission type for /mission/state. Stays "ExplorationMission"
        # across the EXPLORING→MONITORING swap even though the model changes.
        self._mission_type: str = ''
        self._mission_started_at = TimeMsg()  # zero-stamp until first start
        self._last_error: str = ""

        # ─── exploration / discovery state ─────────────────────────────
        # Discovered tags keyed by id → lupin_msgs/DiscoveredTag, fed by the
        # /perception/discovered_tags subscription.
        self._discovered: dict = {}
        self._active_discovery_goal: int = 0
        self._latest_map: Optional[OccupancyGrid] = None
        self._exploring_started_at: float = self._monotonic()
        # When the most recent frontier goal was attempted — throttles the
        # bootstrap re-select so a frontier-less map doesn't spin at 10 Hz.
        self._last_frontier_attempt: float = self._monotonic()
        # First time select returned no frontier (None when frontiers exist);
        # paired with the exploration timeout to bound the bootstrap wait.
        self._no_frontier_since: Optional[float] = None

        # safety flags — see module docstring for the model.
        self._paused: bool = False
        self._estop_engaged: bool = False  # mirrors EStopMonitor.engaged
        self._return_resumable = False
        # Which mission family a battery/dock RETURNING should resume into —
        # captured the moment we divert to the dock, so resume re-enters the
        # right sub-machine instead of always falling into INSPECTING (which
        # would, e.g., terminate a MONITORING loop early). One of
        # 'INSPECTING' | 'MONITORING' | 'EXPLORING' | None.
        self._return_origin: Optional[str] = None
        self._manual_dock_requested = False

        # in-flight Nav2 + bridge futures, used both as identity guards
        # against stale callbacks and as cancel handles for pause/abort.
        self._nav_goal_handle = None
        self._nav_send_goal_future = None
        self._nav_get_result_future = None
        self._nav_state_started_at: float = self._monotonic()
        # Set when the operator cancels (pause / e-stop / abort / skip)
        # between send_goal_async and the server's accept response. The
        # accept callback checks this and cancels the just-accepted goal
        # so a pending goal that lands AFTER the cancel doesn't drive the
        # mission forward unexpectedly.
        self._nav_pending_cancel: bool = False
        # One-shot timer armed when RETURNING defers its dock goal behind a
        # pending cancel; forces the dock goal if the cancel ack never arrives.
        self._return_cancel_watchdog = None
        self._scan_future = None
        self._scan_started_at: float = self._monotonic()
        # Track whether we've requested a return (via abort or battery)
        self._return_requested = False

        # Visual-confirmation in-flight state. None when not waiting on
        # /perception/confirm_tag. Watchdog enforces the timeout.
        self._confirm_future = None
        self._confirm_started_at: float = self._monotonic()

        # tracked separately from the nav-attempt counter so retry logic
        # is per-tag (counter on TagResult), but timeouts are per-attempt.
        # bookkeeping for the watchdog.

        # Latest AMCL pose snapshot, consumed by _poll_localization.
        self._latest_amcl_pose: Optional[PoseWithCovarianceStamped] = None
        # When PREPARE_LOCALIZING was entered (set in on_enter); compared
        # to localization_timeout_s by the watchdog.
        self._localizing_started_at: float = self._monotonic()

        # ─── TF (live robot pose for frontier scoring + discovered approach) ─
        # In SLAM mode /amcl_pose is a static seed, so the real robot pose
        # comes from the map→base_frame TF, not AMCL.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ─── callback group ────────────────────────────────────────────
        # Mutually-exclusive: state machine transitions are serialised
        # under a single-threaded executor, just like v1.
        self._cb_group = MutuallyExclusiveCallbackGroup()

        # ─── HSM ───────────────────────────────────────────────────────
        spec = build_hsm_spec()
        # send_event=True passes EventData into transition callbacks, used
        # by _log_transition to report the trigger name cleanly.
        self._machine = HierarchicalGraphMachine(
            model=self,
            states=spec["states"],
            transitions=spec["transitions"],
            initial=spec["initial"],
            send_event=True,
            queued=True,
            # A stale trigger from an async ROS callback must NO-OP, not raise.
            # The node is driven by concurrent Nav2/bridge/scan callbacks; the
            # per-callback entry guards (e.g. _on_scan_response, line ~2069)
            # catch most stale fires, but under queued=True a trigger that was
            # valid when enqueued (scan_done from SCANNING) can be drained AFTER
            # a concurrently-enqueued abort_to_return/battery divert has moved us
            # to RETURNING. With ignore=False that raised MachineError inside the
            # executor thread and wedged the spin loop (mission stuck in
            # RETURNING, never docking). True makes the stale trigger a graceful
            # no-op — the entry guards remain as defence-in-depth.
            ignore_invalid_triggers=True,
            after_state_change="_log_transition",
        )
        # Recorded by _log_transition so the published MissionState reflects
        # post-transition values consistently.
        self._last_transition_event: str = ""

        # ─── ROS interfaces ────────────────────────────────────────────
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            self._nav_action_name,
            callback_group=self._cb_group,
        )
        self._bridge_client = self.create_client(
            GetTagReading,
            self._bridge_service_name,
            callback_group=self._cb_group,
        )
        # Perception client only created when the gate is enabled. Sim
        # leaves _confirm_client = None and the SCANNING entry skips
        # straight to the bridge call.
        self._confirm_client: Optional[Any] = None
        if self._require_visual_confirmation:
            self._confirm_client = self.create_client(
                ConfirmTag,
                self._visual_confirmation_service,
                callback_group=self._cb_group,
            )

        # Per-pot arm patrol client (optional). Only created when enabled so
        # there's no dangling client / discovery traffic on hardware runs that
        # leave the arm parked.
        self._arm_patrol_enabled = bool(self.get_parameter('arm_patrol_enabled').value)
        self._arm_inspect_preset = str(self.get_parameter('arm_inspect_preset').value)
        self._arm_travel_preset = str(self.get_parameter('arm_travel_preset').value)
        self._monitoring_sweeps = max(1, int(self.get_parameter('monitoring_sweeps').value))
        self._arm_travel_settle_s = float(
            self.get_parameter('arm_travel_settle_s').value
        )
        self._arm_settle_timer: Optional[Any] = None
        self._arm_preset_client: Optional[Any] = None
        if self._arm_patrol_enabled:
            self._arm_preset_client = self.create_client(
                SetArmPreset,
                str(self.get_parameter('arm_preset_service').value),
                callback_group=self._cb_group,
            )
            self.get_logger().info(
                'arm patrol enabled — inspect="%s", travel="%s"'
                % (self._arm_inspect_preset, self._arm_travel_preset)
            )

        # Observations: RELIABLE + TRANSIENT_LOCAL with depth 50 so a late
        # subscriber (e.g. the web Mission tab) sees the mission so far.
        self._obs_pub = self.create_publisher(
            Observation,
            "/floranova/observations",
            QoSProfile(
                depth=50,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        # Mission state: RELIABLE + TRANSIENT_LOCAL depth 1 — latched, so
        # late subscribers immediately see the current snapshot.
        self._state_pub = self.create_publisher(
            MissionState,
            "/mission/state",
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # AMCL pose for the localization gate. Nav2 publishes /amcl_pose
        # with TRANSIENT_LOCAL+depth 1 (latched), so a late subscriber
        # — like us, started after Nav2 — still sees the most recent
        # pose. Match that QoS or the topic stays empty.
        self._amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self._amcl_pose_topic,
            self._on_amcl_pose,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
            callback_group=self._cb_group,
        )

        # Live SLAM map for frontier exploration. slam_toolbox publishes /map
        # RELIABLE + TRANSIENT_LOCAL (latched); match it so a late subscriber
        # gets the current grid.
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            self._map_topic,
            self._on_map,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
            callback_group=self._cb_group,
        )

        # Discovered-tag feed from perception_aggregator (latched). Counted
        # during EXPLORING; poses seed the MONITORING approach goals.
        self._discovered_sub = self.create_subscription(
            DiscoveredTags,
            self._discovered_tags_topic,
            self._on_discovered_tags,
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
            callback_group=self._cb_group,
        )

        # E-stop monitor.
        self._estop = EStopMonitor(
            self,
            self._estop_topic,
            on_engaged=self._on_estop_engaged,
            on_released=self._on_estop_released,
            callback_group=self._cb_group,
        )

        # Battery monitor.
        # True from on_battery_low() until battery recovers; keeps is_blocked() True
        # so no new nav goals are issued while the robot heads to the dock.
        self._battery_low: bool = False
        self._docked_for_battery: bool = False
        self.battery_monitor = BatteryMonitor(
            self,
            str(self.get_parameter("battery_topic").value),
            low_threshold=float(self.get_parameter("battery_low_threshold").value),
            on_low=self.on_battery_low,
            on_recovered=self.on_battery_recovered,
            callback_group=self._cb_group,
        )

        # Operator services. Created with their absolute names per spec.
        self._srv_start = self.create_service(
            StartMission,
            "/mission/start",
            self._handle_start_mission,
            callback_group=self._cb_group,
        )
        self._srv_pause = self.create_service(
            Trigger,
            "/mission/pause",
            self._handle_pause,
            callback_group=self._cb_group,
        )
        self._srv_resume = self.create_service(
            Trigger,
            "/mission/resume",
            self._handle_resume,
            callback_group=self._cb_group,
        )
        self._srv_abort = self.create_service(
            Trigger,
            "/mission/abort",
            self._handle_abort,
            callback_group=self._cb_group,
        )
        self._srv_skip = self.create_service(
            Trigger,
            "/mission/skip_current",
            self._handle_skip_current,
            callback_group=self._cb_group,
        )

        # Manually activated docking (by operator)
        self._srv_dock = self.create_service(
            Trigger,
            "/mission/dock",
            self._handle_dock,
            callback_group=self._cb_group,
        )

        # ─── timers ────────────────────────────────────────────────────
        # State publisher: fixed rate, always running.
        self._state_timer = self.create_timer(
            self._state_publish_period,
            self._publish_state,
            callback_group=self._cb_group,
        )
        # Watchdog: drives BOOT dependency polling, NAVIGATING / SCANNING
        # timeouts, PREPARE.LOCALIZING covariance polling. 100 ms cadence.
        self._watchdog = self.create_timer(
            0.1,
            self._on_watchdog,
            callback_group=self._cb_group,
        )
        # When BOOT was entered. Compared to dependency_timeout_s.
        self._boot_started_at: float = self._monotonic()

        self.get_logger().info(
            f"Mission orchestrator (v2) up. "
            f"nav_action={self._nav_action_name}, "
            f"bridge_service={self._bridge_service_name}, "
            f"estop_topic={self._estop_topic}"
        )
        self.get_logger().info(
            f"Waiting up to {self._dependency_timeout:.1f}s for nav2 + bridge..."
        )

    # ─── time helper ───────────────────────────────────────────────────
    @staticmethod
    def _monotonic() -> float:
        return time.monotonic()

    # ─── lifecycle logging ─────────────────────────────────────────────
    def _log_transition(self, event_data) -> None:
        """``transitions`` after_state_change hook: log every transition.

        Format: ``[lifecycle.sub] EVENT: from→to (tag_id=X, mission_id=Y)``.
        Run via the machine, so individual handlers don't need log calls.
        """
        trigger = event_data.event.name if event_data and event_data.event else "?"
        src = (
            event_data.transition.source
            if event_data and event_data.transition
            else "?"
        )
        dst = self.state
        tag_id = self._mission.current_tag_id() if self._mission is not None else ""
        mission_id = self._mission.mission_id if self._mission is not None else ""
        # `[lifecycle.sub]` prefix: derive from the source state name (the
        # state we're leaving). Top-level states have no separator; nested
        # states are e.g. INSPECTING_NAVIGATING → "[INSPECTING.NAVIGATING]".
        self.get_logger().info(
            f'[{src.replace("_", ".")}] {trigger.upper()}: '
            f'{src}→{dst} (tag_id={tag_id or "-"}, mission_id={mission_id or "-"})'
        )
        self._last_transition_event = trigger

    # ─── safety: pause / e-stop ────────────────────────────────────────
    def _is_blocked(self) -> bool:
        """True iff the orchestrator must hold the active state.

        Either the operator has paused, the E-stop is engaged, or the battery
        is low and the robot is heading to the dock. All three require an
        explicit /mission/resume to unblock.
        """
        return self._paused or self._estop_engaged or self._battery_low

    def _on_estop_engaged(self) -> None:
        """Rising edge of /e_stop_state. Hold pose; require explicit resume."""
        self._estop_engaged = True
        # Implicit pause — release alone does not auto-resume per spec.
        self._paused = True
        self._cancel_inflight_nav("estop_engaged")
        self._last_error = "estop_engaged"

    def _on_estop_released(self) -> None:
        """Falling edge: clear engaged flag, but stay paused awaiting resume."""
        self._estop_engaged = False
        # _paused intentionally untouched.

    def on_battery_low(self) -> None:
        # Rising edge: battery below threshold or insufficient time to reach dock.
        # Cancel any in-flight nav goal, redirect to RETURNING (dock pose).
        # Mirrors on_estop_engaged — operator must call /mission/resume after docking.

        if self._battery_low:
            return
        self._battery_low = True
        self._last_error = "battery_low"

        if self.state == "RETURNING":
            # Already heading to dock — let it complete naturally.

            return

        if self.state in ("READY", "DONE"):
            self._paused = True
            self._manual_dock_requested = False

            self.get_logger().info(
                "Battery low while idle at dock. Entering paused state until charged."
            )
            return

        self._manual_dock_requested = False
        self._docked_for_battery = True
        self._return_resumable = True
        self._return_requested = True
        self._cancel_inflight_nav("battery_low")

        if _is_state_inspecting(self.state) or _is_state_monitoring(self.state) or self.state == 'EXPLORING':
            self._return_origin = _resume_origin_for(self.state)
            self.abort_to_return()  # type: ignore[attr-defined]

    def on_battery_recovered(self) -> None:
        # Falling edge: battery rose back above threshold (e.g. after charging).
        # Clear the flag but leave paused=True — operator resumes explicitly,
        # same policy as E-stop release.
        self._battery_low = False
        self.get_logger().info(
            "Battery recovered above threshold; awaiting /mission/resume."
        )

    # ─── nav cancellation ──────────────────────────────────────────────
    # Operator-initiated cancels (pause / E-stop / abort / skip) drop the
    # goal handle and DECREMENT the attempt counter — this attempt didn't
    # really happen, so the resume kick should re-issue at the same count.
    # Nav2-internal failures (ABORTED, REJECTED, timeout) keep the attempt
    # counted; retry logic lives in _on_inspection_nav_result and
    # _check_nav_timeout.

    def _cancel_inflight_nav(self, reason: str, *, refund_attempt: bool = True) -> None:
        had_inflight = (
            self._nav_goal_handle is not None or self._nav_send_goal_future is not None
        )
        if self._nav_goal_handle is not None:
            try:
                self._nav_pending_cancel = True
                self._nav_goal_handle.cancel_goal_async()
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().warn(f"cancel_goal_async raised: {exc!r}")
        elif self._nav_send_goal_future is not None:
            # Goal was sent but server hasn't accepted yet — flag the
            # accept callback to cancel the goal as soon as it lands.
            self._nav_pending_cancel = True
        if not had_inflight:
            return
        # Refund the attempt: this particular goal didn't get a chance
        # to fail naturally, so it shouldn't burn a retry budget.
        if (
            refund_attempt
            and self._mission is not None
            and not self._mission.is_complete()
        ):
            r = self._mission.current_result()
            if r.nav_attempts > 0:
                r.nav_attempts -= 1
        self.get_logger().info(f"Nav2 goal cancelled ({reason}).")

    # ─── watchdog ──────────────────────────────────────────────────────
    def _on_watchdog(self) -> None:
        """Periodic tick. Per-state housekeeping; non-blocking.

        Only the states that need polling do work here. Most state actions
        are event-driven (action/service callbacks).
        """
        state = self.state
        if state == "BOOT":
            self._poll_boot_dependencies()
        elif state == "PREPARE_LOCALIZING":
            self._poll_localization()
        elif state == 'EXPLORING':
            self._poll_exploration()
        elif state in ('INSPECTING_NAVIGATING', 'MONITORING_NAVIGATING'):
            self._check_nav_timeout()
        elif state in ('INSPECTING_SCANNING', 'MONITORING_SCANNING'):
            # Mid-state we may be waiting on either /perception/confirm_tag
            # or the bridge — the corresponding future is non-None. Both
            # have their own timeouts; check whichever is in flight.
            if self._confirm_future is not None:
                self._check_confirm_timeout()
            else:
                self._check_scan_timeout()
        # READY, *_PUBLISHING, RETURNING, DONE, FAULT: nothing for the
        # watchdog to do.

    # Stubs filled in by later sections — declared here so the watchdog
    # body above type-checks. Concrete logic lands with the inspection
    # sub-machine and the prepare/returning slices.

    def _poll_boot_dependencies(self) -> None:
        elapsed = self._monotonic() - self._boot_started_at
        nav_ready = self._nav_client.server_is_ready()
        bridge_ready = self._bridge_client.service_is_ready()
        # When visual confirmation is required, the perception service is a
        # hard dependency — refuse to leave BOOT without it. Sim leaves the
        # gate off and skips this branch entirely.
        confirm_ready = (
            self._confirm_client is None or self._confirm_client.service_is_ready()
        )
        if nav_ready and bridge_ready and confirm_ready:
            self.get_logger().info("Dependencies up. Orchestrator READY.")
            self.deps_up()  # type: ignore[attr-defined]
            return
        if elapsed > self._dependency_timeout:
            missing = []
            if not nav_ready:
                missing.append(self._nav_action_name)
            if not bridge_ready:
                missing.append(self._bridge_service_name)
            if not confirm_ready:
                missing.append(self._visual_confirmation_service)
            self._last_error = f'dependency_timeout: {", ".join(missing)}'
            self.get_logger().error(
                f"Dependencies did not appear within "
                f'{self._dependency_timeout:.1f}s; missing: {", ".join(missing)}'
            )
            self.fault()  # type: ignore[attr-defined]

    def _poll_localization(self) -> None:
        if self._is_blocked():
            return
        elapsed = self._monotonic() - self._localizing_started_at
        cov = self._latest_amcl_diag()
        if cov is not None and cov <= self._localization_cov_thresh:
            self.get_logger().info(
                f"Localization confident (max diag cov {cov:.3f} "
                f"<= {self._localization_cov_thresh:.3f}); proceeding."
            )
            self.localized()  # type: ignore[attr-defined]
            return
        if elapsed > self._localization_timeout:
            cov_str = f"{cov:.3f}" if cov is not None else "no /amcl_pose received"
            self._last_error = f"localization_failed ({cov_str})"
            self.get_logger().error(
                f"Localization did not converge within "
                f"{self._localization_timeout:.1f}s ({cov_str}); "
                f"transitioning to FAULT."
            )
            self.fault()  # type: ignore[attr-defined]

    def _latest_amcl_diag(self) -> Optional[float]:
        """Return max(x_var, y_var, yaw_var) from latest AMCL pose, or None."""
        msg = self._latest_amcl_pose
        if msg is None:
            return None
        c = msg.pose.covariance  # 36-vector, row-major 6x6
        # Indices: x=0, y=7, yaw=35 (z/roll/pitch are meaningless for our
        # 2D pose; AMCL leaves them huge).
        return float(max(c[0], c[7], c[35]))

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_amcl_pose = msg

    # ─── map / discovery / robot-pose helpers ───────────────────────────
    def _on_map(self, msg: OccupancyGrid) -> None:
        self._latest_map = msg

    def _on_discovered_tags(self, msg: DiscoveredTags) -> None:
        self._discovered = {t.tag_id: t for t in msg.tags}
        if isinstance(self._mission, ExplorationMission):
            self._mission.update_discovered(
                {tid: t.pose_in_map for tid, t in self._discovered.items()}
            )

    def _robot_xy(self) -> Optional[tuple[float, float]]:
        """Live robot (x, y) in the map frame via TF.

        In SLAM mode /amcl_pose is a static seed, so the real pose is the
        map→base_frame transform. Zero-timeout lookup (the single-threaded
        executor that fills the TF buffer must not block on itself); returns
        None until the tree is up.
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self._frame_id, self._base_frame, RclpyTime(),
                timeout=Duration(seconds=0.0),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        return (float(tf.transform.translation.x), float(tf.transform.translation.y))

    # ─── HSM transition conditions ──────────────────────────────────────
    def _is_exploration_mission(self, event_data=None) -> bool:
        return isinstance(self._mission, ExplorationMission)

    def _has_discovered_any(self, event_data=None) -> bool:
        return len(self._discovered) > 0

    # ─── nav localization gate ──────────────────────────────────────────
    def _presence_only_gate(self) -> bool:
        """Exploration-type missions run in SLAM mode where the seeded
        /amcl_pose never updates, so the tight per-leg covariance gate would
        wrongly fail every leg. For EXPLORING/MONITORING we gate on pose
        *presence* and trust slam_toolbox's map→odom. InspectionMission keeps
        the tight gate."""
        return isinstance(self._mission, (ExplorationMission, MonitoringMission))

    def _nav_localization_ok(self) -> tuple[bool, str]:
        """(ok, detail) for whether localization is good enough to send a goal."""
        cov = self._latest_amcl_diag()
        if self._presence_only_gate():
            if self._latest_amcl_pose is None:
                return False, 'no_pose'
            return True, ''
        if cov is None or cov > self._nav_localization_cov_thresh:
            return False, f'amcl_drift_var={cov:.3f}' if cov is not None else 'amcl_drift_var=unknown'
        return True, ''

    def _check_nav_timeout(self) -> None:
        if self._is_blocked() or self._nav_goal_handle is None:
            return
        if self._monotonic() - self._nav_state_started_at <= self._nav_timeout:
            return
        # Treat the timeout as an in-progress attempt that just failed —
        # the attempt has already been counted, so do NOT refund.
        self.get_logger().warn(
            f"Nav2 timeout after {self._nav_timeout:.1f}s for tag "
            f'{self._mission.current_tag_id() if self._mission else "?"}'
        )
        self._cancel_inflight_nav("nav_timeout", refund_attempt=False)
        self._handle_nav_failure("nav_timeout")

    def _check_scan_timeout(self) -> None:
        if self._is_blocked() or self._scan_future is None:
            return
        if self._monotonic() - self._scan_started_at <= self._scan_timeout:
            return
        self.get_logger().warn(
            f"Bridge timeout after {self._scan_timeout:.1f}s for tag "
            f'{self._mission.current_tag_id() if self._mission else "?"}'
        )
        # rclpy futures don't really cancel; just drop the reference so
        # the stale-future guard in _on_scan_response ignores any late
        # response that arrives after this point.
        self._scan_future = None
        if self._mission is not None and not self._mission.is_complete():
            result = self._mission.mark_scan_failed("scan_timeout")
            self._emit_observation_for(result)
        self.scan_done()  # type: ignore[attr-defined]

    def _check_confirm_timeout(self) -> None:
        """Mirror of :meth:`_check_scan_timeout` for /perception/confirm_tag.

        Drops the in-flight future, marks the tag SCAN_FAILED with a
        dedicated detail string the operator can grep ('confirm_timeout'),
        and advances the FSM. Same shape as the scan-timeout path so the
        scan/publish/advance flow downstream is unchanged.
        """
        if self._is_blocked() or self._confirm_future is None:
            return
        elapsed = self._monotonic() - self._confirm_started_at
        if elapsed <= self._visual_confirmation_timeout:
            return
        self.get_logger().warn(
            f"Visual confirm timeout after {self._visual_confirmation_timeout:.1f}s "
            f"for tag "
            f'{self._mission.current_tag_id() if self._mission else "?"}'
        )
        self._confirm_future = None
        if self._mission is not None and not self._mission.is_complete():
            result = self._mission.mark_scan_failed("confirm_timeout")
            self._emit_observation_for(result)
        self.scan_done()  # type: ignore[attr-defined]

    # ─── service handlers ──────────────────────────────────────────────
    def _handle_start_mission(
        self, request: StartMission.Request, response: StartMission.Response
    ) -> StartMission.Response:
        if self.state == "FAULT":
            response.accepted = False
            response.error_message = "orchestrator in FAULT"
            return response
        if self._battery_low:
            response.accepted = False
            response.error_message = "cannot start mission because battery is low"
            return response
        if _is_state_busy(self.state):
            response.accepted = False
            response.error_message = f"mission already running ({self.state})"
            return response
        mission_type = request.mission_type or 'InspectionMission'
        if mission_type not in ('InspectionMission', 'ExplorationMission'):
            response.accepted = False
            response.error_message = (
                f"unknown mission_type '{request.mission_type}' — only "
                "'InspectionMission' and 'ExplorationMission' are supported"
            )
            return response

        mission_id = f'{self._mission_id_prefix}-{uuid.uuid4().hex[:8]}'

        if mission_type == 'ExplorationMission':
            goal = int(request.discovery_goal) or self._discovery_goal_param
            if goal <= 0:
                response.accepted = False
                response.error_message = 'discovery_goal must be > 0'
                return response
            # Fresh discovery state — drop any tags discovered on a prior run.
            self._discovered = {}
            self._active_discovery_goal = goal
            self._mission = ExplorationMission(
                mission_id=mission_id, discovery_goal=goal,
            )
        else:
            # Build the tag sequence. Precedence: explicit request → preset
            # tag_sequence parameter → all tags from tag_locations.json.
            if request.tag_sequence:
                tag_sequence = [str(t) for t in request.tag_sequence]
            elif self._default_tag_sequence:
                tag_sequence = list(self._default_tag_sequence)
            else:
                tag_sequence = sorted(
                    self._tag_locations.keys(), key=numeric_string_sort_key,
                )
            unknown = [t for t in tag_sequence if t not in self._tag_locations]
            if unknown:
                response.accepted = False
                response.error_message = (
                    f'unknown tag id(s) in tag_sequence: {unknown}'
                )
                return response
            if not tag_sequence:
                response.accepted = False
                response.error_message = 'no tags to visit (empty tag_locations)'
                return response
            self._active_discovery_goal = 0
            self._mission = InspectionMission(
                mission_id=mission_id,
                tag_sequence=tag_sequence,
                tag_locations=self._tag_locations,
                nav_max_attempts=self._nav_max_attempts,
                approach_yaw=self._approach_yaw,
            )

        self._mission_type = mission_type
        self._mission_started_at = self.get_clock().now().to_msg()
        self._last_error = ""
        # Resetting safety flags so a new mission starts clean. E-stop
        # engagement at this moment will fire the engagement callback
        # again on the next message and re-block.
        self._paused = False
        self._return_requested = False
        self._return_resumable = False
        self._docked_for_battery = False
        self._manual_dock_requested = False

        # State path: READY → PREPARE → EXPLORING/INSPECTING. From DONE we
        # first have to bounce through READY for the next mission.
        if self.state == 'DONE':
            self.reset_for_next()  # type: ignore[attr-defined]
        self.start_mission()  # type: ignore[attr-defined]

        response.accepted = True
        response.mission_id = mission_id
        return response

    def _handle_pause(self, request, response):  # std_srvs/Trigger
        if not _is_state_busy(self.state):
            response.success = False
            response.message = f"no active mission to pause (state={self.state})"
            return response
        if self._paused:
            response.success = False
            response.message = "already paused"
            return response
        self._paused = True
        self._cancel_inflight_nav("paused_by_operator")
        response.success = True
        response.message = "paused"
        return response

    def _handle_resume(self, request, response):  # std_srvs/Trigger
        if not self._paused:
            response.success = False
            response.message = "not currently paused"
            return response
        if self._estop_engaged:
            response.success = False
            response.message = "cannot resume while E-stop is engaged"
            return response
        if self._battery_low:
            response.success = False
            response.message = (
                "cannot resume while battery is low — charge the robot first"
            )
            return response

        self._paused = False

        # Special case: robot is already docked after low-battery return.
        # In that case, resume means continue the mission, not re-send dock nav.
        if (
            self.state == "RETURNING"
            and self._return_resumable
            and (self._docked_for_battery or self._manual_dock_requested)
            and self._mission is not None
            and not self._mission.is_complete()
        ):
            self._docked_for_battery = False
            self._return_resumable = False
            self._manual_dock_requested = False
            # Resume into the family we left, not always INSPECTING.
            origin = self._return_origin
            self._return_origin = None
            if origin == 'MONITORING':
                self.resume_monitoring()  # type: ignore[attr-defined]
            elif origin == 'EXPLORING':
                self.resume_exploration()  # type: ignore[attr-defined]
            else:
                self.resume_inspection()  # type: ignore[attr-defined]
            response.success = True
            response.message = "resumed"
            return response

        self._docked_for_battery = False
        self._return_resumable = False
        self._manual_dock_requested = False

        # Re-kick the active state so the held action resumes. Only the
        # states that had work-to-do need a kick.
        self._kick_current_state()

        response.success = True
        response.message = "resumed"
        return response

    def _handle_abort(self, request, response):  # std_srvs/Trigger
        if not _is_state_busy(self.state):
            response.success = False
            response.message = f"no active mission to abort (state={self.state})"
            return response
        # Special case: robot already docked and paused after low-battery or manual dock return.
        # Abort should terminate the mission immediately, not try to "return" again.
        if (
            self.state == "RETURNING"
            and self._paused
            and (self._docked_for_battery or self._manual_dock_requested)
        ):
            if self._mission is not None:
                for idx in self._mission.remaining_indices():
                    result = self._mission.force_skip(idx, "mission_aborted")
                    self._emit_observation_for(result)
            self._paused = False
            self._docked_for_battery = False
            self._return_resumable = False
            self._return_requested = False
            self._manual_dock_requested = False
            self.returned()  # type: ignore[attr-defined]
            response.success = True
            response.message = "aborted"
            return response
        if self.state == "RETURNING" and self._battery_low and not self._paused:
            response.success = False
            response.message = (
                "cannot abort: robot is returning to dock due to low battery"
            )
            return response
        self._return_resumable = False
        self._docked_for_battery = False
        self._return_requested = True
        self._manual_dock_requested = False

        self._cancel_inflight_nav("aborted_by_operator")
        # Mark all remaining tags SKIPPED and emit observations for them.
        # Only InspectionMission has a finite remaining-tag list; the
        # monitoring loop and exploration have nothing to "skip".
        if self._mission is not None and _is_state_inspecting(self.state):
            for idx in self._mission.remaining_indices():
                result = self._mission.force_skip(idx, "mission_aborted")
                self._emit_observation_for(result)
        if _is_state_scanning_phase(self.state) or self.state == 'EXPLORING':
            self.abort_to_return()  # type: ignore[attr-defined]
        elif self.state.startswith("PREPARE"):
            # Prep aborted before any tag was attempted; jump straight to
            # DONE via RETURNING so the mission cleans up.
            # transitions doesn't allow a multi-source for the same
            # trigger from PREPARE, so trigger fault → user can restart
            # via a fresh /mission/start. PREPARE-abort is rare; treat as
            # a soft fault rather than a real fault.
            self._last_error = "aborted_in_prepare"
            self.fault()  # type: ignore[attr-defined]
        # RETURNING-abort: no-op, already heading home.
        response.success = True
        response.message = "aborted"
        return response

    def _handle_skip_current(self, request, response):  # std_srvs/Trigger
        if not _is_state_inspecting(self.state):
            response.success = False
            response.message = (
                f"skip_current only valid during INSPECTING (state={self.state})"
            )
            return response
        if self._mission is None or self._mission.is_complete():
            response.success = False
            response.message = "no current tag to skip"
            return response
        self._cancel_inflight_nav("skipped_by_operator")
        result = self._mission.mark_skipped("skipped_by_operator")
        self._emit_observation_for(result)
        # Route to PUBLISHING via the legal trigger for our current sub-
        # state. PUBLISHING's on_enter handles advance + dispatch to
        # next_tag / inspection_complete — don't double-advance.
        self._goto_publishing_from_current()
        response.success = True
        response.message = "skipped"
        return response

    def _handle_dock(self, request, response):
        if not _is_state_busy(self.state) and self.state not in ("READY",):
            response.success = False
            response.message = f"cannot dock in state {self.state}"
            return response
        if self.state == "RETURNING":
            response.success = False
            response.message = "already returning to dock"
            return response
        self._return_resumable = True
        self._docked_for_battery = False
        self._return_requested = True
        self._manual_dock_requested = True

        self._cancel_inflight_nav("manual_dock")
        if _is_state_inspecting(self.state):
            self.abort_to_return()

        response.success = True
        response.message = "heading to dock"
        return response

    # ─── observation / state plumbing ──────────────────────────────────
    def _emit_observation_for(self, result) -> None:
        """Build + publish the Observation for a closed TagResult.

        For OK results we attach the latest AMCL pose so the digital twin
        knows where the robot was standing when it scanned the tag — that's
        the seed for placing tag pins on the operator's map. For non-OK
        results (UNREACHABLE / SCAN_FAILED / SKIPPED) we leave the pose
        unset (orientation.w==0): the robot's pose at that moment isn't a
        meaningful "where is this tag" answer.
        """
        if self._mission is None:
            return
        stamp = self.get_clock().now().to_msg()
        # Pin the tag where it physically is: the discovered-tags feed carries
        # each tag's map-frame pose (perception_aggregator resolves it via TF).
        # Only for OK results — for non-OK we never actually read the tag, so
        # leave the pose unset (the twin drops orientation.w==0 as "missing").
        tag_map_pose = None
        if result.status == Observation.STATUS_OK:
            discovered = self._discovered.get(result.tag_id)
            if discovered is not None:
                tag_map_pose = discovered.pose_in_map
        # Legacy fallback only if we somehow lack the tag's own pose.
        amcl_at_obs = (
            self._latest_amcl_pose
            if (result.status == Observation.STATUS_OK and tag_map_pose is None)
            else None
        )
        msg = make_tag_observation(
            mission_id=self._mission.mission_id,
            source=self._mission.name,
            stamp=stamp,
            status=result.status,
            tag_reading=result.tag_reading,
            status_detail=result.detail,
            frame_id=self._frame_id,
            amcl_pose=amcl_at_obs,
            tag_map_pose=tag_map_pose,
        )
        self._obs_pub.publish(msg)

    def _publish_state(self) -> None:
        msg = MissionState()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        msg.header = header

        state = self.state
        if state.startswith('INSPECTING') or state.startswith('MONITORING'):
            prefix = 'INSPECTING' if state.startswith('INSPECTING') else 'MONITORING'
            msg.lifecycle_state = prefix
            phase = state[len(prefix) + 1:] if '_' in state else ''
            # SCANNING is the umbrella state for both visual confirmation
            # and the bridge call. Surface the finer-grained phase to the
            # operator so the HMI strip and event log can show what the
            # orchestrator is actually waiting on.
            if phase == "SCANNING" and self._confirm_future is not None:
                phase = "CONFIRMING"
            msg.mission_phase = phase
        elif state.startswith("PREPARE"):
            msg.lifecycle_state = "PREPARE"
            msg.mission_phase = state[len("PREPARE_") :] if "_" in state else ""
        else:
            msg.lifecycle_state = state
            msg.mission_phase = ""

        if self._mission is not None:
            msg.mission_id = self._mission.mission_id
            # The user-facing type stays constant across EXPLORING→MONITORING
            # even though the underlying model object swaps.
            msg.mission_type = self._mission_type or self._mission.name
            msg.current_target = self._mission.current_tag_id()
            counters = self._mission.counters()
            msg.targets_total = counters['total']
            msg.targets_completed = counters['completed']
            msg.targets_failed = counters['failed']
            msg.targets_unreachable = counters['unreachable']
            msg.targets_skipped = counters['skipped']
        # Exploration progress — distinct tags found vs the goal N.
        if self._active_discovery_goal > 0:
            msg.tags_discovered = min(len(self._discovered), 65535)
            msg.discovery_goal = min(self._active_discovery_goal, 65535)
        msg.last_error = self._last_error
        msg.estop_engaged = self._estop_engaged
        msg.paused = self._paused
        msg.started_at = self._mission_started_at

        msg.battery_percentage = self.battery_monitor.percentage
        msg.battery_low = self._battery_low

        self._state_pub.publish(msg)

    # ─── kick: re-run the current state's action after resume ──────────
    def _kick_current_state(self) -> None:
        """Re-enter the active state's action after pause/E-stop release.

        Only NAVIGATING and SCANNING have outgoing work that pause/E-stop
        actually interrupts. RETURNING also re-issues its dock goal.
        Everything else is event-driven (action/service callback) and
        naturally re-fires once we stop blocking.
        """
        st = self.state
        if st in ('INSPECTING_NAVIGATING', 'MONITORING_NAVIGATING'):
            if self._nav_goal_handle is None:
                self._send_scan_nav_goal()
        elif st in ('INSPECTING_SCANNING', 'MONITORING_SCANNING'):
            # Re-fire whichever sub-call hasn't started yet. Visual-confirm
            # comes first when enabled; the bridge call only lands once
            # confirmation succeeds. If both futures are None we're either
            # at first entry or just resumed mid-state.
            if self._scan_future is None and self._confirm_future is None:
                if self._require_visual_confirmation:
                    self._call_visual_confirm()
                else:
                    self._call_bridge()
        elif st == 'EXPLORING':
            if self._nav_goal_handle is None:
                self._send_frontier_goal()
        elif st == 'RETURNING':
            if self._nav_goal_handle is None:
                self._send_return_nav_goal()

    # ─── on_enter handlers: the heart of the inspection sub-FSM ────────
    # transitions auto-discovers methods named on_enter_<full_state>; the
    # nested-state form uses the same `_` separator as state names.

    def on_enter_PREPARE_LOCALIZING(self, event_data) -> None:
        self._localizing_started_at = self._monotonic()
        if self._map_yaml_path:
            # TODO(map-loading): load the yaml via the map_server lifecycle
            # interface. For this MR we assume Nav2 already has a map up;
            # warn so the missing piece is visible at runtime.
            self.get_logger().warn(
                f"map_yaml_path={self._map_yaml_path!r} is set but "
                "this build does not load maps — assuming Nav2 already "
                "has one configured."
            )

    # INSPECTING and MONITORING share the same NAVIGATING/SCANNING/PUBLISHING
    # sub-machine, so both on_enter sets delegate to the same handlers. The
    # transitions library routes the shared triggers (nav_succeeded, scan_done,
    # next_tag) to the right sub-state from `self.state`.

    def on_enter_INSPECTING_NAVIGATING(self, event_data) -> None:
        self._enter_navigating()

    def on_enter_MONITORING_NAVIGATING(self, event_data) -> None:
        self._enter_navigating()

    def _enter_navigating(self) -> None:
        # Defensive only: mission-is-None at this depth is a programming
        # error — the only legal entry path is via /mission/start, which
        # creates the mission before triggering any HSM transitions.
        if self._mission is None or self._mission.is_complete():
            self.get_logger().warn(
                "on_enter NAVIGATING with no active mission; holding state."
            )
            return
        if self._is_blocked():
            return
        # Between pots: stow the arm in the travel pose so it isn't waving
        # around mid-drive (and the gripper cam isn't pointed at the floor).
        self._dispatch_arm_preset(self._arm_travel_preset)
        # The preset move takes ~PRESET_TRAVEL_SECONDS to fold the arm in; if we
        # drive immediately the arm sweeps through the pots mid-trajectory. When
        # a settle time is configured (sim patrol) wait for the fold before the
        # nav goal; the callback re-validates state in case we aborted meanwhile.
        if self._arm_patrol_enabled and self._arm_travel_settle_s > 0.0:
            self._schedule_nav_after_arm_settle()
        else:
            self._send_scan_nav_goal()

    def _schedule_nav_after_arm_settle(self) -> None:
        """One-shot: send the scan nav goal after the travel-preset fold has had
        time to finish, so the arm isn't extended while we drive between pots."""
        if self._arm_settle_timer is not None:
            self._arm_settle_timer.cancel()
            self._arm_settle_timer = None

        def _fire() -> None:
            if self._arm_settle_timer is not None:
                self._arm_settle_timer.cancel()
                self._arm_settle_timer = None
            # Re-validate: the mission may have aborted/blocked or advanced
            # during the settle wait.
            if self._mission is None or self._mission.is_complete():
                return
            if self._is_blocked():
                return
            if not str(self.state).endswith('NAVIGATING'):
                return
            self._send_scan_nav_goal()

        self._arm_settle_timer = self.create_timer(
            self._arm_travel_settle_s, _fire,
        )

    def on_enter_INSPECTING_SCANNING(self, event_data) -> None:
        self._enter_scanning()

    def on_enter_MONITORING_SCANNING(self, event_data) -> None:
        self._enter_scanning()

    def _enter_scanning(self) -> None:
        if self._mission is None or self._mission.is_complete():
            self.get_logger().warn(
                "on_enter SCANNING with no active mission; holding state."
            )
            return
        if self._is_blocked():
            return
        # Per-pot patrol: strike the inspect pose so the gripper camera frames
        # the bloom for the flower detector while we read the tag. The flower
        # detector + aggregator attribute whatever they see now to the tag
        # being scanned (temporal co-location), so the arm should be in pose
        # for the duration of the scan.
        self._dispatch_arm_preset(self._arm_inspect_preset)
        # Hardware-style flow: visually confirm the AprilTag is in frame
        # before trusting the bridge reading. Sim leaves the gate off and
        # goes straight to the bridge.
        if self._require_visual_confirmation:
            self._call_visual_confirm()
        else:
            self._call_bridge()

    def _dispatch_arm_preset(self, preset_name: str) -> None:
        """Fire-and-forget arm-preset move for per-pot patrol.

        Never blocks the mission: if patrol is disabled or the
        /lupin/arm/preset service isn't up we just return (the arm staying
        put must not stall a scan or a drive). The preset server clamps the
        pose to the canonical joint window, so a bad name is harmless.
        """
        if not self._arm_patrol_enabled or self._arm_preset_client is None:
            return
        if not self._arm_preset_client.service_is_ready():
            self.get_logger().warn(
                'arm patrol: %s not ready; leaving arm where it is'
                % str(self.get_parameter('arm_preset_service').value),
                throttle_duration_sec=10.0,
            )
            return
        req = SetArmPreset.Request()
        req.name = preset_name
        self._arm_preset_client.call_async(req)  # fire-and-forget
        self.get_logger().info('arm patrol → preset "%s"' % preset_name)

    def on_enter_INSPECTING_PUBLISHING(self, event_data) -> None:
        self._enter_publishing()

    def on_enter_MONITORING_PUBLISHING(self, event_data) -> None:
        self._enter_publishing()

    def _enter_publishing(self) -> None:
        # PUBLISHING is a named gate. The actual publish already happened in
        # _on_scan_response / nav-failure / abort paths. Here we advance the
        # cursor and decide where to go next. Both mission types finish the
        # same way: when is_complete() (InspectionMission's list runs out, or
        # MonitoringMission completes its target_cycles sweeps) →
        # inspection_complete → RETURNING; otherwise next_tag loops on.
        if self._mission is None:
            self.get_logger().warn(
                "on_enter PUBLISHING with no active mission; holding state."
            )
            return
        if self._return_requested or self._manual_dock_requested:
            self.get_logger().info(
                "PUBLISHING entered while dock return is requested; not advancing mission cursor."
            )
            return

        self._mission.advance()

        if self._mission.is_complete():
            self.inspection_complete()  # type: ignore[attr-defined]
        else:
            self.next_tag()  # type: ignore[attr-defined]

    # ─── exploration on_enter ──────────────────────────────────────────
    def on_enter_EXPLORING(self, event_data) -> None:
        self._exploring_started_at = self._monotonic()
        self._no_frontier_since = None
        self.get_logger().info(
            f'EXPLORING: searching for {self._active_discovery_goal} tags '
            f'(discovered so far: {len(self._discovered)}).'
        )
        if self._is_blocked():
            return
        # Already have enough (e.g. tags latched before we started)?
        if len(self._discovered) >= self._active_discovery_goal:
            self.tags_discovered()  # type: ignore[attr-defined]
            return
        self._send_frontier_goal()

    def on_enter_MONITORING(self, event_data) -> None:
        # Build the monitoring loop from whatever was discovered. The parent
        # on_enter fires before the initial child (NAVIGATING), so the model
        # is in place before the first leg navigates.
        discovered_poses = {
            tid: t.pose_in_map for tid, t in self._discovered.items()
        }
        self._mission = MonitoringMission(
            mission_id=self._mission.mission_id if self._mission else
            f'{self._mission_id_prefix}-{uuid.uuid4().hex[:8]}',
            discovered=discovered_poses,
            nav_max_attempts=self._nav_max_attempts,
            approach_yaw=self._approach_yaw,
            standoff_m=self._approach_standoff,
            target_cycles=self._monitoring_sweeps,
            start_xy=self._robot_xy(),
        )
        self.get_logger().info(
            f'MONITORING: continuous re-scan loop over '
            f'{len(discovered_poses)} discovered tag(s).'
        )

    def on_enter_RETURNING(self, event_data) -> None:
        # Guard on E-stop and operator pause only — NOT is_blocked().
        # battery_low is intentionally excluded: it's what triggered RETURNING
        # in the first place.
        if self._estop_engaged or self._paused:
            return

        if self._nav_pending_cancel:
            self.get_logger().info(
                "RETURNING entered while nav cancel is pending; waiting before sending dock goal."
            )
            # Don't trust the cancel ack to always arrive — arm a watchdog that
            # forces the dock goal if it doesn't, so an abort/battery return
            # can't strand the robot in RETURNING.
            if self._return_cancel_watchdog is None:
                self._return_cancel_watchdog = self.create_timer(
                    _RETURN_CANCEL_GRACE_S, self._on_return_cancel_watchdog
                )
            return

        self._send_return_nav_goal()

    def on_enter_DONE(self, event_data) -> None:
        self._log_final_summary()

    def on_enter_FAULT(self, event_data) -> None:
        # Cancel anything still in flight; FAULT is terminal until a
        # fresh start_mission is requested (which will be rejected, per
        # service handler).
        self._cancel_inflight_nav("fault", refund_attempt=False)
        self._scan_future = None
        self._confirm_future = None

    # ─── scan nav: send / response / result (INSPECTING + MONITORING) ───
    def _send_scan_nav_goal(self) -> None:
        """Issue NavigateToPose for the current tag.

        Shared by INSPECTING (a-priori table geometry) and MONITORING
        (discovered tag pose). Increments the attempt counter — refunds if
        cancel-by-operator.
        """
        if self._mission is None or self._mission.is_complete():
            return
        tag_id = self._mission.current_tag_id()

        # Per-leg localization gate. InspectionMission uses the tight AMCL
        # covariance gate (catches drift over a long patrol); exploration /
        # monitoring run in SLAM mode where /amcl_pose is a static seed, so
        # they gate on pose *presence* only (see _nav_localization_ok).
        ok, detail = self._nav_localization_ok()
        if not ok:
            self.get_logger().warn(
                f'Localization gate tripped for tag {tag_id} ({detail}); '
                f'marking unreachable without dispatching nav goal.'
            )
            result = self._mission.mark_unreachable(detail)
            self._emit_observation_for(result)
            self.nav_unreachable()  # type: ignore[attr-defined]
            return

        try:
            approach = self._compute_scan_approach(tag_id)
        except KeyError:
            # Tag missing from the location source — defensive; should have
            # been validated (inspection) or discovered (monitoring).
            result = self._mission.mark_unreachable(f'unknown_tag:{tag_id}')
            self._emit_observation_for(result)
            self.nav_unreachable()  # type: ignore[attr-defined]
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose_stamped(
            approach.goal_x, approach.goal_y, approach.goal_yaw
        )

        self._mission.register_nav_attempt()
        attempt = self._mission.current_result().nav_attempts
        # Log line surfaces derived_from + table_id so the operator can read
        # whether geometry, an override, or the fallback won — same field
        # appears in /mission/state's last_error if the goal gets rejected.
        self.get_logger().info(
            f"NavigateToPose → tag {tag_id} "
            f"(attempt {attempt}/{self._mission.nav_max_attempts}, "
            f"goal=({approach.goal_x:.2f}, {approach.goal_y:.2f}, "
            f"yaw={approach.goal_yaw:.2f}), "
            f"derived_from={approach.derived_from}"
            f"{f', table={approach.table_id}' if approach.table_id else ''}, "
            f"standoff={approach.standoff_m:.2f}m)"
        )

        self._nav_state_started_at = self._monotonic()
        future = self._nav_client.send_goal_async(goal_msg)
        self._nav_send_goal_future = future
        future.add_done_callback(self._on_inspection_nav_goal_response)

    def _compute_scan_approach(self, tag_id: str) -> TagApproach:
        """Approach pose for the current tag, by mission type.

        InspectionMission: a-priori table geometry + overrides.
        MonitoringMission: the tag's discovered map pose + standoff (no table
        geometry exists for runtime-discovered tags). Raises KeyError if the
        tag has no location in either source.
        """
        if isinstance(self._mission, MonitoringMission):
            pose = self._mission.pose_for(tag_id)
            if pose is None:
                raise KeyError(tag_id)
            return compute_discovered_approach(
                tag_id, pose,
                standoff_m=self._approach_standoff,
                fallback_yaw=self._approach_yaw,
                robot_xy=self._robot_xy(),
            )
        return compute_approach(
            tag_id,
            self._tag_locations,
            self._table_locations,
            standoff_m=self._approach_standoff,
            fallback_yaw=self._approach_yaw,
            overrides=self._approach_overrides,
        )

    def _on_inspection_nav_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f"send_goal raised: {exc!r}")
            if future is self._nav_send_goal_future:
                self._nav_send_goal_future = None
                self._nav_goal_handle = None
                self._nav_get_result_future = None
                self._nav_pending_cancel = False
                self._handle_nav_failure(f"send_goal_exception:{exc!r}")
                self._maybe_send_deferred_return()
            return

        # Stale-future guard: if a cancel intervened between send_goal_async
        # and the server's response, _nav_send_goal_future was nulled. Tell
        # the server the goal is no longer wanted and bail.
        if future is not self._nav_send_goal_future:
            if self._nav_pending_cancel and goal_handle.accepted:
                try:
                    goal_handle.cancel_goal_async()
                except Exception as exc:  # pragma: no cover - defensive
                    self.get_logger().warn(f"late-cancel raised: {exc!r}")
            self._nav_pending_cancel = False
            self._maybe_send_deferred_return()
            return

        if not goal_handle.accepted:
            self.get_logger().warn(
                f"Nav2 rejected goal for tag {self._mission.current_tag_id()}"
            )
            self._nav_goal_handle = None
            self._nav_send_goal_future = None
            self._nav_get_result_future = None
            self._nav_pending_cancel = False
            self._handle_nav_failure("nav_rejected")
            self._maybe_send_deferred_return()
            return

        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self._nav_get_result_future = result_future
        result_future.add_done_callback(self._on_inspection_nav_result)

    def _on_inspection_nav_result(self, future) -> None:
        # Stale-future guard: a different goal is now active, ignore.
        if future is not self._nav_get_result_future:
            return
        try:
            result_msg = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f"get_result raised: {exc!r}")
            self._nav_goal_handle = None
            self._nav_send_goal_future = None
            self._nav_get_result_future = None
            self._nav_pending_cancel = False
            self._handle_nav_failure(f"result_exception:{exc!r}")
            self._maybe_send_deferred_return()
            return

        status = result_msg.status
        # Clear handles once the result is in — so cancels mid-handler
        # don't fire on a stale handle.
        self._nav_goal_handle = None
        self._nav_send_goal_future = None
        self._nav_get_result_future = None
        self._nav_pending_cancel = False

        # If we are returning because battery went low, the canceled inspection goal
        # is expected; do not treat it as a failure.
        if status == GoalStatus.STATUS_CANCELED and self.state == "RETURNING":
            self._maybe_send_deferred_return()
            return

        # State guard: an abort/pause that fired between this goal completing
        # and this callback running has already moved us out of NAVIGATING
        # (e.g. to RETURNING). Firing nav_succeeded/nav_failure from there is
        # an illegal transition. The fast monitoring loop makes this race
        # likely; the future-identity guard above doesn't catch the case
        # where the dock goal hasn't reassigned the result future yet.
        if self.state not in ('INSPECTING_NAVIGATING', 'MONITORING_NAVIGATING'):
            return

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.nav_succeeded()  # type: ignore[attr-defined]
            return
        # Anything else (ABORTED / CANCELED / unknown) — failure path.
        # CANCELED arriving here means Nav2 self-cancelled; an operator
        # cancel would have nulled the future before we got here.
        self._handle_nav_failure(f"nav_status_{status}")
        self._maybe_send_deferred_return()

    def _handle_nav_failure(self, detail: str) -> None:
        """Common path for any NAVIGATING failure (timeout, abort, reject).

        Shared by INSPECTING and MONITORING — the retry re-sends via the same
        generalized scan-nav sender.
        """
        if self._mission is None or self._mission.is_complete():
            return
        if self.state not in ('INSPECTING_NAVIGATING', 'MONITORING_NAVIGATING'):
            # Pause/abort path moved us elsewhere; ignore late failure.
            return
        if self._is_blocked():
            return
        if self._mission.can_retry_nav():
            self.get_logger().warn(
                f"Nav failure ({detail}); retrying tag "
                f"{self._mission.current_tag_id()}"
            )
            self._send_scan_nav_goal()
            return
        # Out of retries — mark UNREACHABLE, emit, advance.
        result = self._mission.mark_unreachable(detail)
        self._emit_observation_for(result)
        self.nav_unreachable()  # type: ignore[attr-defined]

    # ─── exploration: frontier drive ───────────────────────────────────
    def _nav_inflight(self) -> bool:
        return (
            self._nav_goal_handle is not None
            or self._nav_send_goal_future is not None
        )

    def _poll_exploration(self) -> None:
        """EXPLORING watchdog tick: stop at goal/timeout, else keep driving."""
        if self._is_blocked():
            return
        # Found enough — cut exploration short even mid-drive.
        if len(self._discovered) >= self._active_discovery_goal:
            self.get_logger().info(
                f'Discovery goal reached ({len(self._discovered)}/'
                f'{self._active_discovery_goal}); switching to MONITORING.'
            )
            self._cancel_inflight_nav('discovery_goal_reached', refund_attempt=False)
            self.tags_discovered()  # type: ignore[attr-defined]
            return
        # Hard time budget on the search.
        if self._monotonic() - self._exploring_started_at > self._exploration_timeout:
            self.get_logger().warn(
                f'Exploration timeout after {self._exploration_timeout:.0f}s; '
                f'discovered {len(self._discovered)}/{self._active_discovery_goal}.'
            )
            self._last_error = 'exploration_timeout'
            self._cancel_inflight_nav('exploration_timeout', refund_attempt=False)
            self.no_frontiers()  # type: ignore[attr-defined]
            return
        # A frontier leg that hung: cancel and re-select.
        if self._nav_inflight():
            if self._monotonic() - self._nav_state_started_at > self._nav_timeout:
                self.get_logger().warn('Frontier nav goal timed out; re-selecting.')
                self._cancel_inflight_nav('frontier_nav_timeout', refund_attempt=False)
                self._send_frontier_goal()
            return
        # No goal in flight (just resumed, or last select found nothing).
        # Re-attempt on a ~1 s throttle so a frontier-less map (bootstrap)
        # doesn't spin select_frontier_goal at the 10 Hz watchdog rate.
        if self._monotonic() - self._last_frontier_attempt > 1.0:
            self._send_frontier_goal()

    def _send_frontier_goal(self) -> None:
        self._last_frontier_attempt = self._monotonic()
        if self._is_blocked():
            return
        ok, detail = self._nav_localization_ok()
        if not ok:
            self.get_logger().warn(
                f'Exploration localization gate ({detail}); waiting.',
                throttle_duration_sec=5.0,
            )
            return
        grid = self._latest_map
        if grid is None:
            self.get_logger().warn('No /map yet; waiting for SLAM scans.',
                                   throttle_duration_sec=5.0)
            return
        robot_xy = self._robot_xy()
        goal = select_frontier_goal(
            grid.data, grid.info.width, grid.info.height,
            grid.info.resolution,
            grid.info.origin.position.x, grid.info.origin.position.y,
            robot_xy=robot_xy,
            free_thresh=self._frontier_free_thresh,
            occupied_thresh=self._frontier_occupied_thresh,
            min_cluster_cells=self._frontier_min_cluster,
            robot_radius_cells=self._frontier_robot_radius,
        )
        if goal is None:
            if self._no_frontier_since is None:
                self._no_frontier_since = self._monotonic()
            self.get_logger().info(
                'No reachable frontier this tick; waiting for more map.',
                throttle_duration_sec=5.0,
            )
            return
        self._no_frontier_since = None
        if robot_xy is not None:
            yaw = math.atan2(goal.y - robot_xy[1], goal.x - robot_xy[0])
        else:
            yaw = 0.0
        self.get_logger().info(
            f'Frontier goal → ({goal.x:.2f}, {goal.y:.2f}) '
            f'(cluster={goal.cluster_size} cells, {goal.num_clusters} frontiers, '
            f'discovered {len(self._discovered)}/{self._active_discovery_goal})'
        )
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose_stamped(goal.x, goal.y, yaw)
        self._nav_state_started_at = self._monotonic()
        future = self._nav_client.send_goal_async(goal_msg)
        self._nav_send_goal_future = future
        future.add_done_callback(self._on_explore_nav_goal_response)

    def _on_explore_nav_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f'frontier send_goal raised: {exc!r}')
            return
        if future is not self._nav_send_goal_future:
            # A cancel intervened — drop the goal if it was accepted.
            if self._nav_pending_cancel and goal_handle.accepted:
                try:
                    goal_handle.cancel_goal_async()
                except Exception:  # pragma: no cover
                    pass
            self._nav_pending_cancel = False
            return
        if not goal_handle.accepted:
            # Nav2 rejected the frontier (e.g. in unknown/lethal). Let the
            # watchdog re-select on its throttle rather than tight-looping.
            self.get_logger().warn('Nav2 rejected frontier goal; will re-select.')
            self._nav_send_goal_future = None
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self._nav_get_result_future = result_future
        result_future.add_done_callback(self._on_explore_nav_result)

    def _on_explore_nav_result(self, future) -> None:
        if future is not self._nav_get_result_future:
            return
        try:
            future.result()  # status not needed — any outcome → re-select
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f'frontier get_result raised: {exc!r}')
        self._nav_goal_handle = None
        self._nav_send_goal_future = None
        self._nav_get_result_future = None
        if self._is_blocked() or self.state != 'EXPLORING':
            return
        # Reached (or failed at) a frontier — let the watchdog drive the
        # goal/timeout decision and the next selection so all the cut-short
        # logic lives in one place.
        if len(self._discovered) >= self._active_discovery_goal:
            self.tags_discovered()  # type: ignore[attr-defined]
            return
        self._send_frontier_goal()

    # ─── scanning: bridge call ─────────────────────────────────────────
    def _call_bridge(self) -> None:
        if self._mission is None or self._mission.is_complete():
            return
        request = GetTagReading.Request()
        request.tag_id = self._mission.current_tag_id()
        self._scan_started_at = self._monotonic()
        self._scan_future = self._bridge_client.call_async(request)
        self._scan_future.add_done_callback(self._on_scan_response)

    # ─── visual confirm: pre-scan AprilTag detection gate ──────────────
    def _call_visual_confirm(self) -> None:
        """Ask /perception/confirm_tag whether the expected AprilTag is in
        frame before we call the bridge. Hardware-only path — sim never
        creates the client.
        """
        if self._mission is None or self._mission.is_complete():
            return
        if self._confirm_client is None:  # belt-and-braces, never in this branch
            self._call_bridge()
            return
        request = ConfirmTag.Request()
        request.expected_tag_id = self._mission.current_tag_id()
        self._confirm_started_at = self._monotonic()
        self._confirm_future = self._confirm_client.call_async(request)
        self._confirm_future.add_done_callback(self._on_visual_confirm_response)

    def _on_visual_confirm_response(self, future) -> None:
        # Stale-future guard, mirrors _on_scan_response. Valid in either
        # sub-machine's SCANNING state.
        if future is not self._confirm_future or self.state not in (
            'INSPECTING_SCANNING', 'MONITORING_SCANNING'
        ):
            return
        self._confirm_future = None
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f"visual confirm raised: {exc!r}")
            if self._mission is not None:
                result = self._mission.mark_scan_failed(f"confirm_exception:{exc!r}")
                self._emit_observation_for(result)
            self.scan_done()  # type: ignore[attr-defined]
            return

        if response.detected:
            self.get_logger().info(
                f"Visual confirm OK for tag "
                f'{self._mission.current_tag_id() if self._mission else "?"} '
                f"(confidence={response.detection_confidence:.2f}); "
                f"proceeding to bridge."
            )
            # Detected — continue with the bridge call. The orchestrator
            # could also use response.tag_pose_in_map for visual servoing
            # in a future MR; v1 just gates on the boolean.
            self._call_bridge()
            return

        # Not detected — same failure shape as scan-timeout: mark, emit,
        # advance. Operator sees the perception's reason in last_error.
        detail = response.error_message or "visual_confirm_missed"
        self.get_logger().warn(
            f"Visual confirm missed tag "
            f'{self._mission.current_tag_id() if self._mission else "?"}: {detail}'
        )
        if self._mission is not None and not self._mission.is_complete():
            result = self._mission.mark_scan_failed(detail)
            self._emit_observation_for(result)
        self.scan_done()  # type: ignore[attr-defined]

    def _on_scan_response(self, future) -> None:
        # Stale-future guard. Valid in either sub-machine's SCANNING state.
        if future is not self._scan_future or self.state not in (
            'INSPECTING_SCANNING', 'MONITORING_SCANNING'
        ):
            return
        self._scan_future = None
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f"bridge call raised: {exc!r}")
            if self._mission is not None:
                result = self._mission.mark_scan_failed(f"service_exception:{exc!r}")
                self._emit_observation_for(result)
            self.scan_done()  # type: ignore[attr-defined]
            return

        if response.status == GetTagReading.Response.STATUS_OK:
            result = self._mission.mark_scan_ok(response.reading)
            self._emit_observation_for(result)
            # Hold SCANNING long enough for the arm to reach the inspect pose
            # and the flower detector/aggregator to read the bloom colour.
            self._finish_scan_with_dwell()
            return
        detail = response.error_message or f"bridge_status_{response.status}"
        result = self._mission.mark_scan_failed(detail)
        self._emit_observation_for(result)
        self.scan_done()  # type: ignore[attr-defined]

    def _finish_scan_with_dwell(self) -> None:
        """Advance out of SCANNING, but not before ``flower_scan_dwell_s`` has
        elapsed since the scan started — gives the patrol arm time to strike the
        inspect pose and the flower detector time to classify the bloom while the
        aggregator's SCANNING-phase gate is still open. The scan-timeout watchdog
        is already inert here (``_scan_future`` is None), so the dwell can't be
        mistaken for a bridge hang."""
        remaining = self._flower_scan_dwell - (self._monotonic() - self._scan_started_at)
        if self._flower_scan_dwell <= 0.0 or remaining <= 0.0:
            self.scan_done()  # type: ignore[attr-defined]
            return
        if self._scan_dwell_timer is not None:
            self._scan_dwell_timer.cancel()
        self._scan_dwell_timer = self.create_timer(remaining, self._on_scan_dwell_done)

    def _on_scan_dwell_done(self) -> None:
        if self._scan_dwell_timer is not None:
            self._scan_dwell_timer.cancel()
            self._scan_dwell_timer = None
        # Only advance if we're still parked at this pot (not aborted/returned).
        if self.state in ('INSPECTING_SCANNING', 'MONITORING_SCANNING'):
            self.scan_done()  # type: ignore[attr-defined]

    # ─── returning: dock goal ──────────────────────────────────────────
    def _send_return_nav_goal(self) -> None:
        # A dock goal is going out now — the deferral watchdog has done its job.
        self._clear_return_cancel_watchdog()
        x, y, yaw = self._dock_pose[0], self._dock_pose[1], self._dock_pose[2]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._build_pose_stamped(x, y, yaw)
        self.get_logger().info(
            f"NavigateToPose (RETURN) → dock=({x:.2f}, {y:.2f}, yaw={yaw:.2f})"
        )
        self._nav_state_started_at = self._monotonic()
        future = self._nav_client.send_goal_async(goal_msg)
        self._nav_send_goal_future = future
        future.add_done_callback(self._on_return_nav_goal_response)

    def _on_return_nav_goal_response(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(
                f"RETURN send_goal raised: {exc!r}; treating as done"
            )
            if future is self._nav_send_goal_future:
                self.returned()  # type: ignore[attr-defined]
            return
        if future is not self._nav_send_goal_future:
            return
        if not goal_handle.accepted:
            self.get_logger().warn("RETURN: Nav2 rejected dock goal; treating as done")
            self.returned()  # type: ignore[attr-defined]
            return
        self._nav_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        self._nav_get_result_future = result_future
        result_future.add_done_callback(self._on_return_nav_result)

    def _on_return_nav_result(self, future) -> None:
        if future is not self._nav_get_result_future:
            return

        try:
            result_msg = future.result()
            status = result_msg.status
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f"RETURN get_result raised: {exc!r}")
            status = GoalStatus.STATUS_UNKNOWN

        self._nav_goal_handle = None
        self._nav_send_goal_future = None
        self._nav_get_result_future = None

        if status != GoalStatus.STATUS_SUCCEEDED:
            # Common race: we cancelled the in-flight inspection goal and immediately
            # sent the dock goal. Sometimes Nav2 delivers the cancel result here
            # first (STATUS_CANCELED=6). In the non-battery abort path, treat that
            # as a stale cancel and re-issue the dock goal once.
            if status == GoalStatus.STATUS_CANCELED and not self._battery_low:
                self.get_logger().warn(
                    "RETURN: got CANCELED while returning without battery_low; "
                    "re-sending dock goal once."
                )
                self._send_return_nav_goal()
                return

            # Per spec: failures while returning to dock are non-fatal. End the
            # mission so the operator can start a fresh one instead of getting
            # stuck forever in RETURNING.
            self.get_logger().warn(
                f"RETURN: dock goal ended with status {status}; transitioning to DONE"
            )
            self._docked_for_battery = False
            self._manual_dock_requested = False
            self._paused = False
            self.returned()  # type: ignore[attr-defined]
            return

        # Successful arrival at dock after low-battery interruption during active
        # inspection: pause and keep mission resumable.
        if (
            self._docked_for_battery
            and self._mission is not None
            and not self._mission.is_complete()
            and self._return_resumable
        ):
            self._paused = True
            self.get_logger().info(
                "Docked due to low battery. Awaiting charge + /mission/resume."
            )
            return

        if self._return_resumable and self._manual_dock_requested:
            self._paused = True
            self.get_logger().info("Docked. Awaiting /mission/resume.")
            return

        # All other successful returns are terminal: normal mission completion,
        # abort-triggered return, or battery-triggered return after mission already
        # became complete.
        if self._docked_for_battery:
            self.get_logger().info(
                "Docked due to low battery but mission is complete; transitioning to DONE."
            )
            self._docked_for_battery = False

        self._paused = False
        self.returned()  # type: ignore[attr-defined]

    # ─── helpers ───────────────────────────────────────────────────────
    def _build_pose_stamped(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation = _yaw_to_quaternion(float(yaw))
        return pose

    def _log_final_summary(self) -> None:
        if self._mission is None:
            self.get_logger().info("Mission DONE: no mission was active.")
            return
        c = self._mission.counters()
        self.get_logger().info(
            f"Mission {self._mission.mission_id} DONE: "
            f'total={c["total"]} ok={c["completed"]} failed={c["failed"]} '
            f'unreachable={c["unreachable"]} skipped={c["skipped"]}'
        )
        # Only InspectionMission carries a per-tag summary; exploration /
        # monitoring report via counters above.
        summary = getattr(self._mission, 'summary_lines', None)
        if callable(summary):
            for line in summary():
                self.get_logger().info(line)

    # ─── routing helpers used by skip_current ──────────────────────────
    def _goto_publishing_from_current(self) -> None:
        """Force-route the active inspection sub-state into PUBLISHING.

        on_enter_PUBLISHING owns the cursor-advance and the
        next_tag / inspection_complete decision, so callers must NOT
        chain a follow-up trigger here.
        """
        st = self.state
        if st == "INSPECTING_NAVIGATING":
            self.nav_unreachable()  # type: ignore[attr-defined]
        elif st == "INSPECTING_SCANNING":
            self.scan_done()  # type: ignore[attr-defined]
        # INSPECTING_PUBLISHING: already there, nothing to do.

    def _on_return_cancel_watchdog(self) -> None:
        """Fallback when a nav cancel ack never arrives (lost over DDS, or the
        action server churned on an abort). Forces the deferred dock goal so the
        robot can't be stranded in RETURNING. One-shot — cancels itself."""
        self._clear_return_cancel_watchdog()
        if self.state != "RETURNING" or self._estop_engaged or self._paused:
            return
        if self._nav_goal_handle is not None or self._nav_send_goal_future is not None:
            return  # a dock goal is already in flight
        if self._nav_pending_cancel:
            self.get_logger().warn(
                f"Nav cancel ack not seen within {_RETURN_CANCEL_GRACE_S:.0f}s; "
                "forcing dock goal (assuming the cancel was lost)."
            )
            self._nav_pending_cancel = False
        self._maybe_send_deferred_return()

    def _clear_return_cancel_watchdog(self) -> None:
        if self._return_cancel_watchdog is not None:
            self._return_cancel_watchdog.cancel()
            self._return_cancel_watchdog = None

    def _maybe_send_deferred_return(self) -> None:
        if self.state != "RETURNING":
            return
        if not self._return_requested:
            return
        if self._nav_pending_cancel:
            return
        if self._nav_goal_handle is not None or self._nav_send_goal_future is not None:
            return
        if self._estop_engaged or self._paused:
            return

        self._return_requested = False
        self.get_logger().info(
            "Prior nav cancel settled; sending deferred dock goal now."
        )
        self._send_return_nav_goal()


# ─── entry point ─────────────────────────────────────────────────────────


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestratorNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
