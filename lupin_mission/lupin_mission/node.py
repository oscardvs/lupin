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
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from transitions.extensions import HierarchicalGraphMachine

from lupin_msgs.msg import MissionState, Observation
from lupin_msgs.srv import GetTagReading, StartMission

from .estop_monitor import EStopMonitor
from .inspection_mission import InspectionMission
from .observations import make_tag_observation
from .tag_locations import load_default_tag_locations, numeric_string_sort_key


# ─── HSM topology ────────────────────────────────────────────────────────
#
# Kept as a free function so the docs exporter can build the diagram
# without standing up a ROS node. Uses the default ``_`` separator from
# transitions, so nested states are referenced as e.g.
# ``INSPECTING_NAVIGATING``.

LIFECYCLE_STATES = ['BOOT', 'READY', 'PREPARE', 'INSPECTING', 'RETURNING', 'DONE', 'FAULT']
INSPECTING_SUBSTATES = ['NAVIGATING', 'SCANNING', 'PUBLISHING']
PREPARE_SUBSTATES = ['LOCALIZING']


def build_hsm_spec() -> dict:
    """States + transitions for the orchestrator. Pure data."""
    states = [
        'BOOT',
        'READY',
        {'name': 'PREPARE', 'children': PREPARE_SUBSTATES, 'initial': 'LOCALIZING'},
        {'name': 'INSPECTING', 'children': INSPECTING_SUBSTATES, 'initial': 'NAVIGATING'},
        'RETURNING',
        'DONE',
        'FAULT',
    ]
    transitions = [
        # BOOT
        {'trigger': 'deps_up', 'source': 'BOOT', 'dest': 'READY'},
        # READY → PREPARE (via /mission/start)
        {'trigger': 'start_mission', 'source': 'READY', 'dest': 'PREPARE'},
        # PREPARE.LOCALIZING → INSPECTING / FAULT
        {'trigger': 'localized', 'source': 'PREPARE_LOCALIZING', 'dest': 'INSPECTING'},
        # INSPECTING sub-machine
        {
            'trigger': 'nav_succeeded',
            'source': 'INSPECTING_NAVIGATING',
            'dest': 'INSPECTING_SCANNING',
        },
        {
            'trigger': 'nav_unreachable',
            'source': 'INSPECTING_NAVIGATING',
            'dest': 'INSPECTING_PUBLISHING',
        },
        {
            'trigger': 'scan_done',
            'source': 'INSPECTING_SCANNING',
            'dest': 'INSPECTING_PUBLISHING',
        },
        {
            'trigger': 'next_tag',
            'source': 'INSPECTING_PUBLISHING',
            'dest': 'INSPECTING_NAVIGATING',
        },
        {
            'trigger': 'inspection_complete',
            'source': 'INSPECTING_PUBLISHING',
            'dest': 'RETURNING',
        },
        # /mission/abort jumps any active inspection state to RETURNING.
        {
            'trigger': 'abort_to_return',
            'source': [
                'INSPECTING_NAVIGATING',
                'INSPECTING_SCANNING',
                'INSPECTING_PUBLISHING',
            ],
            'dest': 'RETURNING',
        },
        # RETURNING → DONE
        {'trigger': 'returned', 'source': 'RETURNING', 'dest': 'DONE'},
        # DONE → READY for next mission
        {'trigger': 'reset_for_next', 'source': 'DONE', 'dest': 'READY'},
        # FAULT — any non-terminal transitions to FAULT on catastrophic error.
        {
            'trigger': 'fault',
            'source': [
                'BOOT',
                'READY',
                'PREPARE',
                'PREPARE_LOCALIZING',
                'INSPECTING',
                'INSPECTING_NAVIGATING',
                'INSPECTING_SCANNING',
                'INSPECTING_PUBLISHING',
                'RETURNING',
            ],
            'dest': 'FAULT',
        },
    ]
    return {'states': states, 'transitions': transitions, 'initial': 'BOOT'}


# Lifecycle-state strings that indicate "a mission is currently running",
# i.e. /mission/start should be rejected.
_BUSY_PREFIXES = ('PREPARE', 'INSPECTING', 'RETURNING')

_TERMINAL_PREFIXES = ('DONE', 'FAULT')


def _is_state_busy(state: str) -> bool:
    return state.startswith(_BUSY_PREFIXES)


def _is_state_inspecting(state: str) -> bool:
    return state == 'INSPECTING' or state.startswith('INSPECTING_')


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

    def __init__(self, node_name: str = 'mission_orchestrator', **node_kwargs):
        super().__init__(node_name, **node_kwargs)

        # ─── parameters ────────────────────────────────────────────────
        self.declare_parameter('nav_action_name', 'navigate_to_pose')
        self.declare_parameter(
            'bridge_service_name', '/greenhouse_bridge/get_tag_reading'
        )
        self.declare_parameter('estop_topic', '/e_stop_state')
        self.declare_parameter('dependency_timeout_s', 30.0)

        self.declare_parameter('map_yaml_path', '')
        self.declare_parameter('localization_timeout_s', 15.0)
        self.declare_parameter('localization_covariance_threshold', 0.25)

        # tag_sequence: type-only declaration so an empty default doesn't
        # infer as BYTE_ARRAY and reject string overrides.
        self.declare_parameter('tag_sequence', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('approach_yaw', 0.0)
        self.declare_parameter('nav_timeout_s', 60.0)
        self.declare_parameter('nav_max_attempts', 2)
        self.declare_parameter('scan_timeout_s', 5.0)

        self.declare_parameter('dock_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('dock_timeout_s', 60.0)

        self.declare_parameter('state_publish_rate_hz', 5.0)
        self.declare_parameter('mission_id_prefix', 'lupin')
        self.declare_parameter('frame_id', 'map')

        self._nav_action_name = str(self.get_parameter('nav_action_name').value)
        self._bridge_service_name = str(self.get_parameter('bridge_service_name').value)
        self._estop_topic = str(self.get_parameter('estop_topic').value)
        self._dependency_timeout = float(self.get_parameter('dependency_timeout_s').value)

        self._map_yaml_path = str(self.get_parameter('map_yaml_path').value)
        self._localization_timeout = float(
            self.get_parameter('localization_timeout_s').value
        )
        self._localization_cov_thresh = float(
            self.get_parameter('localization_covariance_threshold').value
        )

        self._approach_yaw = float(self.get_parameter('approach_yaw').value)
        self._nav_timeout = float(self.get_parameter('nav_timeout_s').value)
        self._nav_max_attempts = int(self.get_parameter('nav_max_attempts').value)
        self._scan_timeout = float(self.get_parameter('scan_timeout_s').value)

        self._dock_pose = list(self.get_parameter('dock_pose').value or [0.0, 0.0, 0.0])
        self._dock_timeout = float(self.get_parameter('dock_timeout_s').value)

        state_rate = float(self.get_parameter('state_publish_rate_hz').value)
        self._state_publish_period = 1.0 / max(state_rate, 0.1)
        self._mission_id_prefix = str(self.get_parameter('mission_id_prefix').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        # ─── tag locations ─────────────────────────────────────────────
        # Loaded once on startup; the bridge uses string IDs.
        self._tag_locations: dict = load_default_tag_locations()

        # ─── runtime state ─────────────────────────────────────────────
        self._mission: Optional[InspectionMission] = None
        self._mission_started_at = TimeMsg()  # zero-stamp until first start
        self._last_error: str = ''

        # safety flags — see module docstring for the model.
        self._paused: bool = False
        self._estop_engaged: bool = False  # mirrors EStopMonitor.engaged

        # in-flight Nav2 + bridge futures, used both as identity guards
        # against stale callbacks and as cancel handles for pause/abort.
        self._nav_goal_handle = None
        self._nav_send_goal_future = None
        self._nav_get_result_future = None
        self._nav_state_started_at: float = self._monotonic()
        self._scan_future = None
        self._scan_started_at: float = self._monotonic()

        # tracked separately from the nav-attempt counter so retry logic
        # is per-tag (counter on TagResult), but timeouts are per-attempt.
        # bookkeeping for the watchdog.

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
            states=spec['states'],
            transitions=spec['transitions'],
            initial=spec['initial'],
            send_event=True,
            queued=True,
            ignore_invalid_triggers=False,
            after_state_change='_log_transition',
        )
        # Recorded by _log_transition so the published MissionState reflects
        # post-transition values consistently.
        self._last_transition_event: str = ''

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

        # Observations: RELIABLE + TRANSIENT_LOCAL with depth 50 so a late
        # subscriber (e.g. the web Mission tab) sees the mission so far.
        self._obs_pub = self.create_publisher(
            Observation,
            '/floranova/observations',
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
            '/mission/state',
            QoSProfile(
                depth=1,
                history=QoSHistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )

        # E-stop monitor.
        self._estop = EStopMonitor(
            self,
            self._estop_topic,
            on_engaged=self._on_estop_engaged,
            on_released=self._on_estop_released,
            callback_group=self._cb_group,
        )

        # Operator services. Created with their absolute names per spec.
        self._srv_start = self.create_service(
            StartMission, '/mission/start', self._handle_start_mission,
            callback_group=self._cb_group,
        )
        self._srv_pause = self.create_service(
            Trigger, '/mission/pause', self._handle_pause,
            callback_group=self._cb_group,
        )
        self._srv_resume = self.create_service(
            Trigger, '/mission/resume', self._handle_resume,
            callback_group=self._cb_group,
        )
        self._srv_abort = self.create_service(
            Trigger, '/mission/abort', self._handle_abort,
            callback_group=self._cb_group,
        )
        self._srv_skip = self.create_service(
            Trigger, '/mission/skip_current', self._handle_skip_current,
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
            0.1, self._on_watchdog, callback_group=self._cb_group,
        )
        # When BOOT was entered. Compared to dependency_timeout_s.
        self._boot_started_at: float = self._monotonic()

        self.get_logger().info(
            f'Mission orchestrator (v2) up. '
            f'nav_action={self._nav_action_name}, '
            f'bridge_service={self._bridge_service_name}, '
            f'estop_topic={self._estop_topic}'
        )
        self.get_logger().info(
            f'Waiting up to {self._dependency_timeout:.1f}s for nav2 + bridge...'
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
        trigger = event_data.event.name if event_data and event_data.event else '?'
        src = event_data.transition.source if event_data and event_data.transition else '?'
        dst = self.state
        tag_id = self._mission.current_tag_id() if self._mission is not None else ''
        mission_id = self._mission.mission_id if self._mission is not None else ''
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

        Either the operator has paused (or the orchestrator implicitly
        paused on E-stop engagement and is waiting for /mission/resume)
        or the E-stop is currently engaged.
        """
        return self._paused or self._estop_engaged

    def _on_estop_engaged(self) -> None:
        """Rising edge of /e_stop_state. Hold pose; require explicit resume."""
        self._estop_engaged = True
        # Implicit pause — release alone does not auto-resume per spec.
        self._paused = True
        self._cancel_inflight_nav('estop_engaged')
        self._last_error = 'estop_engaged'

    def _on_estop_released(self) -> None:
        """Falling edge: clear engaged flag, but stay paused awaiting resume."""
        self._estop_engaged = False
        # _paused intentionally untouched.

    # ─── stub: nav cancel ──────────────────────────────────────────────
    def _cancel_inflight_nav(self, reason: str) -> None:
        """Cancel any in-flight NavigateToPose goal. Safe if none active.

        Filled out properly when the inspection sub-machine lands; for now
        just clears the handle so the stale-future guard kicks in.
        """
        if self._nav_goal_handle is None:
            return
        try:
            self._nav_goal_handle.cancel_goal_async()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f'cancel_goal_async raised: {exc!r}')
        self._nav_goal_handle = None
        self._nav_send_goal_future = None
        self._nav_get_result_future = None
        self.get_logger().info(f'Nav2 goal cancelled ({reason}).')

    # ─── watchdog ──────────────────────────────────────────────────────
    def _on_watchdog(self) -> None:
        """Periodic tick. Per-state housekeeping; non-blocking.

        Only the states that need polling do work here. Most state actions
        are event-driven (action/service callbacks).
        """
        state = self.state
        if state == 'BOOT':
            self._poll_boot_dependencies()
        elif state == 'PREPARE_LOCALIZING':
            self._poll_localization()
        elif state == 'INSPECTING_NAVIGATING':
            self._check_nav_timeout()
        elif state == 'INSPECTING_SCANNING':
            self._check_scan_timeout()
        # READY, INSPECTING_PUBLISHING, RETURNING, DONE, FAULT: nothing
        # for the watchdog to do.

    # Stubs filled in by later sections — declared here so the watchdog
    # body above type-checks. Concrete logic lands with the inspection
    # sub-machine and the prepare/returning slices.

    def _poll_boot_dependencies(self) -> None:
        elapsed = self._monotonic() - self._boot_started_at
        nav_ready = self._nav_client.server_is_ready()
        bridge_ready = self._bridge_client.service_is_ready()
        if nav_ready and bridge_ready:
            self.get_logger().info('Dependencies up. Orchestrator READY.')
            self.deps_up()  # type: ignore[attr-defined]
            return
        if elapsed > self._dependency_timeout:
            missing = []
            if not nav_ready:
                missing.append(self._nav_action_name)
            if not bridge_ready:
                missing.append(self._bridge_service_name)
            self._last_error = f'dependency_timeout: {", ".join(missing)}'
            self.get_logger().error(
                f'Dependencies did not appear within '
                f'{self._dependency_timeout:.1f}s; missing: {", ".join(missing)}'
            )
            self.fault()  # type: ignore[attr-defined]

    def _poll_localization(self) -> None:
        # Stub for this slice; real implementation lands with the
        # PREPARE/RETURNING task. For now, immediately succeed so the
        # lifecycle is testable end-to-end with a fake mission.
        # TODO(prepare-real): replace with AMCL covariance gate.
        if self._is_blocked():
            return
        self.localized()  # type: ignore[attr-defined]

    def _check_nav_timeout(self) -> None:
        # Stub: filled in with the inspection sub-machine task.
        return

    def _check_scan_timeout(self) -> None:
        # Stub: filled in with the inspection sub-machine task.
        return

    # ─── service handlers ──────────────────────────────────────────────
    def _handle_start_mission(self, request: StartMission.Request,
                              response: StartMission.Response) -> StartMission.Response:
        if self.state == 'FAULT':
            response.accepted = False
            response.error_message = 'orchestrator in FAULT'
            return response
        if _is_state_busy(self.state):
            response.accepted = False
            response.error_message = f'mission already running ({self.state})'
            return response
        if request.mission_type and request.mission_type != 'InspectionMission':
            response.accepted = False
            response.error_message = (
                f"unknown mission_type '{request.mission_type}' — "
                "only 'InspectionMission' is supported in this build"
            )
            return response

        # Build the tag sequence: explicit list else default = all known.
        if request.tag_sequence:
            tag_sequence = [str(t) for t in request.tag_sequence]
            unknown = [t for t in tag_sequence if t not in self._tag_locations]
            if unknown:
                response.accepted = False
                response.error_message = (
                    f'unknown tag id(s) in tag_sequence: {unknown}'
                )
                return response
        else:
            tag_sequence = sorted(
                self._tag_locations.keys(), key=numeric_string_sort_key,
            )

        if not tag_sequence:
            response.accepted = False
            response.error_message = 'no tags to visit (empty tag_locations)'
            return response

        # Fresh mission identity.
        mission_id = f'{self._mission_id_prefix}-{uuid.uuid4().hex[:8]}'
        self._mission = InspectionMission(
            mission_id=mission_id,
            tag_sequence=tag_sequence,
            tag_locations=self._tag_locations,
            nav_max_attempts=self._nav_max_attempts,
            approach_yaw=self._approach_yaw,
        )
        self._mission_started_at = self.get_clock().now().to_msg()
        self._last_error = ''
        # Resetting safety flags so a new mission starts clean. E-stop
        # engagement at this moment will fire the engagement callback
        # again on the next message and re-block.
        self._paused = False

        # State path: READY → PREPARE → INSPECTING. From DONE we first
        # have to bounce through READY for the next mission.
        if self.state == 'DONE':
            self.reset_for_next()  # type: ignore[attr-defined]
        self.start_mission()  # type: ignore[attr-defined]

        response.accepted = True
        response.mission_id = mission_id
        return response

    def _handle_pause(self, request, response):  # std_srvs/Trigger
        if not _is_state_busy(self.state):
            response.success = False
            response.message = f'no active mission to pause (state={self.state})'
            return response
        if self._paused:
            response.success = False
            response.message = 'already paused'
            return response
        self._paused = True
        self._cancel_inflight_nav('paused_by_operator')
        response.success = True
        response.message = 'paused'
        return response

    def _handle_resume(self, request, response):  # std_srvs/Trigger
        if not self._paused:
            response.success = False
            response.message = 'not currently paused'
            return response
        if self._estop_engaged:
            response.success = False
            response.message = 'cannot resume while E-stop is engaged'
            return response
        self._paused = False
        # Re-kick the active state so the held action resumes. Only the
        # states that had work-to-do need a kick.
        self._kick_current_state()
        response.success = True
        response.message = 'resumed'
        return response

    def _handle_abort(self, request, response):  # std_srvs/Trigger
        if not _is_state_busy(self.state):
            response.success = False
            response.message = f'no active mission to abort (state={self.state})'
            return response
        self._cancel_inflight_nav('aborted_by_operator')
        # Mark all remaining tags SKIPPED and emit observations for them.
        if self._mission is not None and _is_state_inspecting(self.state):
            for idx in self._mission.remaining_indices():
                result = self._mission.force_skip(idx, 'mission_aborted')
                self._emit_observation_for(result)
        if _is_state_inspecting(self.state):
            self.abort_to_return()  # type: ignore[attr-defined]
        elif self.state.startswith('PREPARE'):
            # Prep aborted before any tag was attempted; jump straight to
            # DONE via RETURNING so the mission cleans up.
            # transitions doesn't allow a multi-source for the same
            # trigger from PREPARE, so trigger fault → user can restart
            # via a fresh /mission/start. PREPARE-abort is rare; treat as
            # a soft fault rather than a real fault.
            self._last_error = 'aborted_in_prepare'
            self.fault()  # type: ignore[attr-defined]
        # RETURNING-abort: no-op, already heading home.
        response.success = True
        response.message = 'aborted'
        return response

    def _handle_skip_current(self, request, response):  # std_srvs/Trigger
        if not _is_state_inspecting(self.state):
            response.success = False
            response.message = (
                f'skip_current only valid during INSPECTING (state={self.state})'
            )
            return response
        if self._mission is None or self._mission.is_complete():
            response.success = False
            response.message = 'no current tag to skip'
            return response
        self._cancel_inflight_nav('skipped_by_operator')
        result = self._mission.mark_skipped('skipped_by_operator')
        self._emit_observation_for(result)
        # Whatever sub-state we were in, jump to PUBLISHING so the normal
        # advance path runs. transitions can't go directly from arbitrary
        # source to PUBLISHING without explicit transitions; cleanest is
        # to advance through next_tag from PUBLISHING. Implement this by
        # routing through the PUBLISHING handler directly.
        self._mission.advance()
        if self._mission.is_complete():
            # If we were in PUBLISHING, machinery would already do this;
            # synthesise the same path from any inspection state.
            self._goto_returning_via_publishing()
        else:
            self._goto_navigating_via_publishing()
        response.success = True
        response.message = 'skipped'
        return response

    # ─── observation / state plumbing ──────────────────────────────────
    def _emit_observation_for(self, result) -> None:
        """Build + publish the Observation for a closed TagResult."""
        if self._mission is None:
            return
        stamp = self.get_clock().now().to_msg()
        msg = make_tag_observation(
            mission_id=self._mission.mission_id,
            source=self._mission.name,
            stamp=stamp,
            status=result.status,
            tag_reading=result.tag_reading,
            status_detail=result.detail,
            frame_id=self._frame_id,
        )
        self._obs_pub.publish(msg)

    def _publish_state(self) -> None:
        msg = MissionState()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        msg.header = header

        state = self.state
        if state.startswith('INSPECTING'):
            msg.lifecycle_state = 'INSPECTING'
            msg.mission_phase = state[len('INSPECTING_'):] if '_' in state else ''
        elif state.startswith('PREPARE'):
            msg.lifecycle_state = 'PREPARE'
            msg.mission_phase = state[len('PREPARE_'):] if '_' in state else ''
        else:
            msg.lifecycle_state = state
            msg.mission_phase = ''

        if self._mission is not None:
            msg.mission_id = self._mission.mission_id
            msg.mission_type = self._mission.name
            msg.current_target = self._mission.current_tag_id()
            counters = self._mission.counters()
            msg.targets_total = counters['total']
            msg.targets_completed = counters['completed']
            msg.targets_failed = counters['failed']
            msg.targets_unreachable = counters['unreachable']
            msg.targets_skipped = counters['skipped']
        msg.last_error = self._last_error
        msg.estop_engaged = self._estop_engaged
        msg.paused = self._paused
        msg.started_at = self._mission_started_at

        self._state_pub.publish(msg)

    # ─── kick: re-run the current state's action after resume ──────────
    def _kick_current_state(self) -> None:
        """Re-enter the active state's action after pause/E-stop release.

        Only INSPECTING.NAVIGATING and INSPECTING.SCANNING have ongoing
        work that pause/E-stop can interrupt; everything else is event-
        driven and naturally re-fires.
        """
        # Filled in by the inspection sub-machine task; for now a no-op
        # so /mission/resume succeeds even if no work was in flight.
        return

    # ─── routing helpers used by skip_current ──────────────────────────
    def _goto_navigating_via_publishing(self) -> None:
        """Force-route an arbitrary inspection sub-state through
        PUBLISHING and on to NAVIGATING for the next tag."""
        # transitions trigger paths require us to be in a valid source.
        # Cheap approach: only accept skip_current from PUBLISHING, and
        # if we're elsewhere, drive through the legal edges.
        st = self.state
        if st == 'INSPECTING_NAVIGATING':
            self.nav_unreachable()  # type: ignore[attr-defined]
            self.next_tag()  # type: ignore[attr-defined]
        elif st == 'INSPECTING_SCANNING':
            self.scan_done()  # type: ignore[attr-defined]
            self.next_tag()  # type: ignore[attr-defined]
        elif st == 'INSPECTING_PUBLISHING':
            self.next_tag()  # type: ignore[attr-defined]

    def _goto_returning_via_publishing(self) -> None:
        st = self.state
        if st == 'INSPECTING_NAVIGATING':
            self.nav_unreachable()  # type: ignore[attr-defined]
            self.inspection_complete()  # type: ignore[attr-defined]
        elif st == 'INSPECTING_SCANNING':
            self.scan_done()  # type: ignore[attr-defined]
            self.inspection_complete()  # type: ignore[attr-defined]
        elif st == 'INSPECTING_PUBLISHING':
            self.inspection_complete()  # type: ignore[attr-defined]


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


if __name__ == '__main__':
    main()
