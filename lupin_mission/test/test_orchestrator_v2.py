"""End-to-end tests for the v2 mission orchestrator.

Each test spins up:
    - a fake NavigateToPose action server (per-test programmable outcomes,
      optional execution delay, accepts cancels).
    - a fake /greenhouse_bridge/get_tag_reading service.
    - a /amcl_pose publisher seeded with a low-covariance pose so the
      PREPARE.LOCALIZING gate clears immediately.
    - a /e_stop_state publisher (only used by the e-stop test).
    - the orchestrator under test.

All nodes share a MultiThreadedExecutor on a background thread so the
orchestrator's send_goal/call_async/service-callback chain can complete
without blocking the test thread. We block the test thread on the
orchestrator reaching DONE (or whichever predicate the case asserts on).

We do NOT depend on real Nav2 / the real greenhouse bridge / the real
MIRTE e-stop publisher.
"""

import os
import threading
import time
import unittest
from unittest import mock

# Isolate this test process from any real ROS nodes a dev might have on
# the default domain. Must be set before rclpy imports/inits anything.
# Unconditional, not setdefault: the team's .bashrc presets DOMAIN_ID=0
# for MIRTE, which would silently defeat isolation otherwise.
os.environ['ROS_DOMAIN_ID'] = '47'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy  # noqa: E402
from action_msgs.msg import GoalStatus  # noqa: E402
from builtin_interfaces.msg import Time as TimeMsg  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav2_msgs.action import NavigateToPose  # noqa: E402
from rclpy.action import ActionServer, CancelResponse  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.parameter import Parameter  # noqa: E402
from rclpy.qos import (  # noqa: E402
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402

from geometry_msgs.msg import Pose  # noqa: E402
from sensor_msgs.msg import BatteryState  # noqa: E402

from lupin_msgs.msg import (  # noqa: E402
    DiscoveredTag, DiscoveredTags, MissionState, Observation, SensorReading,
    TagReading,
)
from lupin_msgs.srv import GetTagReading, StartMission  # noqa: E402

from lupin_mission.node import MissionOrchestratorNode  # noqa: E402


TEST_TAG_IDS = ['1', '2', '3']
FAKE_TAG_LOCATIONS = {
    tid: {'x': float(i), 'y': float(i) + 0.5, 'sensors': ['temperature']}
    for i, tid in enumerate(TEST_TAG_IDS)
}


def _wait_until(predicate, timeout_sec, interval=0.05):
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        v = predicate()
        if v:
            return v
        time.sleep(interval)
    return predicate()


# ─── fakes ───────────────────────────────────────────────────────────────


class FakeNavServer(Node):
    """Programmable NavigateToPose action server.

    Per-call outcome consumed from ``outcomes`` (clamped to last). Each
    execute can optionally sleep up to ``delay_s`` to give the test time
    to interact (pause, abort, e-stop). Cancels are accepted.
    """

    def __init__(self, *, outcomes=None, delay_s=0.0):
        super().__init__('fake_nav_server')
        self.outcomes = list(outcomes) if outcomes is not None else [
            GoalStatus.STATUS_SUCCEEDED
        ]
        self.delay_s = float(delay_s)
        self.goals_received = 0
        self.goals_cancelled = 0
        self._server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self._execute,
            cancel_callback=lambda gh: CancelResponse.ACCEPT,
        )

    def _execute(self, goal_handle):
        idx = min(self.goals_received, len(self.outcomes) - 1)
        status = self.outcomes[idx]
        self.goals_received += 1
        # Wait up to delay_s, watching for cancel.
        deadline = time.monotonic() + self.delay_s
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                self.goals_cancelled += 1
                goal_handle.canceled()
                return NavigateToPose.Result()
            time.sleep(0.02)
        # Under the monitoring loop's rapid goal churn + an abort, this handle
        # may already have been terminated/expired by the time we get here.
        # Calling succeed()/abort()/canceled() on a non-active handle makes
        # rclpy raise "feedback publisher is invalid" from publish_status(),
        # which propagates out of the spin thread and wedges the whole test
        # (the RETURN goal then never completes → stuck in RETURNING). Guard it.
        if not goal_handle.is_active:
            return NavigateToPose.Result()
        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
        elif status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return NavigateToPose.Result()


class FakeBridge(Node):
    """Fake greenhouse bridge service. Defaults to OK for every tag."""

    DEFAULT_READINGS = {
        'temperature': 22.4,
        'humidity': 58.1,
        'co2': 441.0,
        'light': 796.1,
    }

    def __init__(self, behaviour=None):
        super().__init__('fake_bridge')
        self.calls = []
        self._behaviour = behaviour or self._default_ok
        self._service = self.create_service(
            GetTagReading,
            '/greenhouse_bridge/get_tag_reading',
            self._handle,
        )

    @classmethod
    def _default_ok(cls, tag_id):
        return GetTagReading.Response.STATUS_OK, '', cls.DEFAULT_READINGS

    def _handle(self, request, response):
        self.calls.append(request.tag_id)
        status, err, readings = self._behaviour(request.tag_id)
        response.status = status
        response.error_message = err
        if status == GetTagReading.Response.STATUS_OK and readings:
            tr = TagReading()
            tr.tag_id = request.tag_id
            tr.stamp = TimeMsg(sec=0, nanosec=0)
            tr.sim_time_of_day_seconds = 43200.0
            for name, value in readings.items():
                sr = SensorReading()
                sr.name = name
                sr.value = float(value)
                tr.readings.append(sr)
            response.reading = tr
        return response


class AmclSeed(Node):
    """One-shot publisher of a low-covariance /amcl_pose so PREPARE clears."""

    def __init__(self):
        super().__init__('amcl_seed')
        # Match the orchestrator's subscriber QoS so a TRANSIENT_LOCAL
        # subscriber that joins later still receives the seeded pose.
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(
            PoseWithCovarianceStamped, '/amcl_pose', qos
        )

    def publish_low_cov(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        # 36-vector, row-major 6x6. Tight on x/y/yaw; loose on z/roll/pitch.
        cov = [0.0] * 36
        cov[0] = 0.01      # x
        cov[7] = 0.01      # y
        cov[14] = 99999.0  # z (irrelevant for 2D)
        cov[21] = 99999.0  # roll
        cov[28] = 99999.0  # pitch
        cov[35] = 0.01     # yaw
        msg.pose.covariance = cov
        self._pub.publish(msg)


class EStopPub(Node):
    def __init__(self):
        super().__init__('estop_pub')
        # Match orchestrator's subscriber: depth 10, RELIABLE, VOLATILE.
        self._pub = self.create_publisher(Bool, '/e_stop_state', 10)

    def publish(self, engaged: bool) -> None:
        msg = Bool()
        msg.data = bool(engaged)
        self._pub.publish(msg)


class BatteryPub(Node):
    """Publishes sensor_msgs/BatteryState on /io/power/power_watcher.

    Matches BatteryMonitor's subscriber QoS (RELIABLE + VOLATILE depth 10).
    """

    def __init__(self):
        super().__init__('battery_pub')
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(
            BatteryState, '/io/power/power_watcher', qos
        )

    def publish_pct(self, pct: float) -> None:
        msg = BatteryState()
        msg.percentage = float(pct)
        self._pub.publish(msg)


class DiscoveredPub(Node):
    """Latched publisher of /perception/discovered_tags (fakes the aggregator)."""

    def __init__(self):
        super().__init__('discovered_pub')
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(
            DiscoveredTags, '/perception/discovered_tags', qos
        )

    def publish(self, tag_ids) -> None:
        msg = DiscoveredTags()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for i, tid in enumerate(tag_ids):
            t = DiscoveredTag()
            t.tag_id = str(tid)
            p = Pose()
            p.position.x = float(i)
            p.position.y = 0.0
            p.orientation.w = 1.0
            t.pose_in_map = p
            t.sightings = 5
            msg.tags.append(t)
        self._pub.publish(msg)


class StateCollector(Node):
    """Subscribes to /mission/state and /floranova/observations."""

    def __init__(self):
        super().__init__('state_collector')
        self.states: list[MissionState] = []
        self.observations: list[Observation] = []
        latched = QoSProfile(
            depth=50,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_sub = self.create_subscription(
            MissionState, '/mission/state', self._on_state, latched
        )
        self._obs_sub = self.create_subscription(
            Observation, '/floranova/observations', self._on_obs, latched
        )

    def _on_state(self, msg: MissionState) -> None:
        self.states.append(msg)

    def _on_obs(self, msg: Observation) -> None:
        self.observations.append(msg)

    @property
    def latest_state(self):
        return self.states[-1] if self.states else None


# ─── harness ─────────────────────────────────────────────────────────────


class _SpinHarness:
    def __init__(self):
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.nodes: list[Node] = []
        self._thread = None
        self._stop = threading.Event()

    def add(self, node: Node) -> None:
        self.nodes.append(node)
        self.executor.add_node(node)

    def start(self) -> None:
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self) -> None:
        while not self._stop.is_set():
            self.executor.spin_once(timeout_sec=0.05)

    def shutdown(self) -> None:
        # Brief drain so any in-flight action goals can settle before we
        # rip the executor out from under their worker threads. Without
        # this, the FakeNavServer's execute_callback can call goal_handle
        # .succeed() concurrently with destroy_node() and rclpy raises
        # "feedback publisher is invalid".
        time.sleep(0.2)
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        # Stop the executor's OWN worker threads before destroying nodes.
        # The spin thread above only drives spin_once; the MultiThreadedExecutor
        # still owns worker threads that run action execute_callbacks. Without an
        # explicit shutdown those workers (and any in-flight FakeNavServer goal)
        # outlive this harness: a lingering callback calls goal_handle.succeed()
        # after destroy_node() has invalidated the publisher ("feedback publisher
        # is invalid"), and the abandoned executor's still-registered
        # navigate_to_pose server leaks into the next test — which is exactly why
        # test_pause_resume's cancel was never observed in-suite yet passed in
        # isolation. shutdown() joins the workers first, so callbacks finish on a
        # valid handle and no stale server survives.
        try:
            self.executor.shutdown(timeout_sec=2.0)
        except Exception:
            pass
        for node in self.nodes:
            try:
                node.destroy_node()
            except Exception:
                pass


def _orchestrator(*, tag_sequence=None, **overrides) -> MissionOrchestratorNode:
    params = [
        Parameter('nav_timeout_s', Parameter.Type.DOUBLE, 2.0),
        Parameter('scan_timeout_s', Parameter.Type.DOUBLE, 1.0),
        Parameter('dependency_timeout_s', Parameter.Type.DOUBLE, 5.0),
        Parameter('localization_timeout_s', Parameter.Type.DOUBLE, 5.0),
        Parameter('localization_covariance_threshold', Parameter.Type.DOUBLE, 0.25),
        Parameter('nav_max_attempts', Parameter.Type.INTEGER, 2),
        Parameter('state_publish_rate_hz', Parameter.Type.DOUBLE, 20.0),
        Parameter('mission_id_prefix', Parameter.Type.STRING, 'test'),
    ]
    if tag_sequence is not None:
        params.append(
            Parameter('tag_sequence', Parameter.Type.STRING_ARRAY, list(tag_sequence))
        )
    for k, v in overrides.items():
        # type inferred per-value; tests set sane scalar overrides only.
        params.append(Parameter(k, value=v))
    return MissionOrchestratorNode(parameter_overrides=params)


# ─── service-call helpers ────────────────────────────────────────────────


def _call_start(client_node: Node, *, mission_type='InspectionMission',
                tag_sequence=None, discovery_goal=0, timeout=5.0):
    client = client_node.create_client(StartMission, '/mission/start')
    assert client.wait_for_service(timeout_sec=timeout), '/mission/start did not appear'
    req = StartMission.Request()
    req.mission_type = mission_type
    req.tag_sequence = list(tag_sequence) if tag_sequence else []
    req.discovery_goal = int(discovery_goal)
    future = client.call_async(req)
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    return future.result()


def _call_trigger(client_node: Node, srv_name: str, *, timeout=5.0):
    client = client_node.create_client(Trigger, srv_name)
    assert client.wait_for_service(timeout_sec=timeout), f'{srv_name} did not appear'
    future = client.call_async(Trigger.Request())
    deadline = time.monotonic() + timeout
    while not future.done() and time.monotonic() < deadline:
        time.sleep(0.02)
    return future.result()


# ─── tests ───────────────────────────────────────────────────────────────


class TestOrchestratorV2(unittest.TestCase):
    """End-to-end behavioural cases for the v2 orchestrator."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        # Patch tag locations so the orchestrator constructor doesn't need
        # the real mdp-greenhouse package; it'd work either way, but
        # making tests deterministic is worth the patch.
        cls._patcher = mock.patch(
            'lupin_mission.node.load_default_tag_locations',
            return_value=FAKE_TAG_LOCATIONS,
        )
        cls._patcher.start()

    @classmethod
    def tearDownClass(cls):
        cls._patcher.stop()
        rclpy.shutdown()

    # Each test sets up its own harness so process state stays isolated.
    def setUp(self):
        self.harness = _SpinHarness()
        self.collector = StateCollector()
        self.amcl = AmclSeed()
        self.estop = EStopPub()
        self.battery = BatteryPub()
        self.harness.add(self.collector)
        self.harness.add(self.amcl)
        self.harness.add(self.estop)
        self.harness.add(self.battery)

    def tearDown(self):
        self.harness.shutdown()

    def _bringup(
        self,
        *,
        nav_outcomes=None,
        nav_delay_s=0.0,
        bridge_behaviour=None,
        tag_sequence=None,
        seed_amcl=True,
        **orch_overrides,
    ):
        self.nav = FakeNavServer(outcomes=nav_outcomes, delay_s=nav_delay_s)
        self.bridge = FakeBridge(behaviour=bridge_behaviour)
        self.orch = _orchestrator(tag_sequence=tag_sequence, **orch_overrides)
        self.harness.add(self.nav)
        self.harness.add(self.bridge)
        self.harness.add(self.orch)
        self.harness.start()
        # Seed AMCL once the executor is spinning so the latched message
        # has subscribers ready (the orchestrator's pose subscription).
        # seed_amcl=False leaves localization unconverged (PREPARE_LOCALIZING
        # stalls until localization_timeout) for the FAULT/abort-in-prepare cases.
        time.sleep(0.1)
        if seed_amcl:
            self.amcl.publish_low_cov()
        # Wait for orchestrator to reach READY (deps_up).
        self.assertTrue(
            _wait_until(lambda: self.orch.state == 'READY', 5.0),
            f'orchestrator stuck in {self.orch.state}',
        )

    # ── 1. happy path ──────────────────────────────────────────────────
    def test_happy_path_three_tags(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 4,  # 3 inspection + 1 dock
            tag_sequence=TEST_TAG_IDS,
        )
        resp = _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertTrue(resp.accepted, resp.error_message)
        self.assertTrue(resp.mission_id.startswith('test-'))

        ok = _wait_until(lambda: self.orch.state == 'DONE', 15.0)
        self.assertTrue(ok, f'orchestrator ended in state {self.orch.state}')

        ok_obs = [o for o in self.collector.observations
                  if o.status == Observation.STATUS_OK]
        self.assertEqual(len(ok_obs), 3)
        self.assertEqual(
            [o.tag_reading.tag_id for o in ok_obs], TEST_TAG_IDS,
        )

        # MissionState walked through the long-held lifecycle states.
        # PREPARE and RETURNING are intentionally fast and may not get
        # published at 20 Hz, so we only require the steady-state ones.
        # Wait for the latched DONE state to land in the collector so the
        # 20 Hz publisher has had a tick after the orchestrator's internal
        # transition.
        self.assertTrue(_wait_until(
            lambda: self.collector.latest_state
            and self.collector.latest_state.lifecycle_state == 'DONE',
            2.0,
        ))
        seen_states = {s.lifecycle_state for s in self.collector.states}
        self.assertTrue({'INSPECTING', 'DONE'}.issubset(seen_states),
                        f'seen_states={seen_states}')

    # ── 2. mid-mission start rejection ─────────────────────────────────
    def test_start_rejected_during_inspection(self):
        self._bringup(
            # Slow nav so the test can interleave the second start request.
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 4,
            nav_delay_s=0.5,
            tag_sequence=TEST_TAG_IDS,
        )
        resp1 = _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertTrue(resp1.accepted)
        # Wait until orchestrator is actually inside INSPECTING.
        self.assertTrue(_wait_until(
            lambda: self.orch.state.startswith('INSPECTING'), 5.0,
        ), self.orch.state)
        resp2 = _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertFalse(resp2.accepted)
        self.assertIn('mission already running', resp2.error_message)

    # ── 3. pause / resume ──────────────────────────────────────────────
    def test_pause_resume(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 4,
            nav_delay_s=1.0,
            tag_sequence=TEST_TAG_IDS,
        )
        _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertTrue(_wait_until(
            lambda: self.orch.state == 'INSPECTING_NAVIGATING', 5.0,
        ))
        before_obs = len(self.collector.observations)
        pause_resp = _call_trigger(self.collector, '/mission/pause')
        self.assertTrue(pause_resp.success, pause_resp.message)
        # Goal must have been cancelled.
        self.assertTrue(_wait_until(
            lambda: self.nav.goals_cancelled >= 1, 3.0,
        ))
        self.assertTrue(_wait_until(
            lambda: self.collector.latest_state and self.collector.latest_state.paused,
            2.0,
        ))
        # No new observations while paused.
        time.sleep(0.5)
        self.assertEqual(len(self.collector.observations), before_obs)
        # Resume; mission completes.
        resume_resp = _call_trigger(self.collector, '/mission/resume')
        self.assertTrue(resume_resp.success, resume_resp.message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 15.0),
                        f'state={self.orch.state}')
        ok_obs = [o for o in self.collector.observations
                  if o.status == Observation.STATUS_OK]
        self.assertEqual(len(ok_obs), 3)

    # ── 4. abort ───────────────────────────────────────────────────────
    def test_abort_marks_remaining_skipped(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 4,
            nav_delay_s=0.3,
            tag_sequence=TEST_TAG_IDS,
        )
        _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        # Wait until at least one OK observation has landed.
        self.assertTrue(_wait_until(
            lambda: any(o.status == Observation.STATUS_OK
                        for o in self.collector.observations),
            10.0,
        ))
        # Abort while a tag (the 2nd) is in flight.
        abort_resp = _call_trigger(self.collector, '/mission/abort')
        self.assertTrue(abort_resp.success, abort_resp.message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 10.0),
                        f'state={self.orch.state}')
        skipped = [o for o in self.collector.observations
                   if o.status == Observation.STATUS_SKIPPED]
        self.assertGreaterEqual(len(skipped), 1)
        ok_obs = [o for o in self.collector.observations
                  if o.status == Observation.STATUS_OK]
        # At least one tag completed before abort fired.
        self.assertGreaterEqual(len(ok_obs), 1)
        self.assertEqual(len(ok_obs) + len(skipped), 3)

    # ── 5. skip current ────────────────────────────────────────────────
    def test_skip_current_advances(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 4,
            nav_delay_s=0.4,
            tag_sequence=TEST_TAG_IDS,
        )
        _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        # Skip during the very first tag's NAVIGATING.
        self.assertTrue(_wait_until(
            lambda: self.orch.state == 'INSPECTING_NAVIGATING'
            and self.nav.goals_received >= 1,
            5.0,
        ))
        skip_resp = _call_trigger(self.collector, '/mission/skip_current')
        self.assertTrue(skip_resp.success, skip_resp.message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 15.0),
                        f'state={self.orch.state}')
        statuses = [o.status for o in self.collector.observations
                    if o.kind == Observation.KIND_TAG_READING]
        self.assertEqual(len(statuses), 3)
        self.assertEqual(statuses[0], Observation.STATUS_SKIPPED)
        self.assertEqual(statuses[1], Observation.STATUS_OK)
        self.assertEqual(statuses[2], Observation.STATUS_OK)

    # ── 6. retry → UNREACHABLE ─────────────────────────────────────────
    def test_retry_then_unreachable(self):
        # Tag 1: succ. Tag 2: abort, abort → UNREACHABLE. Tag 3: succ.
        # Plus dock at the end. nav_max_attempts=2 (default test override).
        self._bringup(
            nav_outcomes=[
                GoalStatus.STATUS_SUCCEEDED,    # tag 1 attempt 1
                GoalStatus.STATUS_ABORTED,      # tag 2 attempt 1
                GoalStatus.STATUS_ABORTED,      # tag 2 attempt 2
                GoalStatus.STATUS_SUCCEEDED,    # tag 3 attempt 1
                GoalStatus.STATUS_SUCCEEDED,    # dock
            ],
            tag_sequence=TEST_TAG_IDS,
        )
        _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 20.0),
                        f'state={self.orch.state}')
        statuses = [o.status for o in self.collector.observations
                    if o.kind == Observation.KIND_TAG_READING]
        # Expected sequence: OK, UNREACHABLE, OK.
        self.assertEqual(statuses,
                         [Observation.STATUS_OK,
                          Observation.STATUS_UNREACHABLE,
                          Observation.STATUS_OK])
        # The UNREACHABLE observation has no tag_reading payload.
        unreachable_obs = next(
            o for o in self.collector.observations
            if o.status == Observation.STATUS_UNREACHABLE
        )
        self.assertEqual(unreachable_obs.tag_reading.tag_id, '')
        # Two attempts were issued for tag 2.
        self.assertEqual(self.nav.goals_received, 5)

    # ── 7. e-stop preempt + manual resume ──────────────────────────────
    def test_estop_preempt_no_auto_resume(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 4,
            nav_delay_s=1.0,
            tag_sequence=TEST_TAG_IDS,
        )
        _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertTrue(_wait_until(
            lambda: self.orch.state == 'INSPECTING_NAVIGATING', 5.0,
        ))
        # Engage e-stop.
        self.estop.publish(True)
        self.assertTrue(_wait_until(
            lambda: self.collector.latest_state
            and self.collector.latest_state.estop_engaged,
            3.0,
        ))
        # Mission froze, no observations.
        self.assertTrue(_wait_until(
            lambda: self.nav.goals_cancelled >= 1, 3.0,
        ))
        time.sleep(0.5)
        self.assertEqual(len(self.collector.observations), 0)
        # Release; estop_engaged clears, paused stays true.
        self.estop.publish(False)
        self.assertTrue(_wait_until(
            lambda: (self.collector.latest_state
                     and not self.collector.latest_state.estop_engaged),
            2.0,
        ))
        time.sleep(0.5)
        self.assertTrue(self.collector.latest_state.paused,
                        'paused should remain true after estop release')
        self.assertEqual(len(self.collector.observations), 0,
                         'no observations should be emitted before resume')
        # Resume. Mission completes.
        resume_resp = _call_trigger(self.collector, '/mission/resume')
        self.assertTrue(resume_resp.success, resume_resp.message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 15.0),
                        f'state={self.orch.state}')
        ok_obs = [o for o in self.collector.observations
                  if o.status == Observation.STATUS_OK]
        self.assertEqual(len(ok_obs), 3)

    # ── 8. multi-mission within one node lifetime ──────────────────────
    def test_multi_mission_in_one_node(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 6,
            tag_sequence=TEST_TAG_IDS,
        )
        # Mission 1: tags 1, 2.
        r1 = _call_start(self.collector, tag_sequence=['1', '2'])
        self.assertTrue(r1.accepted)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 15.0))
        first_done_count = len(self.collector.observations)
        self.assertEqual(
            [o.status for o in self.collector.observations],
            [Observation.STATUS_OK, Observation.STATUS_OK],
        )

        # Mission 2: tag 3 only.
        r2 = _call_start(self.collector, tag_sequence=['3'])
        self.assertTrue(r2.accepted, r2.error_message)
        self.assertNotEqual(r2.mission_id, r1.mission_id)
        self.assertTrue(_wait_until(
            lambda: self.orch.state == 'DONE'
            and len(self.collector.observations) > first_done_count,
            15.0,
        ))
        new_obs = self.collector.observations[first_done_count:]
        self.assertEqual(len(new_obs), 1)
        self.assertEqual(new_obs[0].mission_id, r2.mission_id)
        self.assertEqual(new_obs[0].status, Observation.STATUS_OK)

    # ── 9. exploration → monitoring → abort ────────────────────────────
    def test_exploration_discovers_then_monitors(self):
        # No /map is published, so EXPLORING just waits on frontiers; the
        # discovery feed (faked) drives the EXPLORING→MONITORING switch.
        self.discovered = DiscoveredPub()
        self.harness.add(self.discovered)
        # Disable discovery-time climate seeding here so the OK-observation
        # count reflects ONLY the monitoring sweep — seeding would emit one
        # extra OK obs per tag at discovery (that path has its own tests).
        self._bringup(nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 50,
                      seed_climate_on_discovery=False)

        resp = _call_start(
            self.collector, mission_type='ExplorationMission', discovery_goal=2,
        )
        self.assertTrue(resp.accepted, resp.error_message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'EXPLORING', 10.0),
                        f'state={self.orch.state}')

        # Publish the discovered tags meeting the goal → switch to MONITORING.
        self.discovered.publish(['1', '2'])
        self.assertTrue(_wait_until(
            lambda: self.orch.state.startswith('MONITORING'), 10.0,
        ), f'state={self.orch.state}')

        # One full sweep over the 2 discovered tags, then RETURNING → DONE.
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 30.0),
                        f'state={self.orch.state}')

        ok_obs = [o for o in self.collector.observations
                  if o.status == Observation.STATUS_OK]
        self.assertEqual(len(ok_obs), 2,
                         'expected exactly one monitoring sweep of 2 tags')
        # mission_type stayed ExplorationMission across the model swap.
        seen_types = {s.mission_type for s in self.collector.states if s.mission_type}
        self.assertIn('ExplorationMission', seen_types)

    def test_exploration_no_tags_returns_home(self):
        # Goal never met and no map → exploration times out, and with nothing
        # discovered it goes straight to RETURNING (not MONITORING).
        self.discovered = DiscoveredPub()
        self.harness.add(self.discovered)
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 4,
            exploration_timeout_s=1.0,
        )
        resp = _call_start(
            self.collector, mission_type='ExplorationMission', discovery_goal=3,
        )
        self.assertTrue(resp.accepted, resp.error_message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 15.0),
                        f'state={self.orch.state}')
        # Never entered MONITORING — nothing was discovered.
        seen = {s.lifecycle_state for s in self.collector.states}
        self.assertIn('EXPLORING', seen)
        self.assertNotIn('MONITORING', seen)

    # ── 10. manual dock during MONITORING actually diverts to RETURNING ─
    def test_dock_during_monitoring_diverts_to_returning(self):
        self.discovered = DiscoveredPub()
        self.harness.add(self.discovered)
        self._bringup(nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 50, nav_delay_s=1.0)
        resp = _call_start(
            self.collector, mission_type='ExplorationMission', discovery_goal=2,
        )
        self.assertTrue(resp.accepted, resp.error_message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'EXPLORING', 10.0))
        self.discovered.publish(['1', '2'])
        self.assertTrue(_wait_until(
            lambda: self.orch.state.startswith('MONITORING'), 10.0,
        ), f'state={self.orch.state}')
        # Operator recalls the robot mid-sweep.
        dock_resp = _call_trigger(self.collector, '/mission/dock')
        self.assertTrue(dock_resp.success, dock_resp.message)
        # Must actually divert (pre-fix: stayed in MONITORING forever).
        self.assertTrue(_wait_until(lambda: self.orch.state == 'RETURNING', 10.0),
                        f'dock did not divert; state={self.orch.state}')

    # ── 11. abort during PREPARE no longer dead-ends in FAULT ───────────
    def test_abort_in_prepare_does_not_fault(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 4,
            seed_amcl=False, localization_timeout_s=30.0,
            tag_sequence=TEST_TAG_IDS,
        )
        _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertTrue(_wait_until(
            lambda: self.orch.state == 'PREPARE_LOCALIZING', 5.0,
        ), f'state={self.orch.state}')
        abort_resp = _call_trigger(self.collector, '/mission/abort')
        self.assertTrue(abort_resp.success, abort_resp.message)
        self.assertTrue(_wait_until(
            lambda: self.orch.state in ('DONE', 'READY'), 10.0,
        ), f'abort-in-PREPARE ended in {self.orch.state}')
        self.assertNotEqual(self.orch.state, 'FAULT')

    # ── 12. /mission/reset recovers a FAULTed orchestrator ──────────────
    def test_reset_clears_fault(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 4,
            seed_amcl=False, localization_timeout_s=1.0,
            tag_sequence=TEST_TAG_IDS,
        )
        _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'FAULT', 8.0),
                        f'state={self.orch.state}')
        reset_resp = _call_trigger(self.collector, '/mission/reset')
        self.assertTrue(reset_resp.success, reset_resp.message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'READY', 5.0),
                        f'state={self.orch.state}')
        # And a real mission can run afterwards.
        self.amcl.publish_low_cov()
        r2 = _call_start(self.collector, tag_sequence=['1'])
        self.assertTrue(r2.accepted, r2.error_message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 15.0),
                        f'state={self.orch.state}')

    # ── 13. pause is refused during a battery-driven return ─────────────
    def test_pause_rejected_during_battery_return(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 10,
            nav_delay_s=2.0, tag_sequence=TEST_TAG_IDS,
        )
        _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertTrue(_wait_until(
            lambda: self.orch.state == 'INSPECTING_NAVIGATING', 5.0,
        ))
        self.battery.publish_pct(0.10)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'RETURNING', 8.0),
                        f'state={self.orch.state}')
        pause_resp = _call_trigger(self.collector, '/mission/pause')
        self.assertFalse(pause_resp.success)
        self.assertIn('battery', pause_resp.message.lower())

    # ── 14. scan dwell does not advance the cursor while paused ─────────
    def test_scan_dwell_holds_when_paused(self):
        self._bringup(
            nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 6,
            flower_scan_dwell_s=3.0, tag_sequence=TEST_TAG_IDS,
        )
        _call_start(self.collector, tag_sequence=TEST_TAG_IDS)
        self.assertTrue(_wait_until(
            lambda: any(o.status == Observation.STATUS_OK
                        for o in self.collector.observations), 10.0,
        ))
        pause_resp = _call_trigger(self.collector, '/mission/pause')
        self.assertTrue(pause_resp.success, pause_resp.message)
        # Pre-fix: the dwell timer fires scan_done() and steps the cursor on
        # despite the pause. Post-fix: it holds in SCANNING.
        time.sleep(4.0)  # longer than the remaining dwell
        self.assertTrue(self.orch.state.endswith('SCANNING'),
                        f'cursor advanced during pause; state={self.orch.state}')
        self.assertEqual(len([o for o in self.collector.observations
                              if o.status == Observation.STATUS_OK]), 1)
        # Resume completes the run WITHOUT re-scanning the paused tag.
        resume_resp = _call_trigger(self.collector, '/mission/resume')
        self.assertTrue(resume_resp.success, resume_resp.message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'DONE', 20.0),
                        f'state={self.orch.state}')
        self.assertEqual(len([o for o in self.collector.observations
                              if o.status == Observation.STATUS_OK]), 3)

    # ── 15. aborting a docked monitoring mission does not crash ─────────
    def test_docked_abort_during_monitoring_no_crash(self):
        self.discovered = DiscoveredPub()
        self.harness.add(self.discovered)
        self._bringup(nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 50, nav_delay_s=0.5)
        resp = _call_start(
            self.collector, mission_type='ExplorationMission', discovery_goal=2,
        )
        self.assertTrue(resp.accepted, resp.error_message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'EXPLORING', 10.0))
        self.discovered.publish(['1', '2'])
        self.assertTrue(_wait_until(
            lambda: self.orch.state.startswith('MONITORING'), 10.0,
        ))
        self.battery.publish_pct(0.10)
        # Battery divert docks then parks paused (resumable monitoring run).
        self.assertTrue(_wait_until(
            lambda: self.orch.state == 'RETURNING'
            and self.collector.latest_state
            and self.collector.latest_state.paused, 14.0,
        ), f'state={self.orch.state}')
        abort_resp = _call_trigger(self.collector, '/mission/abort')
        # Pre-fix: AttributeError (remaining_indices on MonitoringMission).
        self.assertIsNotNone(abort_resp, 'abort service callback crashed')
        self.assertTrue(abort_resp.success, abort_resp.message)
        self.assertTrue(_wait_until(
            lambda: self.orch.state in ('DONE', 'READY'), 8.0,
        ), f'state={self.orch.state}')

    # ── 16. discovery-time climate seeding ─────────────────────────────
    def test_discovery_seeds_climate_during_exploring(self):
        # The moment a NEW tag is discovered during EXPLORING the orchestrator
        # must fire a one-shot bridge read and publish a KIND_TAG_READING/OK
        # observation pinned at the tag's discovered pose — front-loading
        # climate for the twin/HMI before the MONITORING scan ever runs.
        #
        # discovery_goal is set ABOVE the number of tags we publish so the
        # mission STAYS in EXPLORING: monitoring never runs, so any bridge call
        # / OK observation here can only be a discovery seed. That isolates the
        # seed path cleanly.
        self.discovered = DiscoveredPub()
        self.harness.add(self.discovered)
        self._bringup(nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 50)
        resp = _call_start(
            self.collector, mission_type='ExplorationMission', discovery_goal=5,
        )
        self.assertTrue(resp.accepted, resp.error_message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'EXPLORING', 10.0),
                        f'state={self.orch.state}')

        self.discovered.publish(['1', '2', '3'])  # 3 < goal 5 → stays EXPLORING

        # Each newly-discovered tag → exactly one bridge read.
        self.assertTrue(_wait_until(
            lambda: set(self.bridge.calls) >= {'1', '2', '3'}, 5.0),
            f'bridge calls={self.bridge.calls}')
        self.assertEqual(sorted(self.bridge.calls), ['1', '2', '3'],
                         'each discovered tag should be seeded exactly once')

        # One KIND_TAG_READING/OK observation per seeded tag.
        self.assertTrue(_wait_until(
            lambda: len([o for o in self.collector.observations
                         if o.status == Observation.STATUS_OK]) >= 3, 5.0))
        ok = [o for o in self.collector.observations
              if o.status == Observation.STATUS_OK]
        self.assertEqual(len(ok), 3)
        for o in ok:
            self.assertEqual(o.kind, Observation.KIND_TAG_READING)
            # Shaped exactly like a MONITORING reading so the twin ingests it
            # identically (the twin keys on tag_reading.tag_id, not source).
            self.assertEqual(o.source, 'ExplorationMission')
            self.assertEqual(o.status_detail, '')
            self.assertEqual(o.header.frame_id, 'map')
        self.assertEqual({o.tag_reading.tag_id for o in ok}, {'1', '2', '3'})
        # Pose is pinned at the tag's discovered map pose (DiscoveredPub sets
        # position.x = enumerate index): tag '2' is index 1 → x == 1.0.
        by_id = {o.tag_reading.tag_id: o for o in ok}
        self.assertAlmostEqual(by_id['2'].tag_pose_in_map.position.x, 1.0)
        self.assertEqual(by_id['2'].tag_pose_in_map.orientation.w, 1.0)

        # Never left EXPLORING → these are seeds, not monitoring scans.
        self.assertEqual(self.orch.state, 'EXPLORING')

    # ── 17. a tag is seeded only once ──────────────────────────────────
    def test_repeat_discovery_does_not_reseed(self):
        self.discovered = DiscoveredPub()
        self.harness.add(self.discovered)
        self._bringup(nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 50)
        resp = _call_start(
            self.collector, mission_type='ExplorationMission', discovery_goal=5,
        )
        self.assertTrue(resp.accepted, resp.error_message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'EXPLORING', 10.0))

        self.discovered.publish(['1', '2'])
        self.assertTrue(_wait_until(
            lambda: set(self.bridge.calls) >= {'1', '2'}, 5.0))
        self.assertEqual(sorted(self.bridge.calls), ['1', '2'])

        # The discovery feed is latched and re-delivers the same set; that must
        # NOT re-fire the bridge (each tag is seeded at most once).
        self.discovered.publish(['1', '2'])
        self.discovered.publish(['1', '2'])
        time.sleep(0.5)  # give any erroneous re-fire time to land
        self.assertEqual(sorted(self.bridge.calls), ['1', '2'],
                         'repeat discovery must not re-seed')

        # A genuinely NEW tag in a later message IS seeded; the old ones are
        # still not re-fired.
        self.discovered.publish(['1', '2', '3'])
        self.assertTrue(_wait_until(lambda: '3' in self.bridge.calls, 5.0))
        self.assertEqual(sorted(self.bridge.calls), ['1', '2', '3'])

    # ── 18. seeding never blocks the discovery callback / nav ──────────
    def test_seed_does_not_block_discovery_callback(self):
        # The seed must be non-blocking (call_async). If it blocked — e.g. a
        # synchronous bridge call — the orchestrator's single mutually-exclusive
        # callback group would stall on the first seed and frontier nav /
        # subsequent discoveries would freeze. We gate the bridge so it never
        # answers, fire a seed, and prove the orchestrator stays live: a SECOND
        # discovery is still processed while the first seed is parked in flight.
        gate = threading.Event()

        def gated(tag_id):
            gate.wait(timeout=10.0)
            return GetTagReading.Response.STATUS_OK, '', FakeBridge.DEFAULT_READINGS

        self.discovered = DiscoveredPub()
        self.harness.add(self.discovered)
        try:
            # goal high → mission stays EXPLORING, so the bridge is only ever
            # touched by seeds (never a monitoring scan).
            self._bringup(nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 50,
                          bridge_behaviour=gated)
            resp = _call_start(
                self.collector, mission_type='ExplorationMission',
                discovery_goal=5,
            )
            self.assertTrue(resp.accepted, resp.error_message)
            self.assertTrue(_wait_until(lambda: self.orch.state == 'EXPLORING', 10.0))

            # First discovery → seed fires and PARKS on the gated bridge (the
            # bridge records the call_async request before blocking on the gate).
            self.discovered.publish(['1'])
            self.assertTrue(_wait_until(lambda: '1' in self.bridge.calls, 5.0),
                            'seed call never reached the bridge')

            # The crux: if that in-flight seed had blocked the orchestrator's
            # (mutually-exclusive) discovery callback, this second discovery
            # could never be processed. It IS — the new tag lands in the
            # registry AND gets its own seed fired — proving the seed is
            # non-blocking (call_async), so frontier nav never stalls.
            # (We can't assert '2' in bridge.calls: FakeBridge's handler is
            # serialized, so the gated first call holds it until release.)
            self.discovered.publish(['1', '2'])
            self.assertTrue(_wait_until(
                lambda: '2' in self.orch._discovered
                and '2' in self.orch._seeded_tag_ids, 5.0),
                'discovery callback wedged by an in-flight seed')
        finally:
            gate.set()  # release the parked bridge handler(s) for clean teardown

    # ── 19. seeding is gated by the parameter ──────────────────────────
    def test_seeding_disabled_by_param(self):
        self.discovered = DiscoveredPub()
        self.harness.add(self.discovered)
        self._bringup(nav_outcomes=[GoalStatus.STATUS_SUCCEEDED] * 50,
                      seed_climate_on_discovery=False)
        resp = _call_start(
            self.collector, mission_type='ExplorationMission', discovery_goal=5,
        )
        self.assertTrue(resp.accepted, resp.error_message)
        self.assertTrue(_wait_until(lambda: self.orch.state == 'EXPLORING', 10.0))

        self.discovered.publish(['1', '2', '3'])
        time.sleep(0.7)  # ample time for a seed to fire if the gate were open
        self.assertEqual(self.bridge.calls, [],
                         'seeding disabled → no bridge reads on discovery')
        self.assertEqual(
            [o for o in self.collector.observations
             if o.status == Observation.STATUS_OK], [],
            'seeding disabled → no seed observations')
        self.assertEqual(self.orch.state, 'EXPLORING')


if __name__ == '__main__':
    unittest.main()
