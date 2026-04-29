"""End-to-end tests for the mission orchestrator state machine.

Each test spins:
    - a fake NavigateToPose action server (programmable per-test)
    - a fake GetTagReading service     (programmable per-test)
    - the MissionOrchestrator under test

All three nodes share a MultiThreadedExecutor on a background thread so the
orchestrator's async send_goal/call_async chain can complete without blocking
the test thread. We block the test thread on the orchestrator reaching DONE.

We do NOT depend on real Nav2 or the real greenhouse bridge.
"""

import os
import threading
import time
import unittest
from unittest import mock

# Isolate this test process from any real ROS nodes the dev may have running
# on the default domain (e.g. a live greenhouse bridge). Must be set before
# rclpy imports/inits anything. Unconditional assignment, not setdefault:
# the team's .bashrc presets ROS_DOMAIN_ID=0 for MIRTE, which would silently
# defeat isolation otherwise.
os.environ['ROS_DOMAIN_ID'] = '47'
os.environ['ROS_LOCALHOST_ONLY'] = '1'

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as TimeMsg
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from lupin_msgs.msg import SensorReading, TagReading
from lupin_msgs.srv import GetTagReading

from lupin_mission.mission_orchestrator import MissionOrchestrator, State


TEST_TAG_IDS = ['1', '2', '3']
FAKE_TAG_LOCATIONS = {
    tid: {'x': float(i), 'y': float(i) + 0.5, 'sensors': ['temperature']}
    for i, tid in enumerate(TEST_TAG_IDS)
}

DONE_TIMEOUT_SEC = 20.0


def _wait_until(predicate, timeout_sec):
    """Poll predicate() until truthy or timeout. Returns final predicate value."""
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.05)
    return predicate()


class FakeNavServer(Node):
    """Programmable NavigateToPose action server.

    `result_sequence` is a list of GoalStatus codes; the i-th goal received
    yields result_sequence[i] (clamped to last entry if more goals arrive).
    """

    def __init__(self, result_sequence):
        super().__init__('fake_nav_server')
        self.result_sequence = list(result_sequence)
        self.goals_received = 0
        self._server = ActionServer(
            self,
            NavigateToPose,
            'navigate_to_pose',
            execute_callback=self._execute,
        )

    def _execute(self, goal_handle):
        idx = min(self.goals_received, len(self.result_sequence) - 1)
        status = self.result_sequence[idx]
        self.goals_received += 1
        result = NavigateToPose.Result()
        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_handle.succeed()
        elif status == GoalStatus.STATUS_ABORTED:
            goal_handle.abort()
        elif status == GoalStatus.STATUS_CANCELED:
            goal_handle.canceled()
        else:
            goal_handle.abort()
        return result


class FakeBridge(Node):
    """Programmable GetTagReading service.

    `behaviour` is a callable: tag_id -> (status_code, error_message,
    optional readings dict). Default returns STATUS_OK with synthetic readings.
    """

    def __init__(self, behaviour=None):
        super().__init__('fake_bridge')
        self.calls = []
        self._behaviour = behaviour or self._default_ok
        self._service = self.create_service(
            GetTagReading,
            '/greenhouse_bridge/get_tag_reading',
            self._handle,
        )

    @staticmethod
    def _default_ok(tag_id):
        readings = {
            'temperature': 22.4,
            'humidity': 58.1,
            'co2': 441.0,
            'light': 796.1,
        }
        return GetTagReading.Response.STATUS_OK, '', readings

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


class _SpinHarness:
    """Owns the executor + spin thread. Add nodes, then start()."""

    def __init__(self):
        self.executor = MultiThreadedExecutor(num_threads=4)
        self.nodes = []
        self._thread = None
        self._stop = threading.Event()

    def add(self, node):
        self.nodes.append(node)
        self.executor.add_node(node)

    def start(self):
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _spin(self):
        while rclpy.ok() and not self._stop.is_set():
            self.executor.spin_once(timeout_sec=0.1)

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for node in self.nodes:
            self.executor.remove_node(node)
            node.destroy_node()


class MissionOrchestratorIntegrationTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        self.harness = _SpinHarness()
        # Monkey-patch the tag_locations loader so tests do not depend on
        # greenhouse_sim being importable from the test interpreter.
        self._patcher = mock.patch(
            'lupin_mission.mission_orchestrator._load_default_tag_locations',
            return_value=dict(FAKE_TAG_LOCATIONS),
        )
        self._patcher.start()

    def tearDown(self):
        self.harness.stop()
        self._patcher.stop()

    def _build_orchestrator(self, **param_overrides):
        """Construct the orchestrator with parameter_overrides — exercises the
        same wiring that ros2 launch param=value would use, no monkey-poking
        of private state."""
        defaults = dict(
            tag_sequence=TEST_TAG_IDS,
            approach_yaw=0.0,
            nav_timeout_sec=5.0,
            service_timeout_sec=2.0,
            dependency_timeout_sec=10.0,
            nav_retry_limit=1,
            frame_id='map',
        )
        defaults.update(param_overrides)
        overrides = [Parameter(k, value=v) for k, v in defaults.items()]
        return MissionOrchestrator(parameter_overrides=overrides)

    def _start_servers_and_orchestrator(self, nav, bridge, **param_overrides):
        """Add fake servers, start spinning, wait for discovery via the
        orchestrator's own clients (not a sleep), then return the orchestrator."""
        self.harness.add(nav)
        self.harness.add(bridge)
        self.harness.start()
        orchestrator = self._build_orchestrator(**param_overrides)
        self.harness.add(orchestrator)
        # Wait deterministically for the orchestrator to discover its deps.
        # Beats `time.sleep(0.3)` — fails fast if discovery never completes.
        self.assertTrue(
            orchestrator._nav_client.wait_for_server(timeout_sec=5.0),
            'orchestrator never discovered fake nav action server',
        )
        self.assertTrue(
            orchestrator._bridge_client.wait_for_service(timeout_sec=5.0),
            'orchestrator never discovered fake bridge service',
        )
        return orchestrator

    def _wait_for_done(self, orchestrator, timeout=DONE_TIMEOUT_SEC):
        return _wait_until(
            lambda: orchestrator._state == State.DONE, timeout
        )

    # ── happy path ─────────────────────────────────────────────────────
    def test_happy_path_three_tags_all_succeed(self):
        nav = FakeNavServer([GoalStatus.STATUS_SUCCEEDED])
        bridge = FakeBridge()
        orchestrator = self._start_servers_and_orchestrator(nav, bridge)

        self.assertTrue(
            self._wait_for_done(orchestrator),
            f'Orchestrator did not reach DONE; state={orchestrator._state}',
        )
        self.assertEqual(nav.goals_received, 3)
        self.assertEqual(bridge.calls, TEST_TAG_IDS)
        self.assertTrue(all(r.succeeded for r in orchestrator._results))
        self.assertTrue(all(r.nav_attempts == 1 for r in orchestrator._results))
        self.assertEqual(len(orchestrator._mission_log), 3)

    # ── nav failure with retry ─────────────────────────────────────────
    def test_nav_aborted_then_succeeds_on_retry(self):
        nav = FakeNavServer([
            GoalStatus.STATUS_ABORTED,
            GoalStatus.STATUS_SUCCEEDED,
        ])
        bridge = FakeBridge()
        orchestrator = self._start_servers_and_orchestrator(
            nav, bridge, tag_sequence=['1'], nav_retry_limit=1,
        )

        self.assertTrue(
            self._wait_for_done(orchestrator),
            f'Orchestrator did not reach DONE; state={orchestrator._state}',
        )
        # Two nav goals issued: the aborted one + the retry.
        self.assertEqual(nav.goals_received, 2)
        self.assertEqual(bridge.calls, ['1'])
        self.assertTrue(orchestrator._results[0].succeeded)
        # nav_attempts now reflects total attempts including the successful one.
        self.assertEqual(orchestrator._results[0].nav_attempts, 2)

    # ── bridge failure (UNKNOWN_TAG) ───────────────────────────────────
    def test_bridge_unknown_tag_fails_and_advances(self):
        nav = FakeNavServer([GoalStatus.STATUS_SUCCEEDED])

        def behaviour(tag_id):
            if tag_id == '2':
                return (
                    GetTagReading.Response.STATUS_UNKNOWN_TAG,
                    f"unknown '{tag_id}'",
                    None,
                )
            return FakeBridge._default_ok(tag_id)

        bridge = FakeBridge(behaviour=behaviour)
        orchestrator = self._start_servers_and_orchestrator(
            nav, bridge, tag_sequence=['1', '2', '3'],
        )

        self.assertTrue(
            self._wait_for_done(orchestrator),
            f'Orchestrator did not reach DONE; state={orchestrator._state}',
        )
        # Nav succeeds for all three; bridge fails only for tag 2.
        self.assertEqual(nav.goals_received, 3)
        self.assertEqual(bridge.calls, ['1', '2', '3'])
        results_by_id = {r.tag_id: r for r in orchestrator._results}
        self.assertTrue(results_by_id['1'].succeeded)
        self.assertFalse(results_by_id['2'].succeeded)
        self.assertIn('bridge:', results_by_id['2'].failure_reason)
        self.assertTrue(results_by_id['3'].succeeded)


if __name__ == '__main__':
    unittest.main()
