"""Mission orchestrator: drive the MIRTE Master through a tag-scanning routine.

State machine (5 states):
    IDLE        -> watchdog polls Nav2 + bridge availability each tick
    NAVIGATING  -> NavigateToPose to the current tag's (x, y)
    SCANNING    -> call greenhouse bridge for the current tag's readings
    LOGGING     -> append to in-memory mission log, print one-line summary
    DONE        -> terminal; node stays alive so logs/introspection still work

Transitions are event-driven (action/service callbacks, watchdog timer).
We never block the executor waiting on a result.
"""

from __future__ import annotations

import json
import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter

from lupin_msgs.srv import GetTagReading


class State(Enum):
    IDLE = 'IDLE'
    NAVIGATING = 'NAVIGATING'
    SCANNING = 'SCANNING'
    LOGGING = 'LOGGING'
    DONE = 'DONE'


@dataclass
class TagResult:
    tag_id: str
    succeeded: bool = False
    failure_reason: str = ''
    nav_attempts: int = 0  # total NavigateToPose goals issued for this tag
    readings: dict = field(default_factory=dict)
    sim_time_of_day_seconds: float = 0.0
    stamp: Optional[Any] = None  # builtin_interfaces/Time, kept loose to avoid hard import


def _yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def _load_default_tag_locations() -> dict:
    """Return the tags sub-dict from the installed mdp-greenhouse package.

    Path is resolved via importlib.resources so we don't bake a site-packages
    path into the source. mdp-greenhouse is pip-installed (declared in
    setup.py install_requires); colcon won't install it for you.
    """
    try:
        from importlib.resources import files
    except ImportError:  # pragma: no cover - we target py3.10
        from importlib_resources import files  # type: ignore

    try:
        cfg_path = files('greenhouse_sim').joinpath('configs/tag_locations.json')
    except (ModuleNotFoundError, ImportError) as exc:
        raise RuntimeError(
            "Could not import 'greenhouse_sim'. The mdp-greenhouse package is "
            "a runtime dependency of lupin_mission and is not in rosdep — "
            "install it with: pip install 'mdp-greenhouse>=1.0.3,<2'"
        ) from exc
    data = json.loads(cfg_path.read_text())
    return data['tags']


def _numeric_string_sort_key(tag_id: str):
    """Sort numeric tag IDs as ints (numeric IDs first), alpha IDs after."""
    try:
        return (0, int(tag_id))
    except ValueError:
        return (1, tag_id)


class MissionOrchestrator(Node):
    """End-to-end orchestrator: navigate to each tag, scan it, log it."""

    def __init__(self, node_name: str = 'mission_orchestrator', **node_kwargs):
        # Forward parameter_overrides etc. to the rclpy Node constructor so
        # callers (incl. tests) can inject params without monkey-patching.
        super().__init__(node_name, **node_kwargs)

        # Type-only declaration: empty-list defaults infer as BYTE_ARRAY in
        # rclpy and reject a STRING_ARRAY override. Use the type-only form so
        # the param starts unset and accepts strings cleanly.
        self.declare_parameter('tag_sequence', Parameter.Type.STRING_ARRAY)
        self.declare_parameter('approach_yaw', 0.0)
        self.declare_parameter('nav_timeout_sec', 60.0)
        self.declare_parameter('service_timeout_sec', 5.0)
        self.declare_parameter('dependency_timeout_sec', 30.0)
        self.declare_parameter('nav_retry_limit', 1)
        self.declare_parameter('frame_id', 'map')

        self._approach_yaw = float(self.get_parameter('approach_yaw').value)
        self._nav_timeout = float(self.get_parameter('nav_timeout_sec').value)
        self._service_timeout = float(self.get_parameter('service_timeout_sec').value)
        self._dependency_timeout = float(self.get_parameter('dependency_timeout_sec').value)
        self._nav_retry_limit = int(self.get_parameter('nav_retry_limit').value)
        self._frame_id = str(self.get_parameter('frame_id').value)

        # Tag coordinates: {tag_id: {'x': float, 'y': float, ...}}
        # Loaded once on startup; the bridge uses string IDs, so do we.
        self._tag_locations: dict = _load_default_tag_locations()

        # Visit order. Param-not-set or empty list = visit every known tag
        # in numeric-string order.
        try:
            param_seq = self.get_parameter('tag_sequence').value or []
        except rclpy.exceptions.ParameterUninitializedException:
            param_seq = []
        if param_seq:
            self._tag_sequence: list[str] = [str(t) for t in param_seq]
        else:
            self._tag_sequence = sorted(
                self._tag_locations.keys(), key=_numeric_string_sort_key
            )

        # Mutually-exclusive callback group keeps the state machine
        # serialised under a single-threaded executor.
        self._cb_group = MutuallyExclusiveCallbackGroup()

        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self._cb_group,
        )
        self._bridge_client = self.create_client(
            GetTagReading,
            '/greenhouse_bridge/get_tag_reading',
            callback_group=self._cb_group,
        )

        # Mission state
        self._state: State = State.IDLE
        self._current_index: int = 0
        self._current_attempt: int = 0  # 1-based: incremented in _send_nav_goal
        self._results: list[TagResult] = [
            TagResult(tag_id=t) for t in self._tag_sequence
        ]
        self._mission_log: list[dict] = []

        # Active futures / handle. We compare by `is` in callbacks so a stale
        # response from a cancelled or superseded goal can't drive the new
        # attempt's bookkeeping.
        self._goal_handle = None
        self._send_goal_future = None
        self._get_result_future = None
        self._service_future = None

        # Watchdog drives both IDLE dependency-polling and per-state timeouts.
        # _state_started_at is reset on every transition.
        self._state_started_at: float = self._monotonic()
        self._watchdog = self.create_timer(
            0.1, self._on_watchdog, callback_group=self._cb_group
        )

        self.get_logger().info(
            f'Mission orchestrator created: '
            f'{len(self._tag_sequence)} tags in sequence, '
            f'frame_id={self._frame_id}, approach_yaw={self._approach_yaw}'
        )

        if not self._tag_sequence:
            self.get_logger().warn(
                'tag_sequence is empty and no tags were found in '
                'tag_locations.json; nothing to do.'
            )
            self._transition(State.DONE, 'NO_TAGS')
            self._log_final_summary()
            return

        self.get_logger().info(
            f'Waiting up to {self._dependency_timeout:.1f}s for nav2 + bridge...'
        )

    # ── time helper ────────────────────────────────────────────────────
    @staticmethod
    def _monotonic() -> float:
        # Wall-clock monotonic for timeouts; not ROS time. We don't want
        # /clock pauses in sim to wedge the watchdog.
        return time.monotonic()

    # ── state transition helper ────────────────────────────────────────
    def _transition(self, new_state: State, event: str) -> None:
        old = self._state
        tag_id = (
            self._results[self._current_index].tag_id
            if 0 <= self._current_index < len(self._results)
            else '-'
        )
        self.get_logger().info(
            f'[{old.value}] {event}: {old.value}->{new_state.value} (tag_id={tag_id})'
        )
        self._state = new_state
        self._state_started_at = self._monotonic()

    # ── IDLE: dependency polling (driven by watchdog) ──────────────────
    def _poll_dependencies(self) -> None:
        elapsed = self._monotonic() - self._state_started_at
        nav_ready = self._nav_client.server_is_ready()
        bridge_ready = self._bridge_client.service_is_ready()

        if nav_ready and bridge_ready:
            self.get_logger().info('Dependencies up. Starting mission.')
            self._begin_navigating(self._current_index)
            return

        if elapsed > self._dependency_timeout:
            missing = []
            if not nav_ready:
                missing.append('navigate_to_pose action server')
            if not bridge_ready:
                missing.append('get_tag_reading service')
            self.get_logger().error(
                f'Dependencies did not appear within '
                f'{self._dependency_timeout:.1f}s; missing: {", ".join(missing)}'
            )
            self._transition(State.DONE, 'DEPENDENCY_TIMEOUT')
            self._log_final_summary()

    # ── NAVIGATING ─────────────────────────────────────────────────────
    def _begin_navigating(self, index: int) -> None:
        self._current_index = index
        self._current_attempt = 0
        if index >= len(self._tag_sequence):
            self._transition(State.DONE, 'ALL_TAGS_DONE')
            self._log_final_summary()
            return
        self._transition(State.NAVIGATING, 'NEXT_TAG')
        self._send_nav_goal()

    def _retry_or_advance(self, reason: str) -> None:
        """Called when a nav attempt fails (status, timeout, or rejection)."""
        # `_current_attempt` was already bumped in _send_nav_goal for the
        # attempt that just failed; nav_attempts on the result mirrors that.
        result = self._results[self._current_index]
        if self._current_attempt - 1 < self._nav_retry_limit:
            self.get_logger().warn(
                f'Nav failed ({reason}); retrying tag {result.tag_id} '
                f'(attempt {self._current_attempt + 1}/'
                f'{self._nav_retry_limit + 1})'
            )
            self._transition(State.NAVIGATING, f'NAV_RETRY({reason})')
            self._send_nav_goal()
        else:
            result.succeeded = False
            result.failure_reason = f'nav_failed:{reason}'
            self.get_logger().error(
                f'Tag {result.tag_id} marked failed: {result.failure_reason}'
            )
            self._advance_to_next_tag()

    def _send_nav_goal(self) -> None:
        tag_id = self._results[self._current_index].tag_id
        loc = self._tag_locations.get(tag_id)
        if loc is None:
            self.get_logger().error(
                f"Tag '{tag_id}' missing from tag_locations.json; "
                f'skipping (cannot navigate to unknown coordinates).'
            )
            self._results[self._current_index].failure_reason = 'unknown_coordinates'
            self._advance_to_next_tag()
            return

        self._current_attempt += 1
        self._results[self._current_index].nav_attempts = self._current_attempt

        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self._frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(loc['x'])
        pose.pose.position.y = float(loc['y'])
        pose.pose.position.z = 0.0
        pose.pose.orientation = _yaw_to_quaternion(self._approach_yaw)
        goal_msg.pose = pose

        # Reset the timeout clock for THIS attempt.
        self._state_started_at = self._monotonic()
        self._goal_handle = None
        self._get_result_future = None
        self._send_goal_future = self._nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self._on_nav_goal_response)

    def _on_nav_goal_response(self, future) -> None:
        # Stale-future guard: if a watchdog timeout already moved us on,
        # the late response belongs to a goal we no longer care about.
        if future is not self._send_goal_future or self._state != State.NAVIGATING:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f'send_goal_async raised: {exc!r}')
            self._retry_or_advance('send_goal_exception')
            return
        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected the goal.')
            self._retry_or_advance('rejected')
            return
        self._goal_handle = goal_handle
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future) -> None:
        # Stale-future guard: identity check beats state check, because the
        # watchdog can transition NAVIGATING -> NAVIGATING (retry) and the
        # OLD goal's result would otherwise drive the NEW attempt.
        if future is not self._get_result_future or self._state != State.NAVIGATING:
            return
        try:
            wrapped = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f'get_result_async raised: {exc!r}')
            self._retry_or_advance('result_exception')
            return
        status = wrapped.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self._transition(State.SCANNING, 'NAV_SUCCEEDED')
            self._send_scan_request()
        elif status == GoalStatus.STATUS_ABORTED:
            self._retry_or_advance('aborted')
        elif status == GoalStatus.STATUS_CANCELED:
            self._retry_or_advance('canceled')
        else:
            self._retry_or_advance(f'status_{status}')

    # ── SCANNING ───────────────────────────────────────────────────────
    def _send_scan_request(self) -> None:
        tag_id = self._results[self._current_index].tag_id
        request = GetTagReading.Request()
        request.tag_id = tag_id
        # Reset clock for the service-call attempt.
        self._state_started_at = self._monotonic()
        self._service_future = self._bridge_client.call_async(request)
        self._service_future.add_done_callback(self._on_scan_response)

    def _on_scan_response(self, future) -> None:
        # Stale-future guard against a late response after a service-call
        # timeout already advanced us.
        if future is not self._service_future or self._state != State.SCANNING:
            return
        try:
            response = future.result()
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().error(f'bridge call raised: {exc!r}')
            self._mark_failed_and_advance(f'service_exception:{exc!r}')
            return

        if response.status == GetTagReading.Response.STATUS_OK:
            self._record_reading(response.reading)
            self._transition(State.LOGGING, 'BRIDGE_OK')
            self._do_logging()
        else:
            # Includes STATUS_UNKNOWN_TAG and any future non-OK statuses.
            reason = (
                response.error_message
                or f'bridge_status_{response.status}'
            )
            self._mark_failed_and_advance(f'bridge:{reason}')

    # ── LOGGING ────────────────────────────────────────────────────────
    def _record_reading(self, reading: Any) -> None:
        result = self._results[self._current_index]
        result.succeeded = True
        result.failure_reason = ''
        result.stamp = reading.stamp
        result.sim_time_of_day_seconds = float(reading.sim_time_of_day_seconds)
        result.readings = {
            sr.name: float(sr.value) for sr in reading.readings
        }
        self._mission_log.append({
            'tag_id': reading.tag_id,
            'stamp': reading.stamp,
            'sim_time_of_day_seconds': result.sim_time_of_day_seconds,
            'readings': dict(result.readings),
        })

    def _do_logging(self) -> None:
        # WATCHDOG-EXEMPT: this method must stay synchronous. The watchdog
        # skips LOGGING because we transition out of it before the next tick.
        # If a future digital-twin publisher hooks in here and adds *any*
        # async work, extend the watchdog to cover LOGGING too — otherwise a
        # stalled publisher will wedge the mission silently.
        # TODO(digital-twin): hand the latest mission_log entry to the
        # downstream digital-twin publisher (separate piece of work).
        result = self._results[self._current_index]
        readings_str = ' '.join(
            f'{name}={value:.1f}' for name, value in result.readings.items()
        )
        self.get_logger().info(
            f'[tag {result.tag_id}] {readings_str} '
            f'(sim_time={result.sim_time_of_day_seconds:.0f}s)'
        )
        self._advance_to_next_tag()

    # ── advancement / termination ──────────────────────────────────────
    def _mark_failed_and_advance(self, reason: str) -> None:
        result = self._results[self._current_index]
        result.succeeded = False
        result.failure_reason = reason
        self.get_logger().error(
            f'Tag {result.tag_id} marked failed: {reason}'
        )
        self._advance_to_next_tag()

    def _advance_to_next_tag(self) -> None:
        next_index = self._current_index + 1
        if next_index >= len(self._tag_sequence):
            self._transition(State.DONE, 'ALL_TAGS_DONE')
            self._log_final_summary()
            return
        self._begin_navigating(next_index)

    def _log_final_summary(self) -> None:
        total = len(self._results)
        succeeded = sum(1 for r in self._results if r.succeeded)
        failed = total - succeeded
        self.get_logger().info(
            f'Mission DONE: attempted={total} succeeded={succeeded} '
            f'failed={failed}'
        )
        for r in self._results:
            if r.succeeded:
                self.get_logger().info(
                    f'  tag {r.tag_id}: OK (attempts={r.nav_attempts}, '
                    f'sensors={list(r.readings.keys())})'
                )
            else:
                self.get_logger().info(
                    f'  tag {r.tag_id}: FAIL ({r.failure_reason or "unknown"})'
                )

    # ── watchdog ───────────────────────────────────────────────────────
    def _on_watchdog(self) -> None:
        if self._state == State.IDLE:
            self._poll_dependencies()
            return
        if self._state in (State.DONE, State.LOGGING):
            # DONE is terminal. LOGGING is documented WATCHDOG-EXEMPT above.
            return

        elapsed = self._monotonic() - self._state_started_at
        if self._state == State.NAVIGATING and elapsed > self._nav_timeout:
            self.get_logger().warn(
                f'NAV timed out after {elapsed:.1f}s (limit {self._nav_timeout:.1f}s); '
                f'cancelling goal.'
            )
            self._cancel_active_goal()
            self._retry_or_advance('timeout')
        elif self._state == State.SCANNING and elapsed > self._service_timeout:
            self.get_logger().warn(
                f'SCAN timed out after {elapsed:.1f}s (limit {self._service_timeout:.1f}s).'
            )
            # rclpy doesn't actually cancel an in-flight service call; the
            # response will arrive and _on_scan_response's identity check
            # will drop it. Clearing the future here makes that explicit.
            self._service_future = None
            self._mark_failed_and_advance('service_timeout')

    def _cancel_active_goal(self) -> None:
        if self._goal_handle is not None:
            try:
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._on_cancel_done)
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().warn(f'cancel_goal_async failed: {exc!r}')
        self._goal_handle = None
        # Clear get_result_future too; identity check in _on_nav_result will
        # drop the late STATUS_CANCELED that the server eventually sends.
        self._get_result_future = None

    def _on_cancel_done(self, future) -> None:
        try:
            response = future.result()
            self.get_logger().debug(
                f'Goal cancellation acknowledged: '
                f'return_code={response.return_code}'
            )
        except Exception as exc:  # pragma: no cover - defensive
            self.get_logger().warn(f'cancel response raised: {exc!r}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionOrchestrator()
    # TODO(start_mission_service): expose a /lupin_mission/start_mission
    # service so the orchestrator can be triggered on demand instead of
    # running the routine on bringup. v1 just runs once on startup.
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
