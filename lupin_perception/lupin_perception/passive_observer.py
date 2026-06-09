"""passive_observer — climate readings on sight when no mission is running.

The autonomous scan loop polls the greenhouse bridge per tag and emits
KIND_TAG_READING observations that feed the twin's heatmap. With no mission
running (teleop / manual SLAM) that loop is idle, so the heatmap stays empty.
This node fills the gap: it watches /perception/discovered_tags and, while the
mission is idle, polls /greenhouse_bridge/get_tag_reading once per tag and
republishes the reading on the twin's existing /floranova/observations intake.

It defers entirely while a mission is active — the orchestrator owns observation
then ("passive observation when idle; the mission owns observation when active").
"""

from __future__ import annotations

import time
from typing import Optional

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from lupin_msgs.msg import DiscoveredTags, MissionState, Observation
from lupin_msgs.srv import GetTagReading

from .observations import make_tag_reading_observation

# A mission owns observation end-to-end while in these lifecycles; outside them
# (BOOT/READY/DONE/FAULT/empty, or no mission) passive polling may run.
_ACTIVE_LIFECYCLES = ('PREPARE', 'EXPLORING', 'INSPECTING', 'MONITORING', 'RETURNING')


def mission_active(state: Optional[MissionState]) -> bool:
    """True while a mission is driving/scanning (so passive polling must defer)."""
    if state is None:
        return False
    return state.lifecycle_state in _ACTIVE_LIFECYCLES


class PassiveObserver(Node):
    """Poll the greenhouse oracle per discovered tag while the mission is idle."""

    def __init__(self) -> None:
        super().__init__('passive_observer')

        self.declare_parameter('discovered_tags_topic', '/perception/discovered_tags')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('bridge_service_name', '/greenhouse_bridge/get_tag_reading')
        self.declare_parameter('observations_topic', '/floranova/observations')
        self.declare_parameter('map_frame', 'map')
        # 0.0 = read each tag once; > 0 = re-read after this many seconds so the
        # heatmap can track time-of-day drift in the sim oracle.
        self.declare_parameter('refresh_period_s', 0.0)
        self.declare_parameter('tick_period_s', 0.5)
        self.declare_parameter('source_name', 'passive_observer')

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._refresh_period = float(self.get_parameter('refresh_period_s').value)
        self._source = str(self.get_parameter('source_name').value)
        tick = float(self.get_parameter('tick_period_s').value)

        self._tags: dict[str, Pose] = {}        # tag_id -> latest map pose
        self._last_read: dict[str, float] = {}  # tag_id -> monotonic of last read
        self._inflight: set[str] = set()        # tag_ids with a request pending
        self._mission_state: Optional[MissionState] = None

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        # The twin (and every other /floranova/observations producer) uses a
        # RELIABLE + TRANSIENT_LOCAL observation bus. Publishing with the bare
        # `depth` constructor here defaults to VOLATILE durability, which is
        # *incompatible* with the twin's TRANSIENT_LOCAL subscription — DDS then
        # refuses the match ("requesting incompatible QoS … DURABILITY") and the
        # twin silently never receives a single passive climate reading. Match
        # the twin's profile so the readings actually land.
        obs_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._obs_pub = self.create_publisher(
            Observation, str(self.get_parameter('observations_topic').value), obs_qos)
        self.create_subscription(
            DiscoveredTags, str(self.get_parameter('discovered_tags_topic').value),
            self._on_discovered_tags, latched)
        self.create_subscription(
            MissionState, str(self.get_parameter('mission_state_topic').value),
            self._on_mission_state, latched)
        self._bridge = self.create_client(
            GetTagReading, str(self.get_parameter('bridge_service_name').value))

        self._timer = self.create_timer(tick, self._tick)
        self.get_logger().info(
            f'passive_observer ready: polls {self._bridge.srv_name} per discovered '
            f'tag while idle (refresh_period_s={self._refresh_period}).')

    # ── subscriptions ──────────────────────────────────────────────────
    def _on_discovered_tags(self, msg: DiscoveredTags) -> None:
        if msg.header.frame_id and msg.header.frame_id != self._map_frame:
            self.get_logger().warn(
                f'discovered-tags frame {msg.header.frame_id!r} != '
                f'{self._map_frame!r}; ignoring.', throttle_duration_sec=10.0)
            return
        for t in msg.tags:
            if t.tag_id:
                self._tags[t.tag_id] = t.pose_in_map

    def _on_mission_state(self, msg: MissionState) -> None:
        self._mission_state = msg

    # ── periodic poll ──────────────────────────────────────────────────
    def _now(self) -> float:
        return time.monotonic()

    def _due_tags(self) -> list[str]:
        """Tag ids to poll this tick, given current state (defers if active)."""
        if mission_active(self._mission_state):
            return []
        now = self._now()
        due: list[str] = []
        for tag_id in self._tags:
            if tag_id in self._inflight:
                continue
            last = self._last_read.get(tag_id)
            if last is None:
                due.append(tag_id)
            elif self._refresh_period > 0.0 and (now - last) >= self._refresh_period:
                due.append(tag_id)
        return due

    def _tick(self) -> None:
        due = self._due_tags()
        if not due:
            return
        if not self._bridge.service_is_ready():
            self.get_logger().warn(
                f'greenhouse bridge {self._bridge.srv_name} not available yet; '
                'retrying.', throttle_duration_sec=10.0)
            return
        for tag_id in due:
            self._request_reading(tag_id)

    def _request_reading(self, tag_id: str) -> None:
        req = GetTagReading.Request()
        req.tag_id = tag_id
        self._inflight.add(tag_id)
        future = self._bridge.call_async(req)
        future.add_done_callback(lambda f, tid=tag_id: self._on_reading(tid, f))

    def _on_reading(self, tag_id: str, future) -> None:
        self._inflight.discard(tag_id)
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 — log and move on; retry next tick
            self.get_logger().warn(
                f'bridge call for {tag_id!r} failed: {exc}',
                throttle_duration_sec=10.0)
            return
        # Mark read on OK *and* UNKNOWN_TAG so an out-of-greenhouse tag isn't
        # polled every tick.
        self._last_read[tag_id] = self._now()
        if response.status != GetTagReading.Response.STATUS_OK:
            self.get_logger().debug(
                f'tag {tag_id!r} unknown to greenhouse bridge; skipping.')
            return
        obs = make_tag_reading_observation(
            source=self._source,
            stamp=self.get_clock().now().to_msg(),
            tag_reading=response.reading,
            tag_map_pose=self._tags.get(tag_id),
            frame_id=self._map_frame,
        )
        self._obs_pub.publish(obs)


def main(args=None):
    rclpy.init(args=args)
    node = PassiveObserver()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
