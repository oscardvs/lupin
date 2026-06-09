"""Digital-twin node — aggregates per-tag observations from the orchestrator
and exposes the world state to consumers (HMI, FloraNova export, anomaly
detector, …).

ROS interfaces
--------------
* Subscribes ``/floranova/observations`` (lupin_msgs/Observation,
  RELIABLE+TRANSIENT_LOCAL depth 50) — late subscribers get the backlog
  the orchestrator latched.
* Publishes ``/twin/state`` (lupin_msgs/TwinState, RELIABLE+TRANSIENT_LOCAL
  depth 1) at ~1 Hz so a freshly-connected HMI sees the world without a
  bootstrap call.
* Serves ``/twin/get_field`` (lupin_msgs/GetField) — IDW interpolation of
  a sensor over a bbox, NaN beyond max-distance and outside an explored
  mask. Cached by (sensor, observation_count); cache invalidates when the
  store mutates.

Out of scope this MR: no /map subscription (explored mask is a no-op for
v1 — every cell is treated as explored), no history service, no disk
persistence, no anomaly detection. Each is a clean follow-up.
"""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
from typing import Optional

import rclpy
from builtin_interfaces.msg import Time as TimeMsg
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from geometry_msgs.msg import Point, Point32, Polygon
from std_msgs.msg import Header

from lupin_msgs.msg import (
    DiscoveredTags,
    FlowerPoint,
    Observation,
    SensorReading,
    TwinState,
    TwinTagState,
)
from lupin_msgs.srv import GetField

from .idw import (
    DEFAULT_FALLOFF_RADIUS_M,
    DEFAULT_MAX_DISTANCE_M,
    DEFAULT_POWER,
    FieldSample,
    compute_idw_field,
)
from .state import (
    DEFAULT_BUFFER_LEN,
    FlowerUpdate,
    TagSensorEntry,
    TwinFlower,
    TwinObservation,
    TwinStateStore,
)


# Bounded LRU on the field cache. The keyspace is small in practice — a
# handful of (sensor, obs_count, bbox, resolution) combinations per
# session — but a HMI bug that calls GetField with a sliding bbox every
# frame would balloon it. 64 entries is enough to cover steady-state
# repeats without growing without bound on long sessions.
_FIELD_CACHE_CAP = 64


@dataclass(frozen=True)
class _CachedField:
    """Field-grid values cached after a successful /twin/get_field call.

    We deliberately store the *fields* the next response needs, not the
    rclpy Response object the executor handed us. Caching the Response
    aliases that one allocation across calls — fine under
    SingleThreadedExecutor + MutuallyExclusiveCallbackGroup, but a
    footgun if anyone moves to MultiThreadedExecutor later. Fresh-copy
    on the way out costs ~one tuple-unpack and removes the hazard.
    """
    values: tuple
    width: int
    height: int
    origin_x: float
    origin_y: float
    resolution_used: float
    value_min: float
    value_max: float
    sample_count: int


class TwinNode(Node):
    """Long-lived twin node. Single-threaded executor + mutex callback group
    so the store and field-cache mutations are serialised."""

    def __init__(self):
        super().__init__('lupin_twin')

        # ── parameters ─────────────────────────────────────────────────
        self.declare_parameter('observations_topic', '/floranova/observations')
        self.declare_parameter('state_topic', '/twin/state')
        self.declare_parameter('field_service_name', '/twin/get_field')
        self.declare_parameter('state_publish_rate_hz', 1.0)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('buffer_len', DEFAULT_BUFFER_LEN)
        # Discovery feed (ExplorationMission): pin tags as perception finds them,
        # before any bridge scan, so the operator map fills in during exploration.
        self.declare_parameter('discovered_tags_topic', '/perception/discovered_tags')

        # IDW tuning — defaults from the brief; operator-tunable per launch.
        self.declare_parameter('idw_power', DEFAULT_POWER)
        self.declare_parameter('idw_falloff_radius_m', DEFAULT_FALLOFF_RADIUS_M)
        self.declare_parameter('idw_max_distance_m', DEFAULT_MAX_DISTANCE_M)

        self._observations_topic = str(
            self.get_parameter('observations_topic').value
        )
        self._state_topic = str(self.get_parameter('state_topic').value)
        self._field_service_name = str(
            self.get_parameter('field_service_name').value
        )
        self._state_period_s = 1.0 / max(
            0.1, float(self.get_parameter('state_publish_rate_hz').value)
        )
        self._frame_id = str(self.get_parameter('frame_id').value)
        self._buffer_len = int(self.get_parameter('buffer_len').value)
        self._discovered_tags_topic = str(
            self.get_parameter('discovered_tags_topic').value
        )
        self._idw_power = float(self.get_parameter('idw_power').value)
        self._idw_falloff_radius = float(
            self.get_parameter('idw_falloff_radius_m').value
        )
        self._idw_max_distance = float(
            self.get_parameter('idw_max_distance_m').value
        )

        # ── runtime state ──────────────────────────────────────────────
        self._store = TwinStateStore(buffer_len=self._buffer_len)
        # IDW field cache: keyed by (sensor, obs_count, bbox, resolution)
        # → frozen field values. OrderedDict so we can evict the oldest
        # entry when we hit _FIELD_CACHE_CAP. Invalidates lazily on hit:
        # observation_count is part of the key, so a new observation
        # makes the next request miss naturally.
        self._field_cache: OrderedDict[tuple, _CachedField] = OrderedDict()

        # ── ROS interfaces ─────────────────────────────────────────────
        self._cb_group = MutuallyExclusiveCallbackGroup()

        obs_qos = QoSProfile(
            depth=50,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._obs_sub = self.create_subscription(
            Observation,
            self._observations_topic,
            self._on_observation,
            obs_qos,
            callback_group=self._cb_group,
        )
        # Pin tags the instant perception discovers them (latched snapshot,
        # same RELIABLE+TRANSIENT_LOCAL profile as the aggregator's publisher).
        self._discovered_sub = self.create_subscription(
            DiscoveredTags,
            self._discovered_tags_topic,
            self._on_discovered_tags,
            obs_qos,
            callback_group=self._cb_group,
        )

        state_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._state_pub = self.create_publisher(
            TwinState, self._state_topic, state_qos,
        )

        self._field_srv = self.create_service(
            GetField,
            self._field_service_name,
            self._handle_get_field,
            callback_group=self._cb_group,
        )

        self._state_timer = self.create_timer(
            self._state_period_s,
            self._publish_state,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            f'lupin_twin up: obs_topic={self._observations_topic}, '
            f'state_topic={self._state_topic}, '
            f'field_service={self._field_service_name}, '
            f'state_rate={1.0 / self._state_period_s:.1f}Hz, '
            f'idw(power={self._idw_power}, falloff={self._idw_falloff_radius:.2f}m, '
            f'max_dist={self._idw_max_distance:.2f}m)'
        )

    # ─── observation ingestion ─────────────────────────────────────────

    def _on_observation(self, msg: Observation) -> None:
        """Funnel an Observation into the store. Non-OK observations are
        recorded too — the operator wants to see SCAN_FAILED tags listed —
        but only OK observations carry a meaningful pose, so we leave the
        pose unset for the others (the store handles that gracefully)."""
        if msg.kind == Observation.KIND_FLOWER:
            self._ingest_flower(msg)
            return
        if msg.kind != Observation.KIND_TAG_READING:
            # KIND_ANOMALY (standalone anomalies) isn't modelled in the twin
            # yet — the bug flag rides the flower path. Drop it, but loudly the
            # first time so a future producer that wires KIND_ANOMALY upstream
            # sees why its observations vanish instead of debugging a silent
            # dead end (mirrors the frame-mismatch handling below).
            self.get_logger().warn(
                f'Dropping Observation of unmodelled kind {msg.kind} '
                '(twin ingests only KIND_TAG_READING / KIND_FLOWER).',
                throttle_duration_sec=30.0,
            )
            return

        # Frame check — a silently-mismatched frame_id on hardware (the
        # bridge or the orchestrator publishing in /odom by accident) is
        # the kind of bug that costs an afternoon. Log loudly the first
        # time we see one, then drop the message rather than placing the
        # tag at a meaningless map-frame coordinate.
        expected_frame = self._frame_id
        msg_frame = msg.header.frame_id or expected_frame
        if msg_frame != expected_frame:
            self.get_logger().warn(
                f'Dropping Observation in frame {msg_frame!r}; '
                f'twin expects {expected_frame!r}. '
                f'Check orchestrator frame_id parameter.'
            )
            return

        # Field is always present on a ROS message (default-constructed
        # TagReading), so a None check would be dead code; just look up
        # the tag id and rely on the empty-string guard below.
        tag_id = msg.tag_reading.tag_id
        if not tag_id:
            return

        # Pose absence convention: orientation.w == 0 means "missing"
        # (orchestrator zeroes the field for non-OK observations and any
        # pre-localisation slip-throughs). See observations.py docstring.
        pose = msg.tag_pose_in_map
        has_pose = (pose.orientation.w != 0.0) and (
            msg.status == Observation.STATUS_OK
        )

        readings: list[TagSensorEntry] = []
        for r in msg.tag_reading.readings:
            readings.append(TagSensorEntry(name=r.name, value=float(r.value)))

        twin_obs = TwinObservation(
            tag_id=tag_id,
            monotonic_at=self._monotonic_now(),
            pose_x=pose.position.x if has_pose else None,
            pose_y=pose.position.y if has_pose else None,
            pose_qz=pose.orientation.z if has_pose else None,
            pose_qw=pose.orientation.w if has_pose else None,
            readings=readings,
        )
        if self._store.record(twin_obs):
            # Mutation invalidates every IDW cache entry — the observation_count
            # moved, so any future request will miss and re-compute.
            # (We don't need to clear the dict eagerly; stale entries just
            # sit until the cache is dropped on next request that doesn't
            # match. Memory cost is bounded by distinct (sensor, count)
            # pairs we've answered, which is small.)
            pass

    def _ingest_flower(self, msg: Observation) -> None:
        """Record a KIND_FLOWER observation: merge species/anomaly onto the
        co-located tag (keyed by flower.tag_id), pinning it from the flower
        pose if the sensor-reading path hasn't already."""
        flower = msg.flower
        tag_id = flower.tag_id
        if not tag_id:
            return
        # Frame check on the flower's own PoseStamped (falls back to the
        # Observation header, then the expected frame).
        expected = self._frame_id
        msg_frame = flower.pose.header.frame_id or msg.header.frame_id or expected
        if msg_frame != expected:
            self.get_logger().warn(
                f'Dropping flower observation in frame {msg_frame!r}; '
                f'twin expects {expected!r}.'
            )
            return
        pose = flower.pose.pose
        has_pose = pose.orientation.w != 0.0
        flowers = [
            TwinFlower(
                x=float(fp.position.x), y=float(fp.position.y),
                species=fp.species, confidence=float(fp.confidence),
                anomaly=bool(fp.anomaly),
                z=float(fp.position.z), height_m=float(fp.height_m),
            )
            for fp in flower.flowers
        ]
        footprint = [(float(p.x), float(p.y)) for p in flower.box_footprint.points]
        self._store.record_flower(FlowerUpdate(
            tag_id=tag_id,
            monotonic_at=self._monotonic_now(),
            species=flower.species,
            species_confidence=float(flower.confidence),
            anomaly=bool(flower.anomaly),
            flower_count=int(flower.flower_count),
            bug_count=int(flower.bug_count),
            flowers=flowers,
            box_footprint=footprint,
            pose_x=pose.position.x if has_pose else None,
            pose_y=pose.position.y if has_pose else None,
            pose_qz=pose.orientation.z if has_pose else None,
            pose_qw=pose.orientation.w if has_pose else None,
        ))

    def _on_discovered_tags(self, msg: DiscoveredTags) -> None:
        """Pin tags as perception discovers them during exploration — before any
        bridge scan — so the operator watches the map fill in as the robot
        explores. The discovery feed carries each tag's TF-resolved map pose; we
        record a pose-only entry. The later KIND_TAG_READING scan fills the
        sensor readings (record() keeps the pose from whichever arrives first)."""
        expected = self._frame_id
        msg_frame = msg.header.frame_id or expected
        if msg_frame != expected:
            self.get_logger().warn(
                f'Dropping discovered-tags snapshot in frame {msg_frame!r}; '
                f'twin expects {expected!r}.',
                throttle_duration_sec=10.0,
            )
            return
        now = self._monotonic_now()
        for t in msg.tags:
            if not t.tag_id:
                continue
            existing = self._store.tag(t.tag_id)
            if existing is not None and existing.has_pose():
                continue  # already pinned; the scan path owns readings/species
            p = t.pose_in_map
            self._store.record(TwinObservation(
                tag_id=t.tag_id,
                monotonic_at=now,
                pose_x=p.position.x,
                pose_y=p.position.y,
                pose_qz=p.orientation.z,
                pose_qw=p.orientation.w if p.orientation.w != 0.0 else 1.0,
                readings=[],
            ))

    # ─── periodic state publish ────────────────────────────────────────

    def _publish_state(self) -> None:
        msg = TwinState()
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self._frame_id
        msg.header = header

        now_mono = self._monotonic_now()
        out: list[TwinTagState] = []
        for buf in self._store.all_tags():
            entry = TwinTagState()
            entry.tag_id = buf.tag_id

            if buf.has_pose():
                entry.pose.position.x = float(buf.pose_x)  # type: ignore[arg-type]
                entry.pose.position.y = float(buf.pose_y)  # type: ignore[arg-type]
                entry.pose.position.z = 0.0
                entry.pose.orientation.z = float(buf.pose_qz or 0.0)
                entry.pose.orientation.w = float(buf.pose_qw or 1.0)
            else:
                # Leave default zero-Pose; orientation.w == 0 signals
                # "no pose" to the HMI consumer.
                entry.pose.orientation.w = 0.0

            for name, value in buf.latest_readings.items():
                r = SensorReading()
                r.name = name
                r.value = float(value)
                entry.readings.append(r)

            # Flower classification co-located with this tag (empty/false
            # until perception has classified one).
            entry.species = buf.species
            entry.species_confidence = float(buf.species_confidence)
            entry.anomaly = bool(buf.anomaly)
            entry.flower_count = min(int(buf.flower_count), 65535)
            entry.bug_count = min(int(buf.bug_count), 65535)

            # Localized blooms + box footprint (v2 redesign).
            for fl in buf.flowers:
                fp = FlowerPoint()
                fp.position = Point(x=float(fl.x), y=float(fl.y), z=float(fl.z))
                fp.height_m = float(fl.height_m)
                fp.species = fl.species
                fp.confidence = float(fl.confidence)
                fp.anomaly = bool(fl.anomaly)
                entry.flowers.append(fp)
            if buf.box_footprint:
                poly = Polygon()
                for (x, y) in buf.box_footprint:
                    poly.points.append(Point32(x=float(x), y=float(y), z=0.0))
                entry.box_footprint = poly

            # Absolute observation timestamp — durable, survives serialisation
            # to disk, and is what off-line consumers (FloraNova export,
            # anomaly detection) should treat as the source of truth. Build
            # a builtin_interfaces/Time from the cached monotonic seconds;
            # fractional → nanosec.
            t_sec = max(0.0, buf.last_seen_monotonic)
            entry.last_observed = TimeMsg()
            entry.last_observed.sec = int(t_sec)
            entry.last_observed.nanosec = int((t_sec - int(t_sec)) * 1e9)
            entry.stale_seconds = float(max(0.0, now_mono - buf.last_seen_monotonic))
            out.append(entry)

        msg.tags = out
        self._state_pub.publish(msg)

    # ─── /twin/get_field ───────────────────────────────────────────────

    def _handle_get_field(
        self, request: GetField.Request, response: GetField.Response,
    ) -> GetField.Response:
        sensor = (request.sensor_type or '').strip()
        if not sensor:
            return self._field_error(response, 'sensor_type is required')

        if request.resolution <= 0:
            return self._field_error(
                response, f'resolution must be positive, got {request.resolution}',
            )
        if (request.bbox_max_x < request.bbox_min_x or
                request.bbox_max_y < request.bbox_min_y):
            return self._field_error(response, 'bbox max must be ≥ min on both axes')

        cache_key = (
            sensor,
            self._store.observation_count,
            float(request.resolution),
            float(request.bbox_min_x), float(request.bbox_min_y),
            float(request.bbox_max_x), float(request.bbox_max_y),
        )
        cached = self._field_cache.get(cache_key)
        if cached is not None:
            # Touch — promote to most-recently-used so the LRU eviction
            # below targets the actual cold entries.
            self._field_cache.move_to_end(cache_key)
            return self._fill_field_response(response, cached)

        # Sample list: every tag with a pose AND a finite reading for
        # this sensor. samples_for_sensor() handles the filtering, including
        # rejecting NaN/inf values that would otherwise poison the IDW math.
        sample_tuples = self._store.samples_for_sensor(sensor)
        samples = [FieldSample(x=x, y=y, value=v) for (x, y, v) in sample_tuples]

        try:
            grid = compute_idw_field(
                samples,
                bbox_min_x=float(request.bbox_min_x),
                bbox_min_y=float(request.bbox_min_y),
                bbox_max_x=float(request.bbox_max_x),
                bbox_max_y=float(request.bbox_max_y),
                resolution=float(request.resolution),
                falloff_radius_m=self._idw_falloff_radius,
                max_distance_m=self._idw_max_distance,
                power=self._idw_power,
                explored_mask=None,  # /map gating is a follow-up
            )
        except ValueError as exc:
            return self._field_error(response, f'idw error: {exc}')

        # Freeze the grid into a cache entry. Tuple of values keeps it
        # immutable — multiple cache hits all hand back fresh copies on
        # the way out via _fill_field_response, never aliasing rclpy's
        # per-call response object.
        cached_entry = _CachedField(
            values=tuple(float(v) for v in grid.values),
            width=int(grid.width),
            height=int(grid.height),
            origin_x=float(grid.origin_x),
            origin_y=float(grid.origin_y),
            resolution_used=float(grid.resolution),
            value_min=float(grid.value_min),
            value_max=float(grid.value_max),
            sample_count=int(grid.sample_count),
        )
        self._field_cache[cache_key] = cached_entry
        # LRU eviction: drop the oldest entry once over cap. OrderedDict
        # `popitem(last=False)` is the textbook LRU-trim primitive.
        while len(self._field_cache) > _FIELD_CACHE_CAP:
            self._field_cache.popitem(last=False)
        return self._fill_field_response(response, cached_entry)

    @staticmethod
    def _fill_field_response(
        response: GetField.Response, cached: _CachedField,
    ) -> GetField.Response:
        """Copy a cached field into the per-call rclpy response."""
        response.ok = True
        response.error_message = ''
        # ROS float32[] accepts a Python list including NaN.
        response.values = list(cached.values)
        response.width = cached.width
        response.height = cached.height
        response.origin_x = cached.origin_x
        response.origin_y = cached.origin_y
        response.resolution_used = cached.resolution_used
        response.value_min = cached.value_min
        response.value_max = cached.value_max
        response.sample_count = cached.sample_count
        return response

    # ─── helpers ───────────────────────────────────────────────────────

    @staticmethod
    def _field_error(
        response: GetField.Response, message: str,
    ) -> GetField.Response:
        response.ok = False
        response.error_message = message
        response.values = []
        response.width = 0
        response.height = 0
        response.origin_x = 0.0
        response.origin_y = 0.0
        response.resolution_used = 0.0
        response.value_min = 0.0
        response.value_max = 0.0
        response.sample_count = 0
        return response

    def _monotonic_now(self) -> float:
        # Use ROS time (sim time when /clock is up) so playback works.
        return self.get_clock().now().nanoseconds * 1e-9


def main(argv: Optional[list[str]] = None) -> None:
    rclpy.init(args=argv)
    node = TwinNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        finally:
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == '__main__':
    main()
