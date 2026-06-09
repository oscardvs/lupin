"""Perception aggregator — turn raw detections into located, typed flowers.

The two detector nodes (`tag_annotator` ArUco + `yolo_detector` tulip/bug
classifier) each emit low-level, 2-D, un-correlated output. This node fuses
them into the things the mission, twin, and HMI actually need:

* **Discovery feed** — for every AprilTag the detector sees, look the
  ``tag_<id>`` TF up into the ``map`` frame (the detector broadcasts it in the
  camera optical frame; SLAM + the URDF chain provide the rest) and keep a
  de-duplicated, debounced registry of discovered tags with their map pose.
  Published latched on ``/perception/discovered_tags`` so the orchestrator —
  which only subscribes after a mission starts — sees the whole set at once.

* **Flower classification** — co-locate the YOLO flower class with the current
  mission target/base. Association is *temporal*: while the robot is parked
  SCANNING a target (per ``/mission/state.current_target``), the dominant tulip
  class seen on the gripper cam is attributed to that target, and the ``bug``
  class raises the anomaly flag. Emitted as ``KIND_FLOWER`` Observations on
  ``/floranova/observations`` (the twin's existing intake) and folded into the
  discovery feed. Flower map points are estimated from OpenCV/map-derived box
  geometry plus arm pan and YOLO bbox lateral position.

* **/perception/confirm_tag** — the visual-confirmation gate the orchestrator
  already calls (``ConfirmTag.srv``): answers "is ``expected_tag_id`` in view"
  from the registry's freshness.

v1 limitation (documented): flower↔tag association is temporal co-location,
not pixel/3-D overlap — it trusts that what the gripper cam sees while parked
at tag X belongs to tag X. Refine later by aiming the arm at the table.

Compute note: YOLO (torch) is heavy and ``pip install ultralytics`` breaks ROS
numpy/cv2 — run this node and ``yolo_detector`` laptop/Jetson-side, not on the
Orange Pi. This node itself is light (no torch); only its YOLO *input* is.
"""

from __future__ import annotations

import json
import math
import time
from collections import deque
from typing import Optional

import rclpy
from geometry_msgs.msg import Point, Point32, Polygon, Pose
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, String
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)

from lupin_msgs.msg import (
    DiscoveredTag,
    DiscoveredTags,
    FlowerObservation,
    FlowerPoint,
    MissionState,
    Observation,
)
from lupin_msgs.srv import ConfirmTag
from .box_geometry import (
    FlowerPointData,
    MapBox,
    StandardBox,
    bin_detections,
    box_from_map_box,
    box_from_tag,
    lateral_fraction,
)


# Lifecycle/phase strings (from MissionState) during which a flower seen on the
# gripper cam should be attributed to the mission's current_target tag.
_SCANNING_PHASE = 'SCANNING'
_SCANNING_LIFECYCLES = ('MONITORING', 'INSPECTING')


class _TagRecord:
    """In-memory state for one discovered tag. Pure data, no ROS."""

    __slots__ = (
        'tag_id', 'pose', 'best_dist', 'sightings',
        'last_seen_mono', 'last_seen_ros',
        'species', 'species_confidence', 'anomaly', 'flower_dirty',
        'flower_count', 'bug_count',
    )

    def __init__(self, tag_id: str) -> None:
        self.tag_id = tag_id
        self.pose: Optional[Pose] = None
        self.best_dist = math.inf       # keep the pose from the closest view
        self.sightings = 0
        self.last_seen_mono = 0.0
        self.last_seen_ros = Time().to_msg()
        self.species = ''
        self.species_confidence = 0.0
        self.anomaly = False
        self.flower_dirty = False       # flower fields changed since last emit
        self.flower_count = 0           # blooms placed at the last emit
        self.bug_count = 0              # bug tracks/detections at the last emit


class PerceptionAggregator(Node):
    """Fuse tag + YOLO detections into discovered-tag + flower observations."""

    def __init__(self) -> None:
        super().__init__('perception_aggregator')

        # ── Parameters ──────────────────────────────────────────────────
        self.declare_parameter('tag_detections_topic', '/camera/tag_detections_json')
        self.declare_parameter('yolo_detections_topic', '/yolo/detections')
        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('box_geometry_topic', '/perception/box_geometry_json')
        self.declare_parameter('discovered_tags_topic', '/perception/discovered_tags')
        self.declare_parameter('observations_topic', '/floranova/observations')
        self.declare_parameter('confirm_service', '/perception/confirm_tag')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tf_frame_prefix', 'tag_')
        # A tag must be seen this many times (with a successful map lookup)
        # before it counts as discovered — debounces single noisy frames.
        self.declare_parameter('min_sightings', 3)
        # Ignore tag detections farther than this for pose commit (solvePnP
        # error grows with range, especially for a 4 cm tag). 0 = no limit.
        self.declare_parameter('max_tag_distance_m', 2.5)
        # A registry entry is "fresh" (confirmable / live) within this window.
        self.declare_parameter('freshness_s', 5.0)
        # YOLO detections older than this are dropped from the fusion window.
        self.declare_parameter('yolo_window_s', 2.0)
        # Minimum YOLO confidence to consider a flower/anomaly detection.
        self.declare_parameter('yolo_min_confidence', 0.35)
        # class index → name. Default matches best.pt: 0..2 tulips + 3 bug.
        self.declare_parameter(
            'flower_class_names',
            ['tulip_red', 'tulip_white', 'tulip_pink', 'bug'],
        )
        self.declare_parameter('anomaly_class_name', 'bug')
        self.declare_parameter('publish_rate_hz', 2.0)

        # ── flower localization (Stream B) ──────────────────────────────
        self.declare_parameter('joint_states_topic', '/joint_states')
        self.declare_parameter('pan_joint_name', 'shoulder_pan_joint')
        # Gripper-cam width (px) — only used to normalize bbox centre-x to
        # [-1, 1]. 640 matches the sim Gazebo camera; tune if the real cam
        # differs (affects lateral spread magnitude, not correctness).
        self.declare_parameter('detection_image_width', 640.0)
        self.declare_parameter('lateral_columns', 7)
        self.declare_parameter('box_width', 0.80)
        self.declare_parameter('box_depth', 0.40)
        self.declare_parameter('box_height', 0.30)
        self.declare_parameter('tag_mount_height', 0.10)
        self.declare_parameter('tag_lateral_offset', 0.0)
        self.declare_parameter('flower_base_depth_frac', 0.5)
        self.declare_parameter('flower_depth_jitter_frac', 0.18)
        # Half the arm pan-sweep amplitude (rad); matches Stream C's sweep so
        # pan maps cleanly to lateral position once the sweep lands.
        self.declare_parameter('pan_half_span', 0.5)
        self.declare_parameter('camera_half_fov', 0.5)
        # Spatial box-membership gate: drop YOLO blooms whose projected map
        # position falls outside current_target's REGISTERED planter rectangle
        # (a MapBox from the box_layout_publisher), so blooms from an adjacent
        # bench / the over-reaching pan sweep are not misattributed. Default ON
        # (hardware); a no-op in sim where blooms place in-bench. Engages only
        # when a registered MapBox exists for the target — never the legacy
        # tag-anchored fallback — so the temporal path is untouched. See
        # docs/flower_tag_association_audit_2026-06-09.md.
        self.declare_parameter('flower_box_gate', True)
        # Metric slack on the rectangle so nominal-FOV error doesn't drop
        # genuine edge-of-bench blooms.
        self.declare_parameter('flower_box_gate_margin_m', 0.10)

        self._map_frame = str(self.get_parameter('map_frame').value)
        self._tf_prefix = str(self.get_parameter('tf_frame_prefix').value)
        self._min_sightings = int(self.get_parameter('min_sightings').value)
        self._max_dist = float(self.get_parameter('max_tag_distance_m').value)
        self._freshness_s = float(self.get_parameter('freshness_s').value)
        self._yolo_window_s = float(self.get_parameter('yolo_window_s').value)
        self._yolo_min_conf = float(self.get_parameter('yolo_min_confidence').value)
        self._class_names = [str(n) for n in self.get_parameter('flower_class_names').value]
        self._anomaly_name = str(self.get_parameter('anomaly_class_name').value)
        publish_rate = float(self.get_parameter('publish_rate_hz').value)
        self._pan_joint = str(self.get_parameter('pan_joint_name').value)
        self._image_width = max(1.0, float(self.get_parameter('detection_image_width').value))
        self._lateral_columns = max(1, int(self.get_parameter('lateral_columns').value))
        self._box = StandardBox(
            width=float(self.get_parameter('box_width').value),
            depth=float(self.get_parameter('box_depth').value),
            height=float(self.get_parameter('box_height').value),
            tag_mount_height=float(self.get_parameter('tag_mount_height').value),
            tag_lateral_offset=float(self.get_parameter('tag_lateral_offset').value),
        )
        self._flower_base_depth = float(self.get_parameter('flower_base_depth_frac').value)
        self._flower_depth_jitter = float(self.get_parameter('flower_depth_jitter_frac').value)
        self._pan_half_span = float(self.get_parameter('pan_half_span').value)
        self._camera_half_fov = float(self.get_parameter('camera_half_fov').value)
        self._flower_box_gate = bool(self.get_parameter('flower_box_gate').value)
        self._flower_box_gate_margin = float(
            self.get_parameter('flower_box_gate_margin_m').value)

        # ── runtime state ──────────────────────────────────────────────
        self._registry: dict[str, _TagRecord] = {}
        # Map/OpenCV-derived box geometry keyed by mission target/base id.
        self._boxes: dict[str, MapBox] = {}
        # (recv_monotonic, class_name, confidence) within the YOLO window.
        self._yolo_window: deque[tuple[float, str, float]] = deque(maxlen=256)
        # ids + dists from the most recent tag-detection frame (nearest-tag
        # fallback when mission state isn't available).
        self._last_frame: list[tuple[str, float]] = []
        # Latest mission state, for the SCANNING-phase association gate.
        self._mission_state: Optional[MissionState] = None
        self._mission_state_seen = False
        # Latest arm pan angle (rad) — the lateral cue for bloom placement.
        self._shoulder_pan = 0.0
        # Per-scan detection accumulator for the tag currently being scanned:
        # (lateral_fraction f, class_name, confidence, track_id). Reset when
        # the focus tag changes so one box's blooms don't bleed into the next.
        self._scan_tag: Optional[str] = None
        # Bounded so a long dwell / stuck current_target can't grow it without
        # limit (mirrors the _yolo_window cap); a real scan holds far fewer.
        self._scan_dets: deque = deque(maxlen=512)

        # ── TF ──────────────────────────────────────────────────────────
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── QoS ───────────────────────────────────────────────────────
        # Latched snapshot, like /twin/state and /mission/state, so a late
        # orchestrator subscriber gets the current registry immediately.
        latched = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Match the orchestrator/twin observation QoS (depth 50, transient).
        obs_qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        det_qos = QoSProfile(
            depth=5,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        # ── ROS interfaces ──────────────────────────────────────────────
        self._discovered_pub = self.create_publisher(
            DiscoveredTags,
            str(self.get_parameter('discovered_tags_topic').value),
            latched,
        )
        self._obs_pub = self.create_publisher(
            Observation,
            str(self.get_parameter('observations_topic').value),
            obs_qos,
        )
        self.create_subscription(
            String, str(self.get_parameter('tag_detections_topic').value),
            self._on_tag_detections, det_qos,
        )
        self.create_subscription(
            String, str(self.get_parameter('yolo_detections_topic').value),
            self._on_yolo_detections, det_qos,
        )
        self.create_subscription(
            String, str(self.get_parameter('box_geometry_topic').value),
            self._on_box_geometry, det_qos,
        )
        self.create_subscription(
            MissionState, str(self.get_parameter('mission_state_topic').value),
            self._on_mission_state, latched,
        )
        self.create_subscription(
            JointState, str(self.get_parameter('joint_states_topic').value),
            self._on_joint_states, det_qos,
        )
        self._confirm_srv = self.create_service(
            ConfirmTag, str(self.get_parameter('confirm_service').value),
            self._handle_confirm_tag,
        )
        self._publish_timer = self.create_timer(
            1.0 / max(publish_rate, 0.2), self._publish_discovered,
        )

        self.get_logger().info(
            f'perception_aggregator online — map_frame="{self._map_frame}", '
            f'min_sightings={self._min_sightings}, '
            f'classes={self._class_names} (anomaly="{self._anomaly_name}")'
        )

    # ─── time helper ───────────────────────────────────────────────────
    @staticmethod
    def _monotonic() -> float:
        return time.monotonic()

    # ─── tag detections → discovery registry ────────────────────────────
    def _on_tag_detections(self, msg: String) -> None:
        try:
            detections = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn('tag_detections_json not parseable',
                                   throttle_duration_sec=10.0)
            return
        if not isinstance(detections, list):
            return

        now_mono = self._monotonic()
        frame: list[tuple[str, float]] = []
        localized = 0
        for det in detections:
            try:
                tag_id = str(int(det['id']))
                dist = float(det.get('dist', 0.0))
            except (KeyError, TypeError, ValueError):
                continue
            frame.append((tag_id, dist))

            # Pose comes from the map-frame TF lookup, not the JSON (JSON dist
            # is camera-frame z only). Skip far detections for pose quality.
            if self._max_dist > 0.0 and dist > 0.0 and dist > self._max_dist:
                continue
            pose = self._lookup_tag_pose(tag_id)
            if pose is None:
                continue
            self._record_tag(tag_id, pose, dist, now_mono)
            localized += 1
        # Silent demo-killer guard: tags are being seen but none can be put on
        # the map (missing camera intrinsics / map->camera TF), so
        # /perception/discovered_tags stays empty and exploration never reaches
        # its goal — surface the cause instead of an opaque exploration_timeout.
        if frame and localized == 0:
            self.get_logger().warn(
                'AprilTag(s) detected but none map-localisable (no TF) — check '
                'camera_info/intrinsics and the map->camera TF; discovered_tags '
                'will stay empty.',
                throttle_duration_sec=5.0,
            )
        self._last_frame = frame

    def _on_joint_states(self, msg: JointState) -> None:
        """Track the arm pan angle — the lateral cue for bloom placement."""
        try:
            i = list(msg.name).index(self._pan_joint)
        except ValueError:
            return
        if i < len(msg.position):
            self._shoulder_pan = float(msg.position[i])

    def _lookup_tag_pose(self, tag_id: str) -> Optional[Pose]:
        """Look up the tag's pose in the map frame, or None if unavailable.

        Uses a zero-timeout lookup against the already-buffered TF — a blocking
        timeout would deadlock the single-threaded executor that also fills the
        buffer. The detector re-broadcasts the tag TF every frame, so "latest"
        is current to within a frame.
        """
        child = f'{self._tf_prefix}{tag_id}'
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, child, Time(), timeout=Duration(seconds=0.0),
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = t.x, t.y, t.z
        pose.orientation = r
        return pose

    def _record_tag(self, tag_id: str, pose: Pose, dist: float,
                    now_mono: float) -> None:
        rec = self._registry.get(tag_id)
        if rec is None:
            rec = _TagRecord(tag_id)
            self._registry[tag_id] = rec
            self.get_logger().info(f'New tag candidate {tag_id} (map-localised).')
        rec.sightings += 1
        rec.last_seen_mono = now_mono
        rec.last_seen_ros = self.get_clock().now().to_msg()
        # Keep the pose from the closest (best) view we've had.
        d = dist if dist > 0.0 else 0.0
        if rec.pose is None or d < rec.best_dist:
            rec.pose = pose
            rec.best_dist = d if d > 0.0 else rec.best_dist
        if rec.sightings == self._min_sightings:
            self.get_logger().info(
                f'Tag {tag_id} DISCOVERED ({rec.sightings} sightings).'
            )

    # ─── YOLO detections → flower fusion ────────────────────────────────
    def _on_box_geometry(self, msg: String) -> None:
        """Receive map-derived planter boxes from an OpenCV/map node.

        Accepted JSON forms:
          [{"id": "5", "x": 1.2, "y": 3.4, "yaw": 0.0,
            "width": 0.8, "depth": 0.4, "height": 0.3}, ...]
        or {"boxes": [...]}.
        """
        try:
            payload = json.loads(msg.data)
        except (ValueError, TypeError):
            self.get_logger().warn('box_geometry_json not parseable',
                                   throttle_duration_sec=10.0)
            return
        boxes = payload.get('boxes') if isinstance(payload, dict) else payload
        if not isinstance(boxes, list):
            return
        parsed: dict[str, MapBox] = {}
        for item in boxes:
            if not isinstance(item, dict):
                continue
            try:
                box_id = str(item.get('id', item.get('box_id')))
                if not box_id or box_id == 'None':
                    continue
                parsed[box_id] = MapBox(
                    box_id=box_id,
                    x=float(item['x']),
                    y=float(item['y']),
                    yaw=float(item.get('yaw', 0.0)),
                    width=float(item.get('width', self._box.width)),
                    depth=float(item.get('depth', self._box.depth)),
                    height=float(item.get('height', self._box.height)),
                )
            except (KeyError, TypeError, ValueError):
                continue
        if parsed:
            self._boxes.update(parsed)

    def _on_yolo_detections(self, msg: String) -> None:
        try:
            detections = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if not isinstance(detections, list):
            return
        now_mono = self._monotonic()
        frame_dets: list[tuple[float, str, float, Optional[int]]] = []
        for det in detections:
            try:
                cls = int(det['class'])
                conf = float(det['confidence'])
            except (KeyError, TypeError, ValueError):
                continue
            if conf < self._yolo_min_conf:
                continue
            name_from_msg = det.get('class_name')
            if isinstance(name_from_msg, str) and name_from_msg:
                name = name_from_msg
            elif 0 <= cls < len(self._class_names):
                name = self._class_names[cls]
            else:
                continue
            self._yolo_window.append((now_mono, name, conf))
            track_id = None
            if det.get('track_id') is not None:
                try:
                    track_id = int(det['track_id'])
                except (TypeError, ValueError):
                    track_id = None
            # Lateral cue: normalize the bbox centre-x to [-1, 1], combine
            # with the arm pan angle (box_geometry.lateral_fraction).
            bbox = det.get('bbox_xyxy')
            bbox_cx_norm = 0.0
            if isinstance(bbox, (list, tuple)) and len(bbox) == 4:
                try:
                    cx = (float(bbox[0]) + float(bbox[2])) / 2.0
                    bbox_cx_norm = max(-1.0, min(1.0, (cx / self._image_width - 0.5) * 2.0))
                except (TypeError, ValueError):
                    bbox_cx_norm = 0.0
            # Store the UNCLAMPED fraction so the spatial gate can tell a bloom
            # that points past the bench end (|f| > 1) from one merely at the
            # edge; bin_detections re-clamps for placement, so this never moves
            # an in-box bloom.
            f = lateral_fraction(
                self._shoulder_pan, bbox_cx_norm,
                pan_half_span=self._pan_half_span,
                camera_half_fov=self._camera_half_fov,
                clamp=False,
            )
            frame_dets.append((f, name, conf, track_id))
        self._fuse_flower(now_mono, frame_dets)

    def _focus_tag(self) -> Optional[str]:
        """Which mission target/base a fresh flower detection belongs to.

        Primary: the target the mission says it is parked SCANNING. Fallback
        (standalone / no mission): the nearest tag in the latest frame. Returns
        None when we shouldn't attribute (e.g. EXPLORING / driving)."""
        ms = self._mission_state
        if self._mission_state_seen and ms is not None:
            scanning = (
                ms.lifecycle_state in _SCANNING_LIFECYCLES
                and _SCANNING_PHASE in (ms.mission_phase or '')
            )
            if scanning and (ms.current_target in self._boxes
                             or ms.current_target in self._registry):
                return ms.current_target
            # Mission is running but not scanning — don't attribute (avoids
            # drive-by misassociation during EXPLORING).
            return None
        # Standalone: nearest detected tag this frame, if any.
        if not self._last_frame:
            return None
        nearest = min(self._last_frame, key=lambda t: t[1] if t[1] > 0 else math.inf)
        return nearest[0] if nearest[0] in self._registry else None

    def _fuse_flower(self, now_mono: float,
                     frame_dets: list[tuple[float, str, float, Optional[int]]]) -> None:
        # Prune the time window used for the dominant-species summary.
        while self._yolo_window and now_mono - self._yolo_window[0][0] > self._yolo_window_s:
            self._yolo_window.popleft()
        tag_id = self._focus_tag()
        if tag_id is None:
            return
        rec = self._registry.get(tag_id)
        box = self._boxes.get(tag_id)
        if rec is None and box is None:
            return
        if rec is None:
            rec = _TagRecord(tag_id)
            self._registry[tag_id] = rec

        # Per-scan accumulation: reset when the scanned tag changes so one
        # box's blooms never bleed into the next.
        if tag_id != self._scan_tag:
            self._scan_tag = tag_id
            self._scan_dets.clear()
        self._scan_dets.extend(frame_dets)

        # Geometry for this tag: the registered planter rectangle (a MapBox from
        # the box_layout_publisher) or the legacy tag-anchored box. The spatial
        # membership gate runs ONLY against a registered MapBox — never the
        # legacy single-quaternion fallback — so blooms pointing off the bench
        # are dropped on hardware while the temporal path is untouched wherever
        # no registered box exists yet.
        geom = self._box_geometry_for(tag_id, rec)
        if self._flower_box_gate and box is not None and geom is not None:
            dets = self._gate_detections(geom, self._scan_dets)
        else:
            dets = list(self._scan_dets)

        # Dominant tulip species (best confidence) + bug flag for the summary.
        # Derive from the per-scan, tag-scoped accumulator (reset on tag change)
        # rather than the shared time window, so a 'bug' or species seen at the
        # previous pot can't bleed into this pot's summary within yolo_window_s.
        best_species, best_conf = '', 0.0
        anomaly = False
        for _, name, conf, _ in dets:
            if name == self._anomaly_name:
                anomaly = True
                continue
            if conf > best_conf:
                best_species, best_conf = name, conf

        flower_tracks = {
            track_id for _, name, _, track_id in dets
            if name != self._anomaly_name and track_id is not None
        }
        bug_tracks = {
            track_id for _, name, _, track_id in dets
            if name == self._anomaly_name and track_id is not None
        }
        # Locate the blooms inside the box from the (gated) detections.
        flowers = bin_detections(
            [(f, name, conf) for f, name, conf, _ in dets], geom,
            lateral_columns=self._lateral_columns,
            base_depth_frac=self._flower_base_depth,
            depth_jitter_frac=self._flower_depth_jitter,
            anomaly_class=self._anomaly_name,
        )
        flower_count = len(flower_tracks) if flower_tracks else len([
            fp for fp in flowers if fp.species
        ])
        bug_count = len(bug_tracks) if bug_tracks else len([
            fp for fp in flowers if fp.anomaly
        ])

        # Re-emit on a summary change or a change in the column COUNT.
        # Position-only shifts (same count) are intentionally not a trigger —
        # the twin is latest-wins and the next summary/count update carries them.
        changed = (
            rec.species != best_species
            or rec.anomaly != anomaly
            or abs(rec.species_confidence - best_conf) > 0.1
            or rec.flower_count != flower_count
            or rec.bug_count != bug_count
        )
        rec.species = best_species
        rec.species_confidence = best_conf
        rec.anomaly = anomaly
        rec.flower_count = flower_count
        rec.bug_count = bug_count
        if changed:
            rec.flower_dirty = True
            self._emit_flower_observation(tag_id, rec, box, flowers, geom)

    def _box_geometry_for(self, target_id: str,
                          rec: Optional[_TagRecord]) -> Optional[object]:
        box = self._boxes.get(target_id)
        if box is not None:
            return box_from_map_box(box)
        if rec is not None and rec.pose is not None:
            return box_from_tag(rec.pose, self._box)
        return None

    def _gate_detections(self, geom, dets):
        """Drop per-scan detections whose projected bloom falls outside ``geom``
        (the registered bench rectangle, inflated by ``flower_box_gate_margin_m``).

        Uses the UNCLAMPED lateral fraction and the base scan depth, so a bearing
        that points past the bench end is rejected rather than clamped onto the
        current bench's edge column — the core of the misattribution fix. Each
        ``det`` is ``(f, name, conf, track_id)``; the whole tuple is kept."""
        margin = self._flower_box_gate_margin
        kept = []
        for det in dets:
            x, y = geom.place(det[0], self._flower_base_depth)
            if geom.contains(x, y, margin_m=margin):
                kept.append(det)
        return kept

    def _emit_flower_observation(self, target_id: str, rec: Optional[_TagRecord],
                                 box: Optional[MapBox], flowers, geom) -> None:
        if rec is None and box is None:
            return
        est_z, est_height = self._estimated_flower_height(rec, box)
        stamp = self.get_clock().now().to_msg()
        obs = Observation()
        obs.header = Header(stamp=stamp, frame_id=self._map_frame)
        obs.mission_id = (
            self._mission_state.mission_id if self._mission_state else ''
        )
        obs.source = 'perception_aggregator'
        obs.kind = Observation.KIND_FLOWER
        obs.status = Observation.STATUS_OK
        flower = FlowerObservation()
        flower.tag_id = target_id
        flower.pose.header = Header(stamp=stamp, frame_id=self._map_frame)
        flower.pose.pose = self._anchor_pose(target_id, rec, box)
        species = rec.species if rec is not None else ''
        confidence = rec.species_confidence if rec is not None else 0.0
        anomaly = rec.anomaly if rec is not None else False
        flower.species = species
        flower.confidence = confidence
        flower.anomaly = anomaly
        flower.flower_count = min(int(rec.flower_count if rec is not None else 0), 65535)
        flower.bug_count = min(int(rec.bug_count if rec is not None else 0), 65535)

        # Located blooms. Degenerate tag normal (geom is None) -> fall back to
        # a single point at the tag so a classified OR pest-flagged tag still
        # shows a dot (previously a bug-only degenerate tag emitted no point).
        placed = list(flowers)
        if not placed and (species or anomaly):
            anchor = flower.pose.pose
            placed = [FlowerPointData(
                x=anchor.position.x, y=anchor.position.y,
                species=species, confidence=confidence,
                anomaly=anomaly,
            )]
        for fp in placed:
            point = FlowerPoint()
            point.position = Point(
                x=float(fp.x), y=float(fp.y), z=float(fp.z or est_z),
            )
            point.height_m = float(fp.height_m or est_height)
            point.species = fp.species
            point.confidence = float(fp.confidence)
            point.anomaly = bool(fp.anomaly)
            flower.flowers.append(point)

        if geom is not None:
            poly = Polygon()
            for (x, y) in geom.footprint():
                poly.points.append(Point32(x=float(x), y=float(y), z=0.0))
            flower.box_footprint = poly

        obs.flower = flower
        obs.tag_pose_in_map = flower.pose.pose
        self._obs_pub.publish(obs)
        if rec is not None:
            rec.flower_dirty = False

    def _anchor_pose(self, target_id: str, rec: Optional[_TagRecord],
                     box: Optional[MapBox]) -> Pose:
        if box is not None:
            pose = Pose()
            pose.position.x = float(box.x)
            pose.position.y = float(box.y)
            pose.position.z = max(0.0, float(box.height))
            pose.orientation.z = math.sin(float(box.yaw) * 0.5)
            pose.orientation.w = math.cos(float(box.yaw) * 0.5)
            return pose
        if rec is not None and rec.pose is not None:
            return rec.pose
        pose = Pose()
        pose.orientation.w = 1.0
        self.get_logger().warn(
            f'No anchor pose for flower target {target_id}',
            throttle_duration_sec=5.0,
        )
        return pose

    def _estimated_flower_height(
        self, rec: Optional[_TagRecord], box: Optional[MapBox],
    ) -> tuple[float, float]:
        """Approximate bloom z and height from the known planter geometry.

        If a map-derived box is available, its height is the estimate.
        Otherwise, tag mount height approximates the tag's offset above planter
        base and box_height is the expected vertical bloom/plant extent.
        """
        if box is not None:
            height = max(0.0, float(box.height))
            return height, height
        if rec is None or rec.pose is None:
            return max(0.0, float(self._box.height)), max(0.0, float(self._box.height))
        base_z = float(rec.pose.position.z) - float(self._box.tag_mount_height)
        height = max(0.0, float(self._box.height))
        return max(0.0, base_z + height), height

    # ─── mission state ──────────────────────────────────────────────────
    def _on_mission_state(self, msg: MissionState) -> None:
        self._mission_state = msg
        self._mission_state_seen = True

    # ─── confirm_tag service ────────────────────────────────────────────
    def _handle_confirm_tag(self, request: ConfirmTag.Request,
                            response: ConfirmTag.Response) -> ConfirmTag.Response:
        rec = self._registry.get(str(request.expected_tag_id))
        now_mono = self._monotonic()
        if (rec is not None and rec.pose is not None
                and rec.sightings >= self._min_sightings
                and now_mono - rec.last_seen_mono <= self._freshness_s):
            response.detected = True
            response.detection_confidence = min(
                1.0, rec.sightings / float(max(self._min_sightings, 1)),
            )
            response.tag_pose_in_map = rec.pose
            response.error_message = ''
        else:
            response.detected = False
            response.detection_confidence = 0.0
            if rec is None:
                response.error_message = f'tag {request.expected_tag_id} not discovered'
            elif rec.sightings < self._min_sightings:
                response.error_message = (
                    f'tag {request.expected_tag_id} below sighting threshold'
                )
            else:
                response.error_message = f'tag {request.expected_tag_id} stale'
        return response

    # ─── discovery publish ──────────────────────────────────────────────
    def _publish_discovered(self) -> None:
        msg = DiscoveredTags()
        msg.header = Header(stamp=self.get_clock().now().to_msg(),
                            frame_id=self._map_frame)
        for rec in self._registry.values():
            if rec.sightings < self._min_sightings or rec.pose is None:
                continue
            tag = DiscoveredTag()
            tag.tag_id = rec.tag_id
            tag.pose_in_map = rec.pose
            tag.species = rec.species
            tag.species_confidence = rec.species_confidence
            tag.anomaly = rec.anomaly
            tag.sightings = rec.sightings
            msg.tags.append(tag)
        self._discovered_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
