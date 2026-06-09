"""Per-tag observation buffer + materialise as TwinTagState.

Pure Python — separated from the node so the buffer logic is unit-testable
without rclpy. The node owns one :class:`TwinStateStore` instance and
dispatches incoming Observation messages into it.
"""

from __future__ import annotations

import math
from collections import OrderedDict, deque
from dataclasses import dataclass, field
from typing import Optional


# Ring-buffer size per tag. 32 is plenty for the brief's "last few readings"
# without growing memory unboundedly on a long patrol; the heat-map only
# uses the most recent reading per tag anyway.
DEFAULT_BUFFER_LEN = 32


@dataclass
class TagSensorEntry:
    """One sensor's most recent reading at a tag."""
    name: str
    value: float


@dataclass(frozen=True)
class TwinFlower:
    """One localized bloom inside a tag's planter box (map frame). Pure data
    so the store stays ROS-free; the node wraps it in a lupin_msgs/FlowerPoint."""
    x: float
    y: float
    species: str
    confidence: float
    anomaly: bool
    z: float = 0.0
    height_m: float = 0.0


def box_footprint_corners(
    x: float, y: float, yaw: float, width: float, depth: float,
) -> list[tuple[float, float]]:
    """Four map-frame corners (front-left, front-right, back-right, back-left)
    of a box centre+yaw+dims. ``yaw`` is the outward-normal direction; the front
    face is corners[0]→corners[1]. Matches box_geometry.BoxGeometry.footprint
    ordering so HMI/perception agree on which edge is the front."""
    nx, ny = math.cos(yaw), math.sin(yaw)
    lx, ly = -ny, nx
    hw, hd = 0.5 * width, 0.5 * depth
    fx, fy = x + nx * hd, y + ny * hd
    bx, by = x - nx * hd, y - ny * hd
    return [
        (fx + lx * hw, fy + ly * hw),
        (fx - lx * hw, fy - ly * hw),
        (bx - lx * hw, by - ly * hw),
        (bx + lx * hw, by + ly * hw),
    ]


@dataclass
class TagBuffer:
    """Per-tag ring buffer of recent observations.

    Stores enough to populate :class:`TwinTagState` and to support a future
    "history" feature. The cached pose comes from the *first* OK observation
    (orchestrator stamps it from AMCL); we don't update on subsequent visits
    because tags don't move and re-stamping would jitter the HMI marker.
    """
    tag_id: str
    pose_x: Optional[float] = None
    pose_y: Optional[float] = None
    pose_qz: Optional[float] = None
    pose_qw: Optional[float] = None
    # ROS time (seconds) when this tag was most recently observed. Drives
    # the durable `last_observed` Time on TwinTagState so consumers that
    # persist the snapshot (FloraNova export, anomaly detection over
    # historical data) get an absolute timestamp, not a publish-time-relative
    # delta. Set in TwinObservation.monotonic_at by the node — name kept
    # historic for source-stability.
    last_seen_monotonic: float = 0.0
    # Latest reading per sensor name. Order preserved for stable HMI render.
    latest_readings: OrderedDict[str, float] = field(
        default_factory=OrderedDict,
    )
    history: deque = field(default_factory=lambda: deque(maxlen=DEFAULT_BUFFER_LEN))

    # Flower classification co-located with this tag (from perception's
    # KIND_FLOWER observations). Empty/false until a flower is classified.
    species: str = ''
    species_confidence: float = 0.0
    anomaly: bool = False
    flower_count: int = 0
    bug_count: int = 0
    # Localized blooms inside this tag's box + the box footprint as map (x, y)
    # corners. Latest-wins per monitoring sweep. Empty until a flower scan
    # localizes blooms (box_geometry in perception_aggregator).
    flowers: list[TwinFlower] = field(default_factory=list)
    box_footprint: list[tuple[float, float]] = field(default_factory=list)
    # 'channel' once box_geometry_json sets the footprint; 'flower' if only the
    # flower path has. The channel is authoritative — record_flower won't
    # overwrite a channel footprint.
    box_footprint_source: str = ''

    def has_pose(self) -> bool:
        return self.pose_x is not None and self.pose_y is not None


@dataclass
class TwinObservation:
    """The minimum the store needs to record one observation.

    Decoupled from the ROS Observation message so unit tests don't need
    rclpy to drive the store.
    """
    tag_id: str
    monotonic_at: float
    pose_x: Optional[float]
    pose_y: Optional[float]
    pose_qz: Optional[float]
    pose_qw: Optional[float]
    readings: list[TagSensorEntry]


@dataclass
class FlowerUpdate:
    """A flower classification co-located with a tag (KIND_FLOWER).

    Carries the tag's map pose too, so a flower seen at a tag that the
    sensor-reading path never pinned (e.g. discovered during exploration but
    not yet bridge-scanned) still gets a map pin from the detector.
    """
    tag_id: str
    monotonic_at: float
    species: str
    species_confidence: float
    anomaly: bool
    flower_count: int = 0
    bug_count: int = 0
    flowers: list[TwinFlower] = field(default_factory=list)
    box_footprint: list[tuple[float, float]] = field(default_factory=list)
    pose_x: Optional[float] = None
    pose_y: Optional[float] = None
    pose_qz: Optional[float] = None
    pose_qw: Optional[float] = None


class TwinStateStore:
    """Aggregates observations across tags. Thread-unsafe by design — the
    node uses a MutuallyExclusiveCallbackGroup so only one callback ever
    mutates the store at a time."""

    def __init__(self, buffer_len: int = DEFAULT_BUFFER_LEN):
        self._buffer_len = buffer_len
        self._tags: dict[str, TagBuffer] = {}
        # Monotonically-increasing counter that bumps on every accepted
        # observation. Used as a cache key for IDW field results so a
        # single integer comparison invalidates downstream caches.
        self._observation_count: int = 0

    # ── ingestion ──────────────────────────────────────────────────────

    def record(self, obs: TwinObservation) -> bool:
        """Insert an observation. Returns True if the store mutated.

        Returns False for malformed observations (empty tag_id) — the
        caller can then skip increment of the cache key. Pose absence is
        OK (the tag just stays unpinned in TwinState until a later OK
        observation supplies one).
        """
        if not obs.tag_id:
            return False
        buf = self._tags.get(obs.tag_id)
        if buf is None:
            buf = TagBuffer(tag_id=obs.tag_id)
            buf.history = deque(maxlen=self._buffer_len)
            self._tags[obs.tag_id] = buf

        buf.last_seen_monotonic = obs.monotonic_at
        buf.history.append(obs)

        # Cache pose from the FIRST observation that supplies one — tags
        # don't move, so re-stamping on each visit would just inject AMCL
        # jitter into the HMI's tag pin. Subsequent visits update readings
        # but leave the pose as we first saw it.
        if not buf.has_pose() and obs.pose_x is not None and obs.pose_y is not None:
            buf.pose_x = obs.pose_x
            buf.pose_y = obs.pose_y
            buf.pose_qz = obs.pose_qz if obs.pose_qz is not None else 0.0
            buf.pose_qw = obs.pose_qw if obs.pose_qw is not None else 1.0

        for r in obs.readings:
            buf.latest_readings[r.name] = r.value

        self._observation_count += 1
        return True

    def record_flower(self, upd: FlowerUpdate) -> bool:
        """Merge a flower classification onto a tag. Returns True if mutated.

        Latest-wins for species/anomaly (so the anomaly clears on a clean
        re-scan). Pins the tag from the flower pose when no pose is cached
        yet — the detector's tag pose is the plant's actual location.
        """
        if not upd.tag_id:
            return False
        buf = self._tags.get(upd.tag_id)
        if buf is None:
            buf = TagBuffer(tag_id=upd.tag_id)
            buf.history = deque(maxlen=self._buffer_len)
            self._tags[upd.tag_id] = buf

        buf.last_seen_monotonic = upd.monotonic_at
        buf.species = upd.species
        buf.species_confidence = upd.species_confidence
        buf.anomaly = upd.anomaly
        buf.flower_count = max(0, int(upd.flower_count))
        buf.bug_count = max(0, int(upd.bug_count))
        buf.flowers = list(upd.flowers)
        # The live box-geometry channel is authoritative; only fall back to the
        # flower-carried footprint when the channel hasn't claimed this tag.
        if buf.box_footprint_source != 'channel':
            buf.box_footprint = list(upd.box_footprint)
            if upd.box_footprint:
                buf.box_footprint_source = 'flower'
        if not buf.has_pose() and upd.pose_x is not None and upd.pose_y is not None:
            buf.pose_x = upd.pose_x
            buf.pose_y = upd.pose_y
            buf.pose_qz = upd.pose_qz if upd.pose_qz is not None else 0.0
            buf.pose_qw = upd.pose_qw if upd.pose_qw is not None else 1.0

        self._observation_count += 1
        return True

    def record_box(self, tag_id: str, footprint: list[tuple[float, float]]) -> bool:
        """Set a tag's box footprint from the live box-geometry channel.

        Authoritative over the flower path. Deliberately does NOT touch
        last_seen / pose / readings or bump observation_count — box geometry is
        not a sensor observation, so it must not refresh tag staleness or
        invalidate the IDW field cache. Creates a box-only entry if the tag is
        unknown (rare — the discovery feed normally pins it first)."""
        if not tag_id:
            return False
        buf = self._tags.get(tag_id)
        if buf is None:
            buf = TagBuffer(tag_id=tag_id)
            buf.history = deque(maxlen=self._buffer_len)
            self._tags[tag_id] = buf
        buf.box_footprint = list(footprint)
        buf.box_footprint_source = 'channel'
        return True

    # ── inspection ─────────────────────────────────────────────────────

    @property
    def observation_count(self) -> int:
        return self._observation_count

    def tag_ids(self) -> list[str]:
        """Return tag ids in insertion order — stable for the HMI table."""
        return list(self._tags.keys())

    def tag(self, tag_id: str) -> Optional[TagBuffer]:
        return self._tags.get(tag_id)

    def all_tags(self) -> list[TagBuffer]:
        return list(self._tags.values())

    def samples_for_sensor(self, sensor_name: str) -> list[tuple[float, float, float]]:
        """Return (x, y, value) tuples for every tag with both a pose AND
        a finite reading for ``sensor_name``. Used to feed the IDW field
        builder.

        Tags with no pose (no OK observation yet) or no/non-finite reading
        for that sensor are silently skipped — the field builder treats
        their location as "no data" and renders NaN there. The finite
        check matters: a NaN/inf sneaking through (sensor stub returning
        ``float('nan')``, divide-by-zero in a future bridge implementation)
        would poison every IDW cell within ``falloff_radius`` because the
        weighted-sum numerator picks up NaN and never recovers.
        """
        out: list[tuple[float, float, float]] = []
        for buf in self._tags.values():
            if not buf.has_pose():
                continue
            v = buf.latest_readings.get(sensor_name)
            if v is None or not math.isfinite(v):
                continue
            out.append((buf.pose_x, buf.pose_y, float(v)))  # type: ignore[arg-type]
        return out
