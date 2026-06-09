# Passive Observation When Idle — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** During teleop / manual-SLAM sessions (no mission running), make the digital twin keep filling in — flower/pest markers appear, and an opt-in launch adds per-tag climate readings that build the heatmap.

**Architecture:** One runtime signal gates everything — the `/mission/state` lifecycle. (1) The always-on `perception_aggregator` attributes flowers to the nearest seen tag whenever **no mission is active** (previously only when `/mission/state` was never seen). (2) A new opt-in launch runs the greenhouse climate oracle plus a small `passive_observer` node that polls the oracle per discovered tag while idle and republishes `KIND_TAG_READING` observations on the twin's existing intake. When a mission is active, both defer — the orchestrator owns observation.

**Tech Stack:** ROS 2 Humble, `rclpy`, `ament_python`, pytest. Packages: `lupin_perception` (node + aggregator change), `lupin_bringup` (launch), `lupin_greenhouse_bridge` (existing oracle, reused).

**Spec:** `docs/superpowers/specs/2026-06-09-passive-observe-when-idle-design.md`

---

## Conventions (read once)

**Worktree:** `/home/oskrt/worktrees/passive-observe-when-idle` (branch `feat/passive-observe-when-idle`, off `origin/main`).

**Test command** — runs perception tests against the *worktree* source (not the symlink-installed copy), in an isolated DDS domain. Use this everywhere a step says "run pytest":

```bash
cd /home/oskrt/worktrees/passive-observe-when-idle
source /opt/ros/humble/setup.bash >/dev/null && source /home/oskrt/ros2_ws/install/setup.bash >/dev/null
export PYTHONPATH=$PWD/lupin_perception:$PYTHONPATH
export ROS_DOMAIN_ID=97
python3 -m pytest <TESTFILE> -q -p no:cacheprovider
```

**Baseline:** `python3 -m pytest lupin_perception/test/ -q` → **44 passed** (already confirmed). Keep it green.

**Commits:** no `Co-Authored-By` / attribution trailer. `git add` only the listed files.

---

## Task 1: Aggregator attributes flowers when idle

Make `perception_aggregator._focus_tag()` use its nearest-tag fallback whenever there is **no active mission** (not only when `/mission/state` was never seen), gated by a new `attribute_when_idle` param (default `True`).

**Files:**
- Modify: `lupin_perception/lupin_perception/perception_aggregator.py`
- Test: `lupin_perception/test/test_perception_aggregator.py`

- [ ] **Step 1: Write the failing tests**

Append to `lupin_perception/test/test_perception_aggregator.py`:

```python
def _ready(node):
    # Mission orchestrator alive but idle (READY) — the steady state when no
    # mission has been started. This is what blocked flower attribution before.
    ms = MissionState()
    ms.lifecycle_state = 'READY'
    node._on_mission_state(ms)


def test_attributes_flower_when_idle(node):
    # No active mission: a flower seen near a discovered tag is attributed to
    # the nearest tag (teleop / manual-SLAM), so the twin fills in.
    for _ in range(3):
        node._on_tag_detections(_tag_frame((5, 0.8)))
    _ready(node)
    node._on_yolo_detections(_yolo((0, 0.9)))  # class 0 == tulip_red
    assert node._registry['5'].species == 'tulip_red'
    assert any(o.kind == Observation.KIND_FLOWER for o in node.emitted)


def test_idle_attribution_can_be_disabled(node):
    # attribute_when_idle=False restores the old strict behaviour.
    node._attribute_when_idle = False
    for _ in range(3):
        node._on_tag_detections(_tag_frame((5, 0.8)))
    _ready(node)
    node._on_yolo_detections(_yolo((0, 0.9)))
    assert node._registry['5'].species == ''
    assert node.emitted == []
```

- [ ] **Step 2: Run the tests, verify they fail**

Run pytest on `lupin_perception/test/test_perception_aggregator.py`.
Expected: `test_attributes_flower_when_idle` FAILS (no emission — idle path returns `None` today); `test_idle_attribution_can_be_disabled` FAILS with `AttributeError: ... _attribute_when_idle`.

- [ ] **Step 3: Add the active-lifecycle constant**

In `perception_aggregator.py`, below the existing scanning constants (after line 88):

```python
# Lifecycle states during which a mission owns observation end-to-end. Outside
# these (BOOT/READY/DONE/FAULT/empty, or no mission seen) the aggregator may
# passively attribute a flower to the nearest seen tag — teleop / SLAM-test.
_ACTIVE_LIFECYCLES = ('PREPARE', 'EXPLORING', 'INSPECTING', 'MONITORING', 'RETURNING')
```

- [ ] **Step 4: Declare + read the `attribute_when_idle` param**

In `__init__`, after the `publish_rate_hz` declaration (line 151):

```python
        # When no mission is active, attribute flowers to the nearest seen tag
        # so teleop / manual-SLAM sessions still pin flower/pest markers. Set
        # False to require an active SCANNING mission (the pre-2026-06-09 gate).
        self.declare_parameter('attribute_when_idle', True)
```

In the parameter-read block, after `self._camera_half_fov = ...` (line 196):

```python
        self._attribute_when_idle = bool(self.get_parameter('attribute_when_idle').value)
```

- [ ] **Step 5: Replace `_focus_tag()` body**

Replace the whole method (currently lines 481-503) with:

```python
    def _mission_active(self) -> bool:
        """True while a mission is driving/scanning, so it owns observation.
        Idle lifecycles and 'no mission state seen' are not active."""
        ms = self._mission_state
        if not self._mission_state_seen or ms is None:
            return False
        return ms.lifecycle_state in _ACTIVE_LIFECYCLES

    def _focus_tag(self) -> Optional[str]:
        """Which mission target/base a fresh flower detection belongs to.

        Active mission: the target the mission says it is parked SCANNING
        (None otherwise, to avoid drive-by misassociation while EXPLORING/
        RETURNING). No active mission (idle / standalone): the nearest seen
        tag, when ``attribute_when_idle`` — lets teleop pin flowers."""
        ms = self._mission_state
        if self._mission_active():
            scanning = (
                ms.lifecycle_state in _SCANNING_LIFECYCLES
                and _SCANNING_PHASE in (ms.mission_phase or '')
            )
            if scanning and (ms.current_target in self._boxes
                             or ms.current_target in self._registry):
                return ms.current_target
            return None
        if not self._attribute_when_idle:
            return None
        # Nearest detected tag this frame, if any.
        if not self._last_frame:
            return None
        nearest = min(self._last_frame, key=lambda t: t[1] if t[1] > 0 else math.inf)
        return nearest[0] if nearest[0] in self._registry else None
```

- [ ] **Step 6: Run the full aggregator test file, verify all pass**

Run pytest on `lupin_perception/test/test_perception_aggregator.py`.
Expected: PASS, including the unchanged `test_no_attribution_when_not_scanning` (EXPLORING is in `_ACTIVE_LIFECYCLES` → still returns `None`) and `test_species_fusion_emits_flower` (MONITORING+SCANNING → `current_target`).

- [ ] **Step 7: Commit**

```bash
git add lupin_perception/lupin_perception/perception_aggregator.py lupin_perception/test/test_perception_aggregator.py
git commit -m "feat(perception): attribute flowers to nearest tag when no mission is active"
```

---

## Task 2: KIND_TAG_READING observation builder

A small builder local to `lupin_perception` (avoids depending on `lupin_mission`) that produces the same `KIND_TAG_READING` shape the twin already consumes.

**Files:**
- Create: `lupin_perception/lupin_perception/observations.py`
- Test: `lupin_perception/test/test_observations.py`

- [ ] **Step 1: Write the failing test**

Create `lupin_perception/test/test_observations.py`:

```python
"""Unit tests for the passive-perception Observation builder."""

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose

from lupin_msgs.msg import Observation, SensorReading, TagReading
from lupin_perception.observations import make_tag_reading_observation


def _reading():
    tr = TagReading()
    tr.tag_id = '7'
    sr = SensorReading()
    sr.name, sr.value = 'temperature', 21.5
    tr.readings.append(sr)
    return tr


def test_builds_kind_tag_reading_ok():
    obs = make_tag_reading_observation(
        source='passive_observer', stamp=Time(), tag_reading=_reading(),
        tag_map_pose=None, frame_id='map')
    assert obs.kind == Observation.KIND_TAG_READING
    assert obs.status == Observation.STATUS_OK
    assert obs.source == 'passive_observer'
    assert obs.header.frame_id == 'map'
    assert obs.tag_reading.tag_id == '7'
    assert obs.tag_reading.readings[0].name == 'temperature'


def test_normalises_zero_quaternion_when_position_present():
    pose = Pose()            # all-zero orientation (w==0 == "missing" to the twin)
    pose.position.x = 2.0
    obs = make_tag_reading_observation(
        source='x', stamp=Time(), tag_reading=_reading(),
        tag_map_pose=pose, frame_id='map')
    assert obs.tag_pose_in_map.position.x == pytest.approx(2.0)
    assert obs.tag_pose_in_map.orientation.w == pytest.approx(1.0)
```

- [ ] **Step 2: Run the test, verify it fails**

Run pytest on `lupin_perception/test/test_observations.py`.
Expected: FAIL — `ModuleNotFoundError: No module named 'lupin_perception.observations'`.

- [ ] **Step 3: Write the builder**

Create `lupin_perception/lupin_perception/observations.py`:

```python
"""Build lupin_msgs/Observation messages for passive (no-mission) perception.

Mirrors the KIND_TAG_READING shape lupin_mission.observations.make_tag_observation
emits, so the digital twin treats passive and mission readings identically. Kept
local to lupin_perception to avoid depending on lupin_mission (wrong direction);
extract to a shared util only if a third consumer appears.
"""

from __future__ import annotations

from typing import Optional

from geometry_msgs.msg import Pose
from std_msgs.msg import Header

from lupin_msgs.msg import Observation, TagReading


def make_tag_reading_observation(
    *,
    source: str,
    stamp,                              # builtin_interfaces/Time
    tag_reading: TagReading,
    tag_map_pose: Optional[Pose] = None,
    frame_id: str = 'map',
    mission_id: str = '',
) -> Observation:
    """A STATUS_OK KIND_TAG_READING Observation for /floranova/observations.

    Pins the tag at its own map pose (from the discovered-tags feed). The twin
    treats orientation.w == 0 as "no pose", so normalise an all-zero quaternion
    to identity when a position is present.
    """
    msg = Observation()
    msg.header = Header(stamp=stamp, frame_id=frame_id)
    msg.mission_id = mission_id
    msg.source = source
    msg.kind = Observation.KIND_TAG_READING
    msg.status = Observation.STATUS_OK
    msg.tag_reading = tag_reading
    if tag_map_pose is not None:
        pose = Pose()
        pose.position = tag_map_pose.position
        pose.orientation = tag_map_pose.orientation
        q = pose.orientation
        if q.x == 0.0 and q.y == 0.0 and q.z == 0.0 and q.w == 0.0:
            q.w = 1.0
        msg.tag_pose_in_map = pose
    return msg
```

- [ ] **Step 4: Run the test, verify it passes**

Run pytest on `lupin_perception/test/test_observations.py`.
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add lupin_perception/lupin_perception/observations.py lupin_perception/test/test_observations.py
git commit -m "feat(perception): KIND_TAG_READING observation builder for passive perception"
```

---

## Task 3: `passive_observer` node

Polls the greenhouse oracle per discovered tag while idle, republishing `KIND_TAG_READING` observations. Decision logic (`mission_active`, `_due_tags`, `_on_reading`) is split from the service plumbing so it unit-tests without a live bridge.

**Files:**
- Create: `lupin_perception/lupin_perception/passive_observer.py`
- Test: `lupin_perception/test/test_passive_observer.py`
- Modify: `lupin_perception/setup.py` (entry point)

- [ ] **Step 1: Write the failing tests**

Create `lupin_perception/test/test_passive_observer.py`:

```python
"""Unit tests for the passive_observer node decision logic.

The bridge service is not spun up — tests drive the seams (_due_tags,
_on_reading) directly and capture published observations.
"""

import pytest
import rclpy
from geometry_msgs.msg import Pose

from lupin_msgs.msg import DiscoveredTag, DiscoveredTags, MissionState, Observation, SensorReading, TagReading
from lupin_msgs.srv import GetTagReading
from lupin_perception.passive_observer import PassiveObserver, mission_active


@pytest.fixture
def node():
    rclpy.init()
    n = PassiveObserver()
    n.emitted = []
    n._obs_pub.publish = n.emitted.append  # type: ignore[method-assign]
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _discovered(*ids):
    msg = DiscoveredTags()
    msg.header.frame_id = 'map'
    for i in ids:
        t = DiscoveredTag()
        t.tag_id = i
        t.pose_in_map.position.x = 1.0
        t.pose_in_map.orientation.w = 1.0
        msg.tags.append(t)
    return msg


def _state(lifecycle):
    ms = MissionState()
    ms.lifecycle_state = lifecycle
    return ms


class _Future:
    def __init__(self, result):
        self._r = result

    def result(self):
        return self._r


def _response(status, tag_id='3'):
    res = GetTagReading.Response()
    res.status = status
    if status == GetTagReading.Response.STATUS_OK:
        tr = TagReading()
        tr.tag_id = tag_id
        sr = SensorReading()
        sr.name, sr.value = 'temperature', 21.0
        tr.readings.append(sr)
        res.reading = tr
    return res


def test_mission_active_classification():
    assert mission_active(_state('EXPLORING')) is True
    assert mission_active(_state('MONITORING')) is True
    assert mission_active(_state('READY')) is False
    assert mission_active(_state('DONE')) is False
    assert mission_active(None) is False


def test_due_tags_when_idle(node):
    node._on_discovered_tags(_discovered('3', '4'))
    node._on_mission_state(_state('READY'))
    assert set(node._due_tags()) == {'3', '4'}


def test_no_due_tags_when_mission_active(node):
    node._on_discovered_tags(_discovered('3'))
    node._on_mission_state(_state('MONITORING'))
    assert node._due_tags() == []


def test_inflight_tag_excluded(node):
    node._on_discovered_tags(_discovered('3'))
    node._on_mission_state(_state('READY'))
    node._inflight.add('3')
    assert node._due_tags() == []


def test_on_reading_ok_publishes_tag_reading(node):
    node._on_discovered_tags(_discovered('3'))
    node._inflight.add('3')
    node._on_reading('3', _Future(_response(GetTagReading.Response.STATUS_OK, '3')))
    assert len(node.emitted) == 1
    obs = node.emitted[0]
    assert obs.kind == Observation.KIND_TAG_READING
    assert obs.status == Observation.STATUS_OK
    assert obs.header.frame_id == 'map'
    assert obs.tag_reading.tag_id == '3'
    assert obs.tag_pose_in_map.orientation.w == pytest.approx(1.0)
    assert '3' in node._last_read and '3' not in node._inflight


def test_on_reading_unknown_tag_no_publish(node):
    node._on_discovered_tags(_discovered('3'))
    node._inflight.add('3')
    node._on_reading('3', _Future(_response(GetTagReading.Response.STATUS_UNKNOWN_TAG, '3')))
    assert node.emitted == []
    assert '3' in node._last_read  # marked read so we don't hammer the bridge


def test_read_once_then_not_due(node):
    node._on_discovered_tags(_discovered('3'))
    node._on_mission_state(_state('READY'))
    node._on_reading('3', _Future(_response(GetTagReading.Response.STATUS_OK, '3')))
    assert node._due_tags() == []  # refresh_period_s=0 → read each tag once
```

- [ ] **Step 2: Run the tests, verify they fail**

Run pytest on `lupin_perception/test/test_passive_observer.py`.
Expected: FAIL — `ModuleNotFoundError: No module named 'lupin_perception.passive_observer'`.

- [ ] **Step 3: Write the node**

Create `lupin_perception/lupin_perception/passive_observer.py`:

```python
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
        self._obs_pub = self.create_publisher(
            Observation, str(self.get_parameter('observations_topic').value), 50)
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
```

- [ ] **Step 4: Run the tests, verify they pass**

Run pytest on `lupin_perception/test/test_passive_observer.py`.
Expected: PASS (7 tests).

- [ ] **Step 5: Register the executable**

In `lupin_perception/setup.py`, add to `console_scripts` (after the `perception_aggregator` line):

```python
            'passive_observer = lupin_perception.passive_observer:main',
```

- [ ] **Step 6: Run the full perception suite, verify green**

Run pytest on `lupin_perception/test/` (whole dir).
Expected: PASS — 44 baseline + 2 (Task 1) + 2 (Task 2) + 7 (Task 3) = **55 passed**.

- [ ] **Step 7: Commit**

```bash
git add lupin_perception/lupin_perception/passive_observer.py lupin_perception/test/test_passive_observer.py lupin_perception/setup.py
git commit -m "feat(perception): passive_observer node — climate readings on sight when idle"
```

---

## Task 4: `passive_observe.launch.py` (opt-in bringup)

Composes the greenhouse oracle + `passive_observer` into one terminal (mirrors `mission_stack.launch.py`). The aggregator (T7) already attributes flowers when idle after Task 1, so this launch only adds the climate path.

**Files:**
- Create: `lupin_bringup/launch/passive_observe.launch.py`

(No `setup.py` change — `lupin_bringup` already installs `launch/*.launch.py` via glob, same as `mission_stack.launch.py`.)

- [ ] **Step 1: Write the launch file**

Create `lupin_bringup/launch/passive_observe.launch.py`:

```python
"""passive_observe.launch.py — fill the twin during teleop / manual SLAM.

Following DEMO_DAY_WIRED with NO mission running, the map and tag *pins* already
update, and (after the 2026-06-09 aggregator change) flower/pest markers pin too.
This optional terminal adds the missing climate path so the heatmap also builds:

    greenhouse_bridge   → /greenhouse_bridge/get_tag_reading      (climate oracle)
    passive_observer    → polls the oracle per discovered tag while the mission
                          is idle → KIND_TAG_READING on /floranova/observations

Run it as one extra terminal AFTER the base stack (Nav2 / SLAM / twin / perception
T1–T8). It defers automatically while a mission is active.

MUTUALLY EXCLUSIVE with mission_stack.launch.py (T9): both launch
`greenhouse_bridge` (same node name + service). Run THIS for teleop/observe
sessions; run mission_stack for autonomous runs — never both.

Usage:
    ros2 launch lupin_bringup passive_observe.launch.py
    ros2 launch lupin_bringup passive_observe.launch.py refresh_period_s:=30.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_bringup = get_package_share_directory('lupin_bringup')
    pkg_bridge = get_package_share_directory('lupin_greenhouse_bridge')

    # Same tag oracle config mission_stack feeds the bridge — keep in sync.
    tag_locations = os.path.join(pkg_bringup, 'config', 'tag_locations_widened.json')

    args = [
        DeclareLaunchArgument(
            'refresh_period_s', default_value='0.0',
            description='0 = read each tag once; > 0 = re-poll after N seconds '
                        '(track time-of-day drift in the sim oracle).',
        ),
        DeclareLaunchArgument(
            'tag_file', default_value=tag_locations,
            description='Greenhouse tag-oracle config fed to the bridge.',
        ),
        DeclareLaunchArgument(
            'observations_topic', default_value='/floranova/observations',
            description="Twin's observation intake the readings are published on.",
        ),
        DeclareLaunchArgument(
            'map_frame', default_value='map',
            description='Frame the tag poses (and emitted observations) are in.',
        ),
    ]

    bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bridge, 'launch', 'greenhouse_bridge.launch.py'),
        ),
        launch_arguments=[('tag_file', LaunchConfiguration('tag_file'))],
    )

    observer = Node(
        package='lupin_perception',
        executable='passive_observer',
        name='passive_observer',
        output='screen',
        parameters=[{
            'refresh_period_s': LaunchConfiguration('refresh_period_s'),
            'observations_topic': LaunchConfiguration('observations_topic'),
            'map_frame': LaunchConfiguration('map_frame'),
        }],
    )

    return LaunchDescription([
        *args,
        LogInfo(msg='[lupin_bringup] passive_observe: greenhouse_bridge + '
                    'passive_observer. Fills the twin (climate/heatmap) during '
                    'teleop / SLAM tests. Do NOT run alongside mission_stack.'),
        bridge,
        observer,
    ])
```

- [ ] **Step 2: Verify the launch description builds (no DDS needed)**

Run:

```bash
cd /home/oskrt/worktrees/passive-observe-when-idle
source /opt/ros/humble/setup.bash >/dev/null && source /home/oskrt/ros2_ws/install/setup.bash >/dev/null
python3 -c "import importlib.util, sys; \
spec = importlib.util.spec_from_file_location('m', 'lupin_bringup/launch/passive_observe.launch.py'); \
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m); \
ld = m.generate_launch_description(); print('actions:', len(ld.entities))"
```

Expected: prints `actions: 7` with no exception (4 args + LogInfo + bridge include + observer; the count just confirms it constructed). If it raises `PackageNotFoundError: lupin_bringup`/`lupin_greenhouse_bridge`, the workspace isn't sourced — re-source and retry.

- [ ] **Step 3: Commit**

```bash
git add lupin_bringup/launch/passive_observe.launch.py
git commit -m "feat(bringup): passive_observe.launch.py — bridge + passive_observer (opt-in)"
```

---

## Task 5: Document the opt-in path in DEMO_DAY_WIRED.md

Make the feature discoverable where the operator actually looks.

**Files:**
- Modify: `lupin_bringup/DEMO_DAY_WIRED.md`

- [ ] **Step 1: Update the T7 "Markers need a mission" note**

In `lupin_bringup/DEMO_DAY_WIRED.md`, find the blockquote in §4 T7 that begins
`> **Markers need a mission.** Flower/pest markers only pin once T9's SCANNING`
and replace that blockquote with:

```markdown
> **Flowers vs. heatmap without a mission.** After the 2026-06-09 passive-observe
> change, flower/pest markers pin during teleop too (the aggregator attributes a
> bloom to the nearest seen tag whenever no mission is active). The climate
> **heatmap** still needs a reading source — add **T7.5** below for it. The
> green-box tag overlay and tag *pins* work on their own regardless.
```

- [ ] **Step 2: Add the T7.5 section**

Immediately after the end of the T7 section (before `### T8 — Xbox teleop`), insert:

```markdown
### T7.5 — Passive observation (optional; fills the twin without a mission)

When you want the digital twin to keep filling in during **teleop / manual SLAM**
— flower/pest markers *and* the climate heatmap — without starting the autonomous
mission, add this one terminal after T7:

```bash
ros2 launch lupin_bringup passive_observe.launch.py
```

**Brings up:** the `greenhouse_bridge` (climate oracle) + `passive_observer`, which
polls the oracle once per discovered tag while the mission is idle and publishes
`KIND_TAG_READING` on `/floranova/observations` → twin → `/twin/get_field` heatmap.
Drive past the tags and watch the HMI twin/map fill: tag pins, per-tag readings,
heatmap, and flower markers (point the gripper cam at blooms).

> **Do NOT run T7.5 together with T9 (`mission_stack`).** Both launch
> `greenhouse_bridge` (same node + service) and collide. T7.5 is for *no-mission*
> sessions; T9 is for autonomous runs. `passive_observer` also auto-defers while a
> mission is active, so when you switch to T9, stop T7.5 first.
```

- [ ] **Step 3: Note it in the §5 test table**

In §5, find the row `Mission rows (need T7 full stack + T9): ...` and add this line
just above it:

```markdown
No-mission twin fill (**Flower→map**, **Climate heatmap**): add **T7.5**
(`passive_observe.launch.py`) and teleop past the tags — no mission needed.
```

- [ ] **Step 4: Commit**

```bash
git add lupin_bringup/DEMO_DAY_WIRED.md
git commit -m "docs(bringup): document T7.5 passive-observe path in DEMO_DAY_WIRED"
```

---

## Task 6: Workspace build + full verification

Confirm the real (non-PYTHONPATH-overridden) build is clean before handing off.

**Files:** none (build + test).

- [ ] **Step 1: colcon build the touched packages**

```bash
cd /home/oskrt/ros2_ws
colcon build --symlink-install --packages-select lupin_perception lupin_bringup lupin_greenhouse_bridge lupin_msgs
```

Expected: `Finished ... 4 packages` with no failures. (Builds from the worktree only if `~/ros2_ws/src/lupin` points at it; otherwise this validates the base packages — the per-package pytest runs above are the source of truth for the worktree code.)

- [ ] **Step 2: Run the full perception suite once more**

Run pytest on `lupin_perception/test/` (whole dir).
Expected: **55 passed**.

- [ ] **Step 3: Manual hardware/sim checklist (record results, do not auto-check)**

Following `DEMO_DAY_WIRED.md` T1–T8 + **T7.5** (or the sim equivalent), no mission started:
- [ ] Teleop past ≥3 tags → HMI twin shows tag **pins** (regression check — already worked).
- [ ] Per-tag **climate readings** appear on the twin tags.
- [ ] **Heatmap** renders in the HMI twin/map view (confirms `/twin/get_field` has data — risk #1 in the spec).
- [ ] Point the gripper cam at blooms near a tag → **flower/pest markers** pin.
- [ ] No `STATUS_UNKNOWN_TAG` spam in the `passive_observer` log for real tags (risk #2/#3: tag-id ↔ oracle alignment).
- [ ] Start a mission (T9, after stopping T7.5) → behaves exactly as before (passive path defers).

---

## Integration (after manual verification)

Open an MR `feat/passive-observe-when-idle` → `main`. Once merged, fast-forward
`hardware` and `sim` to keep three-branch parity (generic feature, no branch-specific
bits). See `feedback_feature_branches_mrs` / `feedback_branch_split_commits`.
