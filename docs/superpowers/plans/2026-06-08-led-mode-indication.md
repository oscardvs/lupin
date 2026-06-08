# LED Mode Indication Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the onboard LED strip communicate the robot's live activity — green-blink driving, orange-blink arm motion, blue standby, red on e-stop — in every launch config (teleop, HMI-only, idle), without disturbing the existing mission-lifecycle palette.

**Architecture:** Extend `light_strip_bridge` (already onboard, the single LED chokepoint) with three new onboard subscriptions (`/e_stop_state`, twist_mux output `/mirte_base_controller/cmd_vel`, `/joint_states`). A pure precedence function `_decide_style()` returns `(rgb, blink)`; a new render timer turns that into actual strip writes (toggling for blink) through the existing `_send_color` path. The new "activity" rows only apply when no mission is in progress (gap-fill), so mission display is unchanged.

**Tech Stack:** ROS 2 Humble, `rclpy`, ament_python, pytest. Reuses existing palette constants and `lupin_hmi.arm_limits` joint names.

**Spec:** `docs/superpowers/specs/2026-06-08-led-mode-indication-design.md`

**Branch:** Work on `feat/led-mode-indication` off `main` (a clean worktree — `main`'s working tree has unrelated uncommitted xbox-calibration WIP that must stay untouched). No Co-Authored-By trailer on commits.

---

## Environment setup (do once before running any test)

The tests import `lupin_msgs` (a generated message package), so the workspace must be built and sourced. The package is symlink-installed, so Python edits to `light_strip_bridge.py` are live (no rebuild needed for `.py` changes).

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

If `import lupin_msgs` fails, build once:

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select lupin_msgs lupin_hmi && source ~/ros2_ws/install/setup.bash
```

Run the bridge tests with:

```bash
python -m pytest ~/ros2_ws/src/lupin/lupin_hmi/test/test_light_strip_bridge.py -v
```

---

## File Structure

- **Modify** `lupin_hmi/lupin_hmi/light_strip_bridge.py` — all new logic (pure functions + node wiring). Main change.
- **Modify** `lupin_hmi/test/test_light_strip_bridge.py` — add pure-function tests for the new precedence, detection, and blink logic. Existing tests are untouched (the `_state_to_rgb` signature is unchanged).
- **Modify** `lupin_bringup/launch/onboard.launch.py` — pass new params explicitly (defaults are correct; explicit for the sim override hook).
- **Modify** (sim) the launch that starts the bridge in simulation, if any — set `drive_topic` to the sim `_unstamped` topic.
- **Modify** `lupin_hmi/package.xml` — ensure `geometry_msgs`, `sensor_msgs`, `std_msgs` exec deps exist.
- **Modify** `lupin/CHANGELOG.md` — note the feature.

---

## Task 1: Pure precedence — `_decide_style()` returns `(rgb, blink)`

This is the heart: the full priority table as a pure function, reusing the existing palette constants and `_state_to_rgb`/`_safety_rgb`. No ROS wiring yet.

**Files:**
- Modify: `lupin_hmi/lupin_hmi/light_strip_bridge.py`
- Test: `lupin_hmi/test/test_light_strip_bridge.py`

- [ ] **Step 1: Add the lifecycle sets and the joint-name import near the top of `light_strip_bridge.py`**

Add the import alongside the existing imports (after the `from mirte_msgs.srv import SetNeopixel` line):

```python
from lupin_hmi.arm_limits import ARM_JOINT_FULL
```

Add these module-level constants right after the palette block (after the `BATTERY_LOW = (255, 0, 128)` line):

```python
# Lifecycle sets for the activity layer (see 2026-06-08-led-mode-indication spec).
# In-progress: the mission owns the strip; live motion must NOT override it.
IN_PROGRESS_LIFECYCLES = frozenset(
    {'PREPARE', 'EXPLORING', 'INSPECTING', 'MONITORING', 'RETURNING'}
)
# Resting: mission idle/finished; the activity layer applies, and when idle the
# resting lifecycle colour is shown (DONE->green, READY->blue, BOOT->white).
RESTING_LIFECYCLES = frozenset({'BOOT', 'READY', 'DONE'})

# Arm + gripper joint names as they appear in /joint_states (URDF joints). The
# wheel joints also ride /joint_states, so motion detection filters to these.
ARM_STRIP_JOINTS = frozenset(ARM_JOINT_FULL.values()) | {'gripper_joint'}
```

- [ ] **Step 2: Write the failing precedence tests**

Add to `lupin_hmi/test/test_light_strip_bridge.py`. First extend the imports at the top:

```python
from lupin_hmi.light_strip_bridge import (
    AMBER,
    BATTERY_LOW,
    BLUE,
    GREEN,
    LightStripBridge,
    ORANGE,
    RED,
    SPRING,
)
```

Then add the helper and tests:

```python
def _style(mission_fields=None, *, estop=False, mission_fresh=True,
           mode='auto', manual_rgb=None, arm_moving=False, base_driving=False):
    # Exercise the pure precedence without standing up a ROS node.
    bridge = object.__new__(LightStripBridge)
    bridge._unknown_state_off = False
    msg = MissionState()
    for k, v in (mission_fields or {}).items():
        setattr(msg, k, v)
    return bridge._decide_style(
        msg, estop=estop, mission_fresh=mission_fresh, mode=mode,
        manual_rgb=manual_rgb, arm_moving=arm_moving, base_driving=base_driving,
    )


def test_driving_no_mission_is_green_blink():
    assert _style({}, base_driving=True) == (GREEN, True)


def test_arm_moving_no_mission_is_orange_blink():
    assert _style({}, arm_moving=True) == (ORANGE, True)


def test_arm_outranks_base_when_both_move():
    assert _style({}, arm_moving=True, base_driving=True) == (ORANGE, True)


def test_idle_no_mission_node_is_blue_solid():
    assert _style({}, mission_fresh=False) == (BLUE, False)


def test_idle_after_done_shows_green_solid():
    assert _style({'lifecycle_state': 'DONE'}) == (GREEN, False)


def test_idle_at_ready_is_blue_solid():
    assert _style({'lifecycle_state': 'READY'}) == (BLUE, False)


def test_in_progress_mission_overrides_driving():
    # Gap-fill: while a mission is in progress its lifecycle colour wins and
    # stays solid — driving does NOT turn it green-blink.
    assert _style({'lifecycle_state': 'EXPLORING'}, base_driving=True) == (SPRING, False)


def test_estop_standalone_is_red():
    # E-stop with no mission running at all (fresh=False) still shows red.
    assert _style({}, estop=True, mission_fresh=False) == (RED, False)


def test_estop_via_mission_flag_is_red():
    assert _style({'estop_engaged': True}) == (RED, False)


def test_manual_hold_outranks_activity():
    assert _style({}, mode='manual', manual_rgb=(10, 20, 30), base_driving=True) == ((10, 20, 30), False)


def test_safety_outranks_manual():
    assert _style({}, estop=True, mode='manual', manual_rgb=(10, 20, 30)) == (RED, False)


def test_pause_outranks_in_progress_and_manual():
    assert _style({'lifecycle_state': 'INSPECTING', 'paused': True}) == (AMBER, False)
```

- [ ] **Step 3: Run the tests to verify they fail**

```bash
python -m pytest ~/ros2_ws/src/lupin/lupin_hmi/test/test_light_strip_bridge.py -v
```

Expected: the new `test_*` fail with `AttributeError: 'LightStripBridge' object has no attribute '_decide_style'` (or `ImportError` for `BLUE`/`GREEN`/`SPRING` if those aren't exported — they are module constants, so the import resolves; the failure is the missing method).

- [ ] **Step 4: Implement `_decide_style`**

Add this method to the `LightStripBridge` class (place it right after `_state_to_rgb`, keeping the pure-mapping methods together):

```python
def _decide_style(self, mission, *, estop, mission_fresh, mode,
                  manual_rgb, arm_moving, base_driving):
    """Pure precedence: cached inputs -> (rgb, blink).

    Highest priority wins. Rows 1-4 are the existing behaviour (rows 2/4
    gated by mission freshness so a dead orchestrator can't latch the strip);
    rows 5-7 are the activity layer, reached only when no fresh in-progress
    mission owns the strip. See the 2026-06-08-led-mode-indication spec.
    """
    lifecycle = (mission.lifecycle_state or '').upper()

    # 1. Safety — NOT freshness-gated (fail-safe). Live /e_stop_state OR the
    #    mission's own estop flag OR a FAULT lifecycle all force red.
    if estop or self._safety_rgb(mission) is not None:
        return RED, False

    # 2. Operator pause (mission) — a frozen robot must read as held.
    if mission_fresh and mission.paused:
        return AMBER, False

    # 3. Manual HMI colour hold — operator override (auto restores live layer).
    if mode == 'manual' and manual_rgb is not None:
        return manual_rgb, False

    # 4. In-progress mission — existing lifecycle palette, solid, untouched.
    if mission_fresh and lifecycle in IN_PROGRESS_LIFECYCLES:
        return self._state_to_rgb(mission), False

    # 5. Arm moving — orange blink (out-ranks base).
    if arm_moving:
        return ORANGE, True

    # 6. Base driving — green blink.
    if base_driving:
        return GREEN, True

    # 7. Idle — resting mission colour if a mission node is present, else blue.
    if mission_fresh and lifecycle in RESTING_LIFECYCLES:
        return self._state_to_rgb(mission), False
    return BLUE, False
```

- [ ] **Step 5: Run the tests to verify they pass**

```bash
python -m pytest ~/ros2_ws/src/lupin/lupin_hmi/test/test_light_strip_bridge.py -v
```

Expected: PASS (all new tests + the 3 pre-existing tests).

- [ ] **Step 6: Commit**

```bash
git add lupin_hmi/lupin_hmi/light_strip_bridge.py lupin_hmi/test/test_light_strip_bridge.py
git commit -m "feat(hmi): pure LED activity-precedence decision (_decide_style)"
```

---

## Task 2: Pure detection predicates + arm-motion tracking

The timestamp/threshold logic that decides "is the base driving", "is the arm moving", "is mission state fresh". Pure functions reading cached state + an explicit `now` (float seconds), so they unit-test without a clock or a node.

**Files:**
- Modify: `lupin_hmi/lupin_hmi/light_strip_bridge.py`
- Test: `lupin_hmi/test/test_light_strip_bridge.py`

- [ ] **Step 1: Write the failing detection tests**

Add to `test_light_strip_bridge.py`:

```python
from geometry_msgs.msg import Twist


def _detector(**attrs):
    bridge = object.__new__(LightStripBridge)
    # Defaults the predicates read; override via attrs.
    bridge._drive_timeout = 0.4
    bridge._drive_deadband = 1e-3
    bridge._arm_motion_hold = 0.3
    bridge._arm_deadband_rad = 0.0087
    bridge._mission_state_timeout = 2.0
    bridge._twist_nonzero = False
    bridge._twist_stamp = None
    bridge._arm_motion_stamp = None
    bridge._mission_stamp = None
    bridge._arm_last_pos = {}
    for k, v in attrs.items():
        setattr(bridge, k, v)
    return bridge


def _twist(x=0.0, y=0.0, wz=0.0):
    t = Twist()
    t.linear.x = x
    t.linear.y = y
    t.angular.z = wz
    return t


def test_twist_nonzero_detects_motion_components():
    b = _detector()
    assert b._twist_is_nonzero(_twist(x=0.2), b._drive_deadband) is True
    assert b._twist_is_nonzero(_twist(y=0.2), b._drive_deadband) is True
    assert b._twist_is_nonzero(_twist(wz=0.2), b._drive_deadband) is True
    assert b._twist_is_nonzero(_twist(), b._drive_deadband) is False
    assert b._twist_is_nonzero(_twist(x=1e-4), b._drive_deadband) is False


def test_base_driving_requires_recent_nonzero():
    b = _detector(_twist_nonzero=True, _twist_stamp=100.0)
    assert b._base_driving(now=100.2) is True      # within timeout
    assert b._base_driving(now=100.5) is False     # stale (silence/dead-man)
    b2 = _detector(_twist_nonzero=False, _twist_stamp=100.0)
    assert b2._base_driving(now=100.1) is False     # last twist was zero
    b3 = _detector(_twist_nonzero=True, _twist_stamp=None)
    assert b3._base_driving(now=100.0) is False      # never received


def test_arm_moving_holds_then_clears():
    b = _detector(_arm_motion_stamp=50.0)
    assert b._arm_moving(now=50.2) is True          # within hold
    assert b._arm_moving(now=50.4) is False         # hold elapsed
    assert _detector(_arm_motion_stamp=None)._arm_moving(now=50.0) is False


def test_mission_fresh_window():
    b = _detector(_mission_stamp=10.0)
    assert b._mission_fresh(now=11.0) is True
    assert b._mission_fresh(now=13.0) is False
    assert _detector(_mission_stamp=None)._mission_fresh(now=10.0) is False


def test_note_arm_motion_filters_to_arm_joints():
    b = _detector()
    # First sample seeds positions, no motion yet.
    assert b._note_arm_motion(['elbow_joint', 'wheel_left_joint'], [0.0, 0.0], now=1.0) is False
    # Arm joint moved past deadband -> motion stamped.
    assert b._note_arm_motion(['elbow_joint'], [0.5], now=2.0) is True
    assert b._arm_motion_stamp == 2.0
    # A wheel joint moving is ignored (not an arm joint).
    b2 = _detector()
    b2._note_arm_motion(['wheel_left_joint'], [0.0], now=1.0)
    assert b2._note_arm_motion(['wheel_left_joint'], [9.0], now=2.0) is False
    # Gripper counts as arm motion.
    b3 = _detector()
    b3._note_arm_motion(['gripper_joint'], [0.0], now=1.0)
    assert b3._note_arm_motion(['gripper_joint'], [0.2], now=2.0) is True
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
python -m pytest ~/ros2_ws/src/lupin/lupin_hmi/test/test_light_strip_bridge.py -k "twist or driving or arm_moving or mission_fresh or note_arm" -v
```

Expected: FAIL with `AttributeError` for `_twist_is_nonzero` / `_base_driving` / `_arm_moving` / `_mission_fresh` / `_note_arm_motion`.

- [ ] **Step 3: Implement the detection predicates**

Add these methods to `LightStripBridge` (group them after `_decide_style`):

```python
@staticmethod
def _twist_is_nonzero(twist, deadband):
    """True if any mecanum-relevant component exceeds the deadband."""
    return (
        abs(twist.linear.x) > deadband
        or abs(twist.linear.y) > deadband
        or abs(twist.angular.z) > deadband
    )

def _base_driving(self, now):
    """A non-zero twist was received within drive_timeout (silence => stopped)."""
    return (
        self._twist_nonzero
        and self._twist_stamp is not None
        and (now - self._twist_stamp) <= self._drive_timeout
    )

def _arm_moving(self, now):
    """An arm/gripper joint moved within the last arm_motion_hold seconds."""
    return (
        self._arm_motion_stamp is not None
        and (now - self._arm_motion_stamp) <= self._arm_motion_hold
    )

def _mission_fresh(self, now):
    """/mission/state seen within mission_state_timeout (else treat as absent)."""
    return (
        self._mission_stamp is not None
        and (now - self._mission_stamp) <= self._mission_state_timeout
    )

def _note_arm_motion(self, names, positions, now):
    """Update per-joint position cache; stamp motion if a tracked joint moved.

    Filters to ARM_STRIP_JOINTS so the wheel joints that also ride
    /joint_states don't read as arm motion. Returns True if motion stamped.
    """
    moved = False
    for name, pos in zip(names, positions):
        if name not in ARM_STRIP_JOINTS:
            continue
        prev = self._arm_last_pos.get(name)
        if prev is not None and abs(pos - prev) > self._arm_deadband_rad:
            moved = True
        self._arm_last_pos[name] = pos
    if moved:
        self._arm_motion_stamp = now
    return moved
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
python -m pytest ~/ros2_ws/src/lupin/lupin_hmi/test/test_light_strip_bridge.py -v
```

Expected: PASS (all tests).

- [ ] **Step 5: Commit**

```bash
git add lupin_hmi/lupin_hmi/light_strip_bridge.py lupin_hmi/test/test_light_strip_bridge.py
git commit -m "feat(hmi): LED drive/arm/mission-freshness detection predicates"
```

---

## Task 3: Pure blink math — `_blink_on()` and `_effective_rgb()`

Turns a `(rgb, blink)` style plus the current time into the actual colour to write.

**Files:**
- Modify: `lupin_hmi/lupin_hmi/light_strip_bridge.py`
- Test: `lupin_hmi/test/test_light_strip_bridge.py`

- [ ] **Step 1: Write the failing blink tests**

Add to `test_light_strip_bridge.py` (extend the import to include `OFF`):

```python
from lupin_hmi.light_strip_bridge import OFF


def test_blink_on_is_first_half_of_cycle():
    b = object.__new__(LightStripBridge)
    b._blink_hz = 1.0          # 1 Hz -> 0.5 s on / 0.5 s off
    assert b._blink_on(0.0) is True
    assert b._blink_on(0.25) is True
    assert b._blink_on(0.5) is False
    assert b._blink_on(0.75) is False
    assert b._blink_on(1.0) is True    # next cycle


def test_effective_rgb_blanks_on_off_phase_only():
    assert LightStripBridge._effective_rgb(GREEN, True, True) == GREEN
    assert LightStripBridge._effective_rgb(GREEN, True, False) == OFF
    # Solid styles ignore the blink phase entirely.
    assert LightStripBridge._effective_rgb(BLUE, False, False) == BLUE
    assert LightStripBridge._effective_rgb(BLUE, False, True) == BLUE
```

- [ ] **Step 2: Run the tests to verify they fail**

```bash
python -m pytest ~/ros2_ws/src/lupin/lupin_hmi/test/test_light_strip_bridge.py -k "blink or effective" -v
```

Expected: FAIL with `AttributeError` for `_blink_on` / `_effective_rgb`.

- [ ] **Step 3: Implement the blink helpers**

Add to `LightStripBridge`:

```python
def _blink_on(self, now):
    """On for the first half of each blink cycle (blink_hz full cycles/sec)."""
    period = 1.0 / self._blink_hz
    return (now % period) < (period / 2.0)

@staticmethod
def _effective_rgb(rgb, blink, blink_on):
    """The colour to actually write: rgb when solid or in the on-phase, else OFF."""
    return rgb if (not blink or blink_on) else OFF
```

- [ ] **Step 4: Run the tests to verify they pass**

```bash
python -m pytest ~/ros2_ws/src/lupin/lupin_hmi/test/test_light_strip_bridge.py -v
```

Expected: PASS (all tests).

- [ ] **Step 5: Commit**

```bash
git add lupin_hmi/lupin_hmi/light_strip_bridge.py lupin_hmi/test/test_light_strip_bridge.py
git commit -m "feat(hmi): LED blink phase + effective-colour helpers"
```

---

## Task 4: Node wiring — subscriptions, cached state, render timer

Connect the pure functions to live ROS. New params + subscriptions feed the cache; a render timer drives the strip. Refactor the service handlers and `_on_state` to cache-only (the render timer does all sending), and remove the now-superseded `_desired_rgb`. No new unit test — verified by Task 6's smoke test; the existing pure tests must still pass.

**Files:**
- Modify: `lupin_hmi/lupin_hmi/light_strip_bridge.py`
- Modify: `lupin_hmi/package.xml`

- [ ] **Step 1: Add the message imports**

Near the top of `light_strip_bridge.py`, with the other message imports (the `ARM_JOINT_FULL` import from Task 1 is already there):

```python
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
```

- [ ] **Step 2: Declare the new parameters**

In `__init__`, after the existing `self.declare_parameter('color_order', 'RGB')` line, add:

```python
self.declare_parameter('estop_topic', '/e_stop_state')
self.declare_parameter('drive_topic', '/mirte_base_controller/cmd_vel')
self.declare_parameter('joint_states_topic', '/joint_states')
self.declare_parameter('drive_timeout', 0.4)
self.declare_parameter('drive_deadband', 1e-3)
self.declare_parameter('arm_deadband_rad', 0.0087)
self.declare_parameter('arm_motion_hold', 0.3)
self.declare_parameter('mission_state_timeout', 2.0)
self.declare_parameter('render_rate_hz', 10.0)
self.declare_parameter('blink_hz', 1.0)
```

- [ ] **Step 3: Read the new parameters**

After the existing `self._color_order = self._parse_color_order(...)` block, add:

```python
self._estop_topic = str(self.get_parameter('estop_topic').value)
self._drive_topic = str(self.get_parameter('drive_topic').value)
self._joint_states_topic = str(self.get_parameter('joint_states_topic').value)
self._drive_timeout = float(self.get_parameter('drive_timeout').value)
self._drive_deadband = float(self.get_parameter('drive_deadband').value)
self._arm_deadband_rad = float(self.get_parameter('arm_deadband_rad').value)
self._arm_motion_hold = float(self.get_parameter('arm_motion_hold').value)
self._mission_state_timeout = float(self.get_parameter('mission_state_timeout').value)
self._render_rate_hz = float(self.get_parameter('render_rate_hz').value)
self._blink_hz = float(self.get_parameter('blink_hz').value)
```

- [ ] **Step 4: Initialize the activity cache**

After the existing `self._last_msg: Optional[MissionState] = None` line, add:

```python
# Activity-layer cached inputs (timestamps are float seconds from _now()).
self._estop_state = False
self._twist_nonzero = False
self._twist_stamp: Optional[float] = None
self._arm_motion_stamp: Optional[float] = None
self._arm_last_pos: dict = {}
self._mission_stamp: Optional[float] = None
```

- [ ] **Step 5: Create the new subscriptions and the render timer**

After the existing mission-state subscription block (`self._sub = self.create_subscription(MissionState, ...)`), add:

```python
# Sensor-style inputs: tolerate a missed frame (re-evaluated every render
# tick) and stay QoS-compatible with reliable OR best-effort publishers.
sensor_qos = QoSProfile(
    depth=1,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
)
# Match estop_bridge's publisher QoS exactly (RELIABLE, VOLATILE, depth 10).
estop_qos = QoSProfile(
    depth=10,
    history=QoSHistoryPolicy.KEEP_LAST,
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.VOLATILE,
)
self._estop_sub = self.create_subscription(
    Bool, self._estop_topic, self._on_estop, estop_qos
)
self._drive_sub = self.create_subscription(
    Twist, self._drive_topic, self._on_cmd_vel, sensor_qos
)
self._joint_sub = self.create_subscription(
    JointState, self._joint_states_topic, self._on_joint_states, sensor_qos
)
self._render_timer = self.create_timer(
    1.0 / max(self._render_rate_hz, 1.0), self._on_render_tick
)
```

- [ ] **Step 6: Replace `_on_state` with a cache-only version**

Find the existing `_on_state` method and replace its whole body so it only caches (the render timer now does all sending):

```python
def _on_state(self, msg: MissionState) -> None:
    self._last_msg = msg
    self._mission_stamp = self._now()
```

- [ ] **Step 7: Add the new input callbacks, `_now`, and the render tick**

Add these methods to `LightStripBridge` (place near `_on_state`):

```python
def _now(self) -> float:
    return self.get_clock().now().nanoseconds * 1e-9

def _on_estop(self, msg: Bool) -> None:
    self._estop_state = bool(msg.data)

def _on_cmd_vel(self, msg: Twist) -> None:
    self._twist_nonzero = self._twist_is_nonzero(msg, self._drive_deadband)
    self._twist_stamp = self._now()

def _on_joint_states(self, msg: JointState) -> None:
    self._note_arm_motion(msg.name, msg.position, self._now())

def _on_render_tick(self) -> None:
    now = self._now()
    mission = self._last_msg if self._last_msg is not None else MissionState()
    rgb, blink = self._decide_style(
        mission,
        estop=self._estop_state,
        mission_fresh=self._mission_fresh(now),
        mode=self._mode,
        manual_rgb=self._manual_rgb,
        arm_moving=self._arm_moving(now),
        base_driving=self._base_driving(now),
    )
    effective = self._effective_rgb(rgb, blink, self._blink_on(now))
    # Dedup: don't re-send a colour already shown or in flight. Solid styles
    # send once; a blink sends on each on/off transition.
    if effective == self._last_rgb or effective == self._pending_rgb:
        return
    self._send_color(effective, reason=f'{"blink " if blink else ""}{rgb}')
```

- [ ] **Step 8: Simplify the manual/auto service handlers to cache-only**

Replace the body of `_on_set_manual` so it validates and updates state but does NOT send (the render tick applies it, with safety still winning via `_decide_style`):

```python
def _on_set_manual(
    self, request: SetNeopixel.Request, response: SetNeopixel.Response
) -> SetNeopixel.Response:
    """HMI -> hold a manual colour; stop following the activity/mission layer.

    Rejected if the LED service is down, so the HMI never renders a manual
    hold the operator can't actually see. An active e-stop / FAULT still wins
    at render time, so a manual colour can't mask a stopped robot.
    """
    if not self._client.service_is_ready():
        self.get_logger().warn(
            f'manual colour rejected: LED service {self._led_service} not ready'
        )
        response.status = False
        return response

    self._mode = 'manual'
    self._manual_rgb = (
        int(request.color.r),
        int(request.color.g),
        int(request.color.b),
    )
    response.status = True
    return response
```

Replace the body of `_on_set_auto`:

```python
def _on_set_auto(
    self, request: Trigger.Request, response: Trigger.Response
) -> Trigger.Response:
    """HMI -> hand colouring back to the automatic activity/mission layer."""
    self._mode = 'auto'
    self._manual_rgb = None
    response.success = True
    response.message = 'auto'
    return response
```

- [ ] **Step 9: Remove the superseded `_desired_rgb` method**

Delete the entire `_desired_rgb` method — its safety/pause/manual precedence now lives in `_decide_style`, and nothing calls it after Steps 6 and 8. Keep everything else, including `_safety_rgb`, `_state_to_rgb`, `_scan_phase_rgb`, `_describe_state`, `_send_color`, `_on_set_color_done`, `_apply_color_order`, `_parse_color_order`, `_poll_service_ready`, and the startup `if self._set_on_startup: self._send_color(OFF, ...)` block (the render timer supersedes that single OFF ~0.1 s later).

- [ ] **Step 10: Ensure message dependencies are declared**

```bash
grep -E "geometry_msgs|sensor_msgs|std_msgs" ~/ros2_ws/src/lupin/lupin_hmi/package.xml
```

If any of `geometry_msgs`, `sensor_msgs`, `std_msgs` is missing, add it next to the other `<exec_depend>` entries in `lupin_hmi/package.xml`:

```xml
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>sensor_msgs</exec_depend>
<exec_depend>std_msgs</exec_depend>
```

- [ ] **Step 11: Run the existing pure tests to confirm nothing broke**

```bash
python -m pytest ~/ros2_ws/src/lupin/lupin_hmi/test/test_light_strip_bridge.py -v
```

Expected: PASS (all tests — the pure functions are unchanged; this confirms the wiring edits didn't break imports or signatures).

- [ ] **Step 12: Commit**

```bash
git add lupin_hmi/lupin_hmi/light_strip_bridge.py lupin_hmi/package.xml
git commit -m "feat(hmi): wire LED activity layer (subs, render timer, cache-only handlers)"
```

---

## Task 5: Launch parameter plumbing

Pass the new params explicitly on the onboard bridge (defaults are correct; explicit makes the sim override obvious) and point the sim bridge at the sim drive topic.

**Files:**
- Modify: `lupin_bringup/launch/onboard.launch.py`
- Modify: (sim) the launch that starts `light_strip_bridge` in simulation, if it exists

- [ ] **Step 1: Add the activity params to the onboard bridge**

In `lupin_bringup/launch/onboard.launch.py`, in the `light_strip_bridge` Node's `parameters=[{...}]` dict (which currently sets `mission_state_topic`, `led_service`, `manual_service`, `auto_service`, `color_order`), add:

```python
            # Activity layer (see 2026-06-08-led-mode-indication spec): live
            # driving/arm/standby indication when no mission owns the strip.
            'estop_topic': '/e_stop_state',
            'drive_topic': '/mirte_base_controller/cmd_vel',
            'joint_states_topic': '/joint_states',
            'blink_hz': 1.0,
```

- [ ] **Step 2: Check whether the sim bringup launches the bridge**

```bash
grep -rn "light_strip_bridge" ~/ros2_ws/src/lupin/lupin_bringup/launch/sim*.launch.py
```

If there are no matches, the sim doesn't run the bridge — skip Step 3.

- [ ] **Step 3 (only if Step 2 found a sim bridge): point it at the sim drive topic**

In the matching sim launch file, set the bridge's `drive_topic` parameter to the sim chassis output:

```python
            'drive_topic': '/mirte_base_controller/cmd_vel_unstamped',
```

- [ ] **Step 4: Sanity-check the launch files parse**

```bash
python -c "import ast; ast.parse(open('/home/oskrt/ros2_ws/src/lupin/lupin_bringup/launch/onboard.launch.py').read()); print('onboard OK')"
```

Expected: `onboard OK` (and no traceback).

- [ ] **Step 5: Commit**

```bash
git add lupin_bringup/launch/onboard.launch.py
# include the sim launch file too if Step 3 modified it
git commit -m "feat(bringup): pass LED activity-layer params to light_strip_bridge"
```

---

## Task 6: Build, full test, and on-desk smoke test

Verify the whole thing builds and behaves before any push. (Per project practice: smoke-test locally, don't punt verification to the reviewer.)

**Files:** none (verification only)

- [ ] **Step 1: Build and run the full bridge test suite**

```bash
cd ~/ros2_ws && colcon build --symlink-install --packages-select lupin_hmi && source install/setup.bash
python -m pytest ~/ros2_ws/src/lupin/lupin_hmi/test/test_light_strip_bridge.py -v
```

Expected: build succeeds; all tests PASS.

- [ ] **Step 2: Launch the bridge standalone with a fake LED service**

In terminal A, stand up a fake LED service so the bridge's calls succeed and we can watch the colours it writes:

```bash
source ~/ros2_ws/install/setup.bash
python3 -c "
import rclpy
from rclpy.node import Node
from mirte_msgs.srv import SetNeopixel
rclpy.init()
n = Node('fake_led')
def cb(req, resp):
    resp.status = True
    print(f'LED <- ({req.color.r},{req.color.g},{req.color.b})', flush=True)
    return resp
n.create_service(SetNeopixel, '/io/leds/leds/set_color', cb)
rclpy.spin(n)
"
```

In terminal B:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run lupin_hmi light_strip_bridge --ros-args -p color_order:=RGB
```

Expected (terminal A): with nothing else publishing, the bridge settles on standby **blue** — `LED <- (0,0,255)` (printed once; dedup suppresses repeats).

- [ ] **Step 3: Verify driving → green blink**

In terminal C:

```bash
source ~/ros2_ws/install/setup.bash
# Stream a non-zero twist at 20 Hz.
ros2 topic pub -r 20 /mirte_base_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}}'
```

Expected (terminal A): alternating `LED <- (0,255,0)` and `LED <- (0,0,0)` at ~1 Hz (green blink). Stop the publisher (Ctrl-C); within ~0.4 s it returns to `LED <- (0,0,255)` (blue standby).

- [ ] **Step 4: Verify arm motion → orange blink**

In terminal C (after stopping Step 3):

```bash
source ~/ros2_ws/install/setup.bash
# Two distinct elbow positions toggled so the position-delta detector fires.
while true; do
  ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState '{name: [elbow_joint], position: [0.0]}'
  ros2 topic pub -1 /joint_states sensor_msgs/msg/JointState '{name: [elbow_joint], position: [0.6]}'
  sleep 0.2
done
```

Expected (terminal A): orange blink — alternating `LED <- (255,128,0)` and `LED <- (0,0,0)`. Stop it; returns to blue within ~0.3 s.

- [ ] **Step 5: Verify e-stop → solid red, overriding motion**

With the driving publisher from Step 3 running again, in terminal D:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub -r 5 /e_stop_state std_msgs/msg/Bool '{data: true}'
```

Expected (terminal A): solid `LED <- (255,0,0)` (red), no blink, even though the base is being driven. Set `data: false` (Ctrl-C and republish false, or stop) → returns to green blink.

- [ ] **Step 6: Stop all terminals**

Ctrl-C terminals A–D. No commit (verification only). If any step revealed a bug, fix it in the relevant task's file, re-run that task's pytest, and re-smoke before continuing.

---

## Task 7: Changelog

**Files:**
- Modify: `lupin/CHANGELOG.md`

- [ ] **Step 1: Add a changelog entry**

Open `~/ros2_ws/src/lupin/CHANGELOG.md` and add an entry under the current unreleased/top section, matching the file's existing style. Example bullet:

```markdown
- LED strip now communicates live activity in every mode: green-blink while
  driving, orange-blink while moving the arm, blue standby, and red on e-stop
  (incl. standalone teleop, via a direct `/e_stop_state` subscription). The
  mission-lifecycle palette is unchanged. (`light_strip_bridge`)
```

- [ ] **Step 2: Commit**

```bash
git add lupin/CHANGELOG.md
git commit -m "docs(hmi): changelog for LED activity-mode indication"
```

---

## Self-review notes (spec coverage)

- Gap-fill composition → Task 1 (`IN_PROGRESS_LIFECYCLES` gate, rows 4 vs 5–7) + `test_in_progress_mission_overrides_driving`.
- Driving green-blink / arm orange-blink / standby blue → Task 1 rows 5–7 + Task 3 blink + Task 6 smoke Steps 3–4.
- E-stop direct, works in teleop → Task 4 `/e_stop_state` sub + Task 1 row 1 + `test_estop_standalone_is_red` + smoke Step 5.
- 1 Hz blink → Task 3 (`blink_hz=1.0`, on/off) + Task 5 launch param.
- Idle reflects resting mission state → Task 1 row 7 (`RESTING_LIFECYCLES`) + `test_idle_after_done_shows_green_solid` / `test_idle_at_ready_is_blue_solid`.
- Wheel-joint filtering → Task 2 `_note_arm_motion` + `test_note_arm_motion_filters_to_arm_joints`.
- Dead-man silence / staleness → Task 2 `_base_driving` + `test_base_driving_requires_recent_nonzero` + smoke Step 3 stop.
- Render chokepoint / BRG remap preserved → Task 4 routes through existing `_send_color`/`_apply_color_order` (untouched).
- Mission freshness (dead orchestrator can't latch) → Task 2 `_mission_fresh` + Task 1 rows 2/4/7 gating.
- Manual override precedence → Task 1 row 3 + `test_manual_hold_outranks_activity` / `test_safety_outranks_manual`; Task 4 cache-only handlers.
- Sim drive-topic override → Task 5 Steps 2–3.
