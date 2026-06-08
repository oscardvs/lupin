"""Mission-state → LED colour mapping for the status light strip.

Locks the operator-facing contract: every lifecycle maps to a distinct,
legible colour, and battery-low gets its OWN signal rather than hiding inside
the normal RETURNING/ORANGE dock-return colour.
"""

from lupin_hmi.light_strip_bridge import (
    AMBER,
    BATTERY_LOW,
    BLUE,
    GREEN,
    LightStripBridge,
    OFF,
    ORANGE,
    RED,
    SPRING,
)
from lupin_msgs.msg import MissionState


def _rgb(**fields):
    # Exercise the pure colour map without standing up a ROS node.
    bridge = object.__new__(LightStripBridge)
    bridge._unknown_state_off = False
    msg = MissionState()
    for k, v in fields.items():
        setattr(msg, k, v)
    return bridge._state_to_rgb(msg)


def test_battery_low_has_its_own_colour():
    # A low-battery return reads differently from a normal dock return.
    assert _rgb(lifecycle_state='RETURNING', battery_low=True) == BATTERY_LOW
    assert _rgb(lifecycle_state='RETURNING', battery_low=False) == ORANGE


def test_battery_low_colour_is_distinct():
    # Must not collide with the colours an operator would read as something
    # else: dock return (ORANGE), fault/e-stop (RED), or a hold (AMBER).
    assert BATTERY_LOW not in (ORANGE, RED, AMBER)


def test_pause_outranks_battery_low():
    # A held robot stays AMBER even when the battery is low — the pause is the
    # more urgent thing to convey, and it's checked first.
    assert _rgb(lifecycle_state='RETURNING', battery_low=True, paused=True) == AMBER


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
