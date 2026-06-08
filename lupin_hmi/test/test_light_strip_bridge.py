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
