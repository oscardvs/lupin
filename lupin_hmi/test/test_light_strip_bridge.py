"""Mission-state → LED colour mapping for the status light strip.

Locks the operator-facing contract: every lifecycle maps to a distinct,
legible colour, and battery-low gets its OWN signal rather than hiding inside
the normal RETURNING/ORANGE dock-return colour.
"""

from lupin_hmi.light_strip_bridge import (
    AMBER,
    BATTERY_LOW,
    LightStripBridge,
    ORANGE,
    RED,
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
