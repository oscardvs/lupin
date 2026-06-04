"""Polarity contract for the emergency-button → /e_stop_state bridge.

The mapping is the load-bearing, hardware-verifiable part of estop_bridge:
which raw button value means "e-stop engaged". Lock it so a refactor can't
silently invert the safety polarity.
"""

from lupin_hmi.estop_bridge import button_is_engaged


def test_default_polarity_pressed_is_value_true():
    # engaged_value=True: a True reading means pressed/engaged.
    assert button_is_engaged(True, True) is True
    assert button_is_engaged(False, True) is False


def test_inverted_polarity_pressed_is_value_false():
    # engaged_value=False (normally-closed / active-low button).
    assert button_is_engaged(False, False) is True
    assert button_is_engaged(True, False) is False
