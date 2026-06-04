"""Battery-percentage normalisation contract.

The battery-low → dock safety net hinges on one comparison (percentage <
threshold). Real power watchers may publish 0..100 instead of the spec'd 0..1,
or NaN when only voltage is known — either silently disables the net. Lock the
normaliser so those readings are handled, not trusted blindly.
"""

import math

from lupin_mission.battery_monitor import normalize_battery_percentage


def test_fraction_passes_through():
    assert normalize_battery_percentage(0.35) == 0.35
    assert normalize_battery_percentage(0.0) == 0.0
    assert normalize_battery_percentage(1.0) == 1.0


def test_percent_scale_is_normalised():
    assert normalize_battery_percentage(35.0) == 0.35
    assert normalize_battery_percentage(100.0) == 1.0


def test_nan_and_inf_are_rejected():
    assert normalize_battery_percentage(float('nan')) is None
    assert normalize_battery_percentage(float('inf')) is None


def test_negative_is_rejected():
    assert normalize_battery_percentage(-1.0) is None
