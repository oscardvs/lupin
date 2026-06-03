"""Unit tests for the dock-return failure policy (pure, no ROS)."""

from lupin_mission.return_policy import decide_failed_dock


def test_battery_dock_retries_until_max():
    assert decide_failed_dock(
        battery_low=True, docked_for_battery=True, resumable=True,
        retry_count=0, retry_max=3) == 'retry'
    assert decide_failed_dock(
        battery_low=True, docked_for_battery=True, resumable=True,
        retry_count=2, retry_max=3) == 'retry'


def test_battery_dock_pauses_after_max():
    assert decide_failed_dock(
        battery_low=True, docked_for_battery=True, resumable=True,
        retry_count=3, retry_max=3) == 'pause'


def test_non_battery_dock_failure_is_done():
    assert decide_failed_dock(
        battery_low=False, docked_for_battery=False, resumable=False,
        retry_count=0, retry_max=3) == 'done'
    assert decide_failed_dock(
        battery_low=True, docked_for_battery=True, resumable=False,
        retry_count=0, retry_max=3) == 'done'
