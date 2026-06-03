"""Unit tests for mission-event severity classification (pure, no ROS)."""

from lupin_mission.events import (
    SEVERITY_INFO, SEVERITY_WARN, SEVERITY_ERROR, classify_event,
)


def test_empty_is_info_no_label():
    assert classify_event('') == (SEVERITY_INFO, '')
    assert classify_event(None) == (SEVERITY_INFO, '')


def test_known_notices_are_warn_and_friendly():
    assert classify_event('battery_low') == (SEVERITY_WARN, 'battery low')
    assert classify_event('dock_unreachable') == (SEVERITY_WARN, 'dock unreachable — paused')
    assert classify_event('exploration_timeout') == (SEVERITY_WARN, 'exploration timed out')


def test_nav_status_prefix_is_warn():
    sev, label = classify_event('nav_status_6')
    assert sev == SEVERITY_WARN
    assert label == 'navigation retry'


def test_startup_failures_are_error():
    assert classify_event('localization_failed (0.42)')[0] == SEVERITY_ERROR
    assert classify_event('dependency_timeout: amcl')[0] == SEVERITY_ERROR


def test_unknown_code_is_warn_and_passes_through():
    assert classify_event('something_new') == (SEVERITY_WARN, 'something_new')
