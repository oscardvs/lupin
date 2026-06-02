"""Unit tests for MonitoringMission (the continuous re-scan loop model)."""

from __future__ import annotations

from types import SimpleNamespace

from lupin_msgs.msg import Observation, SensorReading, TagReading

from lupin_mission.exploration_mission import ExplorationMission, MonitoringMission


def _pose(x, y):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    )


def _mission(ids=('3', '1', '2')):
    discovered = {i: _pose(float(n), 0.0) for n, i in enumerate(ids)}
    return MonitoringMission(
        mission_id='m', discovered=discovered,
        nav_max_attempts=2, approach_yaw=0.0, standoff_m=0.5,
    )


def test_tags_sorted_and_cursor_starts_at_first():
    m = _mission(('3', '1', '2'))
    assert m.current_tag_id() == '1'  # sorted order
    assert not m.is_complete()


def test_cursor_wraps_and_never_completes():
    m = _mission(('1', '2', '3'))
    seen = [m.current_tag_id()]
    for _ in range(5):
        m.advance()
        seen.append(m.current_tag_id())
        assert not m.is_complete()
    # 1,2,3,1,2,3 — wrapped twice, cycle counter bumped.
    assert seen == ['1', '2', '3', '1', '2', '3']
    assert m.cycles == 1


def test_empty_discovered_is_complete():
    m = MonitoringMission(
        mission_id='m', discovered={}, nav_max_attempts=1,
        approach_yaw=0.0, standoff_m=0.5,
    )
    assert m.is_complete()
    assert m.current_tag_id() == ''


def test_fresh_result_each_visit_and_tally():
    m = _mission(('1', '2'))
    # Visit tag 1: nav attempt + scan ok.
    m.register_nav_attempt()
    assert m.current_result().nav_attempts == 1
    tr = TagReading(tag_id='1')
    tr.readings.append(SensorReading(name='temperature', value=21.0))
    m.mark_scan_ok(tr)
    m.advance()
    # Tag 2 starts with a fresh, zeroed result.
    assert m.current_tag_id() == '2'
    assert m.current_result().nav_attempts == 0
    assert m.current_result().status == Observation.STATUS_OK
    assert not m.current_result().closed
    # The completed leg was tallied.
    assert m.counters()['completed'] == 1
    assert m.counters()['total'] == 2


def test_pose_for_returns_discovered_pose():
    m = _mission(('1',))
    assert m.pose_for('1').position.x == 0.0
    assert m.pose_for('missing') is None


def test_exploration_is_complete_at_goal():
    e = ExplorationMission(mission_id='m', discovery_goal=2)
    assert not e.is_complete()
    e.update_discovered({'1': _pose(0, 0)})
    assert not e.is_complete()
    e.update_discovered({'1': _pose(0, 0), '2': _pose(1, 0)})
    assert e.is_complete()
    assert e.counters() == {
        'total': 2, 'completed': 2, 'failed': 0, 'unreachable': 0, 'skipped': 0,
    }
