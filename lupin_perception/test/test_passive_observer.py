"""Unit tests for the passive_observer node decision logic.

The bridge service is not spun up — tests drive the seams (_due_tags,
_on_reading) directly and capture published observations.
"""

import pytest
import rclpy
from geometry_msgs.msg import Pose

from lupin_msgs.msg import (
    DiscoveredTag,
    DiscoveredTags,
    MissionState,
    Observation,
    SensorReading,
    TagReading,
)
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
