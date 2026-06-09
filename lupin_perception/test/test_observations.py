"""Unit tests for the passive-perception Observation builder."""

import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose

from lupin_msgs.msg import Observation, SensorReading, TagReading
from lupin_perception.observations import make_tag_reading_observation


def _reading():
    tr = TagReading()
    tr.tag_id = '7'
    sr = SensorReading()
    sr.name, sr.value = 'temperature', 21.5
    tr.readings.append(sr)
    return tr


def test_builds_kind_tag_reading_ok():
    obs = make_tag_reading_observation(
        source='passive_observer', stamp=Time(), tag_reading=_reading(),
        tag_map_pose=None, frame_id='map')
    assert obs.kind == Observation.KIND_TAG_READING
    assert obs.status == Observation.STATUS_OK
    assert obs.source == 'passive_observer'
    assert obs.header.frame_id == 'map'
    assert obs.tag_reading.tag_id == '7'
    assert obs.tag_reading.readings[0].name == 'temperature'


def test_normalises_zero_quaternion_when_position_present():
    pose = Pose()            # all-zero orientation (w==0 == "missing" to the twin)
    pose.position.x = 2.0
    obs = make_tag_reading_observation(
        source='x', stamp=Time(), tag_reading=_reading(),
        tag_map_pose=pose, frame_id='map')
    assert obs.tag_pose_in_map.position.x == pytest.approx(2.0)
    assert obs.tag_pose_in_map.orientation.w == pytest.approx(1.0)
