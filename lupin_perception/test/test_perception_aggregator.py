"""Unit tests for perception_aggregator fusion logic.

The TF lookup is monkeypatched so these run on a desktop with no live TF tree
or cameras — they exercise discovery debounce, YOLO→tag species fusion, the
bug anomaly flag, and the confirm_tag service.
"""

import json

import pytest
import rclpy
from geometry_msgs.msg import Pose
from std_msgs.msg import String

from lupin_msgs.msg import MissionState
from lupin_msgs.srv import ConfirmTag
from lupin_perception.perception_aggregator import PerceptionAggregator


@pytest.fixture
def node():
    rclpy.init()
    n = PerceptionAggregator()
    # No TF tree in tests: every detected tag localises to a fixed map pose.
    fake = Pose()
    fake.position.x, fake.position.y = 1.0, 2.0
    fake.orientation.w = 1.0
    n._lookup_tag_pose = lambda tag_id: fake  # type: ignore[method-assign]
    # Capture emitted observations instead of publishing to DDS.
    n.emitted = []
    n._obs_pub.publish = n.emitted.append  # type: ignore[method-assign]
    yield n
    n.destroy_node()
    rclpy.shutdown()


def _tag_frame(*ids_dists):
    return String(data=json.dumps([{'id': i, 'dist': d} for i, d in ids_dists]))


def _yolo(*cls_confs):
    return String(data=json.dumps(
        [{'class': c, 'confidence': f, 'bbox_xyxy': [0, 0, 1, 1]} for c, f in cls_confs]
    ))


def _discovered_ids(node):
    node._publish_discovered_last = None
    msg = None

    def grab(m):
        nonlocal msg
        msg = m
    node._discovered_pub.publish = grab  # type: ignore[method-assign]
    node._publish_discovered()
    return {t.tag_id for t in (msg.tags if msg else [])}


def test_discovery_debounce(node):
    # min_sightings defaults to 3 — below that the tag is a candidate, not
    # published as discovered.
    node._on_tag_detections(_tag_frame((5, 1.0)))
    node._on_tag_detections(_tag_frame((5, 1.0)))
    assert _discovered_ids(node) == set()
    node._on_tag_detections(_tag_frame((5, 1.0)))
    assert _discovered_ids(node) == {'5'}


def test_far_tag_not_pose_committed(node):
    # max_tag_distance_m defaults to 2.5 — a far detection is counted in the
    # frame but not pose-localised, so it never reaches discovered.
    for _ in range(5):
        node._on_tag_detections(_tag_frame((9, 9.0)))
    assert _discovered_ids(node) == set()


def _scan(node, tag_id):
    ms = MissionState()
    ms.lifecycle_state = 'MONITORING'
    ms.mission_phase = 'SCANNING'
    ms.current_target = tag_id
    node._mission_state = ms
    node._mission_state_seen = True


def test_species_fusion_emits_flower(node):
    for _ in range(3):
        node._on_tag_detections(_tag_frame((5, 0.8)))
    _scan(node, '5')
    node._on_yolo_detections(_yolo((0, 0.9)))  # class 0 == tulip_red
    rec = node._registry['5']
    assert rec.species == 'tulip_red'
    assert rec.species_confidence == pytest.approx(0.9)
    assert rec.anomaly is False
    assert len(node.emitted) == 1
    obs = node.emitted[0]
    assert obs.kind == obs.KIND_FLOWER
    assert obs.flower.tag_id == '5'
    assert obs.flower.species == 'tulip_red'
    assert obs.flower.anomaly is False


def test_bug_sets_anomaly(node):
    for _ in range(3):
        node._on_tag_detections(_tag_frame((5, 0.8)))
    _scan(node, '5')
    node._on_yolo_detections(_yolo((0, 0.9), (3, 0.8)))  # tulip_red + bug
    rec = node._registry['5']
    assert rec.species == 'tulip_red'
    assert rec.anomaly is True


def test_no_attribution_when_not_scanning(node):
    # Mission running but EXPLORING (not SCANNING) → no flower attribution,
    # avoiding drive-by misassociation.
    for _ in range(3):
        node._on_tag_detections(_tag_frame((5, 0.8)))
    ms = MissionState()
    ms.lifecycle_state = 'EXPLORING'
    node._mission_state = ms
    node._mission_state_seen = True
    node._on_yolo_detections(_yolo((0, 0.9)))
    assert node._registry['5'].species == ''
    assert node.emitted == []


def test_confirm_tag_service(node):
    req, res = ConfirmTag.Request(), ConfirmTag.Response()
    req.expected_tag_id = '5'
    res = node._handle_confirm_tag(req, res)
    assert res.detected is False  # not discovered yet

    for _ in range(3):
        node._on_tag_detections(_tag_frame((5, 0.8)))
    res = ConfirmTag.Response()
    res = node._handle_confirm_tag(req, res)
    assert res.detected is True
    assert res.tag_pose_in_map.position.x == pytest.approx(1.0)
