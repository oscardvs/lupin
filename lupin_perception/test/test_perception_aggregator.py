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

from lupin_msgs.msg import MissionState, Observation
from sensor_msgs.msg import JointState
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


def _forward_facing_pose():
    # Tag at (1,2) facing +X so box_geometry yields a non-degenerate normal
    # (the default fixture's identity orientation would be degenerate).
    p = Pose()
    p.position.x, p.position.y = 1.0, 2.0
    p.orientation.y = 0.70710678
    p.orientation.w = 0.70710678
    return p


def _scanning(node, tag_id):
    ms = MissionState()
    ms.lifecycle_state = 'MONITORING'
    ms.mission_phase = 'MONITORING_SCANNING'
    ms.current_target = tag_id
    ms.mission_id = 'm-test'
    node._on_mission_state(ms)


def _joints(node, pan):
    js = JointState()
    js.name = ['shoulder_pan_joint', 'shoulder_lift_joint']
    js.position = [float(pan), 0.0]
    node._on_joint_states(js)


def _yolo_cx(*cls_conf_cx):
    # each arg: (class, confidence, bbox_centre_x_px) in a 640-wide image
    return String(data=json.dumps([
        {'class': c, 'confidence': f, 'bbox_xyxy': [cx - 5, 100, cx + 5, 140]}
        for (c, f, cx) in cls_conf_cx
    ]))


def test_flowers_localized_inside_box_not_on_tag(node):
    node._lookup_tag_pose = lambda tag_id: _forward_facing_pose()
    # Discover tag 8 (>= min_sightings).
    for _ in range(3):
        node._on_tag_detections(_tag_frame((8, 1.0)))
    _scanning(node, '8')
    _joints(node, 0.0)
    # Two lateral clusters in image-x: left (cx=80) red, right (cx=560) white.
    node._on_yolo_detections(_yolo_cx((0, 0.9, 80), (1, 0.85, 560)))
    node._on_yolo_detections(_yolo_cx((0, 0.92, 90), (1, 0.88, 550)))

    flower_obs = [o for o in node.emitted
                  if o.kind == Observation.KIND_FLOWER and o.flower.flowers]
    assert flower_obs, 'expected a KIND_FLOWER Observation carrying flowers[]'
    obs = flower_obs[-1]
    # Box footprint emitted as a 4-corner polygon (on the FlowerObservation).
    assert len(obs.flower.box_footprint.points) == 4
    fps = obs.flower.flowers
    assert len(fps) >= 2, 'two lateral clusters -> at least two columns'
    # No bloom is pinned on the tag (1.0, 2.0).
    for fp in fps:
        assert (round(fp.position.x, 3), round(fp.position.y, 3)) != (1.0, 2.0)
    species = {fp.species for fp in fps}
    assert 'tulip_red' in species and 'tulip_white' in species


def test_bug_sets_anomaly_on_localized_flower(node):
    node._lookup_tag_pose = lambda tag_id: _forward_facing_pose()
    for _ in range(3):
        node._on_tag_detections(_tag_frame((8, 1.0)))
    _scanning(node, '8')
    _joints(node, 0.0)
    node._on_yolo_detections(_yolo_cx((2, 0.9, 320), (3, 0.8, 320)))  # pink + bug, centre
    flower_obs = [o for o in node.emitted
                  if o.kind == Observation.KIND_FLOWER and o.flower.flowers]
    assert flower_obs
    assert any(fp.anomaly for fp in flower_obs[-1].flower.flowers)


# ── known-layout box footprint (from tag_locations.json) ────────────────────
# Tags 6 and 29 both sit on Table1 = {x0:1.5, y0:7.3, x1:2.6, y1:7.55} in the
# bundled greenhouse_sim layout the node auto-loads.
_TABLE1_CORNERS = {(1.5, 7.3), (2.6, 7.3), (2.6, 7.55), (1.5, 7.55)}
_TAG_JSON = {'6': (1.5, 7.3), '29': (2.4, 7.3)}


def _pose_at(x, y):
    # Forward-facing (90° about Y) so the LEGACY box_from_tag path would build a
    # (wrong) 0.80×0.40 box here — the assertions below pin the real rectangle.
    p = Pose()
    p.position.x, p.position.y = float(x), float(y)
    p.orientation.y = 0.70710678
    p.orientation.w = 0.70710678
    return p


def test_box_footprint_is_the_real_table_rectangle(node):
    # Map frame == JSON frame: each tag localises to its JSON (x, y).
    node._lookup_tag_pose = lambda tag_id: _pose_at(*_TAG_JSON[tag_id])
    for _ in range(3):
        node._on_tag_detections(_tag_frame((6, 0.8), (29, 0.8)))
    _scanning(node, '6')
    node._on_yolo_detections(_yolo((0, 0.9)))

    flower_obs = [o for o in node.emitted if o.kind == Observation.KIND_FLOWER]
    assert flower_obs, 'expected a KIND_FLOWER Observation'
    pts = flower_obs[-1].flower.box_footprint.points
    assert len(pts) == 4
    got = {(round(p.x, 4), round(p.y, 4)) for p in pts}
    assert got == _TABLE1_CORNERS  # the real bench, not a 0.80×0.40 synthesised box


# ── spatial box-membership gate (fix/flower-tag-association) ────────────────
# With two layout tags (6 & 29 on Table1) registered, the gate projects each
# detection into the bench rectangle and drops the ones whose bearing points
# off the bench, instead of clamping them onto current_target's edge column.


def test_flower_outside_box_is_not_attributed(node):
    # (i) A bloom whose pan/bearing points past the bench end must NOT be
    # attributed to current_target — it is dropped, not clamped onto the edge.
    node._lookup_tag_pose = lambda tag_id: _pose_at(*_TAG_JSON[tag_id])
    for _ in range(3):
        node._on_tag_detections(_tag_frame((6, 0.8), (29, 0.8)))  # 2 tags -> registered
    _scanning(node, '6')
    _joints(node, 1.6)  # pan well past the ±0.5 sweep -> off-bench bearing
    node._on_yolo_detections(_yolo_cx((0, 0.95, 320)))  # centred bbox, tulip_red
    rec = node._registry['6']
    assert rec.species == ''
    assert rec.anomaly is False
    flower_obs = [o for o in node.emitted
                  if o.kind == Observation.KIND_FLOWER and o.flower.flowers]
    assert flower_obs == []


def test_in_box_flower_still_attaches_species_and_anomaly(node):
    # (ii) With the gate active, an in-bench bloom still attaches species AND
    # the bug anomaly flag and emits a located flower.
    node._lookup_tag_pose = lambda tag_id: _pose_at(*_TAG_JSON[tag_id])
    for _ in range(3):
        node._on_tag_detections(_tag_frame((6, 0.8), (29, 0.8)))
    _scanning(node, '6')
    _joints(node, 0.0)
    node._on_yolo_detections(_yolo_cx((0, 0.92, 320), (3, 0.8, 320)))  # red + bug, centre
    rec = node._registry['6']
    assert rec.species == 'tulip_red'
    assert rec.anomaly is True
    flower_obs = [o for o in node.emitted
                  if o.kind == Observation.KIND_FLOWER and o.flower.flowers]
    assert flower_obs
    assert any(fp.anomaly for fp in flower_obs[-1].flower.flowers)


def test_single_tag_fallback_keeps_temporal_attribution(node):
    # (iii) Only ONE layout tag registered -> no JSON->map fit (identity /
    # < 2 tags) -> gate disabled, temporal path preserved: even the off-bench
    # bearing is still attributed, exactly as before this change.
    node._lookup_tag_pose = lambda tag_id: _pose_at(*_TAG_JSON['6'])
    for _ in range(3):
        node._on_tag_detections(_tag_frame((6, 0.8)))  # single tag
    _scanning(node, '6')
    _joints(node, 1.6)  # would be dropped if the gate were active
    node._on_yolo_detections(_yolo_cx((0, 0.95, 320)))
    assert node._registry['6'].species == 'tulip_red'
    flower_obs = [o for o in node.emitted
                  if o.kind == Observation.KIND_FLOWER and o.flower.flowers]
    assert flower_obs


def test_box_gate_param_off_restores_temporal_path(node):
    # The gate is behind a parameter (default ON). With it off, an off-bench
    # bearing is attributed again even with two tags registered.
    node._flower_box_gate = False
    node._lookup_tag_pose = lambda tag_id: _pose_at(*_TAG_JSON[tag_id])
    for _ in range(3):
        node._on_tag_detections(_tag_frame((6, 0.8), (29, 0.8)))
    _scanning(node, '6')
    _joints(node, 1.6)
    node._on_yolo_detections(_yolo_cx((0, 0.95, 320)))
    assert node._registry['6'].species == 'tulip_red'


def test_box_footprint_follows_a_registered_layout(node):
    # Map frame = JSON rotated +90° then translated by (10, -5). Registration
    # must recover it from the two tag correspondences and place the box there.
    tx, ty = 10.0, -5.0

    def to_map(x, y):
        return (-y + tx, x + ty)  # +90° rotation: (x, y) -> (-y, x)

    node._lookup_tag_pose = lambda tag_id: _pose_at(*to_map(*_TAG_JSON[tag_id]))
    for _ in range(3):
        node._on_tag_detections(_tag_frame((6, 0.8), (29, 0.8)))
    _scanning(node, '6')
    node._on_yolo_detections(_yolo((0, 0.9)))

    flower_obs = [o for o in node.emitted if o.kind == Observation.KIND_FLOWER]
    assert flower_obs
    pts = flower_obs[-1].flower.box_footprint.points
    got = sorted((round(p.x, 4), round(p.y, 4)) for p in pts)
    want = sorted((round(mx, 4), round(my, 4))
                  for (jx, jy) in _TABLE1_CORNERS
                  for (mx, my) in [to_map(jx, jy)])
    assert got == want
