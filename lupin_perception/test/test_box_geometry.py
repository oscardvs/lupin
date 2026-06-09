"""Unit tests for :mod:`lupin_perception.box_geometry`.

Pure-Python — no rclpy. Poses are duck-typed via SimpleNamespace, like
lupin_mission/test/test_approach.py. A tag facing +X (quaternion rotating
+Z onto +X: 90 deg about Y) gives an in-plane normal of (1, 0), so the box
extends into -X (behind the tag) and its lateral axis runs along +Y.
"""

from __future__ import annotations

import pytest

from types import SimpleNamespace

from lupin_perception.box_geometry import (
    StandardBox,
    FlowerPointData,
    MapBox,
    box_from_map_box,
    box_from_tag,
    bin_detections,
    lateral_fraction,
)


def _pose(x, y, *, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=0.0),
        orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw),
    )


# Tag at (2, 0) facing +X (normal points +X, into the aisle).
FACING_X = _pose(2.0, 0.0, qy=0.70710678, qw=0.70710678)
BOX = StandardBox(width=0.80, depth=0.40)


def test_box_frame_normal_and_lateral():
    geom = box_from_tag(FACING_X, BOX)
    assert geom is not None
    assert geom.normal[0] == pytest.approx(1.0, abs=1e-6)
    assert geom.normal[1] == pytest.approx(0.0, abs=1e-6)
    # lateral = normal rotated +90 deg -> (0, 1)
    assert geom.lateral[0] == pytest.approx(0.0, abs=1e-6)
    assert geom.lateral[1] == pytest.approx(1.0, abs=1e-6)


def test_place_center_is_into_the_bed_not_on_the_tag():
    geom = box_from_tag(FACING_X, BOX)
    # f=0 (lateral centre), d=0.5 (mid-depth): tag - n*(0.5*depth) + l*0
    x, y = geom.place(0.0, 0.5)
    assert x == pytest.approx(2.0 - 0.5 * 0.40, abs=1e-6)   # 1.80, behind tag
    assert y == pytest.approx(0.0, abs=1e-6)
    # Crucially NOT the tag position -> fixes the "flowers on the tag" bug.
    assert (x, y) != (2.0, 0.0)


def test_place_lateral_extremes_span_box_width():
    geom = box_from_tag(FACING_X, BOX)
    # f=+1 at front (d=0): +width/2 along +Y
    xr, yr = geom.place(1.0, 0.0)
    assert yr == pytest.approx(0.40, abs=1e-6)              # +0.80/2
    xl, yl = geom.place(-1.0, 0.0)
    assert yl == pytest.approx(-0.40, abs=1e-6)
    assert xr == pytest.approx(2.0, abs=1e-6)               # d=0 -> on front face


def test_footprint_four_corners():
    geom = box_from_tag(FACING_X, BOX)
    fp = geom.footprint()
    assert len(fp) == 4
    xs = [p[0] for p in fp]
    ys = [p[1] for p in fp]
    # front face at x=2.0, back face at x=1.60 (2.0 - depth)
    assert min(xs) == pytest.approx(1.60, abs=1e-6)
    assert max(xs) == pytest.approx(2.0, abs=1e-6)
    assert min(ys) == pytest.approx(-0.40, abs=1e-6)
    assert max(ys) == pytest.approx(0.40, abs=1e-6)


def test_map_box_geometry_uses_box_center_and_yaw():
    geom = box_from_map_box(MapBox(
        box_id='base_1', x=1.0, y=2.0, yaw=0.0,
        width=0.80, depth=0.40, height=0.35,
    ))
    assert geom.normal == pytest.approx((1.0, 0.0), abs=1e-6)
    # centre is (1,2), front face is +depth/2 along the normal.
    assert geom.origin == pytest.approx((1.20, 2.0), abs=1e-6)
    x, y = geom.place(0.0, 0.5)
    assert x == pytest.approx(1.0, abs=1e-6)
    assert y == pytest.approx(2.0, abs=1e-6)


def test_lateral_offset_shifts_origin_along_lateral():
    geom = box_from_tag(FACING_X, StandardBox(width=0.80, depth=0.40,
                                              tag_lateral_offset=0.1))
    x, y = geom.place(0.0, 0.0)
    assert y == pytest.approx(0.1, abs=1e-6)


def test_degenerate_normal_returns_none():
    # Identity orientation -> tag +Z is map +Z -> XY projection is (0,0).
    assert box_from_tag(_pose(0.0, 0.0), BOX) is None


def test_degenerate_normal_falls_back_to_robot_vector():
    # Identity orientation but robot_xy given: approach from the robot side.
    geom = box_from_tag(_pose(2.0, 0.0), BOX, robot_xy=(0.0, 0.0))
    assert geom is not None
    # robot is west of the tag -> normal points -X (toward robot/aisle).
    assert geom.normal[0] == pytest.approx(-1.0, abs=1e-6)


def test_lateral_fraction_static_pan_uses_bbox():
    # pan == centre: f is driven purely by the normalized bbox centre-x,
    # scaled by camera_half_fov / (pan_half_span + camera_half_fov).
    f = lateral_fraction(0.0, 1.0, pan_half_span=0.5, camera_half_fov=0.5)
    assert f == pytest.approx(0.5, abs=1e-6)
    assert lateral_fraction(0.0, 0.0, pan_half_span=0.5, camera_half_fov=0.5) == 0.0


def test_lateral_fraction_pan_extends_range_and_clamps():
    # Full pan + same-side bbox saturates to +1 (clamped).
    f = lateral_fraction(0.5, 1.0, pan_half_span=0.5, camera_half_fov=0.5)
    assert f == pytest.approx(1.0, abs=1e-6)
    assert lateral_fraction(10.0, 0.0) == pytest.approx(1.0, abs=1e-6)
    assert lateral_fraction(-10.0, 0.0) == pytest.approx(-1.0, abs=1e-6)


def test_bin_detections_one_point_per_occupied_column():
    geom = box_from_tag(FACING_X, BOX)
    # Two clusters: left (f=-0.9) red, right (f=0.9) white -> two columns.
    dets = [
        (-0.9, 'tulip_red', 0.9),
        (-0.85, 'tulip_red', 0.7),
        (0.9, 'tulip_white', 0.8),
    ]
    flowers = bin_detections(dets, geom, lateral_columns=7)
    assert len(flowers) == 2
    assert all(isinstance(f, FlowerPointData) for f in flowers)
    species = sorted(f.species for f in flowers)
    assert species == ['tulip_red', 'tulip_white']
    # placed inside the box width and behind the front face
    for f in flowers:
        assert -0.40 - 1e-6 <= f.y <= 0.40 + 1e-6
        assert 1.60 - 1e-6 <= f.x <= 2.0 + 1e-6
        assert not f.anomaly


def test_bin_detections_dominant_species_and_anomaly_per_column():
    geom = box_from_tag(FACING_X, BOX)
    # Same column (f near 0): pink wins on confidence; a bug sets anomaly.
    dets = [
        (0.0, 'tulip_white', 0.4),
        (0.02, 'tulip_pink', 0.9),
        (0.01, 'bug', 0.8),
    ]
    flowers = bin_detections(dets, geom, lateral_columns=7, anomaly_class='bug')
    assert len(flowers) == 1
    assert flowers[0].species == 'tulip_pink'
    assert flowers[0].anomaly is True


def test_bin_detections_none_geometry_is_empty():
    assert bin_detections([(0.0, 'tulip_red', 0.9)], None) == []


def test_bin_detections_empty_dets_is_empty():
    geom = box_from_tag(FACING_X, BOX)
    assert bin_detections([], geom) == []


def test_bin_detections_deterministic_depth_jitter():
    geom = box_from_tag(FACING_X, BOX)
    dets = [(-0.9, 'tulip_red', 0.9), (0.9, 'tulip_white', 0.9)]
    a = bin_detections(dets, geom)
    b = bin_detections(dets, geom)
    assert [(f.x, f.y) for f in a] == [(f.x, f.y) for f in b]  # no RNG


# ── lateral_fraction clamp flag ─────────────────────────────────────────────
# The aggregator stores the UNCLAMPED fraction so an over-reach bearing (a bloom
# pointing past the bench end) survives to the spatial gate instead of being
# squashed to the box edge. bin_detections re-clamps for placement.


def test_lateral_fraction_unclamped_preserves_overreach():
    # pan well past the sweep -> bearing/span = 1.6. Default clamps to 1.0;
    # clamp=False keeps 1.6 so the gate can see it is off the bench.
    assert lateral_fraction(1.6, 0.0) == pytest.approx(1.0, abs=1e-6)
    assert lateral_fraction(1.6, 0.0, clamp=False) == pytest.approx(1.6, abs=1e-6)
    assert lateral_fraction(-1.6, 0.0, clamp=False) == pytest.approx(-1.6, abs=1e-6)


# ── BoxGeometry.contains — spatial membership test ──────────────────────────


def test_contains_interior_point():
    geom = box_from_tag(FACING_X, BOX)
    # Box centre (f=0, d=0.5) is inside.
    x, y = geom.place(0.0, 0.5)
    assert geom.contains(x, y) is True


def test_contains_rejects_point_past_bench_end():
    geom = box_from_tag(FACING_X, BOX)
    # f=1.5 -> 0.5*half_w (0.20 m) beyond the +lateral edge: outside.
    x, y = geom.place(1.5, 0.5)
    assert geom.contains(x, y) is False
    # A small margin does not rescue a point that is 0.20 m out.
    assert geom.contains(x, y, margin_m=0.10) is False


def test_contains_margin_admits_near_edge_point():
    geom = box_from_tag(FACING_X, BOX)
    # f=1.1 -> 0.04 m beyond the edge (half_w=0.40): outside at margin 0,
    # inside with a 0.10 m margin.
    x, y = geom.place(1.1, 0.5)
    assert geom.contains(x, y) is False
    assert geom.contains(x, y, margin_m=0.10) is True


def test_contains_rejects_point_behind_back_face():
    geom = box_from_tag(FACING_X, BOX)
    # d=1.6 -> past the back face (depth=0.40): outside.
    x, y = geom.place(0.0, 1.6)
    assert geom.contains(x, y) is False
