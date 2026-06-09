"""Unit tests for the known-layout box geometry + 2D registration added to
:mod:`lupin_perception.box_geometry`.

These cover the planter-box-from-tag_locations.json path: a rigid 2D fit that
registers the known greenhouse layout (JSON frame) onto the live map frame from
detected tag positions, and a BoxGeometry built from a table rectangle's map-frame
corners. Pure-Python — no rclpy, like test_box_geometry.py.
"""

from __future__ import annotations

import math

import pytest

from lupin_perception.box_geometry import (
    IDENTITY_2D,
    Transform2D,
    box_geometry_from_corners,
    nearest_table_rect,
    solve_rigid_2d,
    table_rect_corners,
)


def _pt_set(points, n=6):
    return sorted((round(x, n), round(y, n)) for x, y in points)


# ── Transform2D ────────────────────────────────────────────────────────────


def test_identity_transform_is_a_noop():
    assert IDENTITY_2D.apply(3.0, -4.0) == pytest.approx((3.0, -4.0))


def test_transform_applies_rotation_then_translation():
    theta = math.radians(90.0)
    tf = Transform2D(cos=math.cos(theta), sin=math.sin(theta), tx=1.0, ty=2.0)
    # (1, 0) rotated +90° -> (0, 1), then + (1, 2) -> (1, 3)
    assert tf.apply(1.0, 0.0) == pytest.approx((1.0, 3.0))


# ── solve_rigid_2d ─────────────────────────────────────────────────────────


def test_solve_rigid_2d_empty_is_identity():
    tf = solve_rigid_2d([], [])
    assert (tf.cos, tf.sin, tf.tx, tf.ty) == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_solve_rigid_2d_identity_when_src_equals_dst():
    pts = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    tf = solve_rigid_2d(pts, pts)
    assert (tf.cos, tf.sin, tf.tx, tf.ty) == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_solve_rigid_2d_single_point_is_translation_only():
    tf = solve_rigid_2d([(2.0, 3.0)], [(5.0, 1.0)])
    assert (tf.cos, tf.sin) == pytest.approx((1.0, 0.0))
    assert tf.apply(2.0, 3.0) == pytest.approx((5.0, 1.0))


def test_solve_rigid_2d_pure_translation():
    src = [(0.0, 0.0), (1.0, 0.0), (0.0, 1.0)]
    dst = [(x + 3.0, y - 2.0) for x, y in src]
    tf = solve_rigid_2d(src, dst)
    assert (tf.cos, tf.sin, tf.tx, tf.ty) == pytest.approx((1.0, 0.0, 3.0, -2.0))


def test_solve_rigid_2d_recovers_rotation_and_translation():
    theta = math.radians(30.0)
    c, s = math.cos(theta), math.sin(theta)
    tx, ty = 1.5, -0.7
    src = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)]
    dst = [(c * x - s * y + tx, s * x + c * y + ty) for x, y in src]
    tf = solve_rigid_2d(src, dst)
    assert (tf.cos, tf.sin) == pytest.approx((c, s))
    for (sx, sy), (dx, dy) in zip(src, dst):
        assert tf.apply(sx, sy) == pytest.approx((dx, dy))


# ── table_rect_corners ─────────────────────────────────────────────────────


def test_table_rect_corners_are_ccw_from_min_corner():
    rect = {"x0": 1.5, "y0": 7.3, "x1": 2.6, "y1": 7.55}
    # Order is load-bearing: box_geometry_from_corners derives edges from
    # c0->c1 and c0->c3, so it must be (lo,lo),(hi,lo),(hi,hi),(lo,hi).
    assert table_rect_corners(rect) == [
        (1.5, 7.3), (2.6, 7.3), (2.6, 7.55), (1.5, 7.55),
    ]


def test_table_rect_corners_normalises_swapped_bounds():
    rect = {"x0": 2.6, "y0": 7.55, "x1": 1.5, "y1": 7.3}
    assert table_rect_corners(rect) == [
        (1.5, 7.3), (2.6, 7.3), (2.6, 7.55), (1.5, 7.55),
    ]


# ── nearest_table_rect ─────────────────────────────────────────────────────


_TABLES = {
    "Table1": {"x0": 1.5, "y0": 7.3, "x1": 2.6, "y1": 7.55},   # centre ~ (2.05, 7.425)
    "Table4": {"x0": 1.3, "y0": 4.3, "x1": 1.55, "y1": 5.4},   # centre ~ (1.425, 4.85)
}


def test_nearest_table_rect_picks_closest_centre():
    assert nearest_table_rect((1.4, 4.9), _TABLES) is _TABLES["Table4"]
    assert nearest_table_rect((2.1, 7.4), _TABLES) is _TABLES["Table1"]


def test_nearest_table_rect_empty_returns_none():
    assert nearest_table_rect((0.0, 0.0), {}) is None


# ── box_geometry_from_corners ──────────────────────────────────────────────


def test_box_geometry_footprint_reproduces_the_rectangle():
    rect = {"x0": 1.5, "y0": 7.3, "x1": 2.6, "y1": 7.55}
    corners = table_rect_corners(rect)
    geom = box_geometry_from_corners(corners, toward=(2.05, 9.0))
    assert geom is not None
    assert _pt_set(geom.footprint()) == _pt_set(corners)


def test_box_geometry_long_edge_is_width_short_edge_is_depth():
    rect = {"x0": 1.5, "y0": 7.3, "x1": 2.6, "y1": 7.55}  # 1.10 × 0.25
    geom = box_geometry_from_corners(table_rect_corners(rect), toward=(2.05, 9.0))
    assert geom.box.width == pytest.approx(1.10)
    assert geom.box.depth == pytest.approx(0.25)


def test_box_geometry_normal_points_toward_the_tag():
    rect = {"x0": 1.5, "y0": 7.3, "x1": 2.6, "y1": 7.55}
    # Tag to the north (+y): the box "front" (depth 0) faces +y.
    geom = box_geometry_from_corners(table_rect_corners(rect), toward=(2.05, 9.0))
    assert geom.normal[1] > 0.9
    front = geom.place(0.0, 0.0)
    back = geom.place(0.0, 1.0)
    assert front[1] > back[1]  # depth 0 sits on the +y (aisle) edge


def test_box_geometry_survives_a_rotated_layout():
    # A 1.10 × 0.25 rect, registered through a 40° rotation + translation, must
    # keep its true dimensions and footprint (orientation no longer axis-aligned).
    rect = {"x0": 0.0, "y0": 0.0, "x1": 1.10, "y1": 0.25}
    theta = math.radians(40.0)
    tf = Transform2D(cos=math.cos(theta), sin=math.sin(theta), tx=3.0, ty=-1.0)
    corners = [tf.apply(x, y) for x, y in table_rect_corners(rect)]
    toward = tf.apply(0.55, 5.0)  # tag far on the +y side, in the source frame
    geom = box_geometry_from_corners(corners, toward=toward)
    assert geom is not None
    assert geom.box.width == pytest.approx(1.10)
    assert geom.box.depth == pytest.approx(0.25)
    assert _pt_set(geom.footprint()) == _pt_set(corners)


def test_box_geometry_degenerate_corners_return_none():
    # Zero-area "rectangle" (all corners coincide) has no valid frame.
    assert box_geometry_from_corners([(1.0, 1.0)] * 4, toward=(2.0, 2.0)) is None
