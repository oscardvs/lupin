"""Unit tests for the lidar front-face snap added to box_geometry.

Pure-Python — no rclpy. Backs box_layout_publisher's /map refinement: a 2-DOF
(yaw + normal-offset) fit of a known MapBox's front face onto the occupied
cells of an OccupancyGrid view.
"""

from __future__ import annotations

import math

import pytest

from lupin_perception.box_geometry import (
    GridView,
    MapBox,
    front_face_occupied_points,
    snap_front_face,
)


def _box(yaw=0.0):
    # centre (1,0); width 1.0 spans lateral; depth 0.4 along the normal.
    # yaw=0 → normal +x, front-face centre at x=1.2, spanning y in [-0.5, 0.5].
    return MapBox(box_id='5', x=1.0, y=0.0, yaw=yaw, width=1.0, depth=0.4, height=0.3)


def test_snap_shifts_front_face_onto_line():
    box = _box()
    pts = [(1.25, j / 10.0) for j in range(-4, 5)]  # vertical line 0.05 past the face
    out = snap_front_face(box, pts, min_points=5)
    assert out.x == pytest.approx(1.05, abs=1e-6)  # centre moved +0.05 along +x
    assert out.y == pytest.approx(0.0, abs=1e-6)
    assert out.yaw == pytest.approx(0.0, abs=1e-3)


def test_snap_corrects_small_yaw():
    box = _box()
    th = math.radians(5.0)
    pts = [(1.2 + t * math.sin(th), t * math.cos(th)) for t in [i / 10.0 for i in range(-4, 5)]]
    out = snap_front_face(box, pts, min_points=5, max_yaw_rad=math.radians(12.0))
    assert abs(out.yaw) == pytest.approx(math.radians(5.0), abs=math.radians(1.0))


def test_snap_rejects_when_too_few_points():
    box = _box()
    out = snap_front_face(box, [(1.25, 0.0), (1.25, 0.1)], min_points=8)
    assert out is box  # prior returned unchanged


def test_snap_rejects_blob():
    box = _box()
    pts = [(1.2 + 0.02 * i, 0.02 * j) for i in range(-3, 4) for j in range(-3, 4)]
    out = snap_front_face(box, pts, min_points=8, min_elongation=3.0)
    assert out is box


def test_snap_clamps_large_shift():
    box = _box()
    pts = [(2.0, j / 10.0) for j in range(-4, 5)]  # line far in front (x=2.0)
    out = snap_front_face(box, pts, min_points=5, max_shift_m=0.15)
    assert out.x == pytest.approx(1.15, abs=1e-6)  # Fc x=1.2, d=0.8 clamped to 0.15


def test_front_face_points_selects_band_excludes_far_cells():
    box = _box()
    W = H = 20
    res, ox, oy = 0.1, 0.0, -1.0
    data = [0] * (W * H)
    for row in range(H):
        wy = oy + (row + 0.5) * res
        if -0.4 <= wy <= 0.4:
            data[row * W + 12] = 100  # occupied column at x≈1.25
    data[10 * W + 2] = 100  # far occupied cell at x≈0.25 — must be excluded
    grid = GridView(width=W, height=H, resolution=res, origin_x=ox, origin_y=oy, data=data)
    pts = front_face_occupied_points(box, grid, band_m=0.12, threshold=65)
    assert len(pts) >= 5
    assert all(abs(px - 1.25) < 0.06 for px, _ in pts)
