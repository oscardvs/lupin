"""Planter-box geometry — pure, ROS-free, unit-testable.

Each greenhouse base/box can be represented by a map-frame centre, yaw, width,
depth, and height. A detected bloom's map position is then a deterministic
function of where it sits in that box. This module owns that geometry
(mirroring how ``lupin_mission/approach.py`` isolates the approach-pose math):
no rclpy, no geometry_msgs — the orchestrator/aggregator wrap the plain tuples
in ROS types at the edge, and the unit tests drive it with SimpleNamespace
poses.

Box frame from map geometry
---------------------------
``n`` (normal) points out of the bed into the aisle/front face. ``l`` (lateral)
is ``n`` rotated +90 deg in XY = ``(-n_y, n_x)``. ``origin`` is the centre of
the front face. The box interior extends *opposite* ``n`` by ``depth`` and
spans ``+/- width/2`` along ``l``.

Compatibility: box frame from tag pose
--------------------------------------
``n`` (normal) = the tag's local +Z axis projected onto the map XY plane —
the SAME derivation as ``approach.compute_discovered_approach`` (``nx =
2(qx*qz + qw*qy)``, ``ny = 2(qy*qz - qw*qx)``). It points out of the bed into
the aisle (toward the camera that read the tag). ``l`` (lateral) = ``n``
rotated +90 deg in XY = ``(-n_y, n_x)``. The box interior extends *opposite*
``n`` (into the bed) by ``depth`` and spans ``+/- width/2`` along ``l``, with
an optional ``tag_lateral_offset`` shifting the box centre off the tag.

Placement
---------
A bloom with lateral fraction ``f in [-1, 1]`` and depth fraction
``d in [0, 1]`` maps to ``origin + l*(f*width/2) - n*(d*depth)`` where
``origin = tag_xy + l*tag_lateral_offset``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# Below this the in-plane normal is ill-defined (tag facing up/down or noisy
# orientation). Matches approach.py's _DEGENERATE_NORMAL_EPS.
_DEGENERATE_NORMAL_EPS = 1e-6

# Low-discrepancy multiplier (golden ratio frac) for deterministic, RNG-free
# per-column depth jitter so blooms read as scattered, not a straight line.
_GOLDEN = 0.6180339887498949


@dataclass(frozen=True)
class StandardBox:
    """Standard planter-box dimensions (metres). Defaults are calibrated
    against the greenhouse sim; operators tune via the aggregator params.

    ``height`` and ``tag_mount_height`` are reserved for the HMI box render /
    future Z-aware placement; the current geometry is XY-only and does not
    consume them."""
    width: float = 0.80
    depth: float = 0.40
    height: float = 0.30
    tag_mount_height: float = 0.10
    tag_lateral_offset: float = 0.0


@dataclass(frozen=True)
class FlowerPointData:
    """One placed bloom (map frame). The ROS edge wraps this in a
    lupin_msgs/FlowerPoint."""
    x: float
    y: float
    species: str
    confidence: float
    anomaly: bool
    z: float = 0.0
    height_m: float = 0.0


@dataclass(frozen=True)
class MapBox:
    """Map-derived planter box, typically produced by an OpenCV map pipeline.

    ``x``/``y`` are the box centre in map frame. ``yaw`` is the outward normal
    direction (front face / aisle side)."""
    box_id: str
    x: float
    y: float
    yaw: float
    width: float
    depth: float
    height: float = 0.30


class BoxGeometry:
    """Box frame derived from a tag pose. Precompute once per box-scan, then
    call :meth:`place` per bloom and :meth:`footprint` for the rectangle."""

    __slots__ = ('origin', 'normal', 'lateral', 'box')

    def __init__(
        self,
        origin: Tuple[float, float],
        normal: Tuple[float, float],
        lateral: Tuple[float, float],
        box: StandardBox,
    ) -> None:
        self.origin = origin
        self.normal = normal
        self.lateral = lateral
        self.box = box

    def place(self, f: float, d: float) -> Tuple[float, float]:
        """Map xy for lateral fraction ``f in [-1, 1]`` and depth ``d in [0, 1]``."""
        ox, oy = self.origin
        nx, ny = self.normal
        lx, ly = self.lateral
        half_w = 0.5 * self.box.width
        x = ox + lx * (f * half_w) - nx * (d * self.box.depth)
        y = oy + ly * (f * half_w) - ny * (d * self.box.depth)
        return (x, y)

    def footprint(self) -> List[Tuple[float, float]]:
        """The four interior corners (front-left, front-right, back-right,
        back-left), each a map (x, y)."""
        return [
            self.place(1.0, 0.0),   # front-left  (+lateral, front face)
            self.place(-1.0, 0.0),  # front-right (-lateral, front face)
            self.place(-1.0, 1.0),  # back-right  (-lateral, back)
            self.place(1.0, 1.0),   # back-left   (+lateral, back)
        ]

    def contains(self, x: float, y: float, margin_m: float = 0.0) -> bool:
        """Is map point ``(x, y)`` inside this box (optionally inflated by
        ``margin_m`` metres on every side)?

        Exact inverse of :meth:`place`: project the point onto the box's own
        lateral and normal axes to recover its ``(f, d)`` and test the unit
        box bounds. Used by the aggregator's spatial membership gate to reject
        blooms whose bearing points past the current bench. Returns False for a
        degenerate (zero width/depth) box.
        """
        ox, oy = self.origin
        nx, ny = self.normal
        lx, ly = self.lateral
        half_w = 0.5 * self.box.width
        depth = self.box.depth
        if half_w <= 0.0 or depth <= 0.0:
            return False
        vx, vy = x - ox, y - oy
        f = (vx * lx + vy * ly) / half_w        # lateral fraction in [-1, 1]
        d = -(vx * nx + vy * ny) / depth        # depth fraction in [0, 1]
        mf = margin_m / half_w
        md = margin_m / depth
        return (-1.0 - mf) <= f <= (1.0 + mf) and (-md) <= d <= (1.0 + md)


def tag_normal_xy(
    pose_in_map, robot_xy: Optional[Tuple[float, float]] = None,
) -> Optional[Tuple[float, float]]:
    """Unit in-plane normal of the tag (out of the bed, into the aisle).

    Returns None when the normal is degenerate and no ``robot_xy`` is given.
    With ``robot_xy`` the degenerate case falls back to the direction from
    the tag toward the robot (the side it can already see) — same posture as
    ``approach.compute_discovered_approach``.
    """
    qx = float(pose_in_map.orientation.x)
    qy = float(pose_in_map.orientation.y)
    qz = float(pose_in_map.orientation.z)
    qw = float(pose_in_map.orientation.w)
    nx = 2.0 * (qx * qz + qw * qy)
    ny = 2.0 * (qy * qz - qw * qx)
    norm = math.hypot(nx, ny)
    if norm >= _DEGENERATE_NORMAL_EPS:
        return (nx / norm, ny / norm)
    if robot_xy is not None:
        tx = float(pose_in_map.position.x)
        ty = float(pose_in_map.position.y)
        vx, vy = robot_xy[0] - tx, robot_xy[1] - ty
        vn = math.hypot(vx, vy)
        if vn >= _DEGENERATE_NORMAL_EPS:
            return (vx / vn, vy / vn)
    return None


def box_from_tag(
    pose_in_map,
    box: StandardBox = StandardBox(),
    *,
    robot_xy: Optional[Tuple[float, float]] = None,
) -> Optional[BoxGeometry]:
    """Build the box frame from a tag's map pose, or None if the tag
    orientation is too degenerate to derive a normal (no robot fallback)."""
    n = tag_normal_xy(pose_in_map, robot_xy)
    if n is None:
        return None
    nx, ny = n
    lx, ly = (-ny, nx)  # +90 deg rotation in XY
    tx = float(pose_in_map.position.x)
    ty = float(pose_in_map.position.y)
    origin = (tx + lx * box.tag_lateral_offset, ty + ly * box.tag_lateral_offset)
    return BoxGeometry(origin, (nx, ny), (lx, ly), box)


def box_from_map_box(box: MapBox) -> BoxGeometry:
    """Build a box frame from map-derived centre/dimensions.

    ``MapBox.yaw`` is the outward normal direction. The stored centre is the
    rectangle centre, so the front-face origin is half a depth along ``n``.
    """
    width = max(0.0, float(box.width))
    depth = max(0.0, float(box.depth))
    spec = StandardBox(width=width, depth=depth, height=max(0.0, float(box.height)))
    nx = math.cos(float(box.yaw))
    ny = math.sin(float(box.yaw))
    lx, ly = (-ny, nx)
    origin = (
        float(box.x) + nx * depth * 0.5,
        float(box.y) + ny * depth * 0.5,
    )
    return BoxGeometry(origin, (nx, ny), (lx, ly), spec)


def lateral_fraction(
    pan: float,
    bbox_cx_norm: float,
    *,
    pan_center: float = 0.0,
    pan_half_span: float = 0.5,
    camera_half_fov: float = 0.5,
    clamp: bool = True,
) -> float:
    """Lateral fraction ``f in [-1, 1]`` across the box from the arm pan
    angle (coarse cue, primary once the arm sweeps in Stream C) refined by the
    detection's normalized image centre-x ``bbox_cx_norm in [-1, 1]``.

    ``bearing = (pan - pan_center) + bbox_cx_norm * camera_half_fov``; ``f``
    scales that by the full angular half-range the sweep + FOV can cover and
    clamps to [-1, 1]. With a static pan (pre-Stream-C) ``f`` is driven by the
    image-x cue alone, scaled by ``camera_half_fov / (pan_half_span +
    camera_half_fov)``.

    ``clamp=False`` returns the raw (unclamped) fraction so a caller can tell a
    bloom that points *past* the bench end (``|f| > 1``) from one merely at the
    edge — the spatial membership gate relies on this. ``bin_detections``
    re-clamps for placement, so storing the unclamped value never moves a bloom.
    """
    bearing = (pan - pan_center) + bbox_cx_norm * camera_half_fov
    span = pan_half_span + camera_half_fov
    if span <= 0.0:
        return 0.0
    f = bearing / span
    return max(-1.0, min(1.0, f)) if clamp else f


def bin_detections(
    dets: Sequence[Tuple[float, str, float]],
    geom: Optional[BoxGeometry],
    *,
    lateral_columns: int = 7,
    base_depth_frac: float = 0.5,
    depth_jitter_frac: float = 0.18,
    anomaly_class: str = 'bug',
) -> List[FlowerPointData]:
    """Bin per-scan detections ``(f, class_name, confidence)`` into lateral
    columns across the box and place one bloom per occupied column.

    Per column: the dominant (best-confidence) non-anomaly species names the
    bloom; the anomaly class anywhere in the column sets ``anomaly``; depth is
    ``base_depth_frac`` plus a deterministic per-column jitter so blooms
    scatter in depth rather than forming a straight line. Returns [] when
    ``geom`` is None (degenerate tag — the caller may fall back to the tag pose).
    """
    if geom is None or lateral_columns < 1:
        return []
    # column index for f in [-1, 1]
    cols: dict = {}
    for f, name, conf in dets:
        fc = max(-1.0, min(1.0, float(f)))
        k = min(lateral_columns - 1, int((fc + 1.0) * 0.5 * lateral_columns))
        slot = cols.setdefault(k, {'species': '', 'conf': 0.0, 'anomaly': False})
        if name == anomaly_class:
            slot['anomaly'] = True
            continue
        if float(conf) > slot['conf']:
            slot['species'] = name
            slot['conf'] = float(conf)
    out: List[FlowerPointData] = []
    for k in sorted(cols.keys()):
        slot = cols[k]
        if not slot['species'] and not slot['anomaly']:
            continue
        f_center = -1.0 + (2.0 * k + 1.0) / lateral_columns
        jitter = ((k * _GOLDEN) % 1.0 - 0.5) * 2.0 * depth_jitter_frac
        d = max(0.0, min(1.0, base_depth_frac + jitter))
        x, y = geom.place(f_center, d)
        out.append(FlowerPointData(
            x=x, y=y, species=slot['species'],
            confidence=slot['conf'], anomaly=slot['anomaly'],
            z=max(0.0, float(geom.box.height)),
            height_m=max(0.0, float(geom.box.height)),
        ))
    return out


# ── Known-layout registration: greenhouse layout -> map-frame MapBox ─────────
#
# The course ``tag_locations.json`` gives every planter bench as an exact
# rectangle in the JSON frame; the SLAM ``map`` frame is wherever the robot
# started. ``solve_rigid_2d`` recovers the rigid JSON->map transform from the
# detected-tag constellation (positions only — robust, unlike a single noisy
# tag quaternion), and ``map_box_from_rect`` turns a transformed rectangle into
# the ``MapBox`` the aggregator already consumes. The ``box_layout_publisher``
# node wires these to ``/perception/box_geometry_json``; drawing the FULL known
# rectangle is also what fills the cells the lidar can't see behind the near
# face.


@dataclass(frozen=True)
class Transform2D:
    """Rigid 2-D transform: ``map = R(cos, sin)·p + (tx, ty)``."""
    cos: float = 1.0
    sin: float = 0.0
    tx: float = 0.0
    ty: float = 0.0

    def apply(self, x: float, y: float) -> Tuple[float, float]:
        return (self.cos * x - self.sin * y + self.tx,
                self.sin * x + self.cos * y + self.ty)


IDENTITY_2D = Transform2D()


def solve_rigid_2d(
    src: Sequence[Tuple[float, float]],
    dst: Sequence[Tuple[float, float]],
) -> Transform2D:
    """Best-fit rigid (rotation + translation, no scale) mapping ``src`` to ``dst``.

    Closed-form 2-D Kabsch: rotation ``theta = atan2(sum cross, sum dot)`` over
    the mean-centred point pairs, translation ``t = dst_centroid - R*src_centroid``.
    Returns :data:`IDENTITY_2D` for empty / mismatched input, and an identity
    *rotation* (translation only) when the points are too few or too
    coincident/collinear to define a rotation — so a single seen tag still pins
    position without inventing an orientation.
    """
    n = len(src)
    if n == 0 or n != len(dst):
        return IDENTITY_2D
    sx = sum(p[0] for p in src) / n
    sy = sum(p[1] for p in src) / n
    dx = sum(p[0] for p in dst) / n
    dy = sum(p[1] for p in dst) / n
    dot = 0.0    # sum of a·b over the centred point pairs
    cross = 0.0  # sum of a×b
    for (ax, ay), (bx, by) in zip(src, dst):
        cax = ax - sx
        cay = ay - sy
        cbx = bx - dx
        cby = by - dy
        dot += cax * cbx + cay * cby
        cross += cax * cby - cay * cbx
    norm = math.hypot(dot, cross)
    if norm < _DEGENERATE_NORMAL_EPS:
        cos, sin = 1.0, 0.0
    else:
        cos, sin = dot / norm, cross / norm
    tx = dx - (cos * sx - sin * sy)
    ty = dy - (sin * sx + cos * sy)
    return Transform2D(cos, sin, tx, ty)


def table_rect_corners(rect) -> List[Tuple[float, float]]:
    """The four corners of a table bbox, CCW from the min corner:
    ``(x0,y0), (x1,y0), (x1,y1), (x0,y1)`` with bounds normalised. This order is
    relied on by :func:`map_box_from_rect`."""
    x0 = float(rect['x0'])
    y0 = float(rect['y0'])
    x1 = float(rect['x1'])
    y1 = float(rect['y1'])
    lo_x, hi_x = min(x0, x1), max(x0, x1)
    lo_y, hi_y = min(y0, y1), max(y0, y1)
    return [(lo_x, lo_y), (hi_x, lo_y), (hi_x, hi_y), (lo_x, hi_y)]


def nearest_table_rect(tag_xy, tables):
    """Return the table rect whose centre is nearest ``tag_xy``, or None if
    ``tables`` is empty. Ties broken by table id for determinism (mirrors
    ``lupin_mission.approach._nearest_table``; kept local so lupin_perception
    needs no dependency on lupin_mission)."""
    tx, ty = float(tag_xy[0]), float(tag_xy[1])
    best: Optional[Tuple[float, dict]] = None
    for tid in sorted(tables.keys()):
        rect = tables[tid]
        try:
            cx = (float(rect['x0']) + float(rect['x1'])) / 2.0
            cy = (float(rect['y0']) + float(rect['y1'])) / 2.0
        except (KeyError, TypeError, ValueError):
            continue
        d = math.hypot(tx - cx, ty - cy)
        if best is None or d < best[0]:
            best = (d, rect)
    return best[1] if best is not None else None


def map_box_from_rect(
    box_id: str,
    corners: Sequence[Tuple[float, float]],
    *,
    toward: Optional[Tuple[float, float]] = None,
    height: float = 0.30,
) -> Optional[MapBox]:
    """Build a :class:`MapBox` (centre + outward-normal yaw + dims) from a table
    rectangle's four map-frame ``corners`` (from :func:`table_rect_corners`,
    optionally passed through a :class:`Transform2D`).

    The longer edge is the width (lateral) axis, the shorter the depth/normal.
    ``toward`` (e.g. the tag or robot xy) signs the yaw so the box *front* faces
    the aisle. Returns None for a degenerate (zero-area) rectangle. The result
    round-trips through :func:`box_from_map_box` back to ``corners``.
    """
    if len(corners) < 4:
        return None
    c0, c1, c2, c3 = corners[0], corners[1], corners[2], corners[3]
    ax, ay = c1[0] - c0[0], c1[1] - c0[1]   # edge c0->c1 ("x" extent)
    bx, by = c3[0] - c0[0], c3[1] - c0[1]   # edge c0->c3 ("y" extent)
    la = math.hypot(ax, ay)
    lb = math.hypot(bx, by)
    if la >= lb:
        long_len, short_vec, short_len = la, (bx, by), lb
    else:
        long_len, short_vec, short_len = lb, (ax, ay), la
    if long_len < _DEGENERATE_NORMAL_EPS or short_len < _DEGENERATE_NORMAL_EPS:
        return None
    nx, ny = short_vec[0] / short_len, short_vec[1] / short_len
    cx = (c0[0] + c2[0]) / 2.0
    cy = (c0[1] + c2[1]) / 2.0
    if toward is not None:
        if (toward[0] - cx) * nx + (toward[1] - cy) * ny < 0.0:
            nx, ny = -nx, -ny
    return MapBox(
        box_id=str(box_id), x=cx, y=cy, yaw=math.atan2(ny, nx),
        width=long_len, depth=short_len, height=float(height),
    )


# ── Lidar front-face snap: refine a registered MapBox against the map ────────
#
# The tag-registered MapBox is a prior; the SLAM occupancy grid is an
# independent measurement of the bench's aisle-facing face. `front_face_occupied_points`
# gathers the occupied cells in a thin band around that face and `snap_front_face`
# fits them with a line, correcting the box yaw and its offset along the normal
# (2-DOF). Width/depth/height and the along-edge position stay from the tags.
# Pure: the node wraps the live OccupancyGrid in a GridView (no copy).


@dataclass(frozen=True)
class GridView:
    """Read-only view of a nav_msgs/OccupancyGrid for the snap. ROS-free so the
    snap is unit-testable; ``data`` is row-major (length width*height) and may
    alias the live message's data array."""
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    data: Sequence[int]


def front_face_occupied_points(
    box: MapBox, grid: GridView, *, band_m: float = 0.12, threshold: int = 65,
) -> List[Tuple[float, float]]:
    """Map-frame centres of occupied cells (value ≥ ``threshold``) within
    ``band_m`` of ``box``'s front face, inside the face's width extent. Scans
    only the grid's local bounding box. Empty for a degenerate grid."""
    res = float(grid.resolution)
    if res <= 0.0 or grid.width <= 0 or grid.height <= 0:
        return []
    nx, ny = math.cos(box.yaw), math.sin(box.yaw)
    lx, ly = -ny, nx
    half_w = 0.5 * float(box.width)
    fcx = float(box.x) + nx * 0.5 * float(box.depth)
    fcy = float(box.y) + ny * 0.5 * float(box.depth)
    # World AABB of the ±half_w (lateral) × ±band_m (normal) band.
    cs = [
        (fcx + lx * half_w + nx * band_m, fcy + ly * half_w + ny * band_m),
        (fcx + lx * half_w - nx * band_m, fcy + ly * half_w - ny * band_m),
        (fcx - lx * half_w + nx * band_m, fcy - ly * half_w + ny * band_m),
        (fcx - lx * half_w - nx * band_m, fcy - ly * half_w - ny * band_m),
    ]
    xs = [c[0] for c in cs]
    ys = [c[1] for c in cs]
    col0 = max(0, int((min(xs) - grid.origin_x) / res))
    col1 = min(grid.width - 1, int((max(xs) - grid.origin_x) / res))
    row0 = max(0, int((min(ys) - grid.origin_y) / res))
    row1 = min(grid.height - 1, int((max(ys) - grid.origin_y) / res))
    pts: List[Tuple[float, float]] = []
    for row in range(row0, row1 + 1):
        wy = grid.origin_y + (row + 0.5) * res
        base = row * grid.width
        for col in range(col0, col1 + 1):
            if int(grid.data[base + col]) < threshold:
                continue
            wx = grid.origin_x + (col + 0.5) * res
            dx, dy = wx - fcx, wy - fcy
            if abs(dx * nx + dy * ny) <= band_m and abs(dx * lx + dy * ly) <= half_w:
                pts.append((wx, wy))
    return pts


def _principal_axis(points: Sequence[Tuple[float, float]]):
    """(dirx, diry, lambda1, lambda2, mean_x, mean_y) of a 2-D point set via
    closed-form 2x2 PCA. ``dir`` is the major-axis unit vector; lambda1 ≥ lambda2."""
    n = len(points)
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = syy = sxy = 0.0
    for x, y in points:
        dx, dy = x - mx, y - my
        sxx += dx * dx
        syy += dy * dy
        sxy += dx * dy
    sxx /= n
    syy /= n
    sxy /= n
    tr = sxx + syy
    det = sxx * syy - sxy * sxy
    root = math.sqrt(max(0.0, tr * tr - 4.0 * det))
    lam1 = 0.5 * (tr + root)
    lam2 = 0.5 * (tr - root)
    if abs(sxy) > 1e-12:
        vx, vy = lam1 - syy, sxy
    elif sxx >= syy:
        vx, vy = 1.0, 0.0
    else:
        vx, vy = 0.0, 1.0
    norm = math.hypot(vx, vy) or 1.0
    return (vx / norm, vy / norm, lam1, lam2, mx, my)


def snap_front_face(
    box: MapBox,
    points: Sequence[Tuple[float, float]],
    *,
    min_points: int = 8,
    min_elongation: float = 3.0,
    max_yaw_rad: float = 0.20944,  # 12 deg
    max_shift_m: float = 0.15,
) -> MapBox:
    """Refine ``box`` (a tag-registered prior) against front-face ``points``.

    Fits a line, then corrects yaw to the line and shifts the box along its
    normal so the front face lands on the line — both clamped. Returns the
    prior unchanged (same object) when the points are too few, too blob-like
    (PCA elongation below ``min_elongation``) to define a line."""
    if len(points) < min_points:
        return box
    dirx, diry, lam1, lam2, mx, my = _principal_axis(points)
    if lam1 < min_elongation * max(lam2, 1e-9):
        return box  # a blob, not a line
    nx, ny = math.cos(box.yaw), math.sin(box.yaw)
    cnx, cny = -diry, dirx  # normal ⟂ line direction
    if cnx * nx + cny * ny < 0.0:
        cnx, cny = -cnx, -cny  # keep the prior outward hemisphere
    new_yaw = math.atan2(cny, cnx)
    dyaw = math.atan2(math.sin(new_yaw - box.yaw), math.cos(new_yaw - box.yaw))
    dyaw = max(-max_yaw_rad, min(max_yaw_rad, dyaw))
    out_yaw = box.yaw + dyaw
    fcx = box.x + nx * 0.5 * box.depth
    fcy = box.y + ny * 0.5 * box.depth
    d = (mx - fcx) * nx + (my - fcy) * ny  # signed gap face→line along normal
    d = max(-max_shift_m, min(max_shift_m, d))
    return MapBox(
        box_id=box.box_id, x=box.x + d * nx, y=box.y + d * ny,
        yaw=out_yaw, width=box.width, depth=box.depth, height=box.height,
    )
