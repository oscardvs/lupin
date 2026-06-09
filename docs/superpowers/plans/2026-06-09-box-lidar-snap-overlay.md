# Planter-box lidar-snap + staged-confidence overlay — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the HMI planter-box overlay correct and trustworthy — snap each box's front face onto the lidar map, show all registered benches as a standing layout, and render staged ghost→solid confidence.

**Architecture:** Server-side 2-DOF front-face snap in `box_layout_publisher` (against `/map`); a new `box_geometry_json` subscription in `lupin_twin` that sets per-tag `box_footprint` for all benches (authoritative over the flower path, dissolving the staleness bug); client-side coverage computed in the HMI to drive the confidence styling. No `.msg` changes, no new topics.

**Tech Stack:** Python (rclpy, `lupin_perception`, `lupin_twin`), pytest; TypeScript/React (`lupin_web`), vitest.

**Spec:** `docs/superpowers/specs/2026-06-09-box-lidar-snap-overlay-design.md`

---

## Setup (before Task 1)

- [ ] Branch off `main` in the `lupin` repo: `git -C /home/oskrt/ros2_ws/src/lupin checkout -b feat/box-lidar-snap-overlay`
- [ ] Commit the spec + this plan:
```bash
git -C /home/oskrt/ros2_ws/src/lupin add docs/superpowers/specs/2026-06-09-box-lidar-snap-overlay-design.md docs/superpowers/plans/2026-06-09-box-lidar-snap-overlay.md
git -C /home/oskrt/ros2_ws/src/lupin commit -m "docs: spec + plan for box lidar-snap + staged-confidence overlay"
```

**Conventions:**
- No `Co-Authored-By` trailer on any commit.
- **Every Python test command assumes the ROS workspace is sourced first** so `rclpy` and the `lupin_msgs` interfaces import: `source /home/oskrt/ros2_ws/install/setup.bash`. The `PYTHONPATH=$PWD` prefix shown in each step then overrides the symlink-installed copy with `src/` (the worktree-testing gotcha — without it, edits to `src/` may not be what pytest imports). So each `python3 -m pytest` step is really:
  ```bash
  source /home/oskrt/ros2_ws/install/setup.bash
  cd <package dir>
  PYTHONPATH=$PWD:$PYTHONPATH python3 -m pytest <args>
  ```

## File Structure

| File | Responsibility | Change |
|---|---|---|
| `lupin_perception/lupin_perception/box_geometry.py` | Pure geometry; add `GridView`, `front_face_occupied_points`, `snap_front_face`, `_principal_axis` | Modify (append) |
| `lupin_perception/test/test_box_snap.py` | Unit tests for the snap | Create |
| `lupin_perception/lupin_perception/box_layout_publisher.py` | Subscribe `/map`, snap each box before publishing | Modify |
| `lupin_twin/lupin_twin/state.py` | `box_footprint_corners`, `record_box`, authority flag | Modify |
| `lupin_twin/test/test_state.py` | Tests for corners + record_box authority | Modify (append) |
| `lupin_twin/lupin_twin/node.py` | Subscribe `box_geometry_json` → `record_box` | Modify |
| `lupin_web/web/src/lib/box-coverage.ts` | Pure `frontFaceCoverage(corners, grid)` | Create |
| `lupin_web/web/src/lib/box-coverage.test.ts` | vitest for coverage helper | Create |
| `lupin_web/web/src/components/widgets/MapCanvas.tsx` | Confidence-styled box render | Modify |

---

### Task 1: Pure front-face snap in `box_geometry.py`

**Files:**
- Modify: `lupin_perception/lupin_perception/box_geometry.py` (append after `map_box_from_rect`)
- Test: `lupin_perception/test/test_box_snap.py`

- [ ] **Step 1: Write the failing tests**

Create `lupin_perception/test/test_box_snap.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_perception
PYTHONPATH=$PWD python3 -m pytest test/test_box_snap.py -v
```
Expected: FAIL — `ImportError: cannot import name 'GridView'` (and `front_face_occupied_points`, `snap_front_face`).

- [ ] **Step 3: Implement the snap in `box_geometry.py`**

Append to `lupin_perception/lupin_perception/box_geometry.py` (after `map_box_from_rect`). `math`, `dataclass`, `List`, `Sequence`, `Tuple` are already imported at the top.

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_perception
PYTHONPATH=$PWD python3 -m pytest test/test_box_snap.py -v
```
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
git -C /home/oskrt/ros2_ws/src/lupin add lupin_perception/lupin_perception/box_geometry.py lupin_perception/test/test_box_snap.py
git -C /home/oskrt/ros2_ws/src/lupin commit -m "feat(perception): lidar front-face snap for registered planter boxes"
```

---

### Task 2: Wire the snap into `box_layout_publisher`

**Files:**
- Modify: `lupin_perception/lupin_perception/box_layout_publisher.py`

No new unit test: the algorithm is covered by Task 1's pure tests; the node glue (the `/map` subscription) is verified in Task 7's sim acceptance. This task adds the wiring and must not regress the existing suite.

- [ ] **Step 1: Add imports**

In `box_layout_publisher.py`, after `import json`:
```python
import math
```
After `from std_msgs.msg import String`:
```python
from nav_msgs.msg import OccupancyGrid
```
Extend the existing `from .box_geometry import (...)` block to add three names:
```python
from .box_geometry import (
    GridView,
    front_face_occupied_points,
    map_box_from_rect,
    nearest_table_rect,
    snap_front_face,
    solve_rigid_2d,
    table_rect_corners,
)
```

- [ ] **Step 2: Declare params + state + subscription**

In `__init__`, after the `self.declare_parameter('publish_rate_hz', 1.0)` line, add:
```python
self.declare_parameter('map_topic', '/map')
self.declare_parameter('snap_enable', True)
self.declare_parameter('snap_occupied_threshold', 65)
self.declare_parameter('snap_band_m', 0.12)
self.declare_parameter('snap_min_points', 8)
self.declare_parameter('snap_min_elongation', 3.0)
self.declare_parameter('snap_max_yaw_deg', 12.0)
self.declare_parameter('snap_max_shift_m', 0.15)

self._snap_enable = bool(self.get_parameter('snap_enable').value)
self._snap_threshold = int(self.get_parameter('snap_occupied_threshold').value)
self._snap_band_m = float(self.get_parameter('snap_band_m').value)
self._snap_min_points = int(self.get_parameter('snap_min_points').value)
self._snap_min_elongation = float(self.get_parameter('snap_min_elongation').value)
self._snap_max_yaw_deg = float(self.get_parameter('snap_max_yaw_deg').value)
self._snap_max_shift_m = float(self.get_parameter('snap_max_shift_m').value)
self._latest_grid: OccupancyGrid | None = None
```

After the `self._box_pub = self.create_publisher(...)` block (before the discovered-tags subscription is fine), add a `/map` subscription. `/map` from slam_toolbox is latched (TRANSIENT_LOCAL), so reuse the `latched` QoS already defined in `__init__`:
```python
self.create_subscription(
    OccupancyGrid, str(self.get_parameter('map_topic').value),
    self._on_map, latched,
)
```

- [ ] **Step 3: Add the map callback + snap helper**

Add two methods to `BoxLayoutPublisher` (e.g. after `_on_discovered`):
```python
def _on_map(self, msg: OccupancyGrid) -> None:
    self._latest_grid = msg

def _snap_box(self, mb):
    g = self._latest_grid
    if g is None:
        return mb
    grid = GridView(
        width=g.info.width, height=g.info.height, resolution=g.info.resolution,
        origin_x=g.info.origin.position.x, origin_y=g.info.origin.position.y,
        data=g.data,
    )
    pts = front_face_occupied_points(
        mb, grid, band_m=self._snap_band_m, threshold=self._snap_threshold,
    )
    return snap_front_face(
        mb, pts,
        min_points=self._snap_min_points,
        min_elongation=self._snap_min_elongation,
        max_yaw_rad=math.radians(self._snap_max_yaw_deg),
        max_shift_m=self._snap_max_shift_m,
    )
```

- [ ] **Step 4: Call the snap in `_publish_boxes`**

In `_publish_boxes`, replace:
```python
            mb = map_box_from_rect(tid, corners, toward=mxy, height=self._box_height)
            if mb is None:
                continue
```
with:
```python
            mb = map_box_from_rect(tid, corners, toward=mxy, height=self._box_height)
            if mb is None:
                continue
            if self._snap_enable:
                mb = self._snap_box(mb)
```

- [ ] **Step 5: Verify no regression in the perception suite**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_perception
PYTHONPATH=$PWD python3 -m pytest test/ -q
```
Expected: PASS (existing tests + Task 1's `test_box_snap.py`), no import/collection errors.

- [ ] **Step 6: Commit**

```bash
git -C /home/oskrt/ros2_ws/src/lupin add lupin_perception/lupin_perception/box_layout_publisher.py
git -C /home/oskrt/ros2_ws/src/lupin commit -m "feat(perception): box_layout_publisher snaps boxes to /map before publishing"
```

---

### Task 3: Twin store — `box_footprint_corners` + `record_box` authority

**Files:**
- Modify: `lupin_twin/lupin_twin/state.py`
- Test: `lupin_twin/test/test_state.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `lupin_twin/test/test_state.py` (it already imports from `lupin_twin.state`; add `box_footprint_corners` to that import, and `import math`, `import pytest` if not present):

```python
def test_box_footprint_corners_order_and_geometry():
    # yaw=0 → normal +x, lateral +y. centre (1,0); width 1.0 (lateral), depth 0.4 (normal).
    c = box_footprint_corners(1.0, 0.0, 0.0, 1.0, 0.4)
    assert c[0] == pytest.approx((1.2, 0.5))   # front-left
    assert c[1] == pytest.approx((1.2, -0.5))  # front-right
    assert c[2] == pytest.approx((0.8, -0.5))  # back-right
    assert c[3] == pytest.approx((0.8, 0.5))   # back-left


def test_record_box_sets_footprint_on_unknown_tag():
    store = TwinStateStore()
    assert store.record_box('7', [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)])
    buf = store.tag('7')
    assert buf is not None
    assert buf.box_footprint == [(1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]
    assert buf.box_footprint_source == 'channel'


def test_record_box_channel_wins_over_flower():
    store = TwinStateStore()
    store.record_box('7', [(2.0, 0.0)])
    store.record_flower(FlowerUpdate(
        tag_id='7', monotonic_at=1.0, species='tulip',
        species_confidence=0.9, anomaly=False, box_footprint=[(9.0, 9.0)],
    ))
    assert store.tag('7').box_footprint == [(2.0, 0.0)]  # channel preserved


def test_flower_footprint_used_when_no_channel_box():
    store = TwinStateStore()
    store.record_flower(FlowerUpdate(
        tag_id='7', monotonic_at=1.0, species='tulip',
        species_confidence=0.9, anomaly=False, box_footprint=[(3.0, 3.0)],
    ))
    assert store.tag('7').box_footprint == [(3.0, 3.0)]
    assert store.tag('7').box_footprint_source == 'flower'


def test_record_box_does_not_bump_observation_count():
    store = TwinStateStore()
    c0 = store.observation_count
    store.record_box('7', [(1.0, 0.0)])
    assert store.observation_count == c0
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_twin
PYTHONPATH=$PWD python3 -m pytest test/test_state.py -v -k "box_footprint or record_box or flower_footprint"
```
Expected: FAIL — `ImportError: cannot import name 'box_footprint_corners'` / `TagBuffer` has no `box_footprint_source` / `TwinStateStore` has no `record_box`.

- [ ] **Step 3: Implement in `state.py`**

In `TagBuffer`, after the `box_footprint: list[...] = field(default_factory=list)` line, add:
```python
    # 'channel' once box_geometry_json sets the footprint; 'flower' if only the
    # flower path has. The channel is authoritative — record_flower won't
    # overwrite a channel footprint.
    box_footprint_source: str = ''
```

Add a module-level pure function (e.g. after the `TwinFlower` dataclass; `math` is already imported):
```python
def box_footprint_corners(
    x: float, y: float, yaw: float, width: float, depth: float,
) -> list[tuple[float, float]]:
    """Four map-frame corners (front-left, front-right, back-right, back-left)
    of a box centre+yaw+dims. ``yaw`` is the outward-normal direction; the front
    face is corners[0]→corners[1]. Matches box_geometry.BoxGeometry.footprint
    ordering so HMI/perception agree on which edge is the front."""
    nx, ny = math.cos(yaw), math.sin(yaw)
    lx, ly = -ny, nx
    hw, hd = 0.5 * width, 0.5 * depth
    fx, fy = x + nx * hd, y + ny * hd
    bx, by = x - nx * hd, y - ny * hd
    return [
        (fx + lx * hw, fy + ly * hw),
        (fx - lx * hw, fy - ly * hw),
        (bx - lx * hw, by - ly * hw),
        (bx + lx * hw, by + ly * hw),
    ]
```

In `record_flower`, replace the line `buf.box_footprint = list(upd.box_footprint)` with:
```python
        # The live box-geometry channel is authoritative; only fall back to the
        # flower-carried footprint when the channel hasn't claimed this tag.
        if buf.box_footprint_source != 'channel':
            buf.box_footprint = list(upd.box_footprint)
            if upd.box_footprint:
                buf.box_footprint_source = 'flower'
```

Add a method to `TwinStateStore` (after `record_flower`):
```python
    def record_box(self, tag_id: str, footprint: list[tuple[float, float]]) -> bool:
        """Set a tag's box footprint from the live box-geometry channel.

        Authoritative over the flower path. Deliberately does NOT touch
        last_seen / pose / readings or bump observation_count — box geometry is
        not a sensor observation, so it must not refresh tag staleness or
        invalidate the IDW field cache. Creates a box-only entry if the tag is
        unknown (rare — the discovery feed normally pins it first)."""
        if not tag_id:
            return False
        buf = self._tags.get(tag_id)
        if buf is None:
            buf = TagBuffer(tag_id=tag_id)
            buf.history = deque(maxlen=self._buffer_len)
            self._tags[tag_id] = buf
        buf.box_footprint = list(footprint)
        buf.box_footprint_source = 'channel'
        return True
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_twin
PYTHONPATH=$PWD python3 -m pytest test/test_state.py -v
```
Expected: PASS (new tests + existing `test_state.py`, including the existing `box_footprint` tests).

- [ ] **Step 5: Commit**

```bash
git -C /home/oskrt/ros2_ws/src/lupin add lupin_twin/lupin_twin/state.py lupin_twin/test/test_state.py
git -C /home/oskrt/ros2_ws/src/lupin commit -m "feat(twin): authoritative box-channel footprint in the state store"
```

---

### Task 4: Twin node — subscribe `box_geometry_json`

**Files:**
- Modify: `lupin_twin/lupin_twin/node.py`

Node glue (rclpy subscription); the parsing→corners→record_box logic reuses the unit-tested `box_footprint_corners`/`record_box`. Verified end-to-end in Task 7.

- [ ] **Step 1: Add imports**

In `node.py`, add at the top with the other stdlib imports:
```python
import json
```
Change `from std_msgs.msg import Header` to:
```python
from std_msgs.msg import Header, String
```
Add `box_footprint_corners` to the `from .state import (...)` block.

- [ ] **Step 2: Declare the topic param**

After `self.declare_parameter('discovered_tags_topic', '/perception/discovered_tags')`, add:
```python
self.declare_parameter('box_geometry_topic', '/perception/box_geometry_json')
self._box_geometry_topic = str(self.get_parameter('box_geometry_topic').value)
```

- [ ] **Step 3: Add the subscription**

After the `self._discovered_sub = self.create_subscription(...)` block (which uses `obs_qos`), add:
```python
# Live planter-box geometry from box_layout_publisher: set per-tag
# box_footprint for ALL registered benches, authoritative over the flower
# path, so the overlay tracks registration/snap updates instead of freezing
# at the last flower-summary change.
self._box_geom_sub = self.create_subscription(
    String,
    self._box_geometry_topic,
    self._on_box_geometry,
    obs_qos,
    callback_group=self._cb_group,
)
```

- [ ] **Step 4: Add the handler**

Add a method to `TwinNode` (e.g. after `_on_discovered_tags`):
```python
def _on_box_geometry(self, msg: String) -> None:
    """Set per-tag box footprints from box_layout_publisher's snapped boxes.

    Accepts ``[{id,x,y,yaw,width,depth,height}, …]`` or ``{"boxes": [...]}``
    (same shape the aggregator parses)."""
    try:
        payload = json.loads(msg.data)
    except (ValueError, TypeError):
        self.get_logger().warn(
            'box_geometry_json not parseable', throttle_duration_sec=10.0,
        )
        return
    boxes = payload.get('boxes') if isinstance(payload, dict) else payload
    if not isinstance(boxes, list):
        return
    for item in boxes:
        if not isinstance(item, dict):
            continue
        box_id = str(item.get('id', item.get('box_id', '')))
        if not box_id or box_id == 'None':
            continue
        try:
            corners = box_footprint_corners(
                float(item['x']), float(item['y']), float(item.get('yaw', 0.0)),
                float(item['width']), float(item['depth']),
            )
        except (KeyError, TypeError, ValueError):
            continue
        self._store.record_box(box_id, corners)
```

- [ ] **Step 5: Verify the twin suite + import**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_twin
PYTHONPATH=$PWD python3 -m pytest test/ -q
```
Expected: PASS, no collection/import errors (confirms `node.py` imports cleanly with the new `String`/`json`/`box_footprint_corners` references).

- [ ] **Step 6: Commit**

```bash
git -C /home/oskrt/ros2_ws/src/lupin add lupin_twin/lupin_twin/node.py
git -C /home/oskrt/ros2_ws/src/lupin commit -m "feat(twin): subscribe box_geometry_json to publish all bench footprints"
```

---

### Task 5: HMI `frontFaceCoverage` helper

**Files:**
- Create: `lupin_web/web/src/lib/box-coverage.ts`
- Test: `lupin_web/web/src/lib/box-coverage.test.ts`

- [ ] **Step 1: Write the failing tests**

Create `lupin_web/web/src/lib/box-coverage.test.ts`:

```ts
import { describe, expect, it } from 'vitest'

import { frontFaceCoverage } from './box-coverage'
import type { OccupancyGrid, Point32 } from '@/types/ros'

function makeGrid(
  width: number, height: number, resolution: number, ox: number, oy: number, data: number[],
): OccupancyGrid {
  return {
    header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'map' },
    info: {
      map_load_time: { sec: 0, nanosec: 0 }, resolution, width, height,
      origin: { position: { x: ox, y: oy, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
    },
    data,
  }
}
const pt = (x: number, y: number): Point32 => ({ x, y, z: 0 })

// Box front face (FL→FR) is the vertical segment x=1.25, y from 0.4 to -0.4.
const FACE: Point32[] = [pt(1.25, 0.4), pt(1.25, -0.4), pt(0.85, -0.4), pt(0.85, 0.4)]

describe('frontFaceCoverage', () => {
  it('is ~1 when the whole front face overlaps occupied cells', () => {
    const W = 20, H = 20
    const data = new Array(W * H).fill(0)
    for (let row = 0; row < H; row++) data[row * W + 12] = 100 // occupied col at x≈1.25
    const cov = frontFaceCoverage(FACE, makeGrid(W, H, 0.1, 0, -1, data), { bandM: 0.05 })
    expect(cov).toBeGreaterThan(0.9)
  })

  it('is 0 on an empty grid', () => {
    const W = 20, H = 20
    const cov = frontFaceCoverage(FACE, makeGrid(W, H, 0.1, 0, -1, new Array(W * H).fill(0)))
    expect(cov).toBe(0)
  })

  it('is 0 for a null grid or degenerate face', () => {
    expect(frontFaceCoverage(FACE, null)).toBe(0)
    expect(frontFaceCoverage([pt(0, 0)], makeGrid(4, 4, 0.1, 0, 0, new Array(16).fill(100)))).toBe(0)
  })
})
```

- [ ] **Step 2: Run tests to verify they fail**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_web/web
npx vitest run src/lib/box-coverage.test.ts
```
Expected: FAIL — cannot resolve `./box-coverage`.

- [ ] **Step 3: Implement `box-coverage.ts`**

Create `lupin_web/web/src/lib/box-coverage.ts`:

```ts
import type { OccupancyGrid, Point32 } from '@/types/ros'

/**
 * Fraction [0,1] of a box's FRONT face (corners[0]→corners[1]) corroborated by
 * the lidar: of `samples` points along that edge, how many have an occupied
 * cell (value ≥ `threshold`) within `bandM` metres. Returns 0 for a null/empty
 * grid or a degenerate (<2-corner) face. Corners are FL, FR, BR, BL — the
 * twin/perception order, so the front face is always corners[0]→corners[1].
 */
export function frontFaceCoverage(
  corners: Point32[],
  grid: OccupancyGrid | null | undefined,
  { samples = 24, bandM = 0.1, threshold = 65 }: {
    samples?: number; bandM?: number; threshold?: number
  } = {},
): number {
  if (!grid || corners.length < 2) return 0
  const { width, height, resolution } = grid.info
  if (width <= 0 || height <= 0 || resolution <= 0) return 0
  const ox = grid.info.origin.position.x
  const oy = grid.info.origin.position.y
  const a = corners[0]
  const b = corners[1]
  const r = Math.max(1, Math.ceil(bandM / resolution))
  let covered = 0
  for (let i = 0; i < samples; i++) {
    const t = (i + 0.5) / samples
    const col = Math.floor((a.x + (b.x - a.x) * t - ox) / resolution)
    const row = Math.floor((a.y + (b.y - a.y) * t - oy) / resolution)
    if (occupiedNear(grid, col, row, r, threshold)) covered++
  }
  return covered / samples
}

function occupiedNear(
  grid: OccupancyGrid, col: number, row: number, r: number, threshold: number,
): boolean {
  const { width, height } = grid.info
  for (let dr = -r; dr <= r; dr++) {
    const rr = row + dr
    if (rr < 0 || rr >= height) continue
    for (let dc = -r; dc <= r; dc++) {
      const cc = col + dc
      if (cc < 0 || cc >= width) continue
      if (grid.data[rr * width + cc] >= threshold) return true
    }
  }
  return false
}
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_web/web
npx vitest run src/lib/box-coverage.test.ts
```
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git -C /home/oskrt/ros2_ws/src/lupin add lupin_web/web/src/lib/box-coverage.ts lupin_web/web/src/lib/box-coverage.test.ts
git -C /home/oskrt/ros2_ws/src/lupin commit -m "feat(web): frontFaceCoverage helper for box overlay confidence"
```

---

### Task 6: HMI `MapCanvas` — staged confidence render

**Files:**
- Modify: `lupin_web/web/src/components/widgets/MapCanvas.tsx`

Render wiring; the coverage/lerp logic is unit-tested in Task 5. Verified visually in Task 7.

- [ ] **Step 1: Import the helper**

Add to the imports at the top of `MapCanvas.tsx`:
```ts
import { frontFaceCoverage } from '@/lib/box-coverage'
```

- [ ] **Step 2: Replace the box-footprint draw block**

In the `if (layers.flowers)` loop, replace the existing block:
```ts
        // Box footprint rectangle (faint), from the localized corners.
        const corners = t.box_footprint?.points ?? []
        if (corners.length >= 3) {
          ctx.save()
          ctx.beginPath()
          corners.forEach((p, i) => {
            const c = proj.worldToCanvas(p.x, p.y)
            if (i === 0) ctx.moveTo(c.x, c.y)
            else ctx.lineTo(c.x, c.y)
          })
          ctx.closePath()
          ctx.fillStyle = 'hsla(140, 35%, 50%, 0.06)'
          ctx.lineWidth = 1
          ctx.strokeStyle = 'hsla(140, 35%, 62%, 0.35)'
          ctx.fill()
          ctx.stroke()
          ctx.restore()
        }
```
with:
```ts
        // Box footprint rectangle, styled by staged lidar confidence: how much
        // of the FRONT face (corners[0]→[1]) the occupancy grid corroborates.
        // Faint dashed "predicted" → solid "confirmed".
        const corners = t.box_footprint?.points ?? []
        if (corners.length >= 3) {
          const cov = frontFaceCoverage(corners, mapRef.current)
          const conf = Math.max(0, Math.min(1, (cov - 0.1) / 0.5)) // 0 at ≤0.1, 1 at ≥0.6
          const strokeA = 0.18 + (0.5 - 0.18) * conf
          const fillA = 0.1 * conf
          ctx.save()
          ctx.beginPath()
          corners.forEach((p, i) => {
            const c = proj.worldToCanvas(p.x, p.y)
            if (i === 0) ctx.moveTo(c.x, c.y)
            else ctx.lineTo(c.x, c.y)
          })
          ctx.closePath()
          ctx.fillStyle = `hsla(140, 35%, 50%, ${fillA.toFixed(3)})`
          ctx.lineWidth = conf >= 0.999 ? 1.5 : 1
          ctx.strokeStyle = `hsla(140, 38%, 62%, ${strokeA.toFixed(3)})`
          if (conf < 1) ctx.setLineDash([4, 3])
          ctx.fill()
          ctx.stroke()
          ctx.setLineDash([])
          ctx.restore()
        }
```

- [ ] **Step 3: Type-check + build**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_web/web
npm run build
```
Expected: PASS (tsc + vite build clean — confirms `mapRef`/`proj` are in scope and the import resolves).

- [ ] **Step 4: Run the web suite**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_web/web
npm run test
```
Expected: PASS (all vitest, including `box-coverage.test.ts`).

- [ ] **Step 5: Commit**

```bash
git -C /home/oskrt/ros2_ws/src/lupin add lupin_web/web/src/components/widgets/MapCanvas.tsx
git -C /home/oskrt/ros2_ws/src/lupin commit -m "feat(web): staged ghost→solid confidence for the planter-box overlay"
```

---

### Task 7: Acceptance in sim + branch finish

**Files:** none (verification + integration).

- [ ] **Step 1: Rebuild the ROS packages**

```bash
cd /home/oskrt/ros2_ws
colcon build --packages-select lupin_perception lupin_twin --symlink-install
source install/setup.bash
```
Expected: build OK.

- [ ] **Step 2: Launch the full sim**

```bash
ros2 launch lupin_bringup sim_full.launch.py
```
Then drive the robot (Xbox pad or the HMI Teleop tab) so slam_toolbox builds `/map` and tags get discovered.

- [ ] **Step 3: Verify the live geometry channel**

From the operator terminal:
```bash
ros2 topic echo --once /perception/box_geometry_json
ros2 topic echo --once /twin/state | grep -A6 box_footprint
```
Expected: boxes carry ~1.10×0.25 dims; once a bench's front face is mapped, its published box centre/yaw track the occupied strip (snap engaged); `/twin/state` carries `box_footprint` for discovered benches, not only scanned ones.

- [ ] **Step 4: Verify the HMI overlay**

Open the HMI Map view. Expected:
- Every discovered bench shows a box rectangle (not only scanned ones).
- A bench not yet driven past renders as a **faint dashed** ghost.
- After driving past a bench, its box snaps onto the lidar occupied strip and renders **solid**.
- `snap_enable:=false` (relaunch `box_layout_publisher` with the param) reverts to the un-snapped boxes — a quick A/B sanity check.

- [ ] **Step 5: Full regression**

```bash
cd /home/oskrt/ros2_ws/src/lupin/lupin_perception && PYTHONPATH=$PWD python3 -m pytest test/ -q
cd /home/oskrt/ros2_ws/src/lupin/lupin_twin && PYTHONPATH=$PWD python3 -m pytest test/ -q
cd /home/oskrt/ros2_ws/src/lupin/lupin_web/web && npm run test && npm run build
```
Expected: all PASS.

- [ ] **Step 6: Finish the branch**

Use the `superpowers:finishing-a-development-branch` skill to merge `feat/box-lidar-snap-overlay` → `main`, then fast-forward `sim` and `hardware` per the team branch-split convention. (Do not push without the user's go-ahead.)

---

## Notes for the implementer

- **Frame:** boxes, footprints, and `/map` are all in the `map` frame. The snap and coverage are pure 2-D map-frame math.
- **Corner order is load-bearing:** FL, FR, BR, BL. The HMI treats corners[0]→corners[1] as the front face for coverage; `box_footprint_corners` and `box_geometry.footprint()` must agree on it.
- **Why `record_box` skips `observation_count`/`last_seen`:** box geometry is not a sensor reading. Bumping them would fake tag freshness (pin desaturation) and needlessly invalidate the IDW field cache every 1 Hz.
- **Perf:** `frontFaceCoverage` is ~24 samples × (2r+1)² cell reads per box per frame — negligible; computed inline rather than memoized.
