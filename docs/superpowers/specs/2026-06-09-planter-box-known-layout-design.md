# Planter-box overlay — known-layout rectangles registered to the live map

**Date:** 2026-06-09
**Status:** approved, implementing on `main`
**Author:** Oscar (Team Lupin)

## Problem

On the HMI **Map · navigation (PNL-NAV-01)** view the planter "boxes" overlaid on
the SLAM map are visibly wrong — wrong size, wrong aspect, rotated oddly, and
overlapping. They don't line up with the lidar/occupancy obstacles.

## Root cause

The box rectangles are **synthesised**, not measured:

- `perception_aggregator._fuse_flower` builds each box with
  `box_from_tag(rec.pose, StandardBox(width=0.80, depth=0.40))` and ships the 4
  corners as `box_footprint` → `/floranova/observations` → `lupin_twin` →
  `/twin/state` → `MapCanvas.tsx:716` draws them.
- The **real** planters are not 0.80 × 0.40. The course package
  `greenhouse_sim/configs/tag_locations.json` (the same file the SMI layout came
  from) defines all 10 tables exactly, in metres: six **1.10 × 0.25** E–W benches
  and four **0.25 × 1.10** N–S benches.
- Orientation comes from the **noisy AprilTag quaternion** (the in-plane normal),
  not the known table axis.
- There are **29 tags but 10 tables** (≈4 tags/bench). The code draws **one box
  per tag**, so each bench gets ~4 overlapping oversized rectangles — the clutter
  the operator sees.

Two enabling facts:
1. The runtime already loads these rectangles — `lupin_mission/tag_locations.py`
   (`load_default_tables` / `load_default_tag_locations`) + `approach.py` use them
   for approach poses. The aggregator simply doesn't use them for the render.
2. The map frame is effectively aligned to the JSON layout in sim (the world is
   generated from this JSON), so the JSON rectangles are ground truth.

## Decision

**Known-layout boxes, registered to the live map.** Use the exact table rectangle
from `tag_locations.json` (one per table), positioned by a robust JSON→map
registration fit from the detected tag constellation — not one noisy per-tag
quaternion. This inherently "fills the unseen cells" because we draw the full
known rectangle; lidar snapping is unnecessary as a geometry source (kept as a
possible future refinement). Display timing keeps the current behaviour (a box
appears per scanned tag, just correct now) — option **(A)**.

## Design

Fix lives in `lupin_perception` (`perception_aggregator.py` + the pure
`box_geometry.py`). The twin and HMI contracts are **unchanged** — `box_footprint`
still flows the same way; it's just correct. `MapCanvas.tsx` needs no change.

### Pure additions to `box_geometry.py` (ROS-free, unit-tested)

- `Transform2D(cos, sin, tx, ty)` + `apply((x,y))` — rigid 2D transform
  `map = R·json + t`. `IDENTITY_2D`.
- `solve_rigid_2d(src, dst) -> Transform2D` — closed-form 2D Kabsch (no scale):
  `θ = atan2(Σ cross, Σ dot)` over centred points, `t = dst_c − R·src_c`. Returns
  identity for 0 points and identity-rotation (translation only) for 1 point or a
  degenerate (collinear/coincident) set.
- `table_rect_corners(rect) -> [(x,y)×4]` — the four `x0,y0,x1,y1` corners, CCW.
- `nearest_table_rect(tag_xy, tables) -> rect | None` — nearest table by centre
  distance (mirrors `approach._nearest_table`; kept local to avoid a cross-package
  dependency).
- `box_geometry_from_corners(corners, toward=None) -> BoxGeometry | None` — builds
  a `BoxGeometry` whose `footprint()` reproduces the (map-frame) rectangle: long
  edge → lateral axis, short edge → depth/normal, normal signed toward `toward`
  (the tag, so "front" faces the aisle), origin = front-edge centre. `place()` then
  scatters blooms *inside* the real rectangle.

### `perception_aggregator.py`

- New param `tag_locations_file` (default `''` → bundled `greenhouse_sim`). Load
  `tables` + tag JSON `(x,y)` at init in a `try/except` — on failure log once and
  fall back to the legacy synthesised box (keeps fixtures/tests green without the
  package).
- `_layout_transform()` — build `(json_xy, map_xy)` correspondences from
  registry tags with a committed pose and `sightings ≥ min_sightings`; return
  `solve_rigid_2d(...)`. Identity when < 2 tags (sim is already frame-aligned).
- `_box_geom_for(rec)` — look up the tag's JSON xy → `nearest_table_rect` →
  transform corners by `_layout_transform()` → `box_geometry_from_corners(...,
  toward=tag)`. Fall back to `box_from_tag(rec.pose, self._box)` when layout is
  unavailable/degenerate.
- `_fuse_flower` uses `_box_geom_for(rec)` in place of `box_from_tag`, so both
  `box_footprint` **and** bloom placement use the corrected geometry.

### Launch wiring

`sim_full.launch.py` and `sim_autonomy.launch.py` already compute
`widened_tag_locations`; pass it as the aggregator's `tag_locations_file` so
perception and mission share one source. `hardware.launch.py` likewise. Default
`''` keeps every other launch correct (same bundled coords).

## Tests

- `test_box_geometry.py` (TDD, pure): `solve_rigid_2d` for identity / translation
  / rotation+translation / degenerate / empty; `table_rect_corners`;
  `nearest_table_rect`; `box_geometry_from_corners` footprint round-trip + normal
  orientation + recovered width/depth.
- `test_perception_aggregator.py`: with two fake tags at known map poses, assert
  the emitted `box_footprint` equals the registered table rectangle (size +
  position), not the 0.80 × 0.40 synthesised box.
- Sim run (`sim_full`) to eyeball the overlay against the occupancy grid.

## Limitations (v1)

- Registration needs ≥ 2 discovered tags; before that it falls back to identity
  (fine in sim, may be offset on hardware until two tags are seen).
- One rigid fit for the whole layout assumes the physical greenhouse matches the
  JSON (true for the course demo). Per-bench lidar snapping is the documented
  upgrade path if localisation drift ever shows.
