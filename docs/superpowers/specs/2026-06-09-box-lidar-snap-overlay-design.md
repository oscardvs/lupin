# Planter-box lidar-snap + staged-confidence map overlay

- **Date:** 2026-06-09
- **Status:** Approved (design) — pending implementation plan
- **Scope:** `lupin_perception` (box_layout_publisher), `lupin_twin`, `lupin_web` (MapCanvas)
- **No `.msg` / contract changes.** No new topics.

## 1. Problem & context

The planter-box overlay on the HMI map ("map digital twin") is visibly off — wrong
position relative to the lidar-built occupied cells, and not reliably anchored to the
benches. Investigation (2026-06-09) established:

- **Box position is NOT lidar-derived.** It comes from AprilTag detection + a rigid
  registration of the known JSON layout onto the SLAM `map`
  (`box_geometry.solve_rigid_2d`, driven by the discovered-tag constellation in
  `box_layout_publisher`). The occupancy grid is an *independent* source. So lidar
  coverage today is corroboration, not placement.
- **Registration is the dominant position-error source.** In sim the SLAM `map` frame
  is rotated 90° + translated from the JSON frame (robot spawns at `2.0, 3.0, yaw=π/2`,
  slam_toolbox anchors `map` at start), so the whole transform must be recovered from
  tags; the recovery is corrupted because `generate_greenhouse_world.py` *snaps* each
  tag onto its table face (mean 3.3 cm, up to 10.5 cm, inconsistent directions) while
  the publisher registers against the *raw* JSON coords. On hardware the analogue is
  JSON tag coords ≠ true mounted positions. Under-constrained early (< 2 tags →
  identity), rotation error amplifies with distance.
- **Sizes already match** (operator-confirmed). Bundled `greenhouse_sim` JSON and
  `tag_locations_widened.json` have byte-identical table rectangles (1.10 × 0.25), and
  `generate_greenhouse_world.py` renders the sim collision box from those same numbers.
  The fallback `StandardBox` (0.80 × 0.40) would mismatch, but that is not the reported
  issue here.
- **A staleness bug freezes the overlay.** `box_footprint` is written *only* inside
  `perception_aggregator._emit_flower_observation`, which fires only when the flower
  *summary* changes (`if changed`). The twin copies whatever the last `FlowerObservation`
  carried and has no independent box source. So the rectangle freezes at the geometry
  that existed at the last species/count change and never refreshes as registration
  improves. Boxes also only appear for *scanned* tags, not as a standing layout.
- **The HMI render is faithful.** `MapCanvas` draws both the OccupancyGrid and the box
  rectangle through the same `proj.worldToCanvas`, so a visible offset is a true
  world-coordinate offset, not a canvas bug.

The environment is only partially mapped while boxes are being placed, so the overlay
must degrade gracefully: show a confident, lidar-corrected box where the bench has been
sensed, and a clearly-provisional box where it has not.

## 2. Goals / non-goals

**Goals**
1. Correct box *position* against the lidar where the bench has been sensed (fix
   "off in position" for real, not just hide it).
2. Show **all registered benches** as a standing layout overlay, independent of flower
   scanning.
3. Render staged **confidence**: continuous ghost → solid as lidar corroborates each box.
4. Dissolve the staleness bug (geometry refreshes live, latest-wins).
5. The corrected geometry also benefits the aggregator's flower-box gate (free, no change
   there).
6. No message/contract changes; fully revertible via a parameter.

**Non-goals (YAGNI for v1)**
- Global layout-to-map ICP (Approach C) — reserved as an upgrade if per-box snaps prove
  wobbly.
- Correcting box *size* or the along-edge (tangential) position — dimensions are known
  and the bench slides freely along its own length in lidar; tags own that.
- Server-published confidence in `box_geometry_json` (the HMI computes coverage against
  the same `/map` it already renders, guaranteeing the styling matches the on-screen
  lidar with no frame/timing skew).
- Removing the legacy `FlowerObservation.box_footprint` write (kept as fallback).

## 3. Approach (B + staged confidence + all benches)

Snap the box server-side where the registration already lives (`box_layout_publisher`);
route the snapped geometry to the twin as a **standing per-tag box channel**; compute the
staged confidence client-side in the HMI against the OccupancyGrid it already holds.

## 4. Data flow (4 touch points; everything else unchanged)

```
box_layout_publisher    register layout→map (tags)   +NEW snap front-face to /map
        │                                              publish snapped boxes (SAME schema)
        ├─► perception_aggregator   flower-gate consumes snapped boxes  (free win, no change)
        └─► lupin_twin   +NEW subscribe box_geometry_json → set per-tag box_footprint
                         for ALL discovered tags (authoritative; reuses TwinState)
                         │
                         ▼
              HMI MapCanvas   +NEW front-face coverage vs OccupancyGrid → ghost→solid alpha
```

No new topics, no new messages, no new HMI data source.

## 5. Component 1 — `box_layout_publisher` front-face snap

The placement fix. Each 1 Hz publish tick, after building the tag-registered prior
`MapBox`, refine it against the latest `/map` before publishing.

**New input:** subscribe `map_topic` (`/map`, `nav_msgs/OccupancyGrid`,
RELIABLE; store latest). Snap is a no-op until a grid arrives.

**Per box (snap the freshly-registered prior, never the previous output → no drift):**
1. **Front face** = aisle-facing long edge. With `n = (cos yaw, sin yaw)` (outward
   normal) and `l = (-sin yaw, cos yaw)` (lateral): face centre `Fc = (x, y) + n·depth/2`,
   spanning `±width/2` along `l`.
2. **Gather occupied cells** with value ≥ `snap_occupied_threshold` (65) inside a band
   around the face: `|along-n distance to face| ≤ snap_band_m` and within the width
   extent along `l`. Scan only the grid's local bounding box (compute min/max row/col
   from the band rectangle), not the whole map. Convert cells →
   map coords: `wx = origin.x + (col+0.5)·res`, `wy = origin.y + (row+0.5)·res`.
3. **Fit a line** (2×2 PCA on the points). Principal axis = line direction.
4. **2-DOF correction:**
   - `yaw` ← orientation of the new outward normal (perpendicular to the line),
     sign-disambiguated toward the prior `n` (`dot(new_n, prior_n) > 0`).
   - shift along `n` by `Δ = (centroid − Fc)·n_prior` so the front face lands on the line.
   - Width, depth, height and the along-`l` position are unchanged (tags own them).
5. **Guards — on any failure keep the prior box unchanged (graceful fallback):**
   - `len(points) ≥ snap_min_points`,
   - PCA elongation `λ1/λ2 ≥ snap_min_elongation` (a blob is not a line),
   - clamp `|Δyaw| ≤ snap_max_yaw_deg` and `|Δ| ≤ snap_max_shift_m`.

**New parameters**

| param | default | meaning |
|---|---|---|
| `map_topic` | `/map` | OccupancyGrid to snap against |
| `snap_enable` | `true` | master switch; `false` ⇒ today's behaviour |
| `snap_occupied_threshold` | `65` | cell value ≥ ⇒ occupied (matches HMI) |
| `snap_band_m` | `0.12` | half-band around the front face (along `n`) |
| `snap_min_points` | `8` | min occupied cells to attempt a fit |
| `snap_min_elongation` | `3.0` | min PCA λ1/λ2 to trust a line |
| `snap_max_yaw_deg` | `12` | clamp on yaw correction |
| `snap_max_shift_m` | `0.15` | clamp on normal shift |

The snap math is pure and lives in `box_geometry.py` (ROS-free), wrapped by the node —
mirroring how `solve_rigid_2d` / `map_box_from_rect` are already factored.

## 6. Component 2 — `lupin_twin` standing box channel

All benches + staleness dissolved, reusing the existing `TwinTagState.box_footprint`
(Polygon) — **no `.msg` change**.

- **New subscription:** `box_geometry_topic` (`/perception/box_geometry_json`,
  `std_msgs/String`), RELIABLE+VOLATILE (compatible with the latched RELIABLE producer;
  1 Hz republish covers late joins). Parse the same JSON the aggregator parses
  (`[{id,x,y,yaw,width,depth,height}, …]`).
- **Per box:** upsert the tag entry keyed by `id` (create a box-only entry if discovery
  has not pinned it yet — a box without a pose still renders its rectangle in the HMI, so
  `_publish_state` must include box-only entries rather than filtering for a missing pose),
  and set `box_footprint` to the 4 corners of the snapped rectangle in the canonical
  **FL, FR, BR, BL** order (matches `box_geometry.footprint()`), computed inline (no
  dependency on `lupin_perception`):
  ```
  n=(cos yaw, sin yaw); l=(-sin yaw, cos yaw); c=(x,y)
  FL=c + n·d/2 + l·w/2 ; FR=c + n·d/2 - l·w/2 ; BR=c - n·d/2 - l·w/2 ; BL=c - n·d/2 + l·w/2
  ```
- **Authority rule:** the box channel is the source of truth for `box_footprint`. The
  `FlowerObservation` footprint path (`_ingest_flower` → `FlowerUpdate.box_footprint` →
  `state.py` merge) becomes **fallback-only**: it sets `box_footprint` only when no
  box-channel footprint exists for that tag. Implemented with a small per-tag source flag
  in `TwinStateStore` so the flower merge does not clobber a channel footprint. This is
  what dissolves the staleness bug — geometry now refreshes at 1 Hz, latest-wins,
  decoupled from flower-summary changes.

`_publish_state` is unchanged: it already emits `buf.box_footprint` as the
`TwinTagState.box_footprint` Polygon.

## 7. Component 3 — HMI `MapCanvas` staged-confidence render

The only HMI change, contained to the existing box-drawing block (the flowers layer,
~lines 836–853 today).

- Add a pure helper `frontFaceCoverage(corners, occupancyGrid)`:
  - front edge = `corners[0] → corners[1]` (FL → FR), length = box width;
  - sample ~24 points along it; a sample is *covered* if an occupied cell
    (value ≥ 65) lies within ~0.10 m in `mapRef.current`;
  - return `covered / sampled ∈ [0, 1]`.
- **Ghost → solid lerp** by coverage (continuous):
  - `coverage ≤ 0.1` → faint dashed "predicted": stroke α ≈ 0.18, no fill;
  - `coverage ≥ 0.6` → solid "confirmed": stroke α ≈ 0.5, fill α ≈ 0.10;
  - linear interpolation between; dash fades out as coverage rises.
- Low-confidence boxes stay **faintly visible** (not hidden) — predicted layout remains
  legible during early exploration.
- Recompute coverage only when the map bitmap key or the box set changes (reuse the
  existing throttle/animation-loop machinery); not per frame.

## 8. What explicitly does NOT change

- `box_geometry_json` schema (same fields; values are now snapped).
- `perception_aggregator` — its flower-box gate already consumes `box_geometry_json`, so
  it inherits the corrected geometry for free. The legacy `box_footprint`-on-flowers
  write stays (twin treats it as fallback).
- `TwinState` / any `.msg`. `_publish_state`. The HMI's map/grid/projection code.
- Registration (`solve_rigid_2d`) — the snap refines its output, it does not replace it.

## 9. Testing

- **`box_geometry.py` snap unit tests** (pure, ROS-free, as the module is already tested):
  synthetic occupied-point sets →
  - recovers a known yaw + normal-offset correction;
  - rejects on too-few-points, blob (low elongation), and over-clamp, leaving the prior.
- **Twin unit tests:** a `box_geometry_json` message sets `box_footprint` on a
  discovered-but-unscanned tag; the box channel wins over a subsequent flower-obs
  footprint; a flower-obs footprint is used when no channel box exists.
- **HMI unit test:** `frontFaceCoverage` against a hand-built OccupancyGrid (full / half
  / no coverage; rotated face).
- **Manual (sim):** run `sim_full`, drive past a bench, confirm the box snaps onto the
  occupied strip and the render goes ghost → solid; confirm an un-driven bench shows a
  faint ghost.

## 10. Risks & fallbacks

- **Sparse / early map** → snap guards fail → prior shown as faint ghost. Correct and
  graceful.
- **Clutter in the band (wrong-bench cells)** → elongation + clamp guards reject or bound
  the nudge.
- **Snap oscillation as the map fills** → bounded by clamps; always snapping the prior
  (not the previous output) prevents accumulation.
- **Global kill switch:** `snap_enable=false` restores today's exact behaviour.
- **Twin authority regression:** if the source flag is mis-wired, worst case is the old
  flower-coupled footprint — i.e. today's behaviour.

## 11. Rollout

- Generic infra (perception + twin + web), applies to sim and hardware → land on `main`,
  fast-forward `sim` / `hardware` per the team branch-split convention.
- Verify in `sim_full` first (cheap, observable), then on hardware during the next bring-up.
