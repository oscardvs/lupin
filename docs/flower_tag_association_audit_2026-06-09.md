# Flower→tag association audit + hardening (perception aggregator)

**Date:** 2026-06-09
**Author:** Oscar (Team Lupin)
**Scope:** `lupin_perception/lupin_perception/perception_aggregator.py` +
`box_geometry.py`
**Branch:** `fix/flower-tag-association`

---

## 1. The exact attribution path

The only thing that binds a YOLO bloom to an AprilTag is **which tag the mission
is parked SCANNING**. The box geometry only *places* the bloom inside that tag's
box; it never re-chooses the tag. Step by step:

1. **`_on_yolo_detections(msg)`** (`:433`) parses the gripper-cam YOLO JSON. For
   each detection it gates on `confidence ≥ yolo_min_confidence` (0.35) and a
   valid class index, then computes a lateral cue:
   - `bbox_cx_norm = clamp((cx / detection_image_width − 0.5)·2, −1, 1)`
     (`detection_image_width = 640`).
   - `f = lateral_fraction(shoulder_pan, bbox_cx_norm, pan_half_span=0.5,
     camera_half_fov=0.5)` — **clamped to `[−1, 1]`** (`box_geometry.py:178`).
   - appends `(f, name, conf)` to `frame_dets`, then calls `_fuse_flower`.

2. **`_fuse_flower(now, frame_dets)`** (`:535`) prunes the time window and asks
   `tag_id = _focus_tag()`.

3. **`_focus_tag()`** (`:472`) — **the sole association decision, and it is purely
   temporal**:
   - mission seen & *scanning* (`lifecycle_state ∈ {MONITORING, INSPECTING}` and
     `'SCANNING' ∈ mission_phase`) and `current_target ∈ registry`
     → return `current_target`;
   - mission running but not scanning → `None` (no attribution — avoids drive-by);
   - standalone (no mission) → nearest tag in the last tag-detection frame.

4. `_scan_dets` is reset when `tag_id` changes (so one box's blooms don't bleed
   into the next) and extended with `frame_dets`.

5. **Summary** (`:558`): dominant non-bug species (best confidence) + `anomaly`
   if the bug class appears, computed over the tag-scoped `_scan_dets`.

6. **Placement** (`:569`): `geom = _box_geom_for(rec)` (the registered real table
   rectangle, the legacy tag-anchored `StandardBox`, or `None`); `bin_detections`
   bins the `(f, …)` tuples into lateral columns and places one bloom per column
   *inside* that box.

7. Emit a `KIND_FLOWER` Observation on summary or column-count change.

**Key point:** `lateral_fraction` and `bin_detections` consume `f` only to decide
*where in `current_target`'s box* a bloom is drawn. The clamp in step 1 means a
bloom whose true bearing points **past the end of the current bench** is squashed
to `f = ±1` and dumped into that bench's edge column — silently misattributed
rather than rejected. There is no test that the bloom actually lies on the
current tag's table.

---

## 2. Geometry of the failure — worked example

Real parameters pulled from the nodes:

| Quantity | Value | Source |
|---|---|---|
| Camera half-FOV | `0.5 rad` (28.6°) | `camera_half_fov` default |
| Arm pan half-span | `0.5 rad` (28.6°) | `pan_half_span` default |
| Image width | `640 px` | `detection_image_width` default |
| Approach standoff | `0.4 m` (clamped 0.2–1.5) | mission `approach_standoff_m` (`node.py:347`) |
| E–W bench (Table1) | `1.10 m` lateral × `0.25 m` depth | `tag_locations.json` |
| Tags per bench | ≈ 4 (29 tags / 10 tables) | `tag_locations.json` + spec |

`lateral_fraction` forms `bearing = pan + bbox_cx_norm·camera_half_fov` and
`f = clamp(bearing / span, −1, 1)` with `span = pan_half_span + camera_half_fov =
1.0 rad`. So **`f` saturates once `|bearing| ≥ 1.0 rad` (57°)**.

The camera optical centre sits ≈ `standoff` (0.4 m) out from the bench front
edge, so the bench centre is ≈ `0.4 + 0.125 = 0.525 m` away. Lateral half-width
imaged at the bench plane:

- FOV only (`pan = 0`): `0.525·tan(0.5) = 0.287 m` → **0.57 m** wide.
- With the full pan sweep (`±0.5 rad`): angular `±1.0 rad` →
  `0.525·tan(1.0) = 0.818 m` → **1.64 m** swept.

The bench is **1.10 m** long, so the swept field **over-covers it by ≈ 0.53 m**
(≈ 0.27 m of aisle past *each* end). The bench's own subtended half-angle is
`atan(0.55 / 0.525) = 0.808 rad`, so bearings in `(0.808, 1.0] rad` already point
**past the bench end** yet still produce `|f| ≤ 1` (in-box placement), and
bearings `> 1.0 rad` saturate to `f = ±1` exactly.

### When misattribution actually happens

- **Scenario A — same-bench sibling tags (common).** Scanning tag 6 (x = 1.5, the
  left end of Table1), the ±0.82 m sweep spans x ≈ [0.68, 2.32], covering tag 21
  (1.6) and much of the bench. Tulips physically near a sibling tag are folded
  into tag 6's summary. **The spatial gate intentionally does *not* drop these** —
  they are genuinely on tag 6's table (same rectangle), so placement is correct;
  only the per-tag *summary* over-credits. (Residual limitation, §5.)

- **Scenario B — cross-bench over-reach (the bug target).** When the robot parks
  at a bench *end*, or AMCL drift offsets the park pose by ~0.2 m, or the pan
  swings to its limit, the far edge of the sweep images a *neighbouring* bench or
  a stray plant in the aisle. Those detections have `|bearing| > 0.808 rad` (their
  true position is off the current bench) → currently **clamped to `f = ±1` and
  attributed to `current_target`**. This is the documented v1 failure.

- **Scenario D — empty bench inherits a neighbour's species.** `current_target`
  is a real tag on a bench with no blooms, but the sweep edge catches a
  neighbour's blooms → the empty bench reports the neighbour's species/anomaly.
  This is the "a tag with no real blooms can be assigned a neighbour's
  species/anomaly" case from the task.

### When it is geometrically impossible

- **Scenario C — the bench across the aisle / behind the robot.** The camera
  faces *into* the current bed (yaw points back at the tag); the opposite bench is
  behind the camera and is never imaged. No gate is needed for it.
- **Depth-separated background** only contaminates if a far bench is both inside
  the ±0.82 m lateral window *and* tall enough to clear the front bench — rare in
  the sparse course layout (benches ≥ 0.45 m apart, mostly perpendicular or
  across the aisle).

**Net:** in the demo layout the dominant real error is **Scenario A (same-bench,
not fixable by a box gate and arguably correct)**; the box gate's value is killing
**B and D** — the cross-bench/over-reach blooms that the clamp currently hides.

---

## 3. Proposed options (ranked)

**(a) Spatial box-membership gate — CHOSEN.** Project each detection's
*unclamped* lateral fraction into the map via the existing registered box
geometry and **drop** detections whose projected position falls outside
`current_target`'s table rectangle (with a small metric margin). Reuses
`_box_geom_for` / `_layout_transform`; the geometry already exists. Lowest risk:
it only *removes* clearly-off-bench detections and is a strict no-op for blooms
that already place in-box (which is every legitimate sim/in-bench bloom). Drop
(not reassign) keeps the change local — the neighbour gets credited when the
mission parks there.

**(b) Confidence/stability gate over the dwell window.** Require a species to be
seen for N frames / a fraction of the dwell before committing. Cheap and
complementary, but it suppresses *transient* noise, not a *persistently* visible
neighbour bench — it does not solve the spatial bug. Good follow-up.

**(c) Depth-based association.** Use the Orbbec depth at the bloom pixel to reject
blooms outside the bench depth band. Most physically correct, but the *gripper*
cam (YOLO input) is uncalibrated and has no registered depth, and depth at 0.4 m
is at the near edge of the Orbbec range. High integration cost; deferred.

---

## 4. Implemented fix

A spatial **box-membership gate**, behind `flower_box_gate` (default **ON**):

- `box_geometry.py`
  - `lateral_fraction(..., clamp=True)` — new `clamp` flag so the node can store
    the **unclamped** fraction (over-reach survives to the gate);
    `bin_detections` already re-clamps for placement, so placement is unchanged.
  - `BoxGeometry.contains(x, y, margin_m=0.0)` — exact point-in-box test in the
    box's own lateral/normal frame: inside iff `|f| ≤ 1 + margin/half_w` and
    `−margin/depth ≤ d ≤ 1 + margin/depth`.
- `perception_aggregator.py`
  - params `flower_box_gate` (bool, default `True`) and `flower_box_gate_margin_m`
    (default `0.10`).
  - store the **unclamped** lateral fraction in `_scan_dets`.
  - `_box_geom_for` now reports its source (`layout` vs `legacy`); a registration
    helper counts the `(json_xy, map_xy)` tag correspondences.
  - in `_fuse_flower`, the gate engages **only** when `flower_box_gate` is on, the
    box is a **registered layout** box, **and ≥ 2 tags are registered** (a real
    JSON→map fit exists). It then drops `_scan_dets` entries whose
    `geom.place(f, base_depth)` lands outside `geom.contains(...)`, and computes
    both the species summary and `bin_detections` from the filtered set.

**Fallback preserved (no temporal-path regression):** when the gate is off, the
box is the legacy tag-anchored `StandardBox`, the box is `None` (degenerate tag
normal), or `< 2` tags are registered (identity / unregistered layout), **all
detections are kept** exactly as today. This keeps the sim flower-scan demo and
the `< 2`-tag fallback behaving identically.

---

## 5. Remaining limitations

- **Same-bench sibling over-crediting (Scenario A)** is unchanged: all tags on one
  1.10 m bench share one rectangle, so a sibling's blooms still fold into
  `current_target`'s *summary*. Fixing this needs per-tag sub-regions of the bench
  or true depth/3-D association — out of scope here.
- The membership projection uses the base depth (mid-box), so the gate is
  effectively **lateral** (front/back rejection needs real depth — option (c)).
- The gate trusts the JSON→map registration; with only 1 tag registered it is
  deliberately disabled (the unregistered box position is not yet trustworthy on
  hardware).
- `pan_half_span` / `camera_half_fov` are nominal; the gate margin
  (`flower_box_gate_margin_m`, 0.10 m) absorbs the resulting slack. Tighten once
  the real gripper-cam FOV is measured.
