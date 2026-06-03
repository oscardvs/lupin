# Greenhouse Mission Redesign — Design Spec

**Date:** 2026-06-03
**Branch:** `feat/mission-perception-redesign`
**Author:** Oscar (Team Lupin) · RO47007 MDP
**Status:** Approved design, pre-implementation

---

## 1. Motivation

A full sim mission run (exploration → monitoring → return) surfaced five problems.
Investigation (file:line confirmed below) showed three are real bugs, one is an
HMI mislabel, and one is *by-design behaviour* that the map presents misleadingly.
This spec redesigns the mission logic, flower localization, map rendering, and arm
scan behaviour around a single foundation: **the AprilTag is a calibrated 3D anchor,
and the planter boxes are standard, so box geometry is a rigid transform from the tag.**

User decisions taken during brainstorming:

- **Horizon:** deeper redesign (no demo pressure; do it properly).
- **Flower position:** tag-anchored geometry (not depth projection — the wrist cam is
  uncalibrated with no depth; geometry is deterministic and works in sim + hardware).
- **Scan motion:** arm pan-sweep with the base parked.
- **Mission completion:** one full monitoring sweep, then return and finish.
- **Discovery scope:** keep the `discovery_goal` cap (operator bumps it in the HMI).
- **Box on map:** draw the box footprint rectangle.

---

## 2. Confirmed root causes (investigation record)

| # | Symptom | Reality | Evidence |
|---|---------|---------|----------|
| 1 | "orchestrator error" in the log | **HMI mislabel.** `mission.ts` labels *every* `last_error` change as a red "orchestrator error". The value was just `battery_low`. | `lupin_web/web/src/lib/mission.ts:258-266`; `node.py:906` sets `last_error="battery_low"` |
| 2 | Mission ended in DONE on low battery instead of docking + pausing | **Real bug.** The dock `NavigateToPose` returned ABORTED (`nav_status_6`), and `_on_return_nav_result` treats any non-SUCCEEDED dock as "non-fatal → DONE", even on the battery path. | `node.py` `_on_return_nav_result` (non-SUCCEEDED branch → `returned()`) |
| 3 | Monitoring re-visited tags forever in order 15,18,2,3,5 | **By design + ordering bug.** `MonitoringMission.is_complete()` is `not self._tags` (never true → infinite loop); tags are **string-sorted**. | `exploration_mission.py:90,101-103,120-137` |
| 4 | Only tag 15 showed full climate | **By design.** Each tag has a fixed sensor subset in the greenhouse sim's `tag_locations.json` (tag 15 = temp/humidity/co2/light; tag 2 = soil_moisture; tag 5 = light; …). The HMI falls back to a misleading mid-ramp colour for absent sensors. | bridge `get_sensor_data(tag_id)`; HMI pin colour `MapCanvas.tsx:547-552` |
| 5 | Every tulip classified white | **Aim + timing, not the classifier.** HSV bands are correct/non-overlapping; the wrist cam frames the **white AprilTag backing plate** / grey walls and the largest-blob rule picks white. Arm has ~1 s margin (3 s to reach `inspect`, 4 s dwell). | `sim_flower_detector_node.py` HSV bands; arm timing in `node.py` |
| 6 | Tulips render on top of the tag | **Real bug.** Flowers are pinned to the tag's pose; association is pure temporal co-location with no 3D position. | `perception_aggregator.py:396` `flower.pose.pose = rec.pose`; `:401`; `_focus_tag()` `:323-344`; HMI `MapCanvas.tsx:588` |

---

## 3. Foundation — box-from-tag geometry

New pure-Python module **`lupin_perception/box_geometry.py`** (no ROS imports,
unit-testable, mirrors how `lupin_mission/approach.py` isolates geometry).

**Inputs:** a tag's map-frame `Pose`, a `StandardBox` config, optional per-tag
`tables` bbox override, optional Stream-D vision refinement.

**Box frame from the tag pose:**

- Normal `n` = tag local **+Z** projected to the XY plane (same derivation as
  `approach.compute_discovered_approach()`, `approach.py:223-235`). Points out of
  the bed into the aisle.
- Lateral axis `l` = `n` rotated +90° in XY.
- Box interior extends **opposite** `n` (into the bed) by `depth`, spans `±width/2`
  along `l`, with a configurable tag-mount offset (tags sit mid-front-face at
  `tag_mount_height`).

**Bloom placement:** a detection with lateral fraction `f ∈ [-1, 1]` and depth
fraction `d ∈ [0, 1]` maps to `xy = tag_xy − n·(d·depth) + l·(f·width/2)`.

**Box footprint:** the four interior corners in the map frame (returned as a
`geometry_msgs/Polygon` by the ROS callers; the module returns plain tuples).

**`StandardBox` defaults** (parameters, calibrate against the world/hardware):
`width=0.80 m`, `depth=0.40 m`, `height=0.30 m`, `tag_mount_height=0.10 m`,
`tag_lateral_offset=0.0 m`. When a `tables` bbox is available for the tag's nearest
table (`tag_locations.load_default_tables`), use its extent in place of
`width`/`depth`; Stream D refines pose/extent on top. Degrades to pure
tag-anchored geometry when neither override is present.

---

## 4. Stream A — Mission logic pass

All changes in `lupin_mission/` (`node.py`, `exploration_mission.py`) +
`lupin_web` for the label. Generic → **main**.

### A1. One full sweep, then return
- `MonitoringMission.is_complete()` (`exploration_mission.py:101`): return
  `self.cycles >= self._target_cycles` (default `target_cycles=1`) instead of
  `not self._tags`.
- Wire `inspection_complete` (existing trigger, `node.py:158`) to also fire from
  `MONITORING_PUBLISHING → RETURNING`. `_enter_publishing` already calls
  `self.inspection_complete()` when `is_complete()` (`node.py:1724`); add the
  `MONITORING_PUBLISHING` source to that transition.
- This RETURNING leg is **terminal, non-battery**: `_docked_for_battery=False`,
  `_return_resumable=False`, so dock arrival → `returned()` → DONE.

### A2. Coherent sweep order
- Replace `self._tags = sorted(discovered.keys())` (`exploration_mission.py:90`)
  with **greedy nearest-neighbour** over the discovered map poses, starting from
  the robot's current pose; fall back to `numeric_string_sort_key`
  (`tag_locations.py:83`) when poses are unavailable.

### A3. Battery → dock-and-pause that survives a failed dock
- In `_on_return_nav_result`, the non-SUCCEEDED branch must branch on intent:
  - **Battery return** (`_docked_for_battery`): retry the dock goal up to
    `dock_retry_max` (default 3); if still failing, **pause in place**
    (`_paused=True`, keep resumable) — never DONE.
  - **Completion / abort return:** unchanged (failure → DONE so the operator
    isn't stuck in RETURNING).

### A4. Honest event labelling
- Add `string last_event` + `uint8 last_event_severity` (INFO/WARN/ERROR
  constants) to `MissionState.msg`; the orchestrator classifies known notices
  (`battery_low`, `*_timeout`, `nav_status_*`, `scan_failed`) as INFO/WARN and
  reserves ERROR for genuine faults / `state==FAULT`.
- `mission.ts:258-266`: drive the event log from `last_event_severity` (tone +
  friendly label) instead of hard-coding `'orchestrator error'`/`tone:'err'`.

---

## 5. Stream B — Flower localization + map render

### B1. Message schema (`lupin_msgs`)
- **New** `FlowerPoint.msg`:
  ```
  geometry_msgs/Point position   # map frame
  string species                 # tulip_red | tulip_white | tulip_pink
  float32 confidence             # [0,1]
  bool anomaly
  ```
- `FlowerObservation.msg`: **add** `FlowerPoint[] flowers` and
  `geometry_msgs/Polygon box_footprint`; keep `tag_id` and the summary
  `species`/`confidence`/`anomaly` (dominant across `flowers`). `pose` stays the
  tag pose (now correctly documented as the tag, not the plant).
- `TwinTagState.msg`: **add** `FlowerPoint[] flowers` and
  `geometry_msgs/Polygon box_footprint`. Existing `pose`/`species`/`anomaly`
  remain as the tag pose + summary.

### B2. `perception_aggregator.py`
- Stop pinning flowers to the tag (`:396`).
- Parse `bbox_xyxy` in `_on_yolo_detections` (currently dropped); add a
  `/joint_states` subscription to read `shoulder_pan` (the lateral cue — the
  pan-sweep makes pan ↔ lateral position clean), refined by bbox centre-x.
- During a scan, accumulate `(pan_angle, bbox_cx, class, conf)` and **bin into
  `lateral_columns` (default 7)** across the box width. Per occupied column:
  dominant species → one `FlowerPoint` placed via `box_geometry` (lateral
  fraction from the column, a representative depth fraction with small
  deterministic per-column jitter so blooms read as scattered, not a line).
  `bug` class → `anomaly`.
- Emit one `KIND_FLOWER` `Observation` per box-scan carrying the located
  `flowers[]` + `box_footprint`.

### B3. `lupin_twin`
- Store per-box `FlowerPoint[]` + `box_footprint` (latest-wins per sweep; a
  merge-by-proximity hook left for a future continuous-patrol mode). Publish in
  `/twin/state`.

### B4. HMI `MapCanvas.tsx`
- Tag marker → **diamond** on the box edge (`:556-565`), distinct from flowers.
- Draw the **box footprint** as a faint rectangle from `box_footprint`.
- Flowers → **species-coloured filled dots** at `FlowerPoint.position`
  (`:585-606`, using `flowers[]`, not `t.pose`). `speciesColor()` already maps
  all three colours (`lib/flowers.ts`).
- **Honest climate:** when the selected sensor channel is absent on a tag, render
  a clearly-neutral "no sensor" pin (hollow/hatched) instead of the mid-ramp
  colour (`:547-552`); surface each tag's available sensors. (Addresses the
  "only tag 15 coloured" perception — by-design, so we make the map honest, not
  "fix" a non-bug.)
- Update TS types in `types/ros.ts` (`TwinTagState`, new `FlowerPoint`).

---

## 6. Stream C — Arm pan-sweep + only-white fix

### C1. Settle before drive
- Set `arm_travel_settle_s` (~3.2 s) in the sim launch (currently defaults to 0,
  so the arm sweeps through space while the base drives). Confirm
  `_schedule_nav_after_arm_settle` fires. Sim-launch param → **sim** branch.

### C2. Pan-sweep scan
- Add a sweep-trajectory builder to `lupin_hmi/arm_traj.py` and a `scan`
  capability on `arm_preset_server` (single interface — no speculative duplicate
  topic+service): hold a bloom-framing `(shoulder_lift, elbow, wrist)` pose while
  sweeping `shoulder_pan` from `−θ → +θ` (default `θ=0.5 rad`, clamped to joint
  limits) over the dwell.
- The mission triggers `scan` at `MONITORING_SCANNING`; `flower_scan_dwell_s` =
  sweep duration so the scan window matches the motion. The pan trajectory is the
  lateral cue consumed by B2.

### C3. Only-white fix
- Tune the bloom-framing pose so the camera looks **into the box at bloom height**,
  above the white tag plate.
- In `sim_flower_detector_node.py` (and the hardware YOLO path config): apply an
  **ROI mask** excluding the frame region where the tag plate sits, plus a
  **blob-size window** so a plate-sized white blob can't beat a bloom-sized one.
  Detector ROI tuning → **sim** branch; the size-window param is generic.

---

## 7. Stream D — OpenCV box detector (refinement layer)

- New node that detects the standard planter-box rectangle in the camera image
  (contour/edge fit or colour segmentation of the box material) and **refines /
  validates** the tag-anchored box footprint: corrects tag-mount error, confirms
  a box is actually present before placing flowers, and supplies the real box
  extent to the arm-sweep limits (C2) and the detector ROI (C3).
- **Degrades gracefully** to pure tag-anchored geometry when no box is found —
  additive, never a single point of failure.
- Prototype with a live overlay in Gazebo first. Node generic → **main**; launch
  wiring → **sim**.

---

## 8. Data flow (after redesign)

```
gripper cam ─▶ sim_flower_detector / yolo_detector  (class, conf, bbox)
                          │
/joint_states (shoulder_pan) ─┐
body Orbbec ─▶ tag_annotator ─┼▶ perception_aggregator
                          │    │   ├─ map→tag_<id> TF  (tag pose)
                          │    │   ├─ box_geometry      (box footprint)
   Stream D box refine ───┘    │   └─ bin detections → FlowerPoint[]
                               ▼
        Observation(KIND_FLOWER: flowers[], box_footprint, tag_pose)
                               ▼
                         lupin_twin ──▶ /twin/state (TwinTagState: flowers[], box_footprint)
                               ▼
                       HMI MapCanvas (diamond tag · box rect · coloured flower dots · honest climate)
```

---

## 9. Branch & sequencing

Per the three-branch model (generic → **main**, sim-only → **sim**, then
fast-forward) and the feature-branch + GitLab-MR workflow (no co-author trailer).

- Generic: `lupin_msgs`, `box_geometry`, `perception_aggregator`, mission logic,
  `lupin_twin`, HMI render + types, `arm_traj`/`arm_preset_server`, `mission.ts`,
  Stream-D node.
- Sim-only: `sim_flower_detector` ROI, sim launch params (`arm_travel_settle_s`,
  dwell, sweep wiring), world tweaks, Stream-D launch wiring.

**Order:** A (independent, high value) → schema + B → C (couples to B via the pan
cue) → D (refinement, last).

---

## 10. Testing

- **Unit:** `box_geometry` (frame derivation, placement, degenerate normals,
  tables-bbox override) — pure Python, no rclpy, alongside the existing
  `approach.py` tests. Greedy nearest-neighbour ordering. Dock-retry/pause state
  logic.
- **Integration (sim):** full run — exploration caps at `discovery_goal`,
  monitoring does one nearest-neighbour sweep, flowers land **inside** the box as
  coloured dots, tag renders as a diamond on the edge, box rectangle drawn,
  three colours detected after the aim fix, battery mid-sweep → dock + pause (not
  DONE), HMI event log shows "battery low" not "orchestrator error".
- Verify locally on Oscar's desktop before pushing (no "needs verification" notes
  in the MR).

---

## 11. v1 limitations (documented, accepted)

- Flower positions are **geometric estimates** (tag anchor + lateral cue), not
  true depth-projected points. Lateral resolution ≈ box width / `lateral_columns`;
  depth is a representative fraction with jitter, not measured. Clean seam left to
  upgrade to depth projection later.
- Climate per tag is fixed by the greenhouse sim's sensor placement — the map
  shows what each tag actually has; it does not synthesise missing sensors.
- Stream D refines but does not replace the tag anchor; with no tag, no box.
