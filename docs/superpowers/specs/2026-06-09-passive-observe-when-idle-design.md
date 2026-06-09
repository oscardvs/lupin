# Passive observation when idle — design

- **Date:** 2026-06-09
- **Status:** Approved (design); ready for implementation plan
- **Branch:** `feat/passive-observe-when-idle` (off `origin/main` @ eba3269) → MR into `main`, then fast-forward `hardware` + `sim`
- **Author:** Oscar (Lupin)

## Problem

Following `DEMO_DAY_WIRED.md` end-to-end and **not** starting a mission (teleop, or
manual SLAM testing), the operator wants the SLAM map **and** the digital-twin map to
keep updating with what the robot sees: AprilTags + flowers appended to the map, the
climate **heatmap** built up, and the per-tag info displayed.

What happens today during teleop (T1–T8 up, no T9/mission):

| Surface | Works without a mission? | Why |
|---|---|---|
| SLAM `/map` | ✅ | T3 (`slam_hardware.launch.py`) runs standalone. |
| AprilTag camera overlay (green box) | ✅ | `tag_annotator` runs in T7. |
| Tag **location pins** on twin map | ✅ | aggregator publishes `/perception/discovered_tags` unconditionally; twin pins pose-only entries (`lupin_twin/node.py:334`). |
| **Flower / pest markers** | ❌ | `perception_aggregator._focus_tag()` (`:481`) only uses its standalone "nearest tag" fallback when **no** `/mission/state` was ever seen. The orchestrator publishes idle `READY` continuously, so the aggregator is stuck in "mission mode" and refuses to attribute unless `SCANNING`. |
| Per-tag **climate readings** → **heatmap** | ❌ | only the mission orchestrator emits `KIND_TAG_READING` (and only while `SCANNING` — `node.py:1673` returns early when `self._mission is None`), and the climate oracle `/greenhouse_bridge/get_tag_reading` is only launched by T9/`mission_stack`. No readings → `/twin/get_field` heatmap is empty. |

So two real gaps remain: **flower markers** and **climate readings / heatmap**.

## Guiding principle

> **Passive observation when idle; the mission owns observation when a mission is active.**

Every change keys off one runtime signal — the `/mission/state` lifecycle. "Active" =
the mission is driving/scanning; "idle" = no mission running. This guarantees we never
double-emit or conflict with the orchestrator during an autonomous run.

- **Active lifecycle states:** `PREPARE`, `EXPLORING`, `INSPECTING`, `MONITORING`, `RETURNING`.
- **Idle:** `BOOT`, `READY`, `DONE`, `FAULT`, empty, or no `/mission/state` seen.

(`lifecycle_state` is published as the bare state name, except `INSPECTING*`/`MONITORING*`
which publish the prefix — see `node.py:_publish_state`.)

## What this design does NOT change

`slam_toolbox`, the digital twin (`lupin_twin`), `/twin/get_field` IDW math, the mission
orchestrator/HSM, and the HMI. The twin and HMI are pure consumers — they light up
automatically once observations flow on the existing `/floranova/observations` topic.

## Design

### A. Flower gate — `perception_aggregator._focus_tag()` (always-on, in T7)

Make the existing standalone fallback fire whenever there is **no active mission**, not
only when `/mission/state` was never seen.

- New helper `_mission_active()` → `True` iff a `/mission/state` was seen, is non-None,
  and `lifecycle_state ∈ _ACTIVE_LIFECYCLES`.
- New parameter `attribute_when_idle` (default **`True`** — approved). When `False`,
  restores today's strict behaviour (no attribution unless `SCANNING`).
- Revised logic:
  - **Active mission** → unchanged: attribute to `current_target` only while `SCANNING`,
    else `None` (preserves the "no drive-by misassociation during EXPLORING" guarantee).
  - **Idle** (and `attribute_when_idle`) → nearest detected tag in the latest frame that
    is in the registry (today's standalone fallback). Else `None`.

Rationale for default `True`: flower/pest markers are **real** YOLO detections pinned to
a real, TF-resolved tag — not synthesized data — so "keep the SLAM-test pure" (the reason
climate is opt-in) does not apply. This makes "teleop → tags + flowers appear" work with
no extra terminal.

### B. New node `passive_observer` (package `lupin_perception`)

The climate/heatmap producer for the no-mission case. One job: when idle, turn a
discovered tag into a climate `KIND_TAG_READING` observation.

- **Subscriptions**
  - `/perception/discovered_tags` (`lupin_msgs/DiscoveredTags`, latched) — tag id + TF-resolved `pose_in_map`.
  - `/mission/state` (`lupin_msgs/MissionState`, latched) — idle/active check.
- **Service client:** `/greenhouse_bridge/get_tag_reading` (`lupin_msgs/srv/GetTagReading`).
- **Publisher:** `/floranova/observations` (`lupin_msgs/Observation`) — the twin's existing intake.
- **Parameters** (defaults)
  - `discovered_tags_topic` = `/perception/discovered_tags`
  - `mission_state_topic`   = `/mission/state`
  - `bridge_service_name`   = `/greenhouse_bridge/get_tag_reading`
  - `observations_topic`    = `/floranova/observations`
  - `map_frame`             = `map`
  - `refresh_period_s`      = `0.0`  (0 = read each tag **once**; >0 = re-read after N s to track time-of-day drift)
  - `source_name`           = `passive_observer`
- **Logic** (driven by a low-rate timer, e.g. 2 Hz)
  1. If a mission is **active** → do nothing (defer to the orchestrator); clears no state.
  2. Else, for each known discovered tag whose last successful read is missing or older
     than `refresh_period_s`, and not already in-flight, and the bridge is ready:
     send an async `GetTagReading(tag_id)`.
  3. On response: `STATUS_OK` → build a `KIND_TAG_READING` `Observation` (tag reading +
     `tag_pose_in_map` from the discovered feed, `frame_id = map_frame`) and publish; mark
     read. `STATUS_UNKNOWN_TAG` → mark read (no retry storm) + debug log.
  4. Bridge not ready → throttled warn (~10 s), retry next tick (no crash).
- **Testable core:** keep the decisions pure — `_mission_active(state)`, "which tags are
  due to read now", and "build observation from (tag_id, pose, reading)" — separated from
  ROS plumbing, mirroring the existing `box_geometry.py` / aggregator split.

### C. Observation builder — replicate, don't add a heavy dep

`make_tag_observation()` lives in `lupin_mission`. `lupin_perception` must **not** depend
on `lupin_mission` (wrong direction, heavy). Add a small local builder in
`lupin_perception` (e.g. `observations.py`) that constructs the same `KIND_TAG_READING`
shape the twin expects (incl. normalising `orientation.w == 0 → 1.0`). A unit test asserts
the shape. If a third consumer ever appears, extract to a shared util then — not now
(no speculative shared interface).

### D. New opt-in launch `lupin_bringup/launch/passive_observe.launch.py`

Composes the two cross-package pieces (mirrors `mission_stack.launch.py`):

```
greenhouse_bridge   → /greenhouse_bridge/get_tag_reading   (climate oracle)
passive_observer    → reads tags on sight when idle → /floranova/observations
```

- Args: `refresh_period_s` (default `0.0`), `tag_file` (default
  `lupin_bringup/config/tag_locations_widened.json` — the same file `mission_stack` feeds
  the bridge), plus topic passthroughs.
- Run as one extra terminal ("**T7.5**") during teleop / SLAM-test sessions. The aggregator
  (T7) already attributes flowers when idle (§A), so this launch does **not** touch it — it
  only adds the climate path.

### Mutual exclusivity with `mission_stack` (T9)

`passive_observe.launch.py` and `mission_stack.launch.py` both launch `greenhouse_bridge`
(same node name + service) → running **both** collides. They are mutually exclusive
workflows: **teleop/observe** runs T7.5; **autonomous** runs T9. Documented in
`DEMO_DAY_WIRED.md`. (`passive_observer` also defers when a mission is active, so a stray
`/mission/state` won't cause double readings — but the bridge collision is the real
blocker, hence "don't run both".) Future refinement (out of scope): factor the bridge into
an always-on perception layer shared by both.

## Data flow (opted-in, idle)

```
teleop drive
  ├─ tag seen → aggregator → /perception/discovered_tags → twin pins location      (already)
  │                                   │
  │                                   └─ passive_observer → GetTagReading(tag) →
  │                                        KIND_TAG_READING → twin stores reading →
  │                                        /twin/get_field IDW → HMI heatmap         (NEW)
  └─ gripper-cam YOLO → aggregator (now attributes when idle, §A) →
       KIND_FLOWER → twin → flower/pest markers                                      (NEW)
```

## Testing

- **Unit — aggregator** (`test_perception_aggregator.py`, extend): `_focus_tag()` returns
  nearest tag when idle (`READY`) + `attribute_when_idle`; `None` when active + not
  scanning; `current_target` when active + `SCANNING`; `None` when idle but
  `attribute_when_idle=False`.
- **Unit — passive_observer** (`test_passive_observer.py`, new): idle + discovered tag +
  bridge `OK` → one `KIND_TAG_READING` with correct frame/pose; active mission → no
  publish; `STATUS_UNKNOWN_TAG` → no publish, no retry storm; bridge-down → no crash;
  `refresh_period_s` semantics (0 = once). Builder shape test (§C).
- **Manual** (hardware or sim): T1–T8 + `passive_observe.launch.py`; teleop past several
  tags; in the HMI twin/map view confirm tag pins (already), per-tag readings appear, the
  heatmap renders, and flower/pest markers pin when the gripper cam frames blooms.

## Risks / assumptions to verify

1. **HMI renders the twin without an active mission.** High confidence (HMI is a pure
   consumer of `/twin/state` + `/twin/get_field`), but confirm the twin/map view and
   heatmap don't themselves gate on mission state.
2. **Tag-id format alignment.** `passive_observer` calls the bridge with the id string
   from `/perception/discovered_tags`. The orchestrator already drives the bridge with
   ids from the same discovered feed during missions, so they should match — confirm no
   `STATUS_UNKNOWN_TAG` for real tags during the manual test.
3. **Bridge tag set.** `tag_locations_widened.json` must contain the tags the robot will
   actually see, else readings come back `UNKNOWN_TAG` (markers/pins still work; only the
   heatmap is affected).

## Out of scope / non-goals

- No change to mission, orchestrator, twin, or HMI code.
- No persistence/export changes.
- Not moving the bridge into the base stack (future refinement).
- No new AMCL-drift gate for passive readings — they pin at the tag's own TF-resolved
  `pose_in_map`, which is already the trusted source.

## Integration plan

Generic feature (works sim + hardware): feature branch → MR into `main`; once merged,
fast-forward `hardware` and `sim` to keep three-branch parity. To exercise on the robot
before merge, the worktree branch can be built into `~/ros2_ws` directly (it includes the
one perception commit `main` is currently behind).
