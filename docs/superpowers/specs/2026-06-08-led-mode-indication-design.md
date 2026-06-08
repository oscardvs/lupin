# LED mode indication — activity-driven status strip

**Date:** 2026-06-08
**Author:** Oscar de Vos
**Status:** Design — approved, pending implementation plan
**Package:** `lupin_hmi` (`light_strip_bridge`)

## Problem

A project requirement (RO47007 MDP) is that the robot's **operating mode must
be clearly communicated to the operator** — by sound, the LED strip, or the
gripper. We already drive the onboard neopixel strip, but only from the mission
state machine. The gap:

- `light_strip_bridge` runs **onboard** (`lupin-onboard.service`) and is the
  single chokepoint for every LED write, but it is driven **only** by
  `/mission/state` (`lupin_msgs/MissionState`).
- `/mission/state` is published **only when a mission is running** (the
  orchestrator launches laptop-side with `mission:=true`). So with **just
  teleop** (Xbox) or **just the HMI** and no mission, the bridge has *no input
  at all* — the strip sits at its startup `OFF` and the robot can drive or move
  its arm while communicating nothing.
- The bridge only emits **solid** whole-strip colours; it has no notion of
  "the base is driving" or "the arm is moving", and cannot blink.

We want the strip to always reflect what the robot is physically doing, in
every launch configuration, not just during an autonomous mission.

## Goals

- Strip communicates live activity whenever **no mission owns it**: driving,
  arm motion, and standby each have a distinct, across-the-room-legible look.
- Works identically in **standalone teleop**, **HMI-only**, and **idle** —
  no laptop / mission dependency (the bridge already runs onboard; new inputs
  are all onboard topics).
- Physical/HMI **e-stop turns the strip red in every mode**, including
  standalone teleop (today it only reaches the strip via mission state).
- **Mission display is untouched** — the existing lifecycle palette and its
  behaviour are preserved exactly (the "gap-fill" decision below).

## Non-goals

- No change to the mission-lifecycle palette or when it shows during a run.
- No sound and no gripper-gesture indication (LED strip only).
- No per-pixel animation — the MIRTE `SetNeopixel` service is whole-strip, so
  blink (time-toggle) is the only animation available.
- No battery indication in standalone (battery rides in `MissionState`; out of
  scope here).

## Key decisions (locked with the user)

1. **Composition = gap-fill.** The new driving/arm/standby layer applies *only*
   when no mission is in progress. During an in-progress mission the lifecycle
   palette wins, unchanged. (Blink conveniently disambiguates the colour
   collisions: green-*blink* = driving vs solid green = mission DONE;
   orange-*blink* = arm vs solid orange = RETURNING.)
2. **E-stop:** subscribe to `/e_stop_state` (`std_msgs/Bool`) directly, so red
   works in every mode.
3. **Blink:** on/off, **1 Hz** (0.5 s on / 0.5 s off).
4. **Idle colour:** reflect the resting mission state when a mission node is
   present — DONE→green, READY→blue, BOOT→white — otherwise (pure teleop, no
   orchestrator) standby **blue**.

## State → colour → style mapping (precedence, highest first)

| # | Condition | Colour | Style | Source |
|---|-----------|--------|-------|--------|
| 1 | E-stop engaged **or** lifecycle FAULT | Red `(255,0,0)` | solid | `/e_stop_state` **or** `MissionState.estop_engaged` / `lifecycle=FAULT` |
| 2 | Mission paused | Amber `(255,191,0)` | solid | `MissionState.paused` *(unchanged)* |
| 3 | Manual HMI hold active | operator colour | solid | `/lupin/leds/set` *(unchanged)* |
| 4 | **In-progress mission** | existing palette | solid | `MissionState`, lifecycle ∈ in-progress set *(unchanged)* |
| 5 | **Arm moving** | Orange `(255,128,0)` | **blink** | `/joint_states` (arm joints) |
| 6 | **Base driving** | Green `(0,255,0)` | **blink** | `/mirte_base_controller/cmd_vel` ≠ 0 |
| 7 | **Idle / standby** | Blue `(0,0,255)` *(or resting mission colour)* | solid | nothing active |

Rows 1–4 keep today's colours and behaviour, with one addition: rows 2 and 4
(mission-derived) are gated by mission **freshness** (see below) so a dead or
stopped orchestrator can no longer latch the strip on a stale pause/lifecycle.
Rows 5–7 are the new activity layer, reached **only** when rows 1–4 don't apply
(i.e. no fresh in-progress mission). Safety (row 1) is deliberately *not*
freshness-gated — a stale `estop_engaged` still shows red (fail-safe), and
`/e_stop_state` is live onboard regardless.

**Lifecycle sets** (from `MissionState.msg`):

- **In-progress** (row 4, mission owns strip): `PREPARE`, `EXPLORING`,
  `INSPECTING`, `MONITORING`, `RETURNING`.
- **Resting** (row 7, activity layer applies; idle shows the lifecycle colour):
  `BOOT`→white, `READY`→blue, `DONE`→green. `FAULT` is handled by row 1.

Rationale for the split: while a mission is *in progress*, its lifecycle colour
is the more meaningful "what is it doing", and driving is expected — so motion
must **not** override it (gap-fill). When the mission is merely *resting*
(finished / idle), real motion (teleop after a run) should override to
green-/orange-blink, and when idle the resting colour is shown so a completed
run still reads as green until the next action.

## Signals & detection

All inputs are onboard topics; the bridge gains three subscriptions.

### Base driving — `/mirte_base_controller/cmd_vel` (`geometry_msgs/Twist`)
- This is the **twist_mux output**, so it captures every source —
  `/cmd_vel_joy` (teleop), `/cmd_vel_manual` (HMI), `/cmd_vel_auto` (Nav2) —
  in one topic. Topic is a **parameter** (`drive_topic`) so the sim bringup can
  point it at `/mirte_base_controller/cmd_vel_unstamped`.
- **Driving** = a twist received within `drive_timeout` (default **0.4 s**)
  whose components exceed a deadband `drive_deadband` (default **1e-3**) on any
  of `linear.x`, `linear.y`, `angular.z` (mecanum base).
- **Not driving** when: an explicit ~zero twist arrives, **or** the topic goes
  silent for `drive_timeout`. The Xbox dead-man makes `/cmd_vel_joy` go *silent*
  (not zero) on release, so the twist_mux output also goes silent and the base
  controller's own timeout stops the wheels — the staleness window handles this.

### Arm moving — `/joint_states` (`sensor_msgs/JointState`)
- `/joint_states` also carries **wheel joints**, so filter to the arm + gripper
  joints **by name**. Reuse `lupin_hmi.arm_limits` as the single source of joint
  names: `ARM_JOINT_FULL` (`shoulder_pan_joint`, `shoulder_lift_joint`,
  `elbow_joint`, `wrist_joint`) plus the gripper joint (`gripper_joint`,
  confirm exact name from live `/joint_states` at implementation).
- **Moving** = position delta on any tracked joint above `arm_deadband_rad`
  (default **0.0087 rad ≈ 0.5°**) between messages, latched until
  `arm_motion_hold` (default **0.3 s**) after the last detected change
  (debounce, so intermittent updates don't flicker). Velocity may be used as an
  accelerator if the broadcaster reports it reliably, but position-delta is the
  primary, robust signal (also catches kinesthetic/manual arm moves).
- Includes the gripper — open/close reads as arm-system motion (orange).

### Safety — `/e_stop_state` (`std_msgs/Bool`)
- Published onboard at ~5 Hz by `estop_bridge` (physical emergency button
  **and** HMI software-STOP both feed it). `True` → red, outranking everything.
- Combined with the existing `MissionState.estop_engaged` / `lifecycle=FAULT`
  so red shows whether or not a mission is running.

### Mission freshness
- `mission_state_timeout` (default **2.0 s**): if `/mission/state` hasn't
  arrived within this window, treat as "no mission node present" → activity
  layer with plain blue standby when idle.

### Conflict resolution
- **Arm out-ranks base** if both are somehow active at once (row 5 before 6).

## Architecture

Restructure the bridge around a clear three-part split (input → decision →
render), keeping the existing `_send_color` chokepoint (BRG remap + async
dedup) intact.

1. **Input callbacks** (thin): mission-state, `/e_stop_state`, `drive_topic`,
   `/joint_states`, and the existing manual/auto services. Each only updates
   cached state + a "last seen" timestamp. No colour logic here.

2. **Pure decision function** `_compute_style(now) -> (rgb, blink)`:
   implements the precedence table from cached state + timestamps. Pure and
   node-free, mirroring the current `_state_to_rgb` so it is fully unit
   testable. The existing safety / paused / manual / `_state_to_rgb` logic is
   reused unchanged for rows 1–4; only rows 5–7 are new.

3. **Render timer** (the genuinely new mechanism), default **10 Hz**:
   - calls `_compute_style(now)`,
   - derives the blink phase from elapsed time (`blink_hz`, default 1.0 →
     0.5 s on / 0.5 s off),
   - `effective_rgb = rgb` when solid or in the on-phase, else `OFF`,
   - sends `effective_rgb` through `_send_color`, which already dedups
     (so solid colours are written once; a blink writes twice per second) and
     applies `color_order`.

   The render loop also makes the strip **proactive**: today it sits `OFF`
   until the first `/mission/state`; with the loop, standby-blue is the default
   the moment the LED service is ready (side-bug fixed).

Manual mode is unchanged: `_on_set_manual` holds a solid colour that outranks
the activity layer (row 3 above 5–7); an operator who wants live indication
back calls `/lupin/leds/auto`. This is a deliberate trade-off — the **default
(auto) mode fully satisfies the "always communicate" requirement**; manual is an
explicit operator opt-out.

## New parameters (with defaults)

| Parameter | Default | Purpose |
|-----------|---------|---------|
| `estop_topic` | `/e_stop_state` | Direct safety input |
| `drive_topic` | `/mirte_base_controller/cmd_vel` | Base-command topic (sim overrides to `…/cmd_vel_unstamped`) |
| `joint_states_topic` | `/joint_states` | Arm-motion input |
| `drive_timeout` | `0.4` | Staleness window for "driving" (s) |
| `drive_deadband` | `1e-3` | Per-component zero threshold |
| `arm_deadband_rad` | `0.0087` | Per-joint motion threshold (≈0.5°) |
| `arm_motion_hold` | `0.3` | Debounce after last arm change (s) |
| `mission_state_timeout` | `2.0` | Freshness window for `/mission/state` (s) |
| `render_rate_hz` | `10.0` | Render/blink tick rate |
| `blink_hz` | `1.0` | Blink full-cycle rate (on/off) |

Existing parameters (`led_service`, `manual_service`, `auto_service`,
`color_order`, `mission_state_topic`, …) are retained.

## Decision pseudocode

```
def _compute_style(now):
    # 1 safety
    if estop_state or mission_estop or lifecycle == 'FAULT':
        return RED, solid
    # 2 paused (mission)
    if mission_fresh(now) and msg.paused:
        return AMBER, solid
    # 3 manual hold
    if mode == 'manual' and manual_rgb is not None:
        return manual_rgb, solid
    # 4 in-progress mission — existing palette, untouched
    if mission_fresh(now) and lifecycle in IN_PROGRESS:
        return _state_to_rgb(msg), solid
    # 5 arm moving
    if arm_moving(now):
        return ORANGE, blink
    # 6 base driving
    if base_driving(now):
        return GREEN, blink
    # 7 idle / standby
    if mission_fresh(now) and lifecycle in RESTING:
        return _state_to_rgb(msg), solid   # DONE→green, READY→blue, BOOT→white
    return BLUE, solid
```

## Edge cases

- **Wheel joints in `/joint_states`** → filtered out by name (only arm+gripper
  tracked); otherwise driving would falsely read as arm motion.
- **Dead-man release** → `/cmd_vel_joy` and thus the mux output go *silent*;
  `drive_timeout` returns the strip to standby (no stuck green).
- **Nav2 zero-hold** (commanding ~0 at a goal) → deadband reads not-driving.
- **Blink vs dedup** → render loop drives the toggle; dedup is computed on the
  *effective* (post-blink) colour, so it never suppresses the toggle but still
  suppresses redundant solid writes.
- **LED service down** → `_send_color` already warns and skips; manual set still
  rejects rather than claiming a colour the operator can't see.
- **MissionState stops mid-run** (orchestrator dies) → after
  `mission_state_timeout` the strip falls through to the activity layer rather
  than freezing on the last lifecycle colour.

## Testing

Extend `lupin_hmi/test/test_light_strip_bridge.py` (pure-function style, no
spin), covering the new precedence:

- driving (no mission) → green + blink; arm (no mission) → orange + blink;
  both → orange (arm out-ranks base).
- idle, no mission node → blue solid; idle + DONE → green solid;
  idle + READY → blue solid.
- in-progress lifecycle + driving → unchanged lifecycle colour, solid
  (gap-fill preserved).
- e-stop via `/e_stop_state` alone (no MissionState) → red.
- manual hold outranks activity; safety outranks manual.
- existing battery-low / pause / scan-phase tests still pass (adapt call sites
  to the `(rgb, blink)` return).

Blink-phase / render-timer mechanics get a light unit test (phase on/off from
elapsed time); LED-write dedup is already covered.

## Files touched

- `lupin_hmi/lupin_hmi/light_strip_bridge.py` — new subs, cached state, render
  timer, `_compute_style`, blink. Main change.
- `lupin_hmi/test/test_light_strip_bridge.py` — new precedence tests + adapt
  existing.
- `lupin_bringup/launch/onboard.launch.py` — pass new params (defaults are
  fine; explicit for clarity / sim override hook).
- (sim) wherever the sim bringup launches the bridge — set
  `drive_topic:=/mirte_base_controller/cmd_vel_unstamped`. Confirm sim runs the
  bridge at all; if not, no change.
- `lupin/CHANGELOG.md` — note the feature.

## Out of scope / future

- Battery level in standalone, sound cues, gripper gestures.
- Per-pixel patterns (would need a different LED driver path).
- Surfacing the live activity mode in the HMI (the strip is the indicator;
  HMI mirroring can come later if wanted).

## References (project memory)

- `project_hmi_light_control` — existing manual LED control + mission palette.
- `project_drive_topic` — twist_mux arbitration + sim vs real cmd_vel topics.
- `project_estop_not_wired` (partially superseded) — `estop_bridge` now
  publishes `/e_stop_state` onboard.
- `project_arm_control_overhaul` — shared joint-limit/name source (`arm_limits`).
- `2026-06-08-voice-led-control-design.md` — sibling spec (voice `set_light`
  manual presets); this feature is the automatic activity layer beneath it.
