# Lupin autonomous mission — sim debug & validation (2026-06-08)

**Engineer:** senior ROS 2 review pass (Claude, Opus 4.8)
**Branch:** `fix/sim-mission-debug-2026-06-08` (worktree off `origin/main` @ `a6b216d`)
**Scope:** the autonomous *Explore & monitor* mission in **simulation only** —
exploration → AprilTag discovery → monitoring/scan (tag read + climate + flower) →
perception aggregator → digital twin → HMI; plus battery return-to-dock, the
two-layer e-stop, and per-leg AMCL drift gating. LED voice tool and arm work are
out of scope and untouched.

**Headline:** the full mission was **dead-on-arrival in `sim_full`** — a cmd_vel
mis-wiring meant Nav2 commands never reached the base, so nothing drove (which is
why the "full run" had stayed unverified). With that fixed, the mission runs
**cleanly end-to-end** (explore → monitor → scan → twin → return → dock → DONE),
and the battery / e-stop / unreachable-leg safety paths all behave correctly.
Two bugs fixed + verified; one perception quality defect and one sim-coverage gap
documented with evidence.

> ✅ verified · 🟡 partial · 📝 noted (not individually reproduced) · ⏭ deferred

---

## 0. Harness / reproduce (isolated from the parallel session's `main` WIP)

| Thing | Path |
|---|---|
| Worktree (`fix/sim-mission-debug-2026-06-08`) | `/home/oskrt/worktrees/sim-mission-debug` |
| Overlay colcon ws (built from worktree over `~/ros2_ws/install`; never writes the shared install) | `/home/oskrt/worktrees/sim-mission-debug-ws` |
| Env (ROS → vendor underlay incl. patched `gazebo_ros2_control` → lupin overlay → gazebo; sim-local DDS) | `…-ws/env.sh` |
| Lean sim launcher | `…-ws/run_sim.sh` |
| Mission bag | `…-ws/missionrun/` |

```bash
colcon --log-base …-ws/log build --symlink-install --base-paths …/sim-mission-debug \
       --build-base …-ws/build --install-base …-ws/install
source …-ws/env.sh
ros2 launch lupin_bringup sim_full.launch.py            # rviz/web/joystick default-on
```
The gazebo_ros2_control regression fix (`_vendor_overlays/gazebo_ros2_control`) is
already built into the underlay and inherited automatically.

**Two operational gotchas (environment, not product):** (1) the `ros2` CLI daemon
caches a stale graph if started before the sim — `ros2 daemon stop && start` after
bring-up; node-spinning tools (`tf2_echo`) see data even when the daemon doesn't.
(2) Short-lived `ros2 topic pub` to a RELIABLE subscriber can miss (discovery race);
use a ≥3-4 s publisher window.

---

## 1. Mission map

### 1.1 Orchestrator FSM (`lupin_mission/node.py`, `transitions` HSM)

`queued=True`, `ignore_invalid_triggers=True`, single-threaded executor +
`MutuallyExclusiveCallbackGroup`. Pause / E-stop / battery-low are **flags**
(`_is_blocked()`), not states; only `/mission/resume` unblocks.

```
BOOT ─deps_up─▶ READY ─start_mission─▶ PREPARE.LOCALIZING
   ├─localized[Exploration]─▶ EXPLORING ─tags_discovered(≥N)─▶ MONITORING.{NAV→SCAN→PUB}⟲
   │                              └─no_frontiers[+any]─▶MONITORING / [none]─▶RETURNING
   └─localized[Inspection]──▶ INSPECTING.{NAV→SCAN→PUB}→…
 MONITORING/INSPECTING ─inspection_complete─▶ RETURNING ─returned─▶ DONE ─reset_for_next─▶ READY
 (active) ─abort_to_return─▶ RETURNING ;  (any) ─fault─▶ FAULT ─recover─▶ READY
 RETURNING ─resume_{inspection,monitoring,exploration}─▶ (captured origin family)   # battery/dock resume
```
EXPLORING drives `frontier.select_frontier_goal` over `/map`, counting distinct
`/perception/discovered_tags` until `discovery_goal`. MONITORING builds a nearest-
first `MonitoringMission` from discovered poses. SCANNING = optional
`/perception/confirm_tag` (hardware gate, off in sim) → bridge `GetTagReading` →
`flower_scan_dwell_s` dwell (sim 4 s). Per-leg gate: Inspection = tight AMCL cov;
Exploration/Monitoring = **presence-only** (`/amcl_pose` is a static seed in SLAM).

### 1.2 Topic / service graph (SIM) — verified live

| Edge | Topic / iface | Producer → Consumer |
|---|---|---|
| laser/map | `/scan`, `/map` (latched) | gazebo ray, slam_toolbox → Nav2, frontier |
| body cam | `/camera/image_raw` (+ zero-K info → fallback intrinsics) | gazebo → `tag_annotator` |
| tags | `tag_<id>` TF + `/camera/tag_detections_json` → `/perception/discovered_tags` (latched) | tag_annotator → aggregator → orchestrator, twin |
| gripper cam | `/gripper_camera/image_raw` → `/yolo/detections` (String JSON) | gazebo → sim_flower_detector → aggregator |
| obs/mission/twin | `/floranova/observations`, `/mission/state` (5 Hz latched), `/twin/state` (1 Hz latched), `/twin/get_field` | orchestrator/aggregator → twin → HMI |
| nav | `navigate_to_pose` action | orchestrator → Nav2 bt_navigator |
| **drive** | **`/cmd_vel`** | twist_mux ← {`/cmd_vel_{joy,manual,auto}`,`/zero_cmd_vel`} → **gazebo_planar_move** |
| battery | `/io/power/power_watcher` (%∈[0,1]) | sim_battery_publisher → BatteryMonitor |
| e-stop | `/e_stop_state` (Bool) | **no sim publisher** → EStopMonitor |

---

## 2. Verification matrix

| Layer | What ran | Result | Evidence |
|---|---|---|---|
| Static build | overlay `colcon build` (9 pkgs) | ✅ rc=0 | §0 |
| Unit — perception | `pytest lupin_perception/test` | ✅ 22/22 | |
| Unit — twin | `pytest lupin_twin/test` | ✅ 34/34 | |
| Unit — mission | `pytest lupin_mission/test` | ✅ 74/74 (after Bug #1; was 73/74, 2× repeat) | §3.1 |
| Bring-up | `sim_full.launch.py` (lean) | ✅ Nav2 active MPPI/Omni, orch READY, SLAM/TF good | sim.log |
| Drive path | topic graph + odom drive | ✅ Bug #2 found, fixed, live-verified (robot drives + auto-stops) | §3.2 |
| **Full mission** | Explore&monitor `discovery_goal=4` | ✅ EXPLORING→4 tags→MONITORING→scan×4→RETURNING→dock→**DONE** (ok=3, unreachable=1) | §3.2, bag |
| Approach geometry | discovered_normal vs live quaternions | ✅ correct (normals point into aisle; nav succeeds) | §3.3 |
| Perception→twin | `/twin/state` after scans | ✅ tags + species/flowers/anomaly populate | §3.4 |
| Failure: battery-low | force %=0.15 mid-MONITORING | ✅ divert→dock→pause→recover→resume_monitoring | §4 |
| Failure: e-stop | engage/release/resume | ✅ preempt + pause + clean release + resume | §4 |
| Failure: unreachable leg | tag 16 | ✅ 2 attempts → UNREACHABLE → mission continues → DONE | §3.2 |
| Failure: no-tags / never-localizes | unit + log | 🟡 unit `test_exploration_no_tags_returns_home` ✅; `min_sightings`+localized-warn observed | §6 |
| Perception: flower colour | gripper image + HSV | 🟡 **defect** — cam frames gray wall, all `tulip_white` | §3.3 |
| AMCL drift gate | — | ⏭ presence-only in SLAM sim; not forceable without a fake AMCL | §6 |

---

## 3. Findings

### Bug #1 — orchestrator unit suite flaky in-suite (test-harness teardown) ✅ FIXED
- **Symptom:** `test_orchestrator_v2.py::test_pause_resume` fails in the full suite
  (`self.nav.goals_cancelled >= 1` → False) but passes 3/3 in isolation; an
  `RCLError: feedback publisher is invalid` is thrown from a `FakeNavServer`
  execute callback during teardown.
- **Root cause:** `_SpinHarness.shutdown()` stopped only the spin thread and
  destroyed nodes, never calling `executor.shutdown()`. The `MultiThreadedExecutor`
  worker threads (running action `execute_callback`) outlive the harness; a lingering
  callback calls `goal_handle.succeed()` after the publisher is destroyed, and the
  abandoned executor's still-registered `navigate_to_pose` server leaks into the
  next test, so its cancel is routed to a dead server → `goals_cancelled` never moves.
- **Fix:** `_SpinHarness.shutdown()` now calls `self.executor.shutdown(timeout_sec=2.0)`
  before `destroy_node()`.
- **Re-verify:** ✅ `lupin_mission` functional suite **74/74**, run **2×**, deterministic.

### Bug #2 — `sim_full` cmd_vel dead-ends; robot never navigates ✅ FIXED + live-verified
- **Symptom:** in `sim_full`, Nav2/teleop velocity never moves the base.
- **Root cause:** `greenhouse_sim.launch.py` drives the kinematic base with the vendor
  `planar_move` plugin on **`/cmd_vel`** and intentionally spawns **no** base
  controller / vendor twist_mux (live: one `/twist_mux`; `ros2 control list_controllers`
  = joint_state/arm/gripper only). But `sim_full.launch.py:456` remapped the lupin
  twist_mux output to **`/mirte_base_controller/cmd_vel_unstamped`**, assuming a vendor
  twist_mux (prio 200) that this launch never includes. Live graph: that topic had
  **0 subscribers**; `gazebo_planar_move` subscribes only to `/cmd_vel`. So
  `FollowPath → /cmd_vel_auto → twist_mux → cmd_vel_unstamped → ∅`. `sim_robot.launch.py:73`
  and the `greenhouse_sim` comment both confirm the correct sim target is `/cmd_vel`.
  (`sim_autonomy.launch.py` is fine — it runs atop `sim_robot`'s correct mux.)
- **Fix:** `sim_full.launch.py` twist_mux remap → `('cmd_vel_out', '/cmd_vel')` (+ honest
  comment). The prio-1 `/zero_cmd_vel` input also tames planar_move's no-timeout latch
  (robot now auto-stops when sources go quiet).
- **Re-verify (live):** after the fix, `twist_mux` is a `/cmd_vel` publisher; a
  `/cmd_vel_auto` nudge moved odom `y 3.00→3.34` then **held** (auto-stop). Full mission
  `lupin-4861a5d4`: EXPLORING frontier goals → 4 tags → MONITORING navs **succeed** →
  `total=4 ok=3 failed=0 unreachable=1` → RETURNING → dock(2.00,3.34) → **DONE**.

### Finding #3 — flower colour classification non-functional in sim (gripper cam doesn't frame blooms) 🟡 documented (display-only)
- **Symptom:** every scanned tag → `species: tulip_white`; `/yolo/detections` is a
  **full-frame** `[0,0,~640,480]` white blob at conf 1.0.
- **Root cause (evidence, not guess):** a captured `/gripper_camera/image_raw` frame is
  **100 % low-saturation** (`frac_lowsat(S<38)=1.00`, meanHSV≈(0,0,168)) — a uniform
  gray wall/structure with the gripper fingers at the frame edges; **no blooms in view**
  (see `gripper.png`/`overlay.png`). The world *does* contain 60 colored flower models
  (`flower_*_red/white/pink`), so it's not missing assets — the `inspect` arm pose /
  gripper-cam mount simply doesn't aim at the blossoms in the 1.0.8 layout. `frac_dark=0`
  → the dark-pixel `bug` mask does **not** false-fire here.
- **Impact:** display-only. Per `docs/CONTRACTS.md`, `species`/`anomaly` are producer-set,
  consumer-unread for mission logic — they never affect the FSM. The mission completes
  regardless. This matches the runbook's known risk ("Gripper cam doesn't frame the bloom").
- **Recommended fix (not applied — tuning loop, adjacent to out-of-scope arm work):** tune
  `arm_preset_server.py` `PRESETS['inspect']` and/or the `gripper_camera` mount in
  `mirte_master_description/urdf/arm.xacro` so the cam frames the bloom; verify against
  `/sim_flower/overlay`. Secondary: tighten the `sim_flower_detector` white band
  (`S<38` is broad) so a desaturated background can't read as `tulip_white`.

### Finding #4 — e-stop has no sim publisher; a stuck-engaged state is unrecoverable in sim 🟡 documented (coverage gap)
- The e-stop **logic is correct and validated** (unit `test_estop_preempt_no_auto_resume`
  ✅; live engage→preempt+pause, release→clear, resume→continue — §4).
- **Gap:** nothing publishes `/e_stop_state` in sim (`estop_bridge` is hardware-only; HMI
  STOP is web-only). So the e-stop path can't be exercised out-of-the-box, and — because
  release requires a `Bool=false` edge — once engaged it stays latched with no in-sim way
  to clear it (resume is then permanently refused). During testing an **unattributed**
  `/e_stop_state=true` arrived with no publisher present (likely a transient during daemon/
  topic churn; could not reproduce the producer) and wedged a battery-resume until I
  published `false` manually.
- **Recommended fix:** add a tiny sim e-stop relay/echo (default false) to `sim_full`, or
  a documented `ros2 topic pub` helper, so the two-layer e-stop is testable and a stuck
  engage is clearable in sim.

### Noted leads (reviewed; not mission-blocking) 📝
- **approach `+Z` direction** — *disproved as a bug.* Live quaternions: tag 4 normal →+Y,
  goal (2.58,4.11) facing the tag (nav OK); tag 3 normal →−Y (opposite table). Normals
  correctly point into the aisle (§3.3).
- **`node.py:1396` dock_pose** persists across missions and falls back to map-origin if TF
  is absent at start. In sim TF is up; dock captured correctly (2.00,3.34). Multi-mission
  edge only.
- **`_enter_publishing` early-return** skips `advance()` on a battery/dock divert at
  PUBLISHING → a duplicate observation for that tag after resume (idempotent in the twin).
- **slam via `online_async` (no respawn)** + `/lupin/nav/clear_map` absent in `sim_full`
  → HMI "Erase map" is dead in `sim_full` (works via `slam_sim`/`_slam_core`).
- **twin SCAN_FAILED** resets `stale_seconds` to 0 (staleness shows "fresh" with no new
  reading) — display only.
- **battery (1,2) false-low normalisation** — **not reachable in sim** (publisher clamps
  %∈[0,1]); hardware-only theoretical edge.
- **`arm_library_server` duplicated** when `enable_web:=true` (sim_full + lupin_web both
  start it). Not triggered in the lean run.

---

## 4. Failure-mode probes (live)

- **Battery-low return-to-dock + resume** ✅ — forced %=0.54→0.15 mid-MONITORING →
  `Battery LOW: 14.9% … Triggering dock` → `ABORT_TO_RETURN: MONITORING_SCANNING→RETURNING`
  → dock(2.00,3.34) → "Docked due to low battery. Awaiting charge + /mission/resume"
  (`paused=true`). Recover %=0.90 → `battery_low=false`. `/mission/resume` →
  `RESUME_MONITORING: RETURNING→MONITORING_NAVIGATING` (correct origin family, not
  INSPECTING) → sweep continues.
- **E-stop engage/release/resume** ✅ — engage → `estop_engaged=true, paused=true`
  (preempts active state, cancels nav); release → `estop_engaged=false` (stays paused);
  `/mission/resume` → resumes. Pause is refused during a battery-return (correct).
- **Unreachable approach leg** ✅ — tag 16 nav failed ×2 (`nav_max_attempts=2`) →
  `NAV_UNREACHABLE` → mission continued and completed (counters: `unreachable=1`).

---

## 5. Evidence appendix
- `…-ws/sim.log` — bring-up cascade + all FSM transitions (timestamped).
- `…-ws/missionrun/` — ros2 bag (`/mission/state`,`/twin/state`,`/perception/discovered_tags`,`/floranova/observations`,`/yolo/detections`,`/cmd_vel`,`/odom`,`/amcl_pose`).
- `…-ws/gripper.png`, `…-ws/overlay.png` — gripper-cam frame + detector overlay (Finding #3).
- Key live commands: `ros2 topic info -v /cmd_vel|/mirte_base_controller/cmd_vel_unstamped`,
  `ros2 control list_controllers`, odom drive test, `/perception/discovered_tags` echo,
  `ros2 param set /sim_battery_publisher override_percentage`, `/e_stop_state` pub.

## 6. Left unverified / deferred (and why)
- **AMCL drift gate per leg** — Exploration/Monitoring use the *presence-only* gate (SLAM
  seed never drifts), and Inspection's tight-cov gate would need a fabricated drifting
  `/amcl_pose`. Logic reviewed (`_nav_localization_ok`/`_presence_only_gate`); not forced live.
- **"tag that never localizes"** — covered indirectly: `min_sightings=3` upstream + the
  aggregator's `localized==0` warning observed at startup; not isolated as a standalone case.
- **Full 29-tag sweep** — validated the pipeline with `discovery_goal=4` (and a 7-tag run);
  a complete 29-tag sweep is longer wall-clock, not re-run.
- **Flower-colour fix & e-stop sim publisher** — root-caused and recommended (Findings #3/#4),
  not implemented (tuning loop / sim-only feature, both display/coverage not mission-logic).
- **Hardware-only paths** (real e-stop bridge, YOLO `best.pt`, real power watcher range) —
  out of sim reach; not tested.
