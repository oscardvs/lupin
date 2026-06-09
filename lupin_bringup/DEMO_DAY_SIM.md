# Demo-day bringup — Gazebo sim (Lupin)

Step-by-step for running the **full autonomous greenhouse mission in Gazebo**, as a
demo or as a faithful rehearsal for the hardware run. Scan-able rather than prose —
look it up under pressure and follow it. For the real robot see `DEMO_DAY.md`.

> **Minimal sim↔hardware gap.** Only **two** things differ from hardware:
> the **drive layer** (sim uses the vendor `gazebo_ros_planar_move` teleport instead
> of real mecanum wheels) and the **flower detector** (sim HSV `sim_flower_detector`
> vs hardware YOLO). Both publish the same topics, so the command bus, TF frames,
> odom topic, Nav2, mission orchestrator, perception fusion, twin and HMI are
> identical. Running this exercises ~everything the hardware demo does.

Layered into 3 terminals so any layer can be killed/restarted on its own.

---

## 0. One-time setup (per laptop)

```bash
# HMI needs its Vite build artefact (T3 serves it in preview mode):
cd ~/ros2_ws/src/lupin/lupin_web/web && npm install && npm run build
# Vendor sim edits (gripper camera) — not in lupin git, re-apply after a vendor checkout:
bash ~/ros2_ws/src/lupin/lupin_bringup/scripts/apply-sim-vendor-edits.sh
# gazebo_ros2_control overlay — WITHOUT it the controllers never start (apt regression):
bash ~/ros2_ws/src/lupin/lupin_bringup/scripts/fix-gazebo-ros2-control.sh
cd ~/ros2_ws && colcon build --symlink-install
```

Assumes ROS 2 Humble + Gazebo Classic 11 + sim deps (`gazebo_ros_pkgs`, `slam_toolbox`,
Nav2, `topic_tools`). **No robot, no DDS-to-robot env, no clock sync** — it's all local.
Every terminal just needs the two source lines:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 1. Clean slate — ALWAYS, before every launch

Gazebo Classic leaves orphan `gzserver`/`gzclient`, and stale FastDDS shared memory
silently wedges new participants (controllers never activate, `ros2 topic list`
returns 2). Wipe first:

```bash
bash ~/ros2_ws/src/lupin/lupin_bringup/scripts/sim-clean.sh
```

**Expect:** `gzserver alive: 0 (want 0); :9090 listeners: 0 (want 0)` … `done.`

**If a launch later shows `open_and_lock_file failed`, controllers never activate, or
only ~22 topics appear:** SHM is still contended (builds up after many rapid
relaunches). `rm -f /dev/shm/fastrtps*` then re-run sim-clean. If it persists,
**reboot** (the clean fix) — or run the whole session on a fresh DDS domain:
`export ROS_DOMAIN_ID=42` in *every* terminal.

---

## 2. T1 — robot + Gazebo

```bash
ros2 launch lupin_bringup sim_robot.launch.py
```

Brings up Gazebo (greenhouse world), the Mirte (planar_move drive), the arm/gripper
controllers, the twist_mux command bus, the continuous `/amcl_pose` AMCL stand-in, and
the sim battery.

**Expect:** the Gazebo window opens with the robot **sitting on the floor** between the
planters (NOT hovering). In a second terminal:

```bash
ros2 control list_controllers              # 3 active: joint_state_broadcaster + arm + gripper
ros2 topic echo /groundtruth/odom --once   # z ≈ 0.0, x/y at the spawn, stable (not NaN)
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}"   # robot drives
```

**3 controllers is correct** — the base is driven by `planar_move`, not a ros2_control
wheel controller (the one sim↔hardware drive difference). Don't wait for 5.

**If the robot hovers, won't drive, or Gazebo dies with an Ogre `AxisAlignedBox`
assertion:** you're on an old build — those are fixed; `git pull`, rebuild
`lupin_bringup`, re-source, sim-clean, relaunch.

> Headless / no monitor: `unset DISPLAY` before the launch — `gzclient` skips, the
> physics server and the whole stack still come up.

---

## 3. T2 — autonomy (SLAM + Nav2 + perception + mission + twin)

```bash
ros2 launch lupin_bringup sim_autonomy.launch.py
```

Bring up **after** T1's 3 controllers are active (SLAM needs `/scan` + TF; the
orchestrator needs Nav2 + the bridge — they self-sequence with a few "waiting" retries).

**Expect (in the log):** `[lifecycle_manager_navigation]: Managed nodes are active`,
then `[mission_orchestrator]: Dependencies up. Orchestrator READY.` SLAM publishes
`/map`; `ros2 run tf2_ros tf2_echo map base_link` resolves.

**If spawners loop on `Could not contact /controller_manager`:** T1 isn't fully up (or
SHM-wedged — see §1). Wait for T1's 3 controllers, or clean + relaunch T1.

> **Granular sim (run SLAM / Nav2 in their own terminals, e.g. to debug one in
> isolation):** instead of the `sim_autonomy` bundle, use the per-subsystem sim
> wrappers — the sim twins of the hardware ones in `DEMO_DAY_WIRED.md` §4:
> ```bash
> ros2 launch lupin_navigation slam_sim.launch.py   # → /map + map→odom TF (+ erase-map service)
> # confirm /map (RViz), then:
> ros2 launch lupin_navigation nav2_sim.launch.py   # Nav2 slam mode, use_sim_time:=true preset
> ```
> Same `/map`-then-Nav2 ordering as hardware (both wrap the shared `nav2.launch.py`
> / `_slam_core.launch.py`; only `use_sim_time` differs). `sim_autonomy.launch.py`
> just bundles these with the bridge + orchestrator + twin.

---

## 4. T3 — HMI

```bash
ros2 launch lupin_bringup sim_hmi.launch.py
```

Open **http://localhost:8090**. Lidar / Map / Cameras views should go live within a few
seconds.

---

## 5. Start the mission

**From the HMI (recommended):** **Map** view → *Mission Control* → the green
**“Explore & monitor”** button. Set the number box to how many tags to discover before
switching to monitoring (e.g. **4**).

- The grey **“Patrol known tags”** button is a *different* mission (revisits
  already-mapped tags, no exploration phase) — **not** the autonomous demo run.

**Or from the CLI:**

```bash
ros2 service call /mission/start lupin_msgs/srv/StartMission \
  "{mission_type: 'ExplorationMission', discovery_goal: 4}"
```

**Expect:** `accepted=True`; `/mission/state` lifecycle walks **PREPARE → EXPLORING →
MONITORING**. The robot frontier-explores, detects AprilTags, then approaches each pot
and the arm strikes the inspect pose (gripper cam on the bloom); flower colour + any
pest land as markers on the twin / HMI Map.

---

## 6. Pre-show test sequence

| Test | How | Pass |
|---|---|---|
| **Grounded** | Look at Gazebo / `tf2_echo map base_link` | Robot on the floor, z ≈ 0 |
| **Drive** | HMI Teleop joystick, or `/cmd_vel` pub | Robot moves in the expected direction |
| **Lidar in HMI** | HMI Lidar view | Scan paints; planters show as obstacles |
| **Map + Nav2 goal** | HMI Map view, click a destination | Robot plans + drives, routes **around** planters |
| **Mission** | HMI **Explore & monitor** = 4 | State walks PREPARE→EXPLORING→MONITORING |
| **Flower → map** | Let the mission inspect a tagged pot | Marker appears on the Map (colour unreliable in the 1.0.8 layout — see §7) |
| **Battery divert** | mid-mission: `ros2 param set /sim_battery_publisher override_percentage 0.15` | State → RETURNING, robot docks + pauses; recover with `0.9` then `/mission/resume` |
| **E-stop** | `timeout 4 ros2 topic pub --qos-reliability reliable -r 20 /e_stop_state std_msgs/msg/Bool '{data: true}'` then `false` | Robot halts + pauses; release + `/mission/resume` continues |

> The e-stop and battery have **no sim publisher of their own** — you drive them by
> hand as above (e-stop needs a ≥4 s reliable-QoS publisher or it loses the discovery
> race against the orchestrator's subscriber).

---

## 7. Known quirks (don't panic mid-demo)

- **Drive is teleport, not rolling wheels** — the base slides under `planar_move`. The
  one accepted sim-only difference; everything upstream matches hardware.
- **`FAULT` recovers with `/mission/reset`** — it is NOT terminal. `/mission/abort` still
  won't clear it, but `ros2 service call /mission/reset std_srvs/srv/Trigger {}` fires the
  `recover` transition (FAULT→READY) without restarting the orchestrator; then start the
  mission again. (Relaunching T2 also works but is heavier.)
- **Flower colour reads as uniform `tulip_white`** in the current 1.0.8 layout: the gripper
  cam frames the planter wall, not the blossoms, so `sim_flower_detector` sees a desaturated
  frame. The marker still appears but species/pest is unreliable; the mission FSM is
  unaffected (species is display-only per `docs/CONTRACTS.md`). Fix = re-aim the `inspect`
  arm preset / gripper-cam mount and verify against `/sim_flower/overlay`.
- **3 controllers, not 5** (see §2). Expected.
- **Planters in the costmap:** the planter bodies are 0.14 m so the ~0.107 m lidar
  plane sees them and Nav2 routes around them. **Do NOT regenerate `greenhouse.world`**
  for tweaks — the original layout args are lost and a regen moves every tag; hand-patch
  the world (see the generator header / `project_2026_06_02d_sim_port_session`).
- **Re-run sim-clean (§1) between every run** or SHM contention accumulates.

---

## 8. Emergency reset

```bash
bash ~/ros2_ws/src/lupin/lupin_bringup/scripts/sim-clean.sh
rm -f /dev/shm/fastrtps* /dev/shm/sem.fastrtps*
# still wedged after a couple of tries → reboot, or use a fresh ROS_DOMAIN_ID everywhere.
```

Then redo §2 onwards.

---

## 9. Reference

- **HMI URL:** http://localhost:8090
- **Drive topic (sim):** `/cmd_vel` → `gazebo_ros_planar_move` (hardware: `/mirte_base_controller/cmd_vel`)
- **Odom:** `planar_move` publishes `/odom`, relayed to `/mirte_base_controller/odom` (hardware-identical topic) + `odom→base_link` TF
- **Mission:** start `/mission/start` (`lupin_msgs/srv/StartMission`) or HMI **Explore & monitor**; state on `/mission/state`; `/mission/abort|pause|resume|skip_current|dock|reset`
- **Clean script:** `lupin_bringup/scripts/sim-clean.sh`
- **Vendor sim edits:** `lupin_bringup/scripts/apply-sim-vendor-edits.sh`
- **Deeper runbook:** `lupin/docs/sim_full_mission_runbook.md`
- **All-in-one `sim_full.launch.py`** (the quick-demo path the deeper runbook uses): on
  `main` its twist_mux is mis-wired to a dead topic, so the robot won't drive — fixed in
  MR !42. Until that merges, use this 3-terminal split (unaffected: `sim_robot`'s mux is
  correct). Full root-cause + validation: `lupin/docs/superpowers/2026-06-08-mission-debug.md`.
