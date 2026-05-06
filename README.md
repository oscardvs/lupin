# MDP – Team Lupin – FloraNova Digital Twin

RO47007 Multidisciplinary Project, 2025–2026.
Robot platform: MIRTE Master V2 (holonomic 4-mecanum base, 4-DOF arm).
Client: FloraNova (commercial greenhouse).

> The team is called **Lupin**. The GitLab path is
> `cor/ro47007/2026/group_14/lupin` — `group_14` is the course-assigned
> subgroup, `lupin` is our project inside it.

## Prerequisites

Before cloning this repo, every team member's laptop must have a working
ROS 2 workspace with the MIRTE vendor stack installed. **All of the
following lives outside the `lupin` package — it's shared infrastructure
that every group on this course needs.** Once it's done, the `lupin`
repo drops in as one more package alongside the vendor folders.

### 1. Operating system
**Ubuntu 22.04** (dual-boot or native, **not** WSL).

### 2. ROS 2 Humble
Follow the
[official install guide](https://docs.ros.org/en/humble/Installation.html)
(`ros-humble-desktop` is enough). Make sure
`source /opt/ros/humble/setup.bash` is in your `~/.bashrc` afterwards.

### 3. Workspace + colcon top-level marker
Create the workspace and install the
[`colcon-top-level-workspace`](https://github.com/rhaschke/colcon-top-level-workspace)
extension so `colcon build` works from anywhere inside `~/ros2_ws/`:

```bash
mkdir -p ~/ros2_ws/src
touch ~/ros2_ws/.colcon_root
pip install colcon-top-level-workspace
```

The empty `.colcon_root` file marks the workspace root; the pip extension
teaches `colcon` to find it.

### 4. MIRTE Master vendor packages
Pull the vendor stack into `~/ros2_ws/src/` via `vcstool` and the
`mirte.repos` manifest, then install rosdeps and build. The full
procedure is documented by the MIRTE team — primary references:

- [MIRTE Master developer docs](https://docs.mirte.org/develop/index.html)
- [Simulation install guide](https://docs.mirte.org/0.2.0/doc/simulation/install_simulation.html)
- [Running MIRTE Master in Gazebo](https://docs.mirte.org/0.2.0/doc/simulation/mirte_master_gazebo.html)
- [Main mirte-ros-packages repo](https://github.com/mirte-robot/mirte-ros-packages/tree/develop/)

Individual upstream repos pulled in by `mirte.repos` (for reference):

| Component | Upstream |
| --- | --- |
| Main MIRTE packages | <https://github.com/mirte-robot/mirte-ros-packages/tree/develop/> |
| Mecanum wheel controller | <https://github.com/clearpathrobotics/clearpath_mecanum_drive_controller> |
| Camera (image transport + lazy publish) | <https://github.com/ArendJan/ros2_astra_camera/tree/fix-ros-jammy> |
| Lidar | <https://github.com/Slamtec/rplidar_ros/tree/ros2> |
| Navigation | <https://github.com/kas-lab/mirte_navigation/tree/physical_robot> |
| Gazebo / sim worlds | `mirte-gazebo`, `aws_robomaker_small_house_world`, `plasys_house_world`, `robocup_home_simulation`, `gazebo_grasp_fix`, `sdf_models` |

After this step, `~/ros2_ws/src/` should contain ~15 vendor folders plus
the `mirte.repos` file. Verify with:

```bash
ls ~/ros2_ws/src
```

You can now build the vendor workspace once before adding `lupin`:

```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

### 5. MDP greenhouse simulator (`mdp-greenhouse`)

The course provides a Python-only environmental simulator that returns
sensor readings (temperature, humidity, CO₂, light, soil moisture) at
fixed tag locations in a virtual greenhouse — released on PyPI as
[`mdp-greenhouse`](https://pypi.org/project/mdp-greenhouse/). This is
the **actual sensing modality** for the project: the robot navigates to
a tag, identifies it, and queries this library for readings. There is
no Gazebo plugin and no ROS topic — integration goes through the
Python API.

Install the package and the dependencies it forgets to declare:

```bash
sudo apt install python3-tk           # required for --edit / --view GUIs
pip install mdp-greenhouse pyyaml matplotlib numpy
```

> **Upstream packaging gotchas (still present as of v1.0.6)** — pinned
> here so nobody loses half a day to them:
> - The wheel declares no runtime dependencies, so a bare
>   `pip install mdp-greenhouse` will crash on first import with
>   `ModuleNotFoundError: No module named 'yaml'`. Install `pyyaml`,
>   `matplotlib`, and `numpy` alongside it.
> - The wheel registers no console script, so the `mdp-greenhouse` CLI
>   shown in the upstream README is **not** on `PATH`. Invoke it as a
>   module instead: `python -m greenhouse_sim.cli ...`

Smoke-test the install:

```bash
python -m greenhouse_sim.cli --read --list-tags     # lists default tag IDs 1..22
python -c "from greenhouse_sim.simulator import GreenhouseSimulator; \
           s = GreenhouseSimulator(); print(s.get_sensor_data(s.tags()[0]))"
```

The default greenhouse ships 22 tag locations and 12 tables in a
~4 m × 8 m footprint; configs live under
`<site-packages>/greenhouse_sim/configs/`. To author your own layout,
use `python -m greenhouse_sim.cli --init <folder>` followed by
`--edit <folder>`.

Lupin consumes this library through a ROS 2 wrapper node (planned home:
`lupin_perception` or a new `lupin_greenhouse_bridge` package) that
holds a long-lived `GreenhouseSimulator` instance and exposes a
service taking a `tag_id` and returning the measurement dict. Tag IDs
are expected to line up with the AprilTag IDs detected by
`lupin_navigation`.

## Cloning this repository

Clone Lupin's code as a **sibling** of the MIRTE vendor packages —
not inside any of them:

```bash
cd ~/ros2_ws/src
git clone git@gitlab.tudelft.nl:cor/ro47007/2026/group_14/lupin.git
```

If the clone fails with a permissions error, you have not yet added
your SSH key to GitLab. See the
[GitLab SSH guide](https://docs.gitlab.com/ee/user/ssh.html), then add
the public key at
<https://gitlab.tudelft.nl/-/user_settings/ssh_keys>.

After cloning, check out the branch that matches what you want to do:

```bash
cd lupin
git checkout sim          # to develop or run in Gazebo
# or
git checkout hardware     # to run on the real MIRTE Master
```

## Branch model

This repository uses **three long-lived branches**:

| Branch | What lives here |
| --- | --- |
| `main` | Shared code that is identical regardless of deployment target — messages, perception, planning, HMI logic. **Default branch.** Most feature MRs target `main`. |
| `sim` | Everything in `main` plus simulation-specific bringup: Gazebo launch files, sim parameters, fake-hardware bridges, custom worlds. |
| `hardware` | Everything in `main` plus real-robot bringup: MIRTE driver launches, calibrated parameters for our unit, real-camera AprilTag config. The final demonstration runs from this branch. |

**Where do my changes go?**

- Shared logic, algorithms, messages → branch off `main` (`feat/<topic>`), MR back to `main`.
- Sim-only artefacts (new Gazebo world, sim debug tool) → branch off `sim` (`sim/<topic>`), MR back to `sim`.
- Hardware-only artefacts (calibration values, real-robot launch tweak) → branch off `hardware` (`hw/<topic>`), MR back to `hardware`.

When something lands on `main`, a maintenance MR merges `main` into `sim` and `hardware` so the deployment branches stay current. This is part of the regular weekly maintenance.

## Building

After cloning `lupin`, rebuild the workspace so the new packages are
indexed alongside the vendor stack:

```bash
cd ~/ros2_ws
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
source install/setup.bash
```

Add the `source` line to `~/.bashrc` to avoid repeating it every shell.

## Running

Top-level launches live in `lupin_bringup`. The recommended entry points
depend on which branch you have checked out and what you're trying to do:

```bash
# On the `sim` branch — full greenhouse demo end-to-end
# (Gazebo + slam_toolbox + Nav2 + bridge + v2 mission orchestrator + rosbridge + web HMI + RViz):
ros2 launch lupin_bringup sim_full.launch.py
# Banner at the top of the log links to http://<your-LAN-IP>:8090.
# See "Sim — full mission run (v2 orchestrator end-to-end)" below
# for the full end-to-end recipe + what to do once it's up.
```

Other sim-side entry points are available for partial bringup (debugging
individual layers, KRR house testing, etc.) — see the per-scenario
sections below.

`hardware.launch.py` is forthcoming — for now, hardware bringup uses
the per-package launches via the multi-terminal recipe documented in the
team wiki.

### Greenhouse Gazebo world

For perception / navigation work that needs the actual greenhouse layout
(matched to `mdp-greenhouse`'s tag and table coordinates), use:

```bash
ros2 launch lupin_bringup greenhouse_sim.launch.py
```

This brings up Gazebo with the generated greenhouse world, spawns the
MIRTE Master with its Astra Pro Plus depth-camera plugin
(`/camera/image_raw`, `/camera/depth/image_raw`, `/camera/points`,
`/camera/camera_info`), and starts the standard ros2_control + twist_mux
pipeline. Override the spawn pose with `x:=`, `y:=`, `yaw:=` if needed —
the default puts the robot in the south aisle facing the tables.

The world's tag and table positions are derived from the
`tag_locations.json` shipped inside the `mdp-greenhouse` Python package,
so the Gazebo origin is the same as the bridge's coordinate frame —
nav2, the bridge, and AprilTag detection all agree about positions.

`lupin_perception` provides a lightweight AprilTag overlay helper for
debugging camera perception. Launch it with:

```bash
ros2 launch lupin_perception perception.launch.py
```

This starts the `tag_annotator` node, which subscribes to `/camera/image_raw`
and publishes `/camera/tag_detections_json` with AprilTag corner and distance
metadata. The web HMI overlays AprilTag boxes on the live MJPEG camera stream
instead of relying on a separate boxed-image topic.

For autonomous navigation in this world, the greenhouse has no
pre-built map — pair it with slam_toolbox + Nav2 in SLAM mode, see
[Sim — Nav2 + slam_toolbox in a world without a saved map](#sim--nav2--slam_toolbox-in-a-world-without-a-saved-map)
below.

#### Regenerating the world

The committed `lupin_bringup/worlds/greenhouse.world` is a deterministic
output of `lupin_bringup/scripts/generate_greenhouse_world.py`. Re-run
the script if `mdp-greenhouse` ever publishes a new layout:

```bash
python3 src/lupin/lupin_bringup/scripts/generate_greenhouse_world.py
colcon build --packages-select lupin_bringup --symlink-install
```

By default the script reads `tag_locations.json` from the installed
`greenhouse_sim` package (via `importlib.resources`); pass `--input` to
point it elsewhere.

#### Heads-up for perception integration

The tag visuals in this world are deliberately ugly **bright magenta
placeholders** (model names `apriltag_<id>_PLACEHOLDER`, visual names
`TAG_<id>_PLACEHOLDER_NEEDS_TEXTURE`) — present at the right pose so
the depth camera registers them, but **an AprilTag detector pointed at
this world will not detect anything** until they're replaced with real
`tag36h11` textures. Don't spend time debugging "why doesn't my
detector find anything" — it's the textures. See the TODO inside
`_render_tag` in `scripts/generate_greenhouse_world.py` for the
swap-in path.

To make the AprilTag models available to Gazebo, run the setup script once:

```bash
cd ~/ros2_ws/src/lupin
./lupin_bringup/setup_apriltag_models.sh
```

This links the `lupin_bringup/models/apriltags` directory into Gazebo's model search path.

### Greenhouse sensor bridge (`lupin_greenhouse_bridge`)

Environmental sensing in this project is **not** simulated in Gazebo —
temperature, humidity, CO₂, light, and soil moisture come from the
course-provided [`mdp-greenhouse`](https://pypi.org/project/mdp-greenhouse/)
Python library, queried at fixed tag locations. `lupin_greenhouse_bridge`
holds a long-lived `GreenhouseSimulator` instance and exposes it as a
ROS 2 service so the rest of the stack can consume sensor readings
through a stable contract.

```bash
ros2 launch lupin_greenhouse_bridge greenhouse_bridge.launch.py
```

That brings up one node (`/greenhouse_bridge`) advertising:

| Service | Type | Purpose |
| --- | --- | --- |
| `/greenhouse_bridge/get_tag_reading` | `lupin_msgs/srv/GetTagReading` | Return a `TagReading` (sim-time-of-day + per-sensor `SensorReading[]`) for the requested `tag_id`. Returns `STATUS_UNKNOWN_TAG` for IDs not in the loaded greenhouse. |

Quick smoke test from a second shell:

```bash
ros2 service call /greenhouse_bridge/get_tag_reading \
  lupin_msgs/srv/GetTagReading "{tag_id: '1'}"
```

**Launch args** (all forward to `GreenhouseSimulator` and the sim-time
config; sentinels mean "leave the upstream default alone"):

| Arg | Default | Meaning |
| --- | --- | --- |
| `tag_file` | `''` | Path to a custom `tag_locations.json` (else use `mdp-greenhouse`'s shipped one). |
| `sim_config_file` | `''` | Path to a custom `greenhouse_config.yaml`. |
| `debug_time_of_day` | `-1.0` | Hours (0–24); setting this flips `debug_mode: true` so sim time freezes. |
| `speedup_factor` | `0.0` | Sim seconds per real second (default config: 1800 → full 24 h cycle in 48 s). |
| `debug_seed` | `-1` | RNG seed for deterministic noise; setting this also flips `debug_mode: true`. |

> **Upstream gotcha (mdp-greenhouse 1.0.6):** `current_time()` now
> wraps to `[0, 86400)`, but the underlying hour-to-seconds conversion
> is still off — debug uses `t_h * 86400` (×24 too big), live uses
> `1440 * hour` (×2.5 too small). Effect: `sim_time_of_day_seconds`
> doesn't correspond to wall-clock time; treat it as opaque. The
> bridge pins `mdp-greenhouse>=1.0.6` for the range guarantee and
> keeps its own modulo wrap as defense-in-depth. `debug_seed`
> unaffected. Tracked with C.Pek@tudelft.nl; partial fix landed
> in 1.0.6, conversion still pending.

The bridge has no Gazebo dependency — you can run it standalone
(useful for offline mission-logic dev), or pair it with
`greenhouse_sim.launch.py` once tag IDs come from a real AprilTag
detector against the world. Tag IDs in the bridge match the IDs baked
into the generated greenhouse SDF, so service calls and AprilTag
detections agree once the textures land.

### Sim — full Nav2 stack

When you want autonomy in Gazebo (KRR Course small-house world,
slam-built map, MPPI local planner with mecanum/Omni motion, the
existing teleop/cmd_vel_mux on `/cmd_vel_auto`), use these launches.
The vendor sim package needs `/usr/share/gazebo/setup.sh` sourced or
spawn_entity hangs — every teammate hits this once.

```bash
# Terminal 1 — Gazebo + the MIRTE + teleop chain:
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source ~/ros2_ws/install/setup.bash

ros2 launch lupin_hmi teleop.launch.py use_sim:=true world:=navigation
# KRR house has ~200 models; expect 3–5 min to spawn on a typical laptop.
# spawn_entity may print a 30 s client-side timeout — ignore, gzserver
# continues. Wait for "Configured and activated mirte_base_controller".
```

```bash
# Terminal 2 — Nav2 (run after the controllers are active):
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch lupin_navigation nav2.launch.py
# Brings up map_server, AMCL (set_initial_pose at the spawn pose),
# controller_server (MPPI, motion_model:Omni), planner_server,
# behavior_server, bt_navigator, waypoint_follower, velocity_smoother,
# and two lifecycle_managers. velocity_smoother's smoothed output is
# remapped to /cmd_vel_auto so it goes through cmd_vel_mux just like
# the joystick — manual override still wins.
```

In RViz, set Fixed Frame to `map` and use the **Nav2 Goal** tool to
send a goal. To re-anchor AMCL after manual driving, use **2D Pose
Estimate**.

For mapping a new world (instead of using the saved
`lupin_navigation/maps/krr_house`):

```bash
# Replace Terminal 2 with slam_toolbox in mapping mode:
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix lupin_navigation)/share/lupin_navigation/config/slam_toolbox_sim.yaml

# Drive around (keyboard fallback if you don't have a joystick):
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r cmd_vel:=/cmd_vel_manual

# Save (the transient_local flag is required — slam_toolbox publishes /map durable):
cd ~/ros2_ws/src/lupin/lupin_navigation/maps
ros2 run nav2_map_server map_saver_cli -f <name> \
  --ros-args -p use_sim_time:=true -p map_subscribe_transient_local:=true
```

#### Sim — Nav2 + slam_toolbox in a world without a saved map

Use this when you want autonomous navigation in a world that has no
prebuilt map yet (e.g. the greenhouse): slam_toolbox builds the map
online and Nav2 plans and follows paths against the live `/map`.
`nav2.launch.py slam:=true` drops `map_server`, AMCL, and
`lifecycle_manager_localization` so they don't fight slam_toolbox over
`/map` and the `map→odom` TF.

```bash
# Terminal 1 — sim:
ros2 launch lupin_bringup greenhouse_sim.launch.py
# (or any other Lupin sim entry point)

# Terminal 2 — slam_toolbox owns localization + the map:
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix lupin_navigation)/share/lupin_navigation/config/slam_toolbox_sim.yaml

# Terminal 3 — Nav2 in SLAM mode (only the navigation half):
ros2 launch lupin_navigation nav2.launch.py slam:=true
```

Send goals via the **Nav2 Goal** tool in RViz as usual. Drive a short
loop manually first so slam_toolbox has a few scans of context — Nav2's
global costmap won't plan beyond the explored region.

To later promote the live map to a saved one, follow the
`map_saver_cli` step from the previous section while slam_toolbox is
still running.

### Sim — full mission run (v2 orchestrator end-to-end)

Drive the entire stack — Gazebo greenhouse + slam_toolbox + Nav2 +
greenhouse bridge + v2 mission orchestrator + rosbridge + the Lupin
web HMI — and have the robot autonomously visit AprilTag stations.

#### Option A: one-shot bringup (recommended)

```bash
ros2 launch lupin_bringup sim_full.launch.py
```

This composes the full chain (Gazebo + slam_toolbox + Nav2 + bridge +
mission orchestrator + rosbridge + web HMI + RViz with the persistent
`rviz/full_bringup_viz.rviz` config + the `/amcl_pose` seed — see
"Why the AMCL seed?" below) and is the right entry point for a demo
or a quick sanity check. The bringup is event-driven, not delay-driven:
slam_toolbox waits for `/scan` to publish, Nav2 waits for `/map`. A
banner at the top of the launch log prints clickable
`http://localhost:8090` and `http://<lan-ip>:8090` URLs for the HMI.

Skip pieces with `rviz:=false`, `enable_web:=false`, `seed_amcl:=false`.

Once the chain settles (Gazebo loaded, `mirte_base_controller` active,
Nav2 lifecycle managers report `Managed nodes are active`, orchestrator
prints `Dependencies up. Orchestrator READY.`):

1. Open **http://localhost:8090** in a browser.
2. **One-time setting** (per browser, persisted to localStorage):
   click the gear icon → in **Drive**, change `cmd_vel topic` to
   `/cmd_vel_manual`. The sim's `twist_mux` arbitrates between Nav2
   (`/cmd_vel_auto`) and manual override (`/cmd_vel_manual`); publishing
   directly to the controller topic bypasses the mux and races Nav2.
3. Open the **Teleop** tab and drive a short loop (left joystick =
   linear x/y, right = angular z). slam_toolbox needs scan context
   before Nav2 will plan — Nav2's global costmap won't extend beyond
   the explored region.
4. Kick off a mission:
   ```bash
   ros2 service call /mission/start lupin_msgs/srv/StartMission \
     "{mission_type: 'InspectionMission', tag_sequence: ['1','2']}"
   ```
   Watch live progress:
   ```bash
   ros2 topic echo /mission/state lupin_msgs/msg/MissionState
   ros2 topic echo /floranova/observations lupin_msgs/msg/Observation
   ```

Operator controls (any time during a mission):

```bash
ros2 service call /mission/pause         std_srvs/srv/Trigger
ros2 service call /mission/resume        std_srvs/srv/Trigger
ros2 service call /mission/abort         std_srvs/srv/Trigger
ros2 service call /mission/skip_current  std_srvs/srv/Trigger
```

The web HMI's permanent E-STOP bar also preempts the mission — release
clears `/e_stop_state` but mission stays paused until you call
`/mission/resume`.

#### Option B: terminal-by-terminal (for debugging individual layers)

Each terminal needs `source /opt/ros/humble/setup.bash`,
`source /usr/share/gazebo/setup.sh`, and
`source ~/ros2_ws/install/setup.bash` first.

```bash
# 1. Gazebo + MIRTE + greenhouse world
ros2 launch lupin_bringup greenhouse_sim.launch.py
# Wait for: "Configured and activated mirte_base_controller"

# 2. slam_toolbox (greenhouse has no saved map)
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true \
  slam_params_file:=$(ros2 pkg prefix lupin_navigation)/share/lupin_navigation/config/slam_toolbox_sim.yaml

# 3. Nav2 (slam mode — drops AMCL + map_server)
ros2 launch lupin_navigation nav2.launch.py slam:=true
# Wait for: "Managed nodes are active"

# 4. Greenhouse sensor bridge
ros2 launch lupin_greenhouse_bridge greenhouse_bridge.launch.py

# 5. Mission orchestrator (idles in READY)
ros2 launch lupin_mission mission.launch.py dependency_timeout_s:=120.0

# 6. rosbridge_websocket :9090 (sim laptop doesn't have it pre-baked)
ros2 run rosbridge_server rosbridge_websocket

# 7. Lupin web HMI :8090
ros2 launch lupin_web lupin_web.launch.py

# 8. AMCL pose seed (one-shot — see below)
ros2 run lupin_bringup seed_amcl_pose

# 9. RViz (optional, useful)
rviz2  # set Fixed Frame=map; add TF, /scan, /map, /plan
```

Then drive via the web HMI and call `/mission/start` exactly as in
Option A.

#### Why the AMCL seed?

The v2 orchestrator's `PREPARE.LOCALIZING` gate subscribes to
`/amcl_pose` and waits for the diagonal covariance (x, y, yaw) to drop
below `localization_covariance_threshold` (default `0.25`). On the real
robot AMCL is running and publishes that pose naturally. In sim with
slam_toolbox, AMCL is intentionally absent — slam_toolbox owns
localization and the `map → odom` TF — so `/amcl_pose` never appears
and the orchestrator times out into `FAULT`.

`ros2 run lupin_bringup seed_amcl_pose` publishes a single
`PoseWithCovarianceStamped` with TRANSIENT_LOCAL durability and tight
diag covariance (`0.01` on x, y, yaw), satisfying the gate. The
`sim_full.launch.py` composition fires it automatically alongside the
orchestrator at startup (the orchestrator's `/amcl_pose` subscription
is already alive, so the seed lands and is cached). Disable with
`seed_amcl:=false`. When running the chain manually, run the script
yourself once after the orchestrator hits `READY`.

#### Tag-coordinate caveat in sim

Tag coordinates in `tag_locations.json` are in the greenhouse-world
frame. slam_toolbox's `map` frame originates at the robot's spawn
pose. The greenhouse SDF is generated such that the spawn pose
(`spawn_x`, `spawn_y`, `spawn_yaw` in `sim_full.launch.py`) puts those
two frames in approximate alignment, but the P3D plugin's pose drift
(see "Sim — known limitations") accumulates over long runs. For the
first end-to-end test, prefer tags near the spawn aisle (`['1','2']`);
expand once the chain is proven.

### Sim — known limitations

- `gazebo_planar_move` (vendor URDF P3D plugin) publishes `odom →
  base_link` at ~7 Hz from commanded velocity. Combined with mecanum
  slip, this gives some drift between global (map) and local (odom)
  costmaps at long range — first goal usually plans, occasional
  long-range goals fail and trigger Nav2's clear-costmap recovery.
  A clean fix needs URDF surgery to silence the vendor plugin's TF
  and run a `/groundtruth/odom` bridge as the sole publisher;
  deferred until after the sim demo. **Hardware is unaffected** —
  real wheel encoders are accurate.
- `cmd_vel_mux` only forwards manual Twists when nonzero, so the
  joystick "release" doesn't actively publish a stop — autonomous
  resumes 0.5 s later. Tracked separately.

## Repository layout

| Package | Purpose |
| --- | --- |
| `lupin_bringup` | Top-level launch files (`sim_full.launch.py`, `greenhouse_sim.launch.py`), the greenhouse SDF + generator, RViz config, and small system-glue helpers (`seed_amcl_pose`). |
| `lupin_navigation` | Nav2 params + slam_toolbox config; saved KRR-house map; planned home for AprilTag pose corrections. |
| `lupin_mission` | Mission orchestrator (v2). Hierarchical state machine via `transitions`: `BOOT → READY → PREPARE → INSPECTING → RETURNING → DONE / FAULT`. Polymorphic observation publisher on `/floranova/observations`, status on `/mission/state` (5 Hz), operator services `/mission/{start,pause,resume,abort,skip_current}`, cross-cutting `/e_stop_state` monitor. See `lupin_mission/README.md`. |
| `lupin_msgs` | Custom messages and services: `Observation`, `MissionState`, `TagReading`, `SensorReading`, `FlowerObservation`, `AnomalyReport`; `GetTagReading`, `StartMission`. |
| `lupin_greenhouse_bridge` | ROS 2 wrapper around the `mdp-greenhouse` simulator. Single `~/get_tag_reading` service. Open-sourced separately at `lupin_greenhouse_ros/`. |
| `lupin_hmi` | PS4 + keyboard teleop, `cmd_vel_mux` for arbitration between manual override / Nav2 / web. |
| `lupin_web` | Browser HMI on `:8090` — Vite + React + shadcn/ui. Tabs: Teleop, Arm, Voice, Cameras, Telemetry, Logs, Map. Talks to rosbridge on `:9090`. The Voice tab is a Gemini Live agent with a 14-tool surface (`drive`, `nav_goto`, `nav_forward`, `rotate`, `arm_preset`, `query_state`, `engage_estop`, …) that drives the robot in natural language. |
| `lupin_perception` | Vision package: AprilTag overlay and future flower detection pipelines. Includes `perception.launch.py`, `tag_annotator`, and `config/tags.yaml`. |
| `docs/` | Architecture diagrams, design notes. |

## Contributing

- All three long-lived branches (`main`, `sim`, `hardware`) are **protected** — direct pushes are blocked. All changes go through MRs.
- Branch naming:
  - `feat/<topic>`, `fix/<topic>`, `docs/<topic>`, `chore/<topic>` for branches targeting `main`.
  - `sim/<topic>` for branches targeting `sim`.
  - `hw/<topic>` for branches targeting `hardware`.
- Every MR needs **one approving review** from a teammate (this is the *buddy check* the course grades).
- Commit messages follow [Conventional Commits](https://www.conventionalcommits.org/),
  e.g. `feat(navigation): add AprilTag pose correction`.
- Commit under your **own** GitLab account — commit distribution is graded.

## Team

| Name | GitLab | Role |
| --- | --- | --- |
| Koen Vogels | @kvogels | Perception, Control Interface |
| Lapo Veca | @lapoveca | Navigation, Manipulation |
| Lievijn Simons | @lwmssimons | Hardware Integration, Logic |
| Oscar Devos | @odevos | Navigation, Manipulation |
| Tejas Stanley | @tstanley | Perception |
| Tibbe Wouters | @twouters | Task Logic, Control Interface |

## Course staff

- Chris Pek — responsible lecturer, main project lead
- Carlos Hernandez Corbato — systems architecture
- Martin Klomp / Arend-Jan van Hilten — MIRTE Master creators & developers
- Martijn Wisse — robot expert
- Thijs Hoedemakers — lecturer
- Gillian Saunders — course support, skills & reflection
