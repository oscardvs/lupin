# Demo-day bringup — Mirte-247264 (Lupin)

Step-by-step procedure for getting the full Lupin stack live in front of an
audience. Each step has *what to expect* and *if it fails*. Scan-able rather
than prose — the goal is "look up this file under pressure and follow it".

> The 8-terminal layout is intentional: every subsystem can be killed and
> restarted in its own terminal without taking the rest down. Recommended
> over `ros2 launch lupin_bringup hardware.launch.py` for demos where
> on-stage debugging may be needed.

---

## First-time laptop setup (one-time per laptop)

Everything in §3 onwards assumes the laptop is already wired to talk to the
robot. The five things below are **not** included in a fresh `git clone` —
do them once per laptop and you never look at this section again. The full
versions live in `lupin_bringup/README.md`; this is the demo-day-only summary.

**1. Clone + build the workspace.** Paths throughout this doc assume
`~/ros2_ws/src/lupin`:

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b hardware git@gitlab.tudelft.nl:cor/ro47007/2026/group_14/lupin.git
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y   # apt deps for all lupin packages
cd ~/ros2_ws/src/lupin/lupin_web/web && npm install && npm run build  # T1 (HMI) needs the Vite artefact
cd ~/ros2_ws && colcon build --symlink-install
```

Assumes ROS 2 Humble + the standard MIRTE laptop setup (rviz2, Nav2,
slam_toolbox, rosbridge_suite, joy). If `colcon build` complains about
missing packages beyond what `rosdep` resolved, install them via apt and
re-run.

**2. SSH key + `lupin` alias** (needed for §1's `post-boot-sync.sh` to run
`sudo` non-interactively on the robot, and for §7's `ssh lupin`):

```bash
[ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
ssh-copy-id mirte@192.168.42.1                          # password: ask team — system mirte password
cat >> ~/.ssh/config <<'EOF'

Host lupin
    HostName 192.168.42.1
    User mirte
    IdentityFile ~/.ssh/id_ed25519
EOF
ssh lupin 'echo ok'                                     # smoke test — should not prompt
```

Update `HostName` if the robot is on a different network (lab WiFi, travel
router). **Don't** use `~/.ssh/id_rsa` if it exists on the robot — that key
is image-baked and shared across all MIRTEs (`project_mirte_shared_image_key`).

**3. DDS env (laptop ↔ robot discovery server).** §3 and §4 source
`~/.config/lupin/ros-env.sh` — that file's not in the repo (it bakes in
your `$HOME` and the robot IP). Generate it:

```bash
cd ~/ros2_ws/src/lupin/lupin_bringup
./scripts/setup-laptop-dds-env.sh                       # default robot IP 192.168.42.1 (robot AP)
./scripts/setup-laptop-dds-env.sh 10.0.0.42             # custom IP if on lab WiFi / travel router
```

It writes `~/.config/lupin/fastdds_super_client.xml` +
`~/.config/lupin/ros-env.sh` and appends a source-line to `~/.bashrc`. zsh
users: append the same line to `~/.zshrc` manually. Re-run with a new IP
whenever you swap networks. To back out: `--uninstall`.

**4. `post-boot-sync.sh` symlink** (§1 calls it):

```bash
mkdir -p ~/.config/lupin
source ~/ros2_ws/install/setup.bash
ln -s "$(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/post-boot-sync.sh" \
      ~/.config/lupin/post-boot-sync.sh
```

Symlinked (not copied) so a future `colcon build` of `lupin_bringup`
auto-updates the script in place. Override the target robot at call-time
with `ROBOT=mirte@<ip> ~/.config/lupin/post-boot-sync.sh`.

**5. Smoke test** — open a fresh terminal so `.bashrc` picks up the DDS
env, then:

```bash
source ~/ros2_ws/install/setup.bash
ros2 daemon start && sleep 5
ros2 topic list | wc -l                                 # expect 50+, not 2
~/.config/lupin/post-boot-sync.sh                       # expect drift ≤1s, all services active
```

If `topic list` returns 2: re-check step 3 (`echo $ROS_DISCOVERY_SERVER` —
should be `<robot-ip>:11811`). If `post-boot-sync.sh` prompts for a
password: step 2's `ssh-copy-id` didn't take — re-run it.

> **Xbox controller**: USB-C is the no-drama path (DEMO_DAY assumes this).
> Only if you must pair over BLE, run
> `lupin_bringup/scripts/install-bluetooth-xbox-fix.sh` once — without it,
> BlueZ 5.64 loops the HID descriptor read every ~4 s
> (`project_xbox_ble_pairing_fix`).

---

## 0. Pre-flight (do this BEFORE the audience walks in)

- [ ] Laptop battery > 60 % AND on charger. Brightness up.
- [ ] Xbox controller: charged, **plugged into the laptop via USB-C** (or BT-paired to the laptop). joy_node now runs laptop-side under `hardware.launch.py joystick:=true` — the robot-side joy path was removed (see `project_xbox_ble_pairing_fix`).
- [ ] Robot powered on, on the floor in the demo space, lidar mast clear of obstacles.
- [ ] Wait ~90 s after robot power-on for the boot storm to settle (load avg 8–12 is normal — see [`project_mirte_boot_storm`]).
- [ ] Laptop WiFi connected to `Mirte-247264` AP. Verify with `ip -br addr | grep wlp` shows `192.168.42.66/24`.
- [ ] Internet on a separate interface only (USB tether to phone is fine; needed for any GenAI/Gemini features in the HMI). The route to the robot stays on the WiFi.

---

## 1. Verify robot health + sync clock

```bash
~/.config/lupin/post-boot-sync.sh
```

**Expect:** drift `0s` or `1s`, discovery server `LISTENING` with `users:(("fast-discovery-",…))`, all services `active`, **all 5 controllers `active`**:

| Controller | Role |
|---|---|
| `joint_state_broadcaster` | publishes `/joint_states` |
| `mirte_master_arm_controller` | JTC for the 4-DOF arm |
| `mirte_master_gripper_controller` | gripper action server |
| `pid_wheels_controller` | per-wheel velocity PID |
| `mirte_base_controller` | mecanum_drive_controller (Twist → 4 wheels) |

The vendor `mirte_ros.sh` first-boot race that used to leave controllers stuck `unconfigured` (and that used to drop `mirte_base_controller` + `mirte_master_gripper_controller` entirely when the vendor `spawner-10` died mid-activation) is now closed by `lupin_hmi/auto_home`. The heal loads anything missing from the loaded set, configures the unconfigured, then activates all 5 — runs as `lupin-auto-home.service` after `lupin-onboard`, and `post-boot-sync.sh` re-triggers it idempotently every operator session (handles resume-from-suspend, where systemd doesn't auto-fire boot units).

**If `post-boot-sync.sh` reports `lupin-auto-home` as `failed` or `activating` for more than ~90 s:** the heal can't reach controller_manager. Within-boot retries are bounded by `Restart=on-failure StartLimitBurst=4` on the unit, so it'll have given up after ~3 min. Force a clean cycle:

```bash
ssh mirte@192.168.42.1 sudo systemctl restart mirte-ros.service
# lupin-onboard + lupin-cameras are PartOf=mirte-ros and follow the restart.
# lupin-auto-home is After= but no longer PartOf, so the next post-boot-sync.sh
# run is what re-fires it. Wait ~25 s after the restart, then:
~/.config/lupin/post-boot-sync.sh   # re-verifies + re-triggers auto-home
```

If the restart still leaves controllers stuck, power-cycle the robot (off → 15 s → on) — that clears any ros2_control wedge cleanly.

The "controllers (should show 5 active)" line at the end of `post-boot-sync.sh` sometimes fails with `rcl node's context is invalid` — that's a `ros2 control` CLI bug, not a real failure. The state to trust is what `ros2 control list_controllers` reports directly.

---

## 2. Laptop hygiene

If any previous session left stale FastDDS shared-memory segments, every new
participant will silently fail — `ros2 topic list` returns 2, RViz shows
nothing, etc. Always wipe before launching:

```bash
pkill -9 -f 'ros2|rviz2|rosbridge|slam_toolbox|twin_node|tag_annotator|web_video_server' 2>/dev/null
ros2 daemon stop 2>/dev/null
sleep 1
pkill -9 -f 'ros2cli' 2>/dev/null
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ls /dev/shm | grep -i fast || echo "laptop SHM clean ✓"
```

---

## 3. First laptop terminal — get topics streaming + sanity check

This is the recipe for the **first** terminal of the session. It points the
laptop's DDS at the robot's discovery server, brings up the ros2 daemon, and
verifies topics are flowing. Every subsequent terminal only needs the two
`source` lines (see §4).

```bash
source ~/.config/lupin/ros-env.sh            # ROS_DISCOVERY_SERVER + super-client XML
source ~/ros2_ws/install/setup.bash          # lupin_* packages + RViz mesh paths
ros2 daemon start                            # one-time per session
sleep 5                                       # let discovery fill (~52 topics)
ros2 topic list | wc -l                      # expect ~52 (systemd services only)
ros2 topic hz /scan                          # expect ~10 Hz, Ctrl-C after 3 s
ros2 topic hz /joy                           # expect ~15 Hz if Xbox on, nothing if off
```

**If `topic list | wc -l == 2`**: DDS is wedged. Re-run step 2. If it still
says 2: check `ip -br addr` — if you're multi-homed (phone tether + WiFi)
and the default route is via the phone, FastDDS might be advertising on the
wrong interface. Bring down the tether (`nmcli connection down "Wired connection 1"`)
and try again.

---

## 4. Bit-by-bit launch (one terminal per subsystem)

> **Full greenhouse demo — read this first.** The T1–T8 layout below brings up
> the *base* stack (drive, SLAM, Nav2, HMI, perception overlay+fusion) but leaves
> the **autonomous mission OFF**. For the complete demo (AprilTag fusion + flower
> & pest **map markers** + explore→monitor autonomy) you need T7's **full**
> perception stack and **T9** (mission), both spelled out below. Two ways:
> - **One command** (simplest, least on-stage control):
>   `ros2 launch lupin_bringup hardware.launch.py mission:=true` — everything
>   below *plus* the mission pipeline, in one process tree.
> - **Granular** (per-subsystem kill/restart): run T1–T8 using
>   `perception_stack.launch.py` in T7, then add T9.
>
> Don't mix the two — `hardware.launch.py` already includes T1–T8, so running it
> *and* the individual terminals double-launches everything.

**Every new terminal opened from this point on** starts with the two `source`
lines (same as §3 but without daemon-start, which is already done):

```bash
source ~/.config/lupin/ros-env.sh            # DDS env: discovery server + super-client
source ~/ros2_ws/install/setup.bash          # workspace overlays: lupin_*, RViz meshes
```

Skip either and the terminal will see 2 topics (DDS) or fail to find
`lupin_*` packages and RViz meshes (overlay). Both are needed.

### T1 — HMI (Vite preview + rosbridge :9090; video proxied to robot :8091)

Source the two §4 lines **in this terminal first** — they're repeated in the
block below because T1 fails *silently* if you skip them: the HMI comes up `LIVE`
but with no battery / no telemetry and can't drive (the §6 MULTICAST bug),
unlike T2+ which fail loudly. The launch's `rosbridge DDS mode:` banner is the tell.

```bash
source ~/.config/lupin/ros-env.sh            # DDS env — skip this → §6 MULTICAST bug (HMI LIVE but dead)
source ~/ros2_ws/install/setup.bash
ros2 launch lupin_web lupin_web.launch.py \
  mode:=preview tls:=true rosbridge:=true \
  video:=false video_target:=http://192.168.42.1:8091 leds:=false
```

`video:=false` skips the laptop-side `web_video_server` because
`lupin-cameras.service` on the robot already exposes one on `0.0.0.0:8091`.
`video_target` retargets Vite's `/_video` proxy at the robot directly — raw
camera frames stay on the Pi and only MJPEG crosses WiFi (matches vendor
`mirte.local/ros-video/` behaviour). `leds:=false` because the robot's
`lupin-onboard.service` already runs `light_strip_bridge` — a second one here
would collide on the node name and the `/lupin/leds/{set,auto}` services. The
LightControl card still works; it reaches the robot's bridge over DDS.

**Check the startup banner** the launch prints: it must say `rosbridge DDS
mode: DISCOVERY-SERVER 192.168.42.1:11811`. If it instead says `MULTICAST —
ROS_DISCOVERY_SERVER is UNSET`, this terminal skipped §4's `source` lines —
Ctrl-C and relaunch with them, or the HMI will show LIVE but no battery /
telemetry and won't drive (see §6).

Open `https://localhost:8090` in your browser. Click **Reset** if the e-stop
banner is visible. If the virtual joystick doesn't drive the robot on first
try, **reload the page once** — known startup-state flap, not a real failure.

### T2 — RViz

```bash
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/lupin/lupin_bringup/rviz/full_bringup_viz.rviz
```

**Expect:** robot model + /scan + TF visible. Fixed Frame defaults to `map`
which won't exist yet (no SLAM); set it to `base_link` for now or wait for T3.

**If robot model is missing or Fixed Frame dropdown is empty:** you forgot
to source `install/setup.bash` (needed for `mirte_master_description` mesh
resolution) and/or `ros-env.sh` (needed for DDS). Both.

### T3 — SLAM

```bash
ros2 run slam_toolbox async_slam_toolbox_node \
  --ros-args \
  --params-file ~/ros2_ws/src/lupin/lupin_navigation/config/slam_toolbox_sim.yaml \
  -p use_sim_time:=false
```

**Expect:** `Registering sensor: [Custom Described Lidar]` then /map publishes
within 5–10 s. RViz Map display (toggle the checkbox) populates.

**One `Message Filter dropping ...` at startup is normal** (scan arrives before
TF buffer warms up). If it keeps firing: clock drift — re-run step 1's clock
sync (cleanly, not while ROS is running on the robot).

### T4 — slam_reset service (HMI's "erase map" button)

```bash
ros2 run lupin_navigation slam_reset_node
```

One line of output then quiet. Idle service.

### T5 — Nav2 (only after T3's /map is alive)

```bash
ros2 launch lupin_navigation nav2.launch.py \
  slam:=true \
  use_sim_time:=false \
  params_file:=$(ros2 pkg prefix lupin_navigation)/share/lupin_navigation/config/nav2_params.yaml \
  map:=$(ros2 pkg prefix lupin_navigation)/share/lupin_navigation/maps/krr_house.yaml
```

The `map:=krr_house.yaml` arg is a *placeholder* — in `slam:=true` mode the
map_server isn't instantiated. `RewrittenYaml` just needs a valid path.

**Expect:** ~30 s of lifecycle activation logs, ending with:
```
[lifecycle_manager_navigation]: Managed nodes are active
[lifecycle_manager_navigation]: Creating bond timer...
```

**If you see `Server controller_server was unable to be reached after 4.00s by bond`:**
clock drift or SHM contention. Ctrl-C only T5, then use the targeted restart
below — **not** step 2's full SHM wipe, which would also break slam/RViz/HMI.

#### Restart just Nav2 (leaves slam / RViz / HMI / robot untouched)

Ctrl-C the T5 terminal first. If a node survives (a *ghost* → duplicate node
name + stale DDS/bond state → errors or stalls on the next launch), sweep only
the Nav2 executables. This is laptop-side and matches *only* Nav2:

```bash
NAV2='controller_server|planner_server|behavior_server|bt_navigator|waypoint_follower|velocity_smoother|lifecycle_manager|nav2\.launch'
pkill -f "$NAV2"; sleep 2          # SIGTERM first → nodes release DDS/SHM cleanly
pkill -9 -f "$NAV2" 2>/dev/null    # force-kill any ghost that ignored SIGTERM
pgrep -af "$NAV2" && echo "⚠ still alive — re-run" || echo "Nav2 clean ✓"
```

Then relaunch T5 (with **both** `source` lines). **Do NOT `rm /dev/shm/fastrtps_*`
here** — that wipes the SHM of the still-running slam/RViz/HMI participants and
breaks them; the full wipe (step 2) is for an all-down reset only.

If a *clean* relaunch still stalls at `Configuring planner_server`, that's the
cold-start discovery-server lag (not a ghost) — a second relaunch usually wins;
the real fix is bumping the lifecycle-manager timeouts / going wired.

### T6 — Digital twin

```bash
ros2 launch lupin_twin twin.launch.py
```

One line: `lupin_twin up: obs_topic=/floranova/observations, …`. Idle until
mission orchestrator publishes observations.

### T7 — Perception (AprilTag detector + flower/pest fusion)

Use `perception_stack.launch.py`, **not** the tag-only `perception.launch.py` —
the stack starts all three perception nodes the demo needs:

```bash
ros2 launch lupin_perception perception_stack.launch.py
```

- **`tag_annotator`** — Orbbec AprilTag detection → `tag_<id>` TFs + `/camera/tag_detections_json` (the HMI Cameras overlay).
- **`yolo_detector`** — gripper-cam YOLO → `/yolo/detections` (flower species + the `bug` anomaly class). **Needs `ultralytics` + `numpy<2`** in this env (see `project_ultralytics_install_gotcha`); if missing it no-ops and flowers stay unclassified.
- **`perception_aggregator`** — fuses tags + YOLO into `/perception/discovered_tags` and `KIND_FLOWER` obs on `/floranova/observations` → twin → `/twin/state` (the flower/pest **map markers**).

**Expect:** `tag_annotator online — image="/camera/color/image_raw" …`, then
`Latched intrinsics from camera_info: …` within 3 s, plus a `perception_aggregator`
start line and a YOLO model-load line (a few seconds on first run).

**If no "Latched intrinsics" within 10 s:** cameras aren't publishing yet.
Check `ros2 topic hz /camera/color/camera_info` — should be ~5 Hz. If silent,
`ssh lupin 'sudo systemctl restart lupin-cameras'`.

> **Markers need a mission.** Flower/pest markers only pin once T9's mission
> SCANNING phase attributes a YOLO reading to a tag — "point the camera at a
> flower" alone won't place a marker. The AprilTag green-box overlay (Cameras
> view) works on its own. To run just the tag overlay (no YOLO/torch), the old
> `ros2 launch lupin_perception perception.launch.py` still works.

### T8 — Xbox teleop (joy_node + teleop_twist_joy + arm_teleop)

```bash
ros2 launch lupin_hmi xbox_teleop.launch.py
```

**Expect:** `joy_node ... Opened joystick: Xbox Series X Controller. deadzone: 0.150000`, then `arm_teleop ready @ 10 Hz, step=0.150 rad, ...`, then `arm_teleop seeded from /joint_states: shoulder_pan_joint=..., shoulder_lift_joint=..., ...` once `/joint_states` arrives. Topics published: `/joy`, `/cmd_vel_joy`, arm trajectories on `/mirte_master_arm_controller/joint_trajectory`, gripper goals via the action client.

**Button map (Series X|S BLE HID — probed live 2026-05-21):**

| Input | Role | Index |
|---|---|---|
| **LB** (hold) | drive dead-man — also silences shoulder while held | 6 |
| **RB** (hold) | turbo (~2× scale) | 7 |
| Left stick | translation (forward/back + strafe — mecanum) | axes 0/1 |
| Right stick X | rotation in place (only while LB held) | axis 2 |
| **A** | shoulder_lift − | 0 |
| **B** | shoulder_pan + | 1 |
| **X** | shoulder_pan − | 3 |
| **Y** | shoulder_lift + | 4 |
| D-pad ↑/↓ | elbow ± | axis 7 |
| D-pad ←/→ | wrist ± | axis 6 |
| **RT** (pull) | gripper open | axis 4 |
| **LT** (pull) | gripper close | axis 5 |

Shoulder pan/lift are gated off while LB is held — don't expect Y/A/B/X to move the arm while driving. Release LB before pressing face buttons.

**If a button does nothing:** the BLE HID indices shift between controllers. Probe with `ros2 topic echo /joy --field buttons` and press one button at a time. If yours differ from the table above, patch `lupin_hmi/launch/xbox_teleop.launch.py:153-159` and rebuild `lupin_hmi`.

**If the pad isn't detected (no `Opened joystick:` line):** joy_node only enumerates SDL2 gamepads at startup. Power the controller on first, *then* launch. If you must hotplug, the udev rule `99-lupin-xbox-rebind.rules` pkills joy_node on Xbox-pad arrival and respawn picks it up.

> Alternative: pass `joystick:=true` to the unified bringup (`ros2 launch lupin_bringup hardware.launch.py joystick:=true ...`) instead of running T8 standalone. Same node graph — pick whichever fits the on-stage debugging story.

### T9 — Mission (autonomous explore → monitor → flower; optional)

Only for the autonomous-mission demo. Bundles the greenhouse bridge (tag
oracle), the mission orchestrator, and the one-shot AMCL pose seed. Run it
**after** T1–T8 are up — it needs Nav2 + SLAM + twin + T7's full perception:

```bash
ros2 launch lupin_bringup mission_stack.launch.py        # discovery_goal:=N to override
```

**Expect:** a `greenhouse_bridge` ready line, `[lupin_bringup] mission_stack: …`,
and the orchestrator idling in `READY` (`/mission/state` lifecycle_state=READY).
Start a run from the HMI **Mission** controls, or:
```bash
ros2 service call /mission/start lupin_msgs/srv/StartMission \
  "{mission_type: 'ExplorationMission', discovery_goal: 4}"
```
The HMI MissionStrip should walk
PREPARE → EXPLORING → MONITORING; flower/pest markers land on the Map view as
tags are scanned.

**If the orchestrator FAULTs after ~120 s:** it never saw Nav2 or the bridge —
confirm T5 reached "Managed nodes are active" and that T9 was started after it.

---

## 5. Demo test sequence — verify everything works before the audience arrives

| Test | How | Pass criterion |
|---|---|---|
| **Xbox drive** | Hold **LB**, push left stick forward | Robot moves **forward** physically (drive fixed at the source — `xbox_config.yaml` scales are positive) |
| **Xbox arm** | Release LB. Press/hold **Y/A** → lift up/down. **B/X** → pan right/left. D-pad → wrist/elbow | Arm joints move at ~1.5 rad/s while held; quick taps step 0.15 rad. Watch `arm joints active:` log to confirm input is reaching `arm_teleop` |
| **Xbox gripper** | Release LB. Pull **RT** (open) or **LT** (close) | Gripper opens/closes between [−0.20, 0.25] rad |
| **HMI joystick** | HMI Teleop view, click Reset on e-stop banner, wiggle virtual stick | Robot drives in operator-expected direction (`polarityInvertHmi` now false — no flip needed) |
| **Lidar visible in HMI** | HMI Lidar view | Live scan paints at ~10 Hz |
| **Map visible in HMI** | HMI Map view | OccupancyGrid renders, rotates with robot |
| **Nav2 goal** | RViz "2D Goal Pose" tool, click+drag a goal 1–2 m ahead | Robot plans + drives to goal |
| **HMI map-click goal** | HMI Map view, click destination | Same as above; map renders in the true frame, goal lands where you click |
| **AprilTag overlay** | HMI Cameras view, point camera at a printed tag | Bounding box + tag ID overlay |
| **E-stop** | HMI e-stop button | Robot stops immediately, banner shows "user" reason |
| **Map erase** | HMI menu → "Erase map" | /map clears and slam_toolbox respawns blank |

The rows below need T7's **full** perception stack (`perception_stack.launch.py`) and, for the mission rows, **T9** (`mission_stack.launch.py`):

| Test | How | Pass criterion |
|---|---|---|
| **AprilTag overlay** | HMI Cameras view, point Orbbec at a printed tag | Green box + tag ID + distance overlay (no QR payload — IDs only) |
| **YOLO alive** | `ros2 topic hz /yolo/detections` with the gripper cam at a flower | Ticks > 0 Hz; detections carry a class (`tulip_*` / `bug`) |
| **Flower → map** | T7 + T9; start a mission, let it SCAN a tag that has a flower | Species-coloured ring appears on the HMI Map at that tag |
| **Pest → map** | T7 + T9; a flower YOLO-classed as `bug` | Red dashed ring + "pest detected" tooltip on that flower's marker |
| **Mission explore→monitor** | T9 up; HMI Mission → Start | MissionStrip walks PREPARE→EXPLORING→MONITORING; robot discovers tags then loops |
| **Voice drive** | HMI Voice; **set a Gemini key in Settings first**; "drive forward one metre" | Tab shows "gemini live" (not the amber "mock" banner); a `nav_forward` tool call fires and the robot drives ~1 m |

---

## 6. Known quirks (so you don't panic mid-demo)

- **Drive direction is physically correct** (forward cmd → forward, odom == physical). The old 180° base-frame inversion was fixed at the source on 2026-06-02 via a telemetrix motor/encoder pin-swap (`lupin_bringup/scripts/apply-drive-pinswap.py`, `project_hardware_axis_inversion`); `polarityInvertHmi` now defaults false and the Xbox scales are positive. If a *different* robot ever drives backward, that's an uncorrected unit — fix its pins, don't re-enable the HMI flip.
- **HMI joystick needs a page reload on first load** sometimes. Reason='startup' e-stop + rosbridge status flap. One reload fixes it.
- **`/rosapi/get_time` errors** in the HMI/log every ~2 s — node-name collision between vendor rosbridge and laptop rosbridge. Cosmetic; latency pill won't populate.
- **`controllers (should show 5 active)`** in `post-boot-sync.sh` *always* fails with `rcl node's context is invalid`. CLI bug, not real failure.
- **`Message Filter dropping ... earlier than transform cache`** once at SLAM startup is normal. Repeated occurrences = clock drift.
- **HMI shows LIVE but no battery / no telemetry / can't drive — *yet* `ros2 topic list` in a fresh terminal shows the robot's topics.** The HMI's rosbridge was launched from a terminal that never sourced `ros-env.sh`, so it's on default-multicast and can't join the robot's discovery-server graph. The DDS env is per-*process*, so a CLI check in a different (correctly-sourced) terminal looks fine and hides it. T1's launch now prints a `rosbridge DDS mode:` banner at startup — if it says `MULTICAST`, that's the bug. Confirm on a running HMI with `tr '\0' '\n' </proc/$(pgrep -f rosbridge_websocket)/environ | grep ROS_DISCOVERY_SERVER` (empty = bug). Fix: Ctrl-C T1, re-source both `source` lines from §4, relaunch, reload the browser. (Diagnosed live 2026-05-29.)

---

## 7. Emergency reset (when in doubt, do this)

```bash
# Ctrl-C every laptop terminal.
pkill -9 -f 'ros2|rviz2|rosbridge|slam_toolbox|twin_node|tag_annotator|web_video_server' 2>/dev/null
ros2 daemon stop; pkill -9 -f ros2cli; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ssh lupin 'sudo systemctl restart mirte-ros lupin-onboard lupin-cameras'
sleep 30
~/.config/lupin/post-boot-sync.sh
# Then redo step 3 onwards.
```

If `mirte-ros.service` *restart* leaves controllers fragile (per
[`feedback_mirte_apt_then_powercycle`] / [`project_robot_clock_skew`]): power-cycle the robot
(off → 15 s → on) and restart from step 0.

---

## 8. Reference

- **Robot IP:** `192.168.42.1` (AP `Mirte-247264`)
- **Laptop IP on AP:** `192.168.42.66`
- **SSH:** `ssh lupin` (alias) or `ssh mirte@192.168.42.1`
- **HMI URL:** `https://localhost:8090` or `https://192.168.42.66:8090`
- **rosbridge:** `wss://<host>:8090/_ros` (same-origin proxy)
- **Robot's DDS discovery server:** `192.168.42.1:11811`
- **DDS env on laptop:** `~/.config/lupin/ros-env.sh` (source in every terminal)
- **Clock sync helper:** `~/.config/lupin/post-boot-sync.sh`
