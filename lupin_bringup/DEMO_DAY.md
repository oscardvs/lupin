# Demo-day bringup — Mirte-247264 (Lupin)

Step-by-step procedure for getting the full Lupin stack live in front of an
audience. Each step has *what to expect* and *if it fails*. Scan-able rather
than prose — the goal is "look up this file under pressure and follow it".

> The 7-terminal layout is intentional: every subsystem can be killed and
> restarted in its own terminal without taking the rest down. Recommended
> over `ros2 launch lupin_bringup hardware.launch.py` for demos where
> on-stage debugging may be needed.

---

## 0. Pre-flight (do this BEFORE the audience walks in)

- [ ] Laptop battery > 60 % AND on charger. Brightness up.
- [ ] Xbox controller: charged, **powered ON before robot boot** (so joy_node enumerates it at startup — saves a `systemctl restart` later).
- [ ] Robot powered on, on the floor in the demo space, lidar mast clear of obstacles.
- [ ] Wait ~90 s after robot power-on for the boot storm to settle (load avg 8–12 is normal — see [`project_mirte_boot_storm`]).
- [ ] Laptop WiFi connected to `Mirte-247264` AP. Verify with `ip -br addr | grep wlp` shows `192.168.42.66/24`.
- [ ] Internet on a separate interface only (USB tether to phone is fine; needed for any GenAI/Gemini features in the HMI). The route to the robot stays on the WiFi.

---

## 1. Verify robot health + sync clock

```bash
~/.config/lupin/post-boot-sync.sh
```

**Expect:** drift `0s` or `1s`, discovery server `LISTENING` with `users:(("fast-discovery-",…))`, all 3 services `active`.

**If discovery server `NOT-LISTENING`** (vendor `mirte_ros.sh` cold-boot race fired):

```bash
ssh lupin 'sudo systemctl stop lupin-onboard lupin-cameras mirte-ros && sleep 3 && sudo pkill -9 -f fast-discovery-server'
LAPTOP_TS=$(date -u +%s); ssh lupin "sudo date -s @${LAPTOP_TS} && sudo systemctl start mirte-ros"
sleep 25
ssh lupin 'sudo systemctl start lupin-onboard lupin-cameras'
sleep 10
~/.config/lupin/post-boot-sync.sh   # re-verify; discovery server should be up now
```

The "controllers (should show 5 active)" line at the end **always fails** with
`rcl node's context is invalid` — that's a `ros2 control` CLI bug, not a real
failure. Ignore it; the controllers are fine.

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

## 3. CLI sanity check

```bash
source ~/.config/lupin/ros-env.sh
ros2 daemon start
sleep 5
ros2 topic list | wc -l       # expect ~52 (only systemd services up so far)
ros2 topic hz /scan           # expect ~10 Hz, Ctrl-C after 3 s
ros2 topic hz /joy            # expect ~15 Hz if Xbox on, nothing if off
```

**If `topic list | wc -l == 2`**: DDS is wedged. Re-run step 2. If it still
says 2: check `ip -br addr` — if you're multi-homed (phone tether + WiFi)
and the default route is via the phone, FastDDS might be advertising on the
wrong interface. Bring down the tether (`nmcli connection down "Wired connection 1"`)
and try again.

---

## 4. Bit-by-bit launch (one terminal per subsystem)

Every terminal starts with:

```bash
source ~/.config/lupin/ros-env.sh
source ~/ros2_ws/install/setup.bash    # for RViz mesh resolution
```

### T1 — HMI (Vite preview + rosbridge :9090 + web_video_server :8091)

```bash
ros2 launch lupin_web lupin_web.launch.py mode:=preview tls:=true rosbridge:=true
```

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
clock drift or SHM contention. Ctrl-C only T5, redo step 2 (laptop hygiene)
and step 1 (clock sync), then retry T5.

### T6 — Digital twin

```bash
ros2 launch lupin_twin twin.launch.py
```

One line: `lupin_twin up: obs_topic=/floranova/observations, …`. Idle until
mission orchestrator publishes observations.

### T7 — Perception (AprilTag detector)

```bash
ros2 launch lupin_perception perception.launch.py use_sim_time:=false
```

**Expect:** `tag_annotator online — image="/camera/color/image_raw" …` then
within 3 s `Latched intrinsics from camera_info: fx=… fy=… cx=… cy=…`.

**If no "Latched intrinsics" within 10 s:** cameras aren't publishing yet.
Check `ros2 topic hz /camera/color/camera_info` — should be ~5 Hz. If silent,
`ssh lupin 'sudo systemctl restart lupin-cameras'`.

---

## 5. Demo test sequence — verify everything works before the audience arrives

| Test | How | Pass criterion |
|---|---|---|
| **Xbox drive** | Hold **LB**, push left stick forward | Robot moves **forward** physically (negative scales in `xbox_config.yaml` compensate for Mirte-247264 polarity) |
| **HMI joystick** | HMI Teleop view, click Reset on e-stop banner, wiggle virtual stick | Robot drives in operator-expected direction (HMI applies `polarityInvertHmi`) |
| **Lidar visible in HMI** | HMI Lidar view | Live scan paints at ~10 Hz |
| **Map visible in HMI** | HMI Map view | OccupancyGrid renders, rotates with robot |
| **Nav2 goal** | RViz "2D Goal Pose" tool, click+drag a goal 1–2 m ahead | Robot plans + drives to goal |
| **HMI map-click goal** | HMI Map view, click destination | Same as above; HMI auto-flips coords per `polarityInvertHmi` |
| **AprilTag overlay** | HMI Cameras view, point camera at a printed tag | Bounding box + tag ID overlay |
| **E-stop** | HMI e-stop button | Robot stops immediately, banner shows "user" reason |
| **Map erase** | HMI menu → "Erase map" | /map clears and slam_toolbox respawns blank |

---

## 6. Known quirks (so you don't panic mid-demo)

- **RViz odometry direction is "wrong"** vs physical motion — *expected*. Mirte-247264 has the mecanum drive wired with motor + encoder leads reversed in pairs, so the vendor frame is rotated 180° relative to physical chassis. Internally consistent, just rotated. See `lupin_web/web/src/lib/polarity.ts:3-9`.
- **HMI joystick needs a page reload on first load** sometimes. Reason='startup' e-stop + rosbridge status flap. One reload fixes it.
- **`/rosapi/get_time` errors** in the HMI/log every ~2 s — node-name collision between vendor rosbridge and laptop rosbridge. Cosmetic; latency pill won't populate.
- **`controllers (should show 5 active)`** in `post-boot-sync.sh` *always* fails with `rcl node's context is invalid`. CLI bug, not real failure.
- **`Message Filter dropping ... earlier than transform cache`** once at SLAM startup is normal. Repeated occurrences = clock drift.

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
