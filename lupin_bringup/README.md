# lupin_bringup

Top-level launch files, configs, and systemd glue for Team Lupin. Composes
the vendor MIRTE Master + Gazebo stack into a single launch per scenario,
so feature MRs in this repo have one stable entry point instead of chasing
vendor file names.

## Hardware bring-up — minimal commands

Lupin's runtime is split across **the robot** (ROS-native, no JSON broker)
and **the operator's laptop** (HMI + Nav2 + RViz). The robot only does what
must live close to the hardware; everything else runs on the laptop where
there's CPU headroom. See `project_offload_strategy` for the rationale.

**On the robot — two systemd services, no HMI:**

| Service | What it does | Lives in |
| --- | --- | --- |
| `lupin-onboard` | `twist_mux` + `arm_preset_server` + `gripper_action_bridge` — operator surfaces wired to controllers. Must live on the robot so manual control still works while the laptop is rebooting. | `lupin_bringup/` |
| `lupin-cameras` | Kills the vendor camera nodes (`usb_cam` + Orbbec component container) after `mirte-ros` is up, then relaunches them with our params: 5 fps RGB on `/camera/color/image_raw`, 5 fps gripper on `/gripper_camera/image_raw`, depth + pointcloud off by default. Topic names are identical to the vendor's so downstream consumers see no interface change. | `lupin_bringup/` |

The vendor `mirte-ros.service` still runs telemetrix, ros2_control, RPLidar,
cameras, and a vendor `rosbridge_websocket :9090` — but the Lupin HMI no
longer connects to that vendor rosbridge; it talks to the laptop's
co-located one. The vendor instance sits idle.

**On the laptop — one command brings up everything:**

```bash
ros2 launch lupin_bringup hardware.launch.py
```

That's the single entry point. It brings up the HMI, SLAM, Nav2, the
digital twin, and RViz in one shot. Open `https://<laptop-ip>:8090` once
Vite logs "ready in NNN ms" — the HMI's rosbridge pill goes green within
a couple of seconds. The browser will warn about the self-signed cert;
accept it once per device.

Each subsystem is behind a boolean flag:

| Flag                  | Default | What it brings up |
| --------------------- | ------- | ----------------- |
| `web:=`               | `true`  | `lupin_web` — Vite preview (HTTPS :8090) + `rosbridge_websocket` :9090 + `web_video_server` :8091. All JSON encoding for the HMI runs here, off the Pi. |
| `slam:=`              | `true`  | `slam_toolbox` (owns `/map`, map→odom TF) + `slam_reset_node` (the HMI Erase-map service `/lupin/nav/clear_map`). |
| `nav2:=`              | `true`  | Nav2 stack in slam mode. Sentinel waits for `/map` + a hot `odom→base_link` tf before activating. |
| `twin:=`              | `true`  | `lupin_twin` — aggregates `/floranova/observations` into `/twin/state` for the HMI Twin tab. |
| `mission:=`           | `false` | Mission pipeline bundle: `greenhouse_bridge` (oracle), `mission_orchestrator` lifecycle node, one-shot `/amcl_pose` seed. |
| `rviz:=`              | `true`  | RViz2 with the persistent `full_bringup_viz.rviz` config. |
| `joystick:=`          | `true`  | Xbox controller teleop on the laptop (`joy_node` + `teleop_twist_joy` + `arm_teleop`). Pad plugs into the laptop (USB-C is the no-drama path); the robot-side joy path was removed because BLE pairing on the vendor image is fragile. Pass `joystick:=false` to silence joy_node when no pad is connected. |
| `dependency_timeout_s:=` | `120.0` | Mission orchestrator wait before FAULT. |

Common invocations:

```bash
# Default operator mode — HMI + SLAM + Nav2 + twin + RViz.
ros2 launch lupin_bringup hardware.launch.py

# Headless smoke test (no HMI, no RViz).
ros2 launch lupin_bringup hardware.launch.py web:=false rviz:=false

# Full mission run.
ros2 launch lupin_bringup hardware.launch.py mission:=true

# Just the HMI (no autonomy stack) — useful while debugging the UI.
ros2 launch lupin_bringup hardware.launch.py slam:=false nav2:=false
```

The vendor `mirte-ros.service` still runs telemetrix, ros2_control,
RPLidar, cameras, and a vendor `rosbridge_websocket :9090` — but the
Lupin HMI no longer connects to that vendor rosbridge; it talks to the
laptop's co-located one. The vendor instance sits idle.

Twist arbitration: HMI is priority 50, Xbox 100, Nav2 10 — so the
operator override always wins.

### Installing the robot-side services (one-time per Mirte image)

The services are not in the stock MIRTE image. Install once per robot:

```bash
# On the robot, from a Lupin checkout under ~/ros2_ws/src/lupin
sudo apt install ros-humble-twist-mux v4l-utils

cd ~/ros2_ws && colcon build --packages-up-to lupin_bringup --symlink-install

source ~/ros2_ws/install/setup.bash
sudo bash $(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/install-onboard-systemd.sh
sudo bash $(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/install-cameras-systemd.sh
```

The cameras install script auto-removes the legacy `lupin-cameras-throttle`
service if it's still installed, so the upgrade is a single command.

Each install script is idempotent — safe to re-run. `--uninstall` undoes it.

If `lupin-web.service` was previously installed on the robot (legacy
configuration before the HMI offload), uninstall it now:

```bash
sudo bash $(ros2 pkg prefix lupin_web)/share/lupin_web/scripts/install-systemd.sh --uninstall
```

### Laptop ↔ robot DDS over WiFi (one-time per laptop)

ROS 2's default FastDDS uses UDP multicast for participant discovery, which
WiFi access points (iPhone hotspot, the robot's own `mirte-ap.service`,
many university APs) silently drop. Symptom: `ssh` and `ping` to the robot
work fine, but `ros2 topic list` from the laptop returns only
`/parameter_events` + `/rosout`. The HMI loads but every panel is blank.

The vendor MIRTE image ships a FastDDS Discovery Server for exactly this
case. The fix is two flags — one on each side.

**On the robot** — enable the vendor's discovery server, then restart the
service stack:

```bash
# In ~/.mirte_settings.sh, ensure this line is set to true:
echo 'export MIRTE_FASTDDS=true' >> /home/mirte/.mirte_settings.sh
sudo systemctl restart mirte-ros.service lupin-onboard.service lupin-cameras.service

# Verify the server is listening (UDP, not TCP):
sudo ss -lnup | grep 11811        # expect "fast-discovery-server" listening on 0.0.0.0:11811
```

**On the laptop** — run the setup script once. It generates the FastDDS
XML profile, writes the env-loader, and wires `~/.bashrc` to source it.
The default robot IP is `192.168.42.1` (the robot's own AP); override if
you're on a shared LAN where the robot has a different address.

```bash
# from the source tree:
./lupin_bringup/scripts/setup-laptop-dds-env.sh                  # uses 192.168.42.1
./lupin_bringup/scripts/setup-laptop-dds-env.sh 10.0.0.42        # custom IP
./lupin_bringup/scripts/setup-laptop-dds-env.sh --uninstall      # back out

# from an installed workspace:
$(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/setup-laptop-dds-env.sh
```

Open a fresh terminal (or `source ~/.config/lupin/ros-env.sh`), then:

```bash
ros2 daemon start && sleep 5
ros2 topic list | wc -l                          # expect 50+, not 2
ros2 topic echo /io/power/power_watcher --once   # should print BatteryState
```

If you swap networks (lab WiFi → robot AP → travel router), re-run the
setup script with the new IP. The robot side doesn't need to change —
its participants always point at `127.0.0.1:11811`.

Three gotchas that wasted a full day on 2026-05-18, captured here so we
never relearn them:

- **Both XML and `ROS_DISCOVERY_SERVER` env var are required** on Humble's
  FastDDS 2.6. Either alone is flaky — the env-var auto-config path is
  inconsistent, and the XML alone doesn't trigger discovery-server
  connection. The setup script wires both in the daemon's environment.
- **Zombie ROS processes from a `Ctrl-C`'d launch** keep binding DDS ports
  (7410–7447) and hijack discovery responses — the discovery server hands
  topic info to the zombie and your fresh CLI/HMI participant gets
  nothing. Before a fresh launch, sweep them:
  ```bash
  pgrep -af 'robot_state_pub|rosbridge_websocket|rviz2|twin_node|tag_annotator|wait_for' \
    | grep -v grep
  # kill -9 any PIDs that show up
  ```
- **`ros2 topic list` auto-starts the daemon but doesn't wait for
  discovery to complete.** Right after env changes, do
  `ros2 daemon start && sleep 5` before the first `topic list` call,
  otherwise the first invocation returns `/parameter_events` + `/rosout`
  while the daemon is still discovering. Subsequent calls are fine.

**Troubleshooting — is this my problem?**

| Symptom | Likely fix |
| --- | --- |
| `ros2 topic list` returns only `/parameter_events` and `/rosout` from the laptop, but `ssh` and `ping` to the robot work | DDS over WiFi — run the setup script. |
| HMI loads and shows "LIVE" but every panel is blank (no battery, no IMU, no lidar) | Same — laptop's rosbridge can't subscribe to anything via DDS. |
| Battery was working, then stopped after a launch Ctrl-C | Zombie ROS process from the killed launch. Sweep PIDs (see above) and re-launch. |
| HMI blank, but `ros2 topic list` in a *fresh* terminal DOES show robot topics | The DDS env is per-process: the HMI's rosbridge was launched from a terminal that never sourced `ros-env.sh`, so only *it* is on multicast. Check `tr '\0' '\n' </proc/$(pgrep -f rosbridge_websocket)/environ \| grep ROS_DISCOVERY_SERVER` — empty confirms it. Relaunch the HMI from a fresh shell. The `lupin_web` launch prints a `rosbridge DDS mode:` banner to catch this. |
| Topic count was 50+ then dropped to 2 after switching WiFi | Network changed, but daemon still has the old config. Re-run the setup script with the new robot IP, open a fresh shell. |
| Setup script ran fine, fresh shell, still 2 topics | Robot's discovery server isn't actually listening. SSH in and `sudo ss -lnup \| grep 11811` — if empty, `MIRTE_FASTDDS=true` wasn't picked up (check `~/.mirte_settings.sh`, then `sudo systemctl restart mirte-ros.service`). |

**Note for zsh users:** the setup script appends a source-line to
`~/.bashrc` only. If your interactive shell is zsh, add the same line to
`~/.zshrc` manually:

```bash
echo '[ -f ~/.config/lupin/ros-env.sh ] && source ~/.config/lupin/ros-env.sh' >> ~/.zshrc
```

### Laptop-side setup (one-time per laptop)

The HMI is part of `hardware.launch.py` — no extra service to install,
just make sure the Vite build artefact exists and the workspace is built:

```bash
# On the laptop, from anywhere on the repo
cd ~/ros2_ws/src/lupin/lupin_web/web && npm install && npm run build

cd ~/ros2_ws && colcon build --symlink-install
```

After that, every `ros2 launch lupin_bringup hardware.launch.py` brings
up the HMI alongside the autonomy stack. If you previously installed the
old `lupin-web` user-mode systemd unit, drop it so it doesn't fight the
launch for port :8090:

```bash
systemctl --user disable --now lupin-web 2>/dev/null
rm -f ~/.config/systemd/user/lupin-web.service \
      ~/.config/systemd/user/default.target.wants/lupin-web.service
systemctl --user daemon-reload
```

### Post-boot clock-sync helper (one-time per laptop)

The Orange Pi 3B has no RTC and in AP mode has no NTP source, so the
robot boots with whatever clock it had at shutdown. `scripts/post-boot-sync.sh`
syncs the clock laptop→robot and sanity-checks the discovery server +
service stack in one shot. `DEMO_DAY.md` and the `project_robot_clock_skew`
memory both reference it as `~/.config/lupin/post-boot-sync.sh`, so set
that path up once:

```bash
mkdir -p ~/.config/lupin
ln -s "$(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/post-boot-sync.sh" \
      ~/.config/lupin/post-boot-sync.sh
```

Symlinked (not copied) so a future `colcon build --packages-select lupin_bringup`
updates the script in-place. Override the target robot at call-time:

```bash
~/.config/lupin/post-boot-sync.sh                              # mirte@192.168.42.1 (default)
ROBOT=mirte@10.0.0.42 ~/.config/lupin/post-boot-sync.sh        # any other robot
```

### Tuning camera rates

Edit `lupin_bringup/config/cameras.yaml` (toggle `enabled`, change `fps`),
then on the robot:

```bash
cd ~/ros2_ws && colcon build --packages-select lupin_bringup
sudo systemctl restart lupin-cameras
```

Defaults: RGB at 5 fps, gripper at 5 fps, depth + pointcloud off. Bump
rates for arm-aiming sessions where you actually need recent frames; turn
on depth/pointcloud only when perception research needs them — they're the
single biggest producer-side CPU drain on the A55 Pi.

## Launches

| File | Purpose |
| --- | --- |
| `launch/onboard.launch.py` | Robot-side glue: `twist_mux` + `arm_preset_server` + `gripper_action_bridge`. Run via `lupin-onboard.service`. |
| `launch/cameras.launch.py` | Reads `config/cameras.yaml` and (re)launches the Orbbec + USB gripper cameras at the configured FPS, on the vendor topic names. Run via `lupin-cameras.service`, which kills the vendor cameras first so the v4l/USB devices are free. |
| `launch/hardware.launch.py` | Laptop-side single entry point: HMI + Nav2 + slam_toolbox + twin + RViz against the real Mirte. Boolean flags per subsystem (`web`, `slam`, `nav2`, `twin`, `mission`, `rviz`, `joystick`). The sentinel cascade waits for `/scan`, `/map`, then a hot `odom→base_link` tf before each next stage. End-to-end mission run is `mission:=true`. |
| `launch/sim.launch.py` | Generic sim entry point. Wraps `mirte_gazebo`'s empty / navigation launches; `nav:=true` brings up Nav2 + RViz against the KRR small-house world. |
| `launch/sim_full.launch.py` | Full sim mission: greenhouse world + Nav2 + slam_toolbox + lupin_twin + lupin_mission + arm_sim_shim + RViz. The sim peer of `hardware.launch.py mission:=true`. |
| `launch/greenhouse_sim.launch.py` | Greenhouse-world entry point. Loads the SDF from `scripts/generate_greenhouse_world.py`, spawns the MIRTE Master, starts ros2_control + twist_mux. Sets Gazebo env vars in-launch so `/usr/share/gazebo/setup.sh` isn't needed. |

Each launch has a thorough docstring header — open the file to see the args
and rationale. See `project_lupin_services_architecture` in the team's
shared memory for how the pieces compose at runtime.

## Worlds and scripts

- `worlds/greenhouse.world` — committed, deterministic output of
  `scripts/generate_greenhouse_world.py`. Regenerate after a
  `mdp-greenhouse` upgrade:
  ```bash
  python3 scripts/generate_greenhouse_world.py
  colcon build --symlink-install --packages-select lupin_bringup
  ```
  By default the script reads `tag_locations.json` from the installed
  `greenhouse_sim` Python package via `importlib.resources`; pass
  `--input` to point it elsewhere.

- AprilTag visuals in the generated world are bright magenta
  `_PLACEHOLDER` stand-ins until real `tag36h11` textures land. See the
  TODO inside `_render_tag` and the heads-up note in the top-level
  README's "Greenhouse Gazebo world" section.

## Where to start

For the multi-terminal sim workflow (with Nav2, slam_toolbox, the sensor
bridge, and known sim limitations), see the **Running** section of the
[top-level README](../README.md#running).

## Maintainer

Oscar Devos — `o.a.e.devos@student.tudelft.nl`
