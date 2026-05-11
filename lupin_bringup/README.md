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
| `lupin-cameras-throttle` | `topic_tools throttle` pipeline → `/lupin/camera/...` at config rates, so the laptop subscribes to throttled streams instead of full-rate vendor feeds. | `lupin_bringup/` |

The vendor `mirte-ros.service` still runs telemetrix, ros2_control, RPLidar,
cameras, and a vendor `rosbridge_websocket :9090` — but the Lupin HMI no
longer connects to that vendor rosbridge; it talks to the laptop's
co-located one. The vendor instance sits idle.

**On the laptop — user-mode systemd HMI auto-starts at login:**

| Service | What it does | Lives in |
| --- | --- | --- |
| `lupin-web` (user) | Vite preview (HTTPS :8090) + rosbridge_websocket :9090 + `web_video_server` :8091. All JSON encoding for the HMI lives here, off the Pi. | `lupin_web/` |

After laptop boot + login, open `https://<laptop-ip>:8090` and the
joystick / arm / gripper widgets work. The browser will warn about the
self-signed cert; accept it once per device.

For Nav2 + SLAM + RViz on top of the running robot, from the laptop:

```bash
ros2 launch lupin_bringup hardware.launch.py
```

That's the one entry point. Each subsystem is behind a boolean flag, so
the operator opts in or out without editing files:

| Flag                  | Default | What it brings up |
| --------------------- | ------- | ----------------- |
| `slam:=`              | `true`  | `slam_toolbox` (owns `/map`, map→odom TF) + `slam_reset_node` (the HMI Erase-map service `/lupin/nav/clear_map`). |
| `nav2:=`              | `true`  | Nav2 stack in slam mode. Sentinel waits for `/map` before activating. |
| `twin:=`              | `true`  | `lupin_twin` — aggregates `/floranova/observations` into `/twin/state` for the HMI Twin tab. |
| `mission:=`           | `false` | Mission pipeline bundle: `greenhouse_bridge` (oracle), `mission_orchestrator` lifecycle node, one-shot `/amcl_pose` seed. |
| `rviz:=`              | `true`  | RViz2 with the persistent `full_bringup_viz.rviz` config. |
| `joystick:=`          | `false` | Xbox controller teleop on the laptop (`joy_node` + `teleop_twist_joy` + `arm_teleop`). |
| `dependency_timeout_s:=` | `120.0` | Mission orchestrator wait before FAULT. |

Common invocations:

```bash
# Default operator mode — SLAM + Nav2 + twin + RViz.
ros2 launch lupin_bringup hardware.launch.py

# Headless smoke test.
ros2 launch lupin_bringup hardware.launch.py rviz:=false

# Full mission run.
ros2 launch lupin_bringup hardware.launch.py mission:=true

# Teleop only — laptop adds no autonomy, RViz still visualises /scan + /tf.
ros2 launch lupin_bringup hardware.launch.py slam:=false nav2:=false
```

Twist arbitration: HMI is priority 50, Xbox 100, Nav2 10 — so the
operator override always wins.

`hardware_full.launch.py` is now a deprecated alias for
`hardware.launch.py mission:=true`; it forwards with a warning banner and
will be removed in a future cleanup.

### Installing the robot-side services (one-time per Mirte image)

The services are not in the stock MIRTE image. Install once per robot:

```bash
# On the robot, from a Lupin checkout under ~/ros2_ws/src/lupin
sudo apt install ros-humble-twist-mux ros-humble-topic-tools

cd ~/ros2_ws && colcon build --packages-up-to lupin_bringup --symlink-install

source ~/ros2_ws/install/setup.bash
sudo bash $(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/install-onboard-systemd.sh
sudo bash $(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/install-cameras-throttle-systemd.sh
```

Each install script is idempotent — safe to re-run. `--uninstall` undoes it.

If `lupin-web.service` was previously installed on the robot (legacy
configuration before the HMI offload), uninstall it now:

```bash
sudo bash $(ros2 pkg prefix lupin_web)/share/lupin_web/scripts/install-systemd.sh --uninstall
```

### Installing the laptop-side HMI service (one-time per laptop)

```bash
# On the laptop, from anywhere on the repo
cd ~/ros2_ws/src/lupin/lupin_web/web && npm install && npm run build

cd ~/ros2_ws && colcon build --packages-select lupin_web --symlink-install

source ~/ros2_ws/install/setup.bash
~/ros2_ws/src/lupin/lupin_web/scripts/install-systemd-laptop.sh --linger
```

No sudo for the unit itself (it's a user-mode systemd unit installed to
`~/.config/systemd/user/`). The optional `--linger` runs one `sudo
loginctl enable-linger $USER` so the HMI starts at boot rather than at GUI
login. Inspect with `journalctl --user -u lupin-web -f`.

### Tuning camera rates

Edit `lupin_bringup/config/cameras.yaml` (toggle `enabled`, change `rate_hz`),
then on the robot:

```bash
cd ~/ros2_ws && colcon build --packages-select lupin_bringup
sudo systemctl restart lupin-cameras-throttle
```

Defaults: RGB + gripper at 1 Hz, depth disabled. Bump rates for arm-aiming
sessions where you actually need recent frames.

## Launches

| File | Purpose |
| --- | --- |
| `launch/onboard.launch.py` | Robot-side glue: `twist_mux` + `arm_preset_server` + `gripper_action_bridge`. Run via `lupin-onboard.service`. |
| `launch/cameras_throttle.launch.py` | Reads `config/cameras.yaml` and spawns one `topic_tools throttle` per enabled camera. Run via `lupin-cameras-throttle.service`. |
| `launch/hardware.launch.py` | Laptop-side: Nav2 + slam_toolbox + RViz against the real Mirte. Sentinel cascade waits for `/scan` then `/map` before each next stage. Slimmed: twist_mux moved to `lupin-onboard.service` so this never fights the robot's instance. |
| `launch/hardware_full.launch.py` | Wraps `hardware.launch.py` plus the lupin_mission orchestrator and lupin_twin. End-to-end mission run. |
| `launch/sim.launch.py` | Generic sim entry point. Wraps `mirte_gazebo`'s empty / navigation launches; `nav:=true` brings up Nav2 + RViz against the KRR small-house world. |
| `launch/sim_full.launch.py` | Full sim mission: greenhouse world + Nav2 + slam_toolbox + lupin_twin + lupin_mission + arm_sim_shim + RViz. The sim peer of `hardware_full.launch.py`. |
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
