# lupin_bringup

Top-level launch files, configs, and systemd glue for Team Lupin. Composes
the vendor MIRTE Master + Gazebo stack into a single launch per scenario,
so feature MRs in this repo have one stable entry point instead of chasing
vendor file names.

## Hardware bring-up — minimal commands

The robot is **operator-controllable from power-on**. Three systemd services
on the Pi cover everything that has to be alive for HMI teleop, arm presets,
and the gripper:

| Service | What it does | Lives in |
| --- | --- | --- |
| `lupin-web` | HMI on `https://<robot-ip>:8090` (HTTPS + same-origin `/_ros` and `/_video` proxies, web_video_server :8091) | `lupin_web/` |
| `lupin-onboard` | `twist_mux` + `arm_preset_server` + `gripper_action_bridge` — operator surfaces wired to controllers | `lupin_bringup/` |
| `lupin-cameras-throttle` | `topic_tools throttle` pipeline → `/lupin/camera/...` at config rates | `lupin_bringup/` |

After power-on, open `https://<robot-ip>:8090` and the joystick / arm /
gripper widgets work. **No laptop launch required.** The browser will warn
about the self-signed cert; accept it once per device.

For Nav2 + SLAM + RViz on top of the running robot, from the laptop:

```bash
ros2 launch lupin_bringup hardware.launch.py
```

That's it. One command, no flags. It assumes the robot is up and reachable
on the same LAN. Useful args: `rviz:=false` (headless), `joystick:=true` (Xbox
controller plugged into the laptop). Twist arbitration: HMI is priority 50,
Xbox 100, Nav2 10 — so the operator override always wins.

### Installing the robot-side services (one-time per Mirte image)

The services are not in the stock MIRTE image. Install once per robot:

```bash
# On the robot, from a Lupin checkout under ~/ros2_ws/src/lupin
sudo apt install ros-humble-twist-mux ros-humble-topic-tools

cd ~/ros2_ws && colcon build --packages-up-to lupin_bringup --symlink-install

source ~/ros2_ws/install/setup.bash
sudo bash $(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/install-onboard-systemd.sh
sudo bash $(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/install-cameras-throttle-systemd.sh
sudo bash $(ros2 pkg prefix lupin_web)/share/lupin_web/scripts/install-systemd.sh
```

Each install script is idempotent — safe to re-run. `--uninstall` undoes it.

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
