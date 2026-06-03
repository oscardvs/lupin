# Demo-day bringup — wired ethernet (Lupin)

Step-by-step for running the full Lupin hardware stack with the laptop **wired
to the robot over an ethernet cable** instead of the robot's WiFi AP. Scan-able
rather than prose — look it up under pressure and follow it.

> **Why wired.** A direct cable is a clean Layer-2 link: no AP multicast drops,
> no eduroam roam-away, no captive portal, far lower latency and jitter than the
> robot AP. It's the most reliable transport for live hardware testing — fewer
> moving parts to fail mid-demo. The sibling guides: `DEMO_DAY.md` (robot WiFi
> AP, "cableless") and `DEMO_DAY_SIM.md` (Gazebo).

> **What actually differs from cableless (`DEMO_DAY.md`).** Only the **physical
> link and the IP scheme**. The robot-side services (`mirte-ros`,
> `lupin-onboard`, `lupin-cameras`, `lupin-auto-home`) are identical and run the
> same regardless of how the laptop connects. The robot's FastDDS discovery
> server listens on `0.0.0.0:11811` (all interfaces — verified in
> `README.md`), so over the cable it's reachable at the robot's **wired** IP.
> Everything downstream of the IP swap — launches, clock sync, the HMI — is the
> same. **If a step here isn't networking, `DEMO_DAY.md` is the deeper
> reference.**

**Addressing used throughout this doc** (a private /24 that avoids the robot AP's
`192.168.42.0/24` and typical home/eduroam ranges):

| Host | Interface | IP |
|---|---|---|
| Robot | `eth0` (confirm the name — see §A) | `10.42.0.1/24` |
| Laptop | USB-C→ethernet adapter (`enx…`) | `10.42.0.2/24` |
| Discovery server | robot, UDP | `10.42.0.1:11811` |

---

## A. One-time ROBOT setup — static IP on the wired port

Do this **once**, while the laptop can still reach the robot the old way (over
the AP `Mirte-247264`, or any existing network). It gives the robot's ethernet
port a fixed address so every wired session is deterministic.

**1. SSH in over the AP** (the way you do today) and find the wired interface:

```bash
ssh lupin                      # 192.168.42.1 over the robot AP
ip -br link                    # find the RJ45 port — usually 'eth0' (Orange Pi 3B);
                               # could be 'end0' / 'enP3p49s0'. NOT wlan0 (that's the AP).
```

**2. Give it a persistent static IP.** The MIRTE image uses NetworkManager
(wifi-connect runs on it), so `nmcli` is the clean path — replace `eth0` with
the name from step 1:

```bash
sudo nmcli connection add type ethernet ifname eth0 con-name lupin-wired \
  ipv4.method manual ipv4.addresses 10.42.0.1/24 \
  ipv4.never-default yes ipv6.method ignore connection.autoconnect yes
sudo nmcli connection up lupin-wired
ip -br addr show eth0           # expect: UP  10.42.0.1/24
```

- `ipv4.never-default yes` — the cable must NOT become the robot's default route
  (it has no upstream; leave the robot's own routing alone).
- `connection.autoconnect yes` — comes up by itself whenever a cable is plugged.

> **If `eth0` is managed by netplan / systemd-networkd instead** (`nmcli device
> status` shows it `unmanaged`): drop a netplan file instead —
> ```yaml
> # /etc/netplan/99-lupin-wired.yaml
> network:
>   version: 2
>   ethernets:
>     eth0: { addresses: [10.42.0.1/24], dhcp4: no }
> ```
> then `sudo netplan apply`.

**3. Disable the robot AP for wired sessions — REQUIRED, not optional.**
Counter-intuitive but **proven live 2026-06-03**: if `wlan0`/the AP stays up, the
robot advertises BOTH its wired (`10.42.0.1`) and AP (`192.168.42.1`) DDS
locators, and the **laptop→robot data path silently dies** — discovery still
works and robot→laptop telemetry still flows, so the HMI looks `LIVE`, but you
**cannot drive, the arm services time out, and `ros2 control list_controllers`
hangs**. Disable it and reboot so every participant re-announces `eth0`-only:

```bash
ssh lupin-wired 'sudo systemctl disable --now mirte-ap mirte-wifi-watchdog'
ssh lupin-wired sudo reboot
```

After reboot the robot is single-homed on `eth0` (the `lupin-wired` NM profile
autoconnects — verified persistent). You lose the AP as an SSH fallback for the
session, which is fine on a cable. **Better fix that keeps the AP as a fallback,
but NOT yet implemented:** pin the robot's FastDDS to `eth0` via an
`interfaceWhiteList` so the AP can stay up without poisoning DDS — see §6.

> **Going back to the AP later?** This whole §A.3 is reversible — see
> `DEMO_DAY_WIRED_REVERT.md` (re-enable `mirte-ap` + reboot, pull the cable,
> repoint the laptop DDS at `192.168.42.1`).

That's the entire robot-side delta. `MIRTE_FASTDDS=true` is already set on
Mirte-247264. The discovery server is *supposed* to come up at boot, but the
vendor's start is a fire-and-forget one-shot that can lose a cold-boot race and
silently leave **nothing on `:11811`** while `mirte-ros` still reports `active`.
§1 verifies it; §1's "nothing on 11811" recovery fixes it without a stack restart.

---

## B. One-time LAPTOP setup

Everything in §1 onward assumes the laptop is built and wired. The items below
are **not** in a fresh `git clone` — do them once per laptop. Most are identical
to `DEMO_DAY.md §First-time setup`; **only B3 (wired IP) and B4 (DDS IP) differ.**

**B1. Clone + build the workspace** (paths assume `~/ros2_ws/src/lupin`):

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b hardware git@gitlab.tudelft.nl:cor/ro47007/2026/group_14/lupin.git
cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y
cd ~/ros2_ws/src/lupin/lupin_web/web && npm install && npm run build   # HMI Vite artefact
cd ~/ros2_ws && colcon build --symlink-install
```

**B2. SSH key + a wired host alias.** Generate a per-laptop key (do **not** use
the image-baked `~/.ssh/id_rsa` — it's shared across all MIRTEs) and add a
`lupin-wired` alias so `ssh lupin-wired` and `post-boot-sync.sh` reach the robot
over the cable:

```bash
[ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
# copy the key over the AP first (one time), so wired ssh is password-less:
ssh-copy-id mirte@192.168.42.1
cat >> ~/.ssh/config <<'EOF'

Host lupin-wired
    HostName 10.42.0.1
    User mirte
    IdentityFile ~/.ssh/id_ed25519
EOF
```

**B3. Wire the laptop's USB-C ethernet adapter to a static IP.** Plug the
adapter in, find its name, and give it `10.42.0.2/24` with **no default route**
(so internet keeps flowing over WiFi):

```bash
ip -br link | grep -E 'enx|eth'        # the adapter shows up as enx<MAC>
sudo nmcli connection add type ethernet ifname enxXXXXXXXXXXXX con-name lupin-wired \
  ipv4.method manual ipv4.addresses 10.42.0.2/24 \
  ipv4.never-default yes ipv6.method ignore connection.autoconnect yes
sudo nmcli connection up lupin-wired
```

If you only ever use one adapter, this persists. Swap adapters → re-run with the
new `enx…` name (the MAC, hence the name, is per-adapter).

> **Substitute the real `enx…` name — don't paste `enxXXXXXXXXXXXX` literally.**
> nmcli will happily create a profile bound to the placeholder, which then fails
> to activate with `No suitable device found … profile is not compatible with
> device (mismatching interface name)`. With **two** adapters plugged (e.g. one to
> the robot, one to an iPhone tether on `172.20.10.x`), pick the robot-facing one
> by link speed — it's the gigabit one:
> `for d in /sys/class/net/enx*; do echo "$(basename $d) $(cat $d/speed 2>/dev/null)"; done`.
> Repair a mis-bound profile in place (no need to delete it):
> `sudo nmcli con mod lupin-wired connection.interface-name enxREAL && sudo nmcli con up lupin-wired enxREAL`.

**B4. DDS env pointed at the WIRED IP.** The DDS env that wires the laptop to the
robot's discovery server lives in `~/.config/lupin/ros-env.sh` (not in the repo —
it bakes in the robot IP). Generate it for the **wired** address:

```bash
cd ~/ros2_ws/src/lupin/lupin_bringup
./scripts/setup-laptop-dds-env.sh 10.42.0.1        # ← the robot's WIRED IP
```

It writes `~/.config/lupin/fastdds_super_client.xml` + `ros-env.sh`
(`ROS_DISCOVERY_SERVER=10.42.0.1:11811`) and appends a source-line to
`~/.bashrc`. zsh users: add the same line to `~/.zshrc`. To switch back to the AP
later: re-run with `192.168.42.1`. Back out entirely: `--uninstall`.

> **This one IP drives everything.** `hardware.launch.py` now auto-derives the
> HMI camera-proxy target from `ROS_DISCOVERY_SERVER`, so setting the wired IP
> here is the *only* place you choose it — the video tab follows automatically.

**B5. `post-boot-sync.sh` symlink** (§1 calls it):

```bash
mkdir -p ~/.config/lupin
source ~/ros2_ws/install/setup.bash
ln -s "$(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/post-boot-sync.sh" \
      ~/.config/lupin/post-boot-sync.sh
```

**B6. Smoke test** — open a fresh terminal (so `.bashrc` picks up the wired DDS
env) with the cable plugged into a booted robot:

```bash
ping -c2 10.42.0.1                                  # cable link is up
source ~/ros2_ws/install/setup.bash
ros2 daemon start && sleep 5
ros2 topic list | wc -l                             # expect 50+, not 2
ROBOT=mirte@10.42.0.1 ~/.config/lupin/post-boot-sync.sh
```

If `topic list` returns 2: `echo $ROS_DISCOVERY_SERVER` — must be
`10.42.0.1:11811`. If empty, B4 didn't take (fresh shell?). If `ping` fails, the
cable/adapter or §A static IP is the problem, not DDS.

> **Xbox controller**: plug it into the **laptop** via USB-C (the no-drama path;
> the robot-side joy path was removed). BLE only if you must — run
> `scripts/install-bluetooth-xbox-fix.sh` once first.

---

## 0. Pre-flight (do this BEFORE the audience walks in)

- [ ] Laptop battery > 60 % AND on charger. Brightness up.
- [ ] **Ethernet cable seated** both ends; USB-C adapter's link LED on.
- [ ] Xbox controller charged, **plugged into the laptop** via USB-C.
- [ ] Robot powered on, on the floor, lidar mast clear.
- [ ] Wait ~90 s after power-on for the boot storm to settle (load avg 8–12 is
      normal; CPU mostly idle — not a fault).
- [ ] `ip -br addr` on the laptop shows the adapter at `10.42.0.2/24`, and WiFi
      (internet) on a **separate** interface. Internet route must NOT be the cable.
- [ ] `ping -c2 10.42.0.1` succeeds.

---

## 1. Verify robot health + sync clock

```bash
ROBOT=mirte@10.42.0.1 ~/.config/lupin/post-boot-sync.sh
```

**Expect:** drift `0s`/`1s`, discovery server `LISTENING` on `0.0.0.0:11811`
with `users:(("fast-discovery-",…))`, all services `active`, **all 5
controllers `active`**:

| Controller | Role |
|---|---|
| `joint_state_broadcaster` | publishes `/joint_states` |
| `mirte_master_arm_controller` | JTC for the 4-DOF arm |
| `mirte_master_gripper_controller` | gripper action server |
| `pid_wheels_controller` | per-wheel velocity PID |
| `mirte_base_controller` | mecanum_drive_controller (Twist → 4 wheels) |

**The `0.0.0.0:11811` bind is what makes wired work** — it means the server
answers on every interface, including `eth0`. If the `ss` line instead shows a
*specific* address (e.g. `192.168.42.1:11811`), the vendor pinned the server to
the AP and it won't answer on the cable: `ros2 topic list` will return 2 even
with a good `ping`. For this session, fall back to `DEMO_DAY.md` (AP); the
durable fix is on the robot side (the discovery server should listen on the
wildcard) — flag it to the team.

**If `ss` shows *nothing* on `:11811`** (empty, not a wrong address), the vendor's
boot-time start lost the cold-boot race and never bound — its log shows
`Discovery Server wasn't able to allocate the specified listening port` +
`fast-discovery-server tool not found!` (check:
`ssh lupin-wired 'sudo journalctl -u mirte-ros -b --no-pager | grep -iE "discovery|11811|allocate"'`).
`mirte-ros` still says `active`; it lied. You do **not** need to restart the
stack — the robot's nodes are clients retrying `127.0.0.1:11811`, so launch the
server standalone and they attach within seconds (verified 2026-06-03: 69 topics
on the laptop, no restart):

```bash
ssh lupin-wired 'env -u FASTRTPS_DEFAULT_PROFILES_FILE setsid bash -c \
  "exec fast-discovery-server -i 0 -l 0.0.0.0 -p 11811" >/tmp/lupin-disc.log 2>&1 </dev/null &'
ssh lupin-wired 'sudo ss -lnup | grep 11811'   # expect: UNCONN 0.0.0.0:11811 users:(("fast-discovery-",…))
```

Two traps: `env -u FASTRTPS_DEFAULT_PROFILES_FILE` is **required** — with the
vendor super_client profile in env the server inherits a pinned locator and fails
"couldn't allocate port" even with the port free; and **never** clean up strays
with `pkill -f fast-discovery-server` — the `-f` pattern matches your own ssh
shell's argv and SIGTERMs the session (`exit 255`). Use `pkill -x
fast-discovery-server` (process-name match) or kill by PID. This manual server
**dies on reboot/power-cycle** — re-run the line after any robot restart. Durable
fix: a `lupin-discovery-server.service` (designed, not yet installed).

The clock sync matters: the Orange Pi has no RTC and over a direct cable there's
no NTP, so it boots with whatever clock it had at shutdown. **`post-boot-sync.sh`
now sets the clock with `mirte-ros` STOPPED** (only when drift > 3 s), wipes stale
SHM, then restarts the stack — because `date -s` on a *live* ros2_control stack
wedges the controllers (see the warning below). It then re-triggers
`lupin-auto-home` (heals any stuck controllers + parks the arm at `home`).

**If `lupin-auto-home` shows `failed`/`activating` > 90 s**, or controllers are
stuck `unconfigured`/`inactive`, or the wheels are dead (confirm with the drive
smoke-test in §5):

```bash
# Reboot-free recovery: stop the robot stack, wipe stale FastDDS SHM, restart
# with the clock already stable. A plain `systemctl restart` does NOT clear it.
ssh lupin-wired 'sudo systemctl stop lupin-onboard lupin-cameras mirte-ros && sleep 3 \
  && sudo rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* \
  && sudo systemctl start mirte-ros'
sleep 25
ssh lupin-wired 'sudo systemctl start lupin-onboard lupin-cameras'
ROBOT=mirte@10.42.0.1 ~/.config/lupin/post-boot-sync.sh   # re-verify + re-trigger auto-home
```

(The `controllers (should show 5 active)` line sometimes ends with `rcl node's
context is invalid` — that's a `ros2 control` CLI bug, not a real failure; trust
`ros2 control list_controllers`.)

> **Never `date -s` on a live stack.** It wedges ros2_control by leaving stale
> FastDDS SHM (`/dev/shm/fastrtps_*`) that a plain `systemctl restart` can't clear
> — symptom: controllers go `configured` but the activation spawner dies with
> `exit code -11`, `controller_state` goes silent, wheels dead (but `ros2 node
> info` still shows the publisher, so it looks alive). **The cure is the reboot-
> free SHM wipe above, NOT a power-cycle.** A power-cycle is *worse* here: the dead
> RTC means a cold boot comes up on the wrong clock, something `date -s`-corrects
> it after `mirte-ros` is already live, and it re-wedges. (Burned hours on
> 2026-06-03; root cause confirmed — see `project_robot_clock_skew`.) Same with
> `systemctl restart` right after a robot `apt upgrade` — wipe SHM + restart first.

---

## 2. Laptop hygiene

Stale FastDDS shared-memory segments from a previous session silently wedge every
new participant (`ros2 topic list` returns 2, RViz blank). Always wipe before
launching:

```bash
pkill -9 -f 'ros2|rviz2|rosbridge|slam_toolbox|twin_node|tag_annotator|web_video_server' 2>/dev/null
ros2 daemon stop 2>/dev/null
sleep 1
pkill -9 -f 'ros2cli' 2>/dev/null
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
ls /dev/shm | grep -i fast || echo "laptop SHM clean ✓"
```

---

## 3. First laptop terminal — get topics streaming

Points the laptop's DDS at the robot's discovery server over the cable and
verifies topics flow. Every subsequent terminal only needs the two `source` lines.

```bash
source ~/.config/lupin/ros-env.sh            # ROS_DISCOVERY_SERVER=10.42.0.1:11811 + super-client XML
source ~/ros2_ws/install/setup.bash          # lupin_* packages + RViz meshes
ros2 daemon start                            # one-time per session
sleep 5                                       # let discovery fill
ros2 topic list | wc -l                      # expect ~52 (systemd services only)
ros2 topic hz /scan                          # ~10 Hz; Ctrl-C after 3 s
ros2 topic hz /joy                           # ~15 Hz if the Xbox pad is on
```

**If `topic list | wc -l == 2`:** DDS wedged → redo §2. Still 2 → confirm
`echo $ROS_DISCOVERY_SERVER` is `10.42.0.1:11811` and `ping 10.42.0.1` works. If
the laptop is **multi-homed** (cable + WiFi) and the default route is wrong, DDS
can still find the robot because the `10.42.0.0/24` route is directly connected —
but if you ever see flakiness, see §6.

---

## 4. Bring-up

Two ways. **Granular (one terminal per subsystem)** is recommended for live
hardware testing — any layer can be killed/restarted on its own, which is what
you want when debugging on the cable. **One-command** is simplest.

> **One-command (least on-stage control):**
> ```bash
> ros2 launch lupin_bringup hardware.launch.py mission:=true
> ```
> Brings up T1–T9 below in one process tree. Because `ros-env.sh` is set to the
> wired IP, `hardware.launch.py` auto-targets the camera proxy at
> `http://10.42.0.1:8091` and runs `leds:=false` (the robot already owns the LED
> bridge). No extra flags needed. **Don't** also run the per-terminal launches —
> that double-launches everything.

**Every new terminal from here starts with the two `source` lines** (same as §3,
minus `daemon start`):

```bash
source ~/.config/lupin/ros-env.sh
source ~/ros2_ws/install/setup.bash
```

Skip either → 2 topics (DDS) or missing `lupin_*` packages/meshes (overlay).

The per-terminal **expectations and failure modes are identical to
`DEMO_DAY.md §4`** — only the IP changes. Below: the command, the **verified**
node/topic effect of each launch, and any wired-specific delta.

### T1 — HMI (Vite preview + rosbridge :9090; video proxied to robot :8091)

Re-source the two §3 lines **in this terminal first** — T1 fails *silently* if
you skip them (HMI comes up `LIVE` but with no battery/telemetry and can't drive;
see §6). The launch's `rosbridge DDS mode:` banner is the tell.

```bash
source ~/.config/lupin/ros-env.sh            # skip → §6 MULTICAST bug (HMI LIVE but dead)
source ~/ros2_ws/install/setup.bash
ros2 launch lupin_web lupin_web.launch.py \
  mode:=preview tls:=true rosbridge:=true \
  video:=false video_target:=http://10.42.0.1:8091 leds:=false
```

**Brings up (verified):** the Vite HMI on HTTPS `:8090`, a co-located
`rosbridge_websocket` on `:9090` (the HMI's DDS↔JSON broker — JSON-encoding load
lives on the laptop, not the Pi). `video:=false` because `lupin-cameras.service`
on the robot already serves MJPEG on `0.0.0.0:8091`; `video_target` retargets
Vite's `/_video` proxy at the robot's wired IP so only MJPEG crosses the cable.
`leds:=false` because the robot's `lupin-onboard.service` already runs
`light_strip_bridge` — running a second on the laptop collides on the node name
and the `/lupin/leds/{set,auto}` services (the LightControl card still works; it
reaches the robot's bridge over DDS).

**Check the banner** the launch prints: must say `rosbridge DDS mode:
DISCOVERY-SERVER 10.42.0.1:11811`. If it says `MULTICAST — ROS_DISCOVERY_SERVER
is UNSET`, this terminal skipped the `source` lines — Ctrl-C and relaunch.

Open `https://localhost:8090`. Click **Reset** if the e-stop banner shows. If the
virtual joystick doesn't drive on first try, **reload the page once** (known
startup-state flap). The camera tab now pulls MJPEG over the cable.

### T2 — RViz

```bash
ros2 run rviz2 rviz2 -d ~/ros2_ws/src/lupin/lupin_bringup/rviz/full_bringup_viz.rviz
```

**Brings up:** the persistent bringup view. **Expect** robot model + `/scan` + TF.
Fixed Frame defaults to `map` (won't exist until T3) — set `base_link` until then.
Empty model/frames → you forgot a `source` line (both are needed).

### T3 — SLAM

```bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  --params-file ~/ros2_ws/src/lupin/lupin_navigation/config/slam_toolbox_sim.yaml \
  -p use_sim_time:=false
```

**Brings up:** `slam_toolbox` owning `/map` + the `map→odom` TF. The
`slam_toolbox_sim.yaml` params are shared sim/hardware; the only sim-specific bit
(`use_sim_time`) is overridden to `false` here. **Expect** `Registering sensor:
[Custom Described Lidar]` then `/map` within 5–10 s. One `Message Filter
dropping…` at startup is normal; repeated = clock drift (redo §1, cleanly).

### T4 — slam_reset service (HMI "Erase map" button)

```bash
ros2 run lupin_navigation slam_reset_node
```

**Brings up:** the `/lupin/nav/clear_map` Trigger service. The HMI's Erase-map
SIGTERMs `slam_toolbox` (which respawns blank). One line of output then idle.

### T5 — Nav2 (only after T3's `/map` is alive)

```bash
ros2 launch lupin_navigation nav2.launch.py \
  slam:=true use_sim_time:=false \
  params_file:=$(ros2 pkg prefix lupin_navigation)/share/lupin_navigation/config/nav2_params.yaml \
  map:=$(ros2 pkg prefix lupin_navigation)/share/lupin_navigation/maps/krr_house.yaml
```

**Brings up:** the Nav2 lifecycle stack (controller/planner/behavior/bt_navigator/
waypoint_follower/velocity_smoother + lifecycle_manager). `velocity_smoother`
outputs `/cmd_vel_auto` → `twist_mux` on the robot. **`use_sim_time:=false` is
required** — `nav2.launch.py` defaults it to `true`. The `map:=` path is a
placeholder (in `slam:=true` mode `map_server` isn't instantiated; `RewrittenYaml`
just needs a valid path). **Expect** ~30 s of activation logs ending with
`Managed nodes are active` + `Creating bond timer…`.

> Over a clean cable, Nav2's discovery is *faster* than over the AP — the bond
> timeout was raised to 20 s for the AP's lag and has even more margin here. If
> you still see `Server controller_server was unable to be reached … by bond`,
> it's SHM/clock, not the link — use the **targeted Nav2-only restart** in
> `DEMO_DAY.md §4 T5` (do **not** full-SHM-wipe; that kills slam/RViz/HMI too).

### T6 — Digital twin

```bash
ros2 launch lupin_twin twin.launch.py
```

**Brings up:** `lupin_twin` aggregating `/floranova/observations` → `/twin/state`
(+ `/twin/get_field`). Idle until the mission publishes observations.

### T7 — Perception (AprilTag detector + flower/pest fusion)

```bash
ros2 launch lupin_perception perception_stack.launch.py
```

**Brings up (verified, this branch):**
- **`tag_annotator`** — AprilTag detection on the **Orbbec** RGB stream
  (`/camera/color/image_raw`, calibrated intrinsics → accurate tag distance/TF)
  → `tag_<id>` TFs + `/camera/tag_detections_json` (the HMI Cameras overlay).
- **`yolo_detector`** — gripper-cam YOLO (`/gripper_camera/image_raw/compressed`)
  → `/yolo/detections` (flower species + `bug`). **Needs `ultralytics` +
  `numpy<2`** in this env; missing → it no-ops and flowers stay unclassified.
- **`perception_aggregator`** (`enable_aggregator:=true` default) — fuses tags +
  YOLO into `/perception/discovered_tags` + `KIND_FLOWER` obs on
  `/floranova/observations` → twin → the flower/pest map markers.

**Expect** `tag_annotator online …`, `Latched intrinsics from camera_info` within
~3 s, an aggregator start line, and a YOLO model-load line (a few seconds first
run). No "Latched intrinsics" in 10 s → cameras aren't publishing; check
`ros2 topic hz /camera/color/camera_info` (~5 Hz). Silent →
`ssh lupin-wired 'sudo systemctl restart lupin-cameras'`.

> AprilTags read off the **Orbbec head camera** (the calibrated, accurate path).
> If during live testing the head camera physically can't frame the tags, you can
> point the detector at the gripper cam instead:
> `perception_stack.launch.py tag_image_topic:=/gripper_camera/image_raw
> tag_camera_info_topic:=/gripper_camera/camera_info` — but that stream is
> uncalibrated, so tag *distance/TF* will be rough (the overlay still draws).
>
> **Markers need a mission.** Flower/pest markers only pin once T9's SCANNING
> phase attributes a YOLO reading to a tag — pointing the camera at a flower
> alone won't place one. The green-box tag overlay works on its own.

### T8 — Xbox teleop

```bash
ros2 launch lupin_hmi xbox_teleop.launch.py
```

**Brings up:** `joy_node` + `teleop_twist_joy` (→ `/cmd_vel_joy`, a `twist_mux`
input) + `arm_teleop`. **Expect** `Opened joystick: Xbox Series X Controller` then
`arm_teleop ready …` and `arm_teleop seeded from /joint_states …` once
`/joint_states` arrives. Button map + "button does nothing" fixes: `DEMO_DAY.md §4
T8` (BLE HID indices shift between pads — probe `ros2 topic echo /joy --field
buttons`).

### T9 — Mission (autonomous explore → monitor → flower; optional)

```bash
ros2 launch lupin_bringup mission_stack.launch.py        # discovery_goal:=N to override
```

**Brings up:** the greenhouse bridge (tag oracle →
`/greenhouse_bridge/get_tag_reading`), the `mission_orchestrator` lifecycle node,
and the one-shot `/amcl_pose` seed. Run **after** T1–T8 (it needs Nav2 + SLAM +
twin + T7). `dependency_timeout_s` is 120 s here (cold Nav2 lifecycle on
hardware). **Expect** a bridge ready line, `[lupin_bringup] mission_stack: …`, and
the orchestrator idling in `READY`. Start from the HMI **Mission** controls, or:
```bash
ros2 service call /mission/start lupin_msgs/srv/StartMission \
  "{mission_type: 'ExplorationMission', discovery_goal: 4}"
```
MissionStrip walks PREPARE → EXPLORING → MONITORING. FAULT after ~120 s → it
never saw Nav2/bridge (confirm T5 reached "active" first).

---

## 5. Pre-show test sequence

Same checks as `DEMO_DAY.md §5` — the link doesn't change behaviour. The
essentials:

| Test | How | Pass |
|---|---|---|
| **Xbox drive** | Hold **LB**, push left stick forward | Robot moves forward physically |
| **Xbox arm** | Release LB; **Y/A** lift, **B/X** pan, D-pad wrist/elbow | Joints move ~1.5 rad/s held |
| **Xbox gripper** | Release LB; **RT** open / **LT** close | Gripper opens/closes |
| **HMI joystick** | HMI Teleop, Reset e-stop, wiggle stick | Robot drives in expected direction |
| **Lidar / Map in HMI** | HMI Lidar / Map views | Scan ~10 Hz; OccupancyGrid renders |
| **Cameras in HMI** | HMI Cameras view | MJPEG streams (proxied to `10.42.0.1:8091`) |
| **Nav2 goal** | RViz "2D Goal Pose", 1–2 m ahead | Robot plans + drives |
| **HMI map-click goal** | HMI Map view, click a destination | Plans + drives to the click |
| **AprilTag overlay** | HMI Cameras, point Orbbec at a tag | Green box + ID + distance |
| **E-stop** | HMI e-stop | Robot stops immediately |
| **Map erase** | HMI menu → Erase map | `/map` clears, slam respawns blank |

Mission rows (need T7 full stack + T9): **Flower→map**, **Pest→map**,
**explore→monitor**, **Voice drive** — see `DEMO_DAY.md §5`.

**Isolate the base controller (Xbox/HMI won't drive and you don't know why).**
Publish a slow Twist *straight to the controller* — bypasses twist_mux, the HMI,
and the e-stop, so wheels-move means the controller + PID + telemetrix + wheels
are all healthy and the fault is upstream. Run it **robot-local** (`ros2 topic
echo`/`pub` from the laptop over the discovery server is flaky). Robot on blocks
or floor clear; `Ctrl-C` to stop:

```bash
ssh lupin-wired
ros2 topic pub -r 20 /mirte_base_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.10}, angular: {z: 0.0}}"
# mecanum: linear.y strafes, angular.z rotates. Rate must be >=4 Hz — the
# controller's command_timeout is 0.25 s, so a one-shot/slow pub just stops.
```

Wheels spin → base stack healthy (chase the HMI/Xbox/twist_mux/e-stop path).
Wheels dead → controller wedged → §1 reboot-free recovery (stop → wipe SHM → start).

---

## 6. Known quirks (so you don't panic mid-demo)

- **HMI LIVE but no battery/telemetry/can't drive — yet a fresh terminal shows
  the robot's topics.** The HMI's rosbridge was launched from a terminal that
  never sourced `ros-env.sh`, so it's on default-multicast and never joined the
  discovery-server graph. The env is per-*process*, so a CLI check elsewhere looks
  fine and hides it. T1's `rosbridge DDS mode:` banner catches it — if it says
  `MULTICAST`, that's the bug. Confirm:
  `tr '\0' '\n' </proc/$(pgrep -f rosbridge_websocket)/environ | grep ROS_DISCOVERY_SERVER`
  (empty = bug). Fix: Ctrl-C T1, re-source both lines, relaunch, reload browser.
- **Multi-homing is a HARD BLOCKER, not a caveat (proven live 2026-06-03).** If
  the robot's AP (`wlan0` `192.168.42.1`) is up alongside the cable, the robot
  advertises both DDS locators and the laptop does **not** cleanly ignore the
  unreachable AP one: **laptop→robot DATA silently fails** — no drive, arm/service
  calls time out, `ros2 control list_controllers` hangs — while discovery and
  robot→laptop telemetry keep working, so the HMI looks `LIVE`. **Dropping only
  the laptop's WiFi association is NOT enough** — the robot still advertises the
  AP locator regardless of what the laptop is associated to. The fix is
  **robot-side**: disable the AP and reboot so every participant re-announces
  `eth0`-only (see §A.3 — `disable --now mirte-ap mirte-wifi-watchdog` + reboot).
  Signature to recognise it: `ros2 topic echo /scan` (robot→laptop) works but
  `ros2 control list_controllers` (laptop→robot round-trip) times out; a
  robot-side packet capture shows the laptop's user-data hitting the wrong ports,
  never the subscriber's. **Durable fix that keeps the AP as a fallback (TODO, not
  yet implemented):** give the robot a FastDDS profile with an `interfaceWhiteList`
  of `127.0.0.1` + the `eth0` IP and `useBuiltinTransports=false`, applied to all
  robot nodes, so DDS only ever advertises the wired locator. A USB→ethernet
  iPhone tether on a *separate* `enx…` is fine to keep (internet only) — it never
  carries robot traffic.
- **Don't let the laptop route internet over the cable.** B3 sets
  `ipv4.never-default yes`, so the WiFi default route wins and the cable carries
  only robot traffic. If you skipped that, `nmcli con mod lupin-wired
  ipv4.never-default yes && nmcli con up lupin-wired`.
- **The bringup banner's `LAN: https://<ip>:8090` may show the WiFi IP or
  `localhost`** — it's computed via a route to 8.8.8.8, which goes out WiFi, not
  the cable. Cosmetic only; open `https://localhost:8090`.
- **Drive direction is physically correct** (fixed at the source 2026-06-02 via a
  telemetrix pin-swap). `polarityInvertHmi` is false; Xbox scales positive. A
  *different* robot driving backward is an uncorrected unit — fix its pins.
- **`/rosapi/get_time` errors every ~2 s** — node-name collision between the
  vendor and laptop rosbridge. Cosmetic.
- **`Message Filter dropping …`** once at SLAM startup is normal; repeated = clock
  drift.

---

## 7. Emergency reset (when in doubt)

```bash
# Ctrl-C every laptop terminal, then wipe laptop SHM:
pkill -9 -f 'ros2|rviz2|rosbridge|slam_toolbox|twin_node|tag_annotator|web_video_server' 2>/dev/null
ros2 daemon stop; pkill -9 -f ros2cli; rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
# Robot: stop -> wipe stale SHM -> start. A plain `restart` does NOT clear a
# clock-step wedge; you must wipe /dev/shm/fastrtps_* with the stack down.
ssh lupin-wired 'sudo systemctl stop lupin-onboard lupin-cameras mirte-ros && sleep 3 \
  && sudo rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* && sudo systemctl start mirte-ros'
sleep 25
ssh lupin-wired 'sudo systemctl start lupin-onboard lupin-cameras'
ROBOT=mirte@10.42.0.1 ~/.config/lupin/post-boot-sync.sh
# Then redo §3 onward.
```

Link itself looks dead? `ping 10.42.0.1` — if it fails, it's the cable/adapter or
the §A static IP, not ROS. Reseat the cable, `nmcli con up lupin-wired` on the
laptop, confirm the robot's `eth0` still shows `10.42.0.1/24`. The SHM-wipe restart
above is the cure for a wedged controller_manager — a power-cycle is a last resort
and *recurs* on this robot (dead RTC re-creates the clock-step wedge), so prefer
the wipe + restart and keep the clock stable after.

---

## 8. Reference

- **Robot wired IP:** `10.42.0.1` (`eth0`, static, `lupin-wired` NM profile)
- **Laptop wired IP:** `10.42.0.2` (USB-C adapter, static, no default route)
- **Robot AP (fallback):** `192.168.42.1` (`Mirte-247264`)
- **SSH:** `ssh lupin-wired` (alias) or `ssh mirte@10.42.0.1`
- **HMI URL:** `https://localhost:8090`
- **rosbridge:** `wss://localhost:8090/_ros` (same-origin proxy)
- **Robot's DDS discovery server:** `10.42.0.1:11811` (binds `0.0.0.0`)
- **DDS env (laptop):** `~/.config/lupin/ros-env.sh` (`setup-laptop-dds-env.sh 10.42.0.1`)
- **Clock + health:** `ROBOT=mirte@10.42.0.1 ~/.config/lupin/post-boot-sync.sh`
- **Cableless (AP) guide:** `DEMO_DAY.md` · **Sim guide:** `DEMO_DAY_SIM.md`
- **Revert wired → AP:** `DEMO_DAY_WIRED_REVERT.md`
