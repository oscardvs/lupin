# Changelog

All notable changes to the Lupin stack are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Because the repository uses three long-lived branches (`main`, `sim`,
`hardware` — see the [README](README.md#branch-model)), entries note where a
change is branch-specific. Unmarked entries apply to all three.

## [Unreleased]

_Nothing yet._

## [1.1.0] — 2026-06-07

Greenhouse mission + perception overhaul on top of `v1.0.0`: per-bloom flower
localization and rendering, an operator arm pose/sequence library, a hardened
mission state machine, the physical e-stop wired into the orchestrator, and a
stabilised OpenCV AprilTag pipeline. Demonstrated end-to-end in Gazebo (`sim`)
and on the physical MIRTE Master V2 (`hardware`).

### Added

- **Flower localization & render** — new `FlowerPoint` message and
  `flowers[]` / `box_footprint` fields on `FlowerObservation` and
  `TwinTagState`. `perception_aggregator` derives a tag-anchored planter-box
  rectangle and scatters per-bloom points inside it from a pan/bbox cue; the
  twin republishes them per tag; the HMI draws the box rectangle, a diamond
  tag marker, species-coloured flower dots, and a 3D bloom health indicator.
- **Arm pose & sequence library** (`arm_library_server`, laptop-side) — named
  pose CRUD, kinesthetic/teleop sequence recording with a waypoint cap, and
  replay with scheduled gripper events and speed scaling. New `lupin_msgs`
  services (`GetArmLibrary`, `SaveArmPose`, `ArmRecord`, `PlayArmSequence`,
  `ArmLibraryEdit`), HMI **Pose Library** and **Sequence Recorder** cards, and
  voice-agent tools for the whole surface.
- **Mission monitoring** — `ExplorationMission` runs a nearest-neighbour
  monitoring sweep that completes after `target_cycles` (default 1), with a
  scan dwell so the per-pot patrol arm and flower detector get their window.
  Mission events are classified so the HMI shows a tone rather than a raw
  "orchestrator error".
- **Physical e-stop** — the hardware emergency-stop button is bridged onto
  `/e_stop_state`, and the HMI STOP now reaches the orchestrator e-stop.
- **LED control** — manual LED control plus mission-state strip colours
  (including EXPLORING/MONITORING and a distinct battery-low return colour),
  an HMI-colocated light-strip bridge, and a BRG wire-order remap.
- **Sim** — vision perception and per-pot arm patrol wired into
  `sim_full.launch.py`, a three-colour flower scan layer (trough clusters +
  bug anomalies + HSV detector), per-environment slam/nav2 launch wrappers,
  a `mission_stack` launch, and DEMO_DAY runbooks.
- Single source of truth for arm joint limits and trajectory building
  (`arm_limits`, mirrored by the frontend `lib/arm.ts`).

### Changed

- **`cmd_vel_mux` → `twist_mux`** — base-velocity arbitration moved to
  `twist_mux` over `/cmd_vel_{joy,manual,auto}` (priorities 100/50/10) on both
  the `sim` and `hardware` branches.
- **AprilTag detection** moved from the standard ROS detector to OpenCV ArUco
  for stability, publishing detections as JSON with an HMI overlay toggle
  (no second bounding-box stream); Gazebo gains real `tag36h11` models in
  place of the magenta placeholders.
- **HMI arm control** — sliders seed from `/joint_states` feedback behind a
  strict Enable arming gate, report convergence-based status, and clamp to the
  real asymmetric per-joint limits.
- **Mission FSM hardened** — recoverable FAULT, working dock recall, nav/dock
  timeouts and a safe freeze; battery docking is resumable into the
  originating sub-machine and retries then pauses instead of silently
  reporting DONE.
- `hardware.launch.py` made wired-demo-ready (`robot_ip` arg, `leds:=false`).

### Fixed

- Metric frontier clearance so exploration goals clear Nav2's inflation layer.
- Twin UI greys out when `/twin/state` goes stale; tag markers are pinned at
  the tag pose, not the robot pose.
- Battery-percentage normalisation and an approach-footprint guard.
- Hardware clock-sync no longer wedges `ros2_control`.
- Sim stability: patched `gazebo_ros2_control` so `controller_manager` starts,
  restored the vendor `planar_move` drive (grounded spawn, no Ogre crash),
  raised planter bodies above the lidar plane, and added a continuous
  `/amcl_pose` stand-in so the mission clears localization.

## [1.0.0] — 2026-06-02

First tagged release: the full FloraNova greenhouse digital-twin stack,
demonstrated end-to-end both in Gazebo (`sim`) and on the physical MIRTE
Master V2 (`hardware`). This tag marks `main` synced up to the `hardware`
branch.

### Added

- **Mission orchestration** (`lupin_mission`) — hierarchical state-machine
  orchestrator (v2) driving `BOOT → READY → PREPARE → {INSPECTING |
  EXPLORING → MONITORING} → RETURNING → DONE/FAULT`. Two mission types:
  `InspectionMission` (fixed tag list, once) and `ExplorationMission`
  (frontier-discover N tags, then monitor them in a loop). Operator services
  `/mission/{start,pause,resume,abort,skip_current,dock}`, 5 Hz
  `/mission/state`, and an `/e_stop_state` safety monitor.
- **Battery & docking** — `BatteryMonitor` with battery-triggered return-to-dock
  and a resumable manual/battery docking flow (`/mission/dock`).
- **Frontier exploration** (`lupin_mission/frontier.py`) — online frontier
  selection over the live SLAM grid for autonomous tag discovery.
- **Navigation** (`lupin_navigation`) — Nav2 with the MPPI controller
  (`motion_model: Omni` for the mecanum base), slam_toolbox SLAM mode, saved
  KRR-house map, and per-leg AMCL drift gating. WiFi cold-start lifecycle
  reliability tuning (`hardware`).
- **Perception** (`lupin_perception`) — OpenCV ArUco AprilTag detector,
  Ultralytics YOLO tulip/anomaly detector on the gripper cam, and a
  `perception_aggregator` fusing them into `/perception/discovered_tags`,
  flower observations, and a `/perception/confirm_tag` service.
- **Greenhouse bridge** (`lupin_greenhouse_bridge`) — ROS 2 wrapper around the
  course `mdp-greenhouse` simulator exposing `~/get_tag_reading`.
- **Digital twin** (`lupin_twin`) — aggregates `/floranova/observations` into
  `/twin/state` and an IDW `/twin/get_field` service.
- **Web HMI** (`lupin_web`) — browser HMI on `:8090` (Vite + React +
  shadcn/ui): Teleop, Arm, Voice (Gemini Live, 14-tool surface), Cameras,
  Telemetry, Logs, and Map tabs over rosbridge. Showpiece redesign: aurora
  background, URDF 3D twin, boot sequence, and data-reactive telemetry.
- **Teleop & control interface** (`lupin_hmi`) — PS4/Xbox + keyboard teleop,
  `cmd_vel_mux` arbitration, arm preset/calibration servers, and
  gripper/light-strip bridges.
- **Custom messages** (`lupin_msgs`) — `Observation`, `MissionState`,
  `TagReading`, `SensorReading`, `FlowerObservation`, `AnomalyReport`,
  `TwinState`/`TwinTagState`, `DiscoveredTag`/`DiscoveredTags`; services
  `GetTagReading`, `StartMission`, `ConfirmTag`, `GetField`, `CalibrateArm`.
- **Bringup** (`lupin_bringup`) — one-shot `sim_full.launch.py`,
  `greenhouse_sim.launch.py` + deterministic greenhouse world generator, and
  hardware bringup (`hardware.launch.py`, `onboard.launch.py`,
  `cameras.launch.py`) with systemd units, FastDDS discovery-server setup,
  Xbox BLE pairing fix, and post-boot clock sync (`hardware`).
- **Hardware integration** (`hardware`) — drive-inversion cancelled at source
  via telemetrix pin-swap, operator-in-the-loop Hiwonder arm zero-offset
  calibration, auto-home to a safe pose on boot, and the camera pipeline that
  relaunches vendor cameras at config-driven low FPS.
- Repository `LICENSE` (Apache-2.0) and this changelog.

### Project meta

- Three-branch model (`main`/`sim`/`hardware`), Conventional Commits, and
  buddy-checked MRs — see [README](README.md#contributing).
- Public documentation site: <https://lupin-robot.vercel.app>.

[Unreleased]: https://github.com/oscardvs/lupin/compare/v1.1.0...main
[1.1.0]: https://github.com/oscardvs/lupin/compare/v1.0.0...v1.1.0
[1.0.0]: https://github.com/oscardvs/lupin/releases/tag/v1.0.0
