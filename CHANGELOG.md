# Changelog

All notable changes to the Lupin stack are documented here.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Because the repository uses three long-lived branches (`main`, `sim`,
`hardware` — see the [README](README.md#branch-model)), entries note where a
change is branch-specific. Unmarked entries apply to all three.

## [Unreleased]

_Nothing yet._

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

[Unreleased]: https://gitlab.tudelft.nl/cor/ro47007/2026/group_14/lupin/-/compare/v1.0.0...main
[1.0.0]: https://gitlab.tudelft.nl/cor/ro47007/2026/group_14/lupin/-/tags/v1.0.0
