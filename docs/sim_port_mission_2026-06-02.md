# Sim port + full-mission test — design (2026-06-02)

Branch: `feat/sim-port-mission-2026-06-02` (off `origin/sim`) → MR into `sim`.

## Goal
Port `main` onto `sim` and run the **full autonomous greenhouse mission in Gazebo**, to
observe the robot's intelligence: frontier ("random") exploration, AprilTag detection +
map-position assignment, digital-twin info updates, per-pot base approach, and (new) the
arm articulating to a patrol/inspect pose at each pot.

## Branch reality (corrected 2026-06-02)
- `main` is **140 commits ahead** of `origin/sim`; `origin/sim` has **12 commits** main
  lacks — all the **sim AprilTag/ArUco vision pipeline** (real `tag36h11` Gazebo textures +
  OpenCV ArUco `tag_annotator`).
- Merging `main` → `sim` yields **9 conflicts**, only 3 real code:
  `tag_annotator.py`, `perception/launch/perception.launch.py`, `web/.../ros.ts`.
  The rest are READMEs + `setup.py`. The 26 tag textures, the material script, and the
  real-texture world generator come in cleanly (main never touched them).
- **Vision-based tag detection in Gazebo is already solved on `sim`** — `generate_greenhouse_world.py`
  renders real textured tag plates (magenta only via a `use_placeholder` toggle).

## A. Port (merge + conflict resolution)
- `tag_annotator.py`: unify — ArUco detection + TF + JSON; prefer live `CameraInfo` (K +
  distortion) when present & non-zero, else fall back to a configurable hardcoded K so sim
  works if `camera_info` is late. `tag_size_m` a param.
- `perception.launch.py`: main's parameterized superset.
- `ros.ts`: union of `TagDetection` fields.
- `setup.py` ×2: keep main installs **and** sim apriltag model/material data-files.
- READMEs: main's, fold in sim perception notes.
- `colcon build` must pass before continuing.

## B. Sim mission-readiness
1. Add **`tag_annotator` + `perception_aggregator`** to `sim_full.launch.py` — without the
   aggregator publishing `/perception/discovered_tags`, frontier exploration discovers 0
   tags and times out.
2. `tag_size_m = 0.16` (world plate size) — else PnP map positions are wrong by ~4×.
3. Reconcile the sim camera image/`camera_info` topics + the optical TF frame so the
   aggregator's `map→tag_<id>` lookup resolves through SLAM (`map→odom`) + URDF.
4. Scanning readings via the **oracle bridge** (`require_visual_confirmation=False`):
   discovery is real vision, sensor data is bridge-by-tag-id (reliable). Flag flippable.

## C. New feature — per-tag arm patrol
- Param `arm_patrol_enabled` (default true in sim). On entering SCANNING the orchestrator
  commands an **`inspect` arm pose with the camera tilted DOWN toward the pot** (low-lift,
  thermally safe); on leaving SCANNING for the next tag it returns to a travel pose.
- Uses `/lupin/arm/preset`; adds an `inspect` preset if missing.

## D. Bring-up & observe
`colcon build` → `ros2 launch lupin_bringup sim_full.launch.py` → wait for
Gazebo→`/scan`→SLAM `/map`→Nav2→mission `READY` → start `ExplorationMission`
(`discovery_goal` = full tag set, 22). Observe via HMI + RViz + topic echoes:
frontier exploration → ArUco detect → `/perception/discovered_tags` map assignment →
`EXPLORING→MONITORING` → base approach per pot → arm inspect pose → `/twin/state` updates.

## Risks validated live (not assumed)
- P3D teleport vs real mecanum drive in `greenhouse_sim`.
- Sim camera optical-frame name in the TF chain.
- ArUco detection range on 0.16 m plates at nav distances.
- Whether the greenhouse world yields frontiers within the exploration timeout (22 tags is
  ambitious — a lower-N smoke pass first is recommended).
