# Sim full-mission runbook (2026-06-02)

Branch `feat/sim-port-mission-2026-06-02`. Goal: rehearse the **full mission in
Gazebo** with minimal gap to tomorrow's hardware run — exploration, AprilTag
detection + map assignment, per-pot arm patrol, base approach, **3-colour flower
detection + map colour**, and bug/anomaly.

> **Minimal-gap contract:** the ONLY sim↔hardware difference is the *detector*.
> Sim runs `sim_flower_detector` (HSV); hardware runs `yolo_detector` (best.pt)
> via `perception_stack.launch.py`. Both publish the identical `/yolo/detections`
> contract, so `perception_aggregator` → twin → HMI are byte-identical.

## 0. One-time build
```bash
cd ~/ros2_ws
colcon build --symlink-install        # already done; rerun if you edit code
source install/setup.bash
```

## 1. Launch the whole sim
```bash
ros2 launch lupin_bringup sim_full.launch.py
```
Cascade (event-driven, no fixed delays): Gazebo + greenhouse world → `/scan` →
`slam_toolbox` (`/map`) → Nav2 (slam mode) → mission orchestrator idles in
`READY`. Also up: bridge, **tag_annotator + perception_aggregator +
sim_flower_detector**, twin, rosbridge, web HMI (`http://localhost:8090`), RViz,
twist_mux, arm shim + preset server.

Expect ~30–60 s before everything is green on a cold start.

### Confirm it came up
```bash
ros2 topic echo /mission/state --once          # lifecycle_state: READY
ros2 topic list | grep -E 'camera|gripper|scan|map|discovered|yolo|twin'
```
You should see `/camera/image_raw`, `/gripper_camera/image_raw`, `/scan`, `/map`,
`/perception/discovered_tags`, `/yolo/detections`, `/twin/state`.

## 2. Start the mission
First a quick smoke pass (discover a few tags) to prove the pipeline:
```bash
ros2 service call /mission/start lupin_msgs/srv/StartMission \
  "{mission_type: 'ExplorationMission', discovery_goal: 4}"
```
Then the full sweep (all 22 tags → continuous monitoring):
```bash
ros2 service call /mission/start lupin_msgs/srv/StartMission \
  "{mission_type: 'ExplorationMission', discovery_goal: 22}"
```
(`discovery_goal: 0` uses the node default of 5. Arm patrol is already on in sim.)

## 3. What to watch (the "intelligence")
- **Frontier exploration** — robot drives into unknown space (RViz: costmap +
  `/map` growing; `/mission/state` `mission_phase` = EXPLORING).
- **Tag detection + map assignment** — body Astra sees a tag →
  `/perception/discovered_tags` gains an entry with a map pose.
  `ros2 topic echo /perception/discovered_tags`
- **EXPLORING → MONITORING** once `discovery_goal` tags are found.
- **Base approach** — robot parks at a standoff facing each tag.
- **Arm patrol** — at each pot the arm strikes the `inspect` pose (gripper cam
  down on the bloom), returns to `home` between pots. Watch the arm in Gazebo;
  orchestrator logs `arm patrol → preset "inspect"`.
- **Flower colour + map** — the gripper cam sees the cluster → `sim_flower_detector`
  publishes `/yolo/detections` → aggregator attributes the dominant colour to the
  scanned tag → `/twin/state` `species` (`tulip_red`=magenta, `tulip_white`,
  `tulip_pink`) → HMI map shows a coloured ring; a `bug` station shows a dashed
  anomaly ring. `ros2 topic echo /twin/state`
- **Flower detector debug view** — `/sim_flower/overlay` shows the gripper-cam
  frame with detected colour boxes (view in RViz/rqt_image_view) — use this to
  confirm/tune detection.

## 4. Known risks — what to look for and how to fix
| Symptom | Cause | Fix |
|---|---|---|
| Arm/base controllers don't spawn; ros2_control errors at startup | The gripper-camera Gazebo sensor (restored in `arm.xacro`) may trip `gazebo_ros2_control` (it was bisected out before) | Tell me — we re-comment the `<gazebo reference="gripper_camera_link">` sensor block in `mirte_master_description/urdf/arm.xacro` and move the camera to a separately-spawned model |
| `tag_annotator` warns "No CameraInfo" / no tag TF | Body cam topic isn't `/camera/image_raw` | `ros2 topic list \| grep camera`; tell me the real topic — 1-line fix in `sim_full.launch.py` (fallback intrinsics keep PnP alive meanwhile) |
| Exploration finds 0 tags, times out | Body cam not seeing the side tags, or detector silent | check `/camera/tag_detections_json` while driving; lower `discovery_goal`; drive a manual loop first (Xbox/web) so SLAM has context |
| Wrong/again flower colour, or none | HSV bands vs actual Gazebo render | open `/sim_flower/overlay`; report the colour that's off — bands are params in `sim_flower_detector_node.py` (`_FLOWER_CLASSES`), quick to retune |
| Gripper cam doesn't frame the bloom | `inspect` arm pose angles or camera mount | report what the cam sees; tune the `inspect` preset in `arm_preset_server.py` (`PRESETS['inspect']`) and/or the camera mount in `arm.xacro` |
| Base teleports / odom lags | P3D plugin (vendor URDF) vs mecanum control | known sim limitation; nav still works visually |

## 5. Vendor edit (not in the lupin repo)
`mirte_master_description/urdf/arm.xacro` — restored the gripper-camera Gazebo
sensor + added its optical frame (the `gripper_camera_link` mount was already a
team edit). Needed on any machine running this sim until we vendor the URDF into
lupin. Validated: `xacro mirte_master.xacro sim:=True …` expands and publishes
`/gripper_camera/image_raw`.
