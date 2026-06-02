# lupin_perception

Vision package for Team Lupin on the **real MIRTE Master**. It ships three
nodes:

- **`tag_annotator`** — detects AprilTags in the Orbbec RGB stream, broadcasts
  a TF per detection, publishes overlay JSON for the web HMI.
- **`yolo_detector`** — Ultralytics YOLO on the gripper cam, classifies tulip
  species (`tulip_red`/`tulip_white`/`tulip_pink`) + a `bug` anomaly class.
- **`perception_aggregator`** — fuses the two: TF-projects discovered tags into
  the `map` frame, co-locates the YOLO flower class with each tag, and exposes
  the result to the mission (discovery feed + `/perception/confirm_tag`) and the
  twin/HMI (`KIND_FLOWER` observations). See "Perception aggregator" below.

## What it does

1. **Detects AprilTags** (family `36h11`) using OpenCV's native ArUco
   detector — no `apriltag_ros` dependency.
2. **Estimates each tag's 3D pose** with `cv2.solvePnP`, using camera
   intrinsics **read live from `CameraInfo`** (so the same code works
   against any calibrated camera, no per-bot hard-coded K).
3. **Broadcasts a TF transform** per tag: parent = whatever frame the
   driver puts in `Image.header.frame_id` (e.g. `camera_color_optical_frame`),
   child = `tag_<id>`.
4. **Publishes overlay JSON** (corners + ID + distance) on
   `/camera/tag_detections_json` for the HMI's `CameraStream` widget to
   draw boxes on the MJPEG stream.

## Launching

```bash
ros2 launch lupin_perception perception.launch.py
```

To start both the AprilTag and YOLO pipelines together:

```bash
ros2 launch lupin_perception perception_stack.launch.py
```

It's also folded into `lupin_bringup hardware.launch.py`: `perception:=true`
(default) starts `tag_annotator` **and** `perception_aggregator` (both light);
`yolo:=true` (default) starts the heavy YOLO detector laptop-side. Pass
`perception:=false` / `yolo:=false` to skip.

## Perception aggregator

`perception_aggregator` is the fusion + discovery node the mission and twin
depend on. It:

1. Subscribes `/camera/tag_detections_json` and looks the per-tag `tag_<id>` TF
   up into the `map` frame (only the **calibrated Orbbec** stream yields a TF;
   the uncalibrated gripper cam doesn't). A tag is "discovered" after a few
   confident sightings (debounce).
2. Subscribes `/yolo/detections` and attributes the dominant tulip class — and
   the `bug` anomaly — to the tag the robot is currently SCANNING (read from
   `/mission/state`), falling back to the nearest detected tag when run
   standalone. This is **temporal co-location**, not 3-D overlap (v1 limitation
   — the two detectors are on different, differently-calibrated cameras).
3. Publishes:
   - `/perception/discovered_tags` (`lupin_msgs/DiscoveredTags`, latched) — the
     orchestrator counts these against `discovery_goal` and uses the poses for
     monitoring approach goals.
   - `KIND_FLOWER` Observations on `/floranova/observations` — the twin merges
     species/anomaly onto the tag and the HMI map renders them.
   - `/perception/confirm_tag` (`lupin_msgs/srv/ConfirmTag`) — the orchestrator's
     hardware visual-confirmation gate.

```bash
ros2 launch lupin_perception perception_aggregator.launch.py
# inspect the discovery registry as the robot explores:
ros2 topic echo /perception/discovered_tags
```

> **Compute:** keep `yolo_detector` + `perception_aggregator` off the Orange Pi.
> YOLO needs torch, and `pip install ultralytics` drags numpy 2 / opencv-python
> that break `cv_bridge` — run them laptop/Jetson-side with `numpy<2` and
> without `opencv-python` (see `project_ultralytics_install_gotcha`).

## Configuration

Launch arguments (with defaults):

| Arg | Default | Notes |
| --- | --- | --- |
| `image_topic` | `/camera/color/image_raw` | Vendor Orbbec RGB. On hardware this is served by `lupin-cameras.service` at a config-driven low FPS — the detection rate is capped to that setting and the HMI overlay stays in sync automatically. Switch to `/gripper_camera/image_raw` for the wrist cam. |
| `camera_info_topic` | `/camera/color/camera_info` | Must be the matching rectified intrinsics for `image_topic`. |
| `detections_topic` | `/camera/tag_detections_json` | What the HMI subscribes to. Don't change unless you also reconfigure the HMI. |
| `tag_size_m` | `0.04` | Physical edge length of the printed tags. |
| `tf_frame_prefix` | `tag_` | Child frame id = `f"{prefix}{id}"`. |
| `image_qos` | `sensor_data` | BEST_EFFORT (KEEP_LAST 5). Matches the vendor driver. Set to `reliable` for sim-style profiles. |
| `use_sim_time` | `false` | Real robot has no `/clock`. |

Example — gripper-cam variant for in-hand tag tracking:

```bash
ros2 launch lupin_perception perception.launch.py \
  image_topic:=/gripper_camera/image_raw \
  camera_info_topic:=/gripper_camera/camera_info
```

## Topic contract

Subscriptions:

| Topic | Type | Notes |
| --- | --- | --- |
| `image_topic` | `sensor_msgs/Image` | First frame ignored if `camera_info` hasn't arrived yet — node logs a throttled warning and waits. |
| `camera_info_topic` | `sensor_msgs/CameraInfo` | First message latches `K` and `D`; subsequent messages ignored. |

Publications:

| Topic | Type | Notes |
| --- | --- | --- |
| `/camera/tag_detections_json` | `std_msgs/String` | JSON array: `[{id, corners: [[x,y]×4], dist}]`. Empty array per frame when no tags are visible (clears the HMI overlay). |
| `/tf` | `tf2_msgs/TFMessage` | One transform per detected tag, stamped with `Image.header.stamp`. |

## Why this differs from the sim build

| Aspect | sim | hardware (this branch) |
| --- | --- | --- |
| `image_topic` | `/camera/image_raw` (Gazebo plugin) | `/camera/color/image_raw` (Orbbec driver) |
| Intrinsics | Hard-coded for Gazebo 640×480, no distortion | Latched from live `CameraInfo` (per-bot calibration) |
| `use_sim_time` | `True` | `False` |
| Image QoS | `RELIABLE` | `BEST_EFFORT` (sensor_data) |

## Debugging

```bash
# Verify the camera stack is alive
ros2 topic hz /camera/color/image_raw
ros2 topic echo --once /camera/color/camera_info

# Watch live detections
ros2 topic echo /camera/tag_detections_json --no-arr

# Inspect TFs (transient, only published while tags are in view)
ros2 run tf2_ros tf2_echo camera_color_optical_frame tag_1
```

If the HMI shows the camera feed but never any boxes:

1. Check the JSON publisher rate: `ros2 topic hz /camera/tag_detections_json`.
   - Zero → the node isn't getting either `image_topic` or `camera_info_topic`.
     Re-check the QoS warning in the node log (a BEST_EFFORT publisher won't
     deliver to a RELIABLE subscriber, but our default goes the other way).
2. If JSON is publishing but boxes don't appear in the HMI, make sure the
   HMI's "Overlay AprilTags" toggle is on (top of the Cameras view).

## Future work

- Per-detection 3-D flower localization (project YOLO boxes via the Orbbec
  depth stream) to replace the v1 temporal co-location with the AprilTag pose.
- Aim the arm/gripper cam at the table during SCANNING so the flower is framed
  reliably (today fusion trusts whatever the gripper sees while parked).
- Move intrinsics latching to a one-shot `wait_for_message` so the
  first frame after startup actually processes instead of being dropped.

Done since the first cut: `/perception/confirm_tag` (now served by
`perception_aggregator`) and the YOLO flower/anomaly classifier.
