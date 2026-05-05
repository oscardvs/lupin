# lupin_perception

`lupin_perception` is the vision package for Team Lupin. It provides a custom AprilTag detection and annotation system using OpenCV, and serves as the home for future flower-detection pipelines.

## Overview

The perception system has been updated to use **OpenCV's native ArUco detector** instead of the external `apriltag_ros` package. This provides better control, fewer external dependencies, and improved integration with the robot's coordinate systems.

### Key Components

- **`tag_annotator` node**: Custom AprilTag detection, 3D pose estimation, and visualization
  - Subscribes to: `/camera/image_raw`
  - Publishes:
    - `/camera/tag_detections_json`: JSON metadata for AprilTag overlays in the web UI
    - TF transforms for each detected tag (frame: `tag_<id>`)
  - Broadcasts 3D poses using TF2 with perspective-n-point (PnP) solver

## What it does

1. **Detects AprilTags** in the camera stream using OpenCV's ArUco detector (36h11 family)
2. **Estimates 3D Pose** for each tag using camera calibration and PnP:
   - Calculates rotation vector and translation vector
   - Broadcasts TF transforms with proper orientation
3. **Visualizes Results**:
- Draws bounding boxes around detected tags locally
- Publishes tag ID, corner coordinates, and distance as JSON for the web overlay
- Uses the live camera MJPEG stream plus JSON metadata instead of a separate boxed image stream
4. **Provides Real-time TF Data** for downstream tasks (navigation, manipulation, etc.)

## Launching

To start the perception pipeline:

```bash
ros2 launch lupin_perception perception.launch.py
```

This launches only the `tag_annotator` node.

## Configuration

Camera calibration is hard-coded in `tag_annotator.py`:
- **Camera Matrix (K)**: Intrinsic parameters (focal length, principal point)
- **Distortion Coefficients**: Currently set to zeros (no distortion model)
- **Tag Size**: 0.1 m (10 cm) - used for distance calculations

To customize for a different camera:
1. Run camera calibration to obtain the intrinsic matrix
2. Update `self.K` in `tag_annotator.py`
3. Update `self.dist_coeffs` if distortion correction is needed

The legacy `config/tags.yaml` file is preserved but no longer used.

## Architecture

### Detection Pipeline

```
Camera Frame
    ↓
Image Callback
    ↓
ArUco Detector (OpenCV)
    ↓
[For each detected tag]
  - Perspective-n-Point (PnP) Solver
  - 3D Pose Calculation
  - TF Transform Broadcast
    ↓
Visualization & Publication
```

### Output Frames

- **Tag transforms**: `<camera_frame> → tag_<id>`
  - Example: `camera_optical_frame → tag_0`
  - Provides 3D position and orientation of each AprilTag
  - Updated at image capture rate (~30 Hz in simulation)

## Current scope

- **AprilTag detection** and 3D pose estimation using OpenCV ArUco
- **Real-time visualization** with annotated video stream
- **TF broadcasting** for integration with navigation and manipulation
- Vision pipeline scaffolding for future flower detection and visual confirmation

## Why we switched from apriltag_ros

| Aspect | apriltag_ros | tag_annotator (OpenCV) |
|--------|-------------|----------------------|
| Dependencies | External ROS package | Built-in OpenCV |
| Configuration | YAML parameters | Python code (more flexible) |
| 3D Pose | May require additional nodes | Native PnP solver |
| Integration | Generic ROS approach | Tailored to Lupin needs |
| Maintainability | External dependency | In-house control |

## Debugging Tips

1. **Check published topics**:
   ```bash
   ros2 topic list | grep -E "(camera|tag)"
   ros2 topic echo /camera/tag_detections_json
   ```

2. **Verify TF transforms**:
   ```bash
   ros2 run tf2_tools view_frames
   ros2 topic echo /tf
   ```

3. **Enable ROS logging**:
   ```bash
   ros2 launch lupin_perception perception.launch.py --ros-args --log-level info
   ```

## Future work

- Add a `/perception/confirm_tag` service for mission gating
- Add flower detection and anomaly detection pipelines
- Improve runtime configuration and HMI integration
