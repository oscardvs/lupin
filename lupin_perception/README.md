# lupin_perception

`lupin_perception` is the vision package for Team Lupin. It currently provides a lightweight AprilTag annotation helper and is the home for future flower-detection pipelines.

## What it does

- Launches `apriltag_ros` to detect AprilTags in the front-facing camera stream.
- Runs the `tag_annotator` node, which subscribes to:
  - `/camera/image_raw`
  - `/detections`
- Publishes an annotated image stream on `/camera/image_raw_boxed`.

This is useful for debugging the camera-based AprilTag pipeline before the mission and bridge stacks are fully integrated.

## Launching

To start the perception pipeline:

```bash
ros2 launch lupin_perception perception.launch.py
```

That launch file starts:

- `apriltag_ros/apriltag_node`
- `lupin_perception/tag_annotator`

The node uses `lupin_perception/config/tags.yaml` for detector parameters.

## Current scope

- AprilTag detection overlay for the camera stream
- Vision pipeline scaffolding for future flower detection and visual confirmation

## Future work

- Add a `/perception/confirm_tag` service for mission gating
- Add flower detection and anomaly detection pipelines
- Improve runtime configuration and HMI integration
