# Flower-detection observation, produced by lupin_perception's
# perception_aggregator when it co-locates a YOLO flower classification with
# a discovered AprilTag. Carried inside Observation (kind == KIND_FLOWER).

# The AprilTag this flower is co-located with. The twin keys its per-tag
# state by this id so species/anomaly merge onto the same map pin as the
# tag's sensor readings. Empty only for a flower seen with no nearby tag
# (not produced in v1).
string tag_id

# Map-frame pose of the co-located tag (header.frame_id == "map"). v1: the
# AprilTag's pose from the detector's solvePnP + TF lookup — i.e. the plant's
# actual location, not the robot's standoff pose.
geometry_msgs/PoseStamped pose

# YOLO class name: "tulip_red" | "tulip_white" | "tulip_pink". Empty when no
# flower was classified at this tag (e.g. only the "bug" anomaly was seen).
string species

# Classification confidence [0, 1] for `species`.
float32 confidence

# True when the YOLO "bug" anomaly class was co-located with this tag. The
# twin copies this onto TwinTagState.anomaly so the HMI can flag pests. Kept
# on the flower path (not AnomalyReport) because it's keyed by the same
# tag_id the twin merges on; standalone anomalies still use KIND_ANOMALY.
bool anomaly
