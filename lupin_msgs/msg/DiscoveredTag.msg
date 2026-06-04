# One AprilTag discovered live by perception during an ExplorationMission.
#
# Produced by lupin_perception's perception_aggregator, which projects the
# tag detector's per-tag TF into the map frame and fuses the YOLO flower
# classifier. Unlike tag_locations.json (a-priori), these are found at
# runtime by the live detector.

# AprilTag id as a string (matches the detector + greenhouse-bridge id space).
string tag_id

# Tag pose in the map frame (from solvePnP + TF lookup). This is the tag's
# actual location — the orchestrator derives a standoff approach pose from it
# (compute_discovered_approach), and the twin/HMI pin the flower here.
geometry_msgs/Pose pose_in_map

# YOLO dominant flower class co-located with this tag, or "" if none yet.
# CONTRACTS: species + anomaly (below) are producer-set but consumer-unread for
# logic — only mirrored into TwinTagState for HMI display, never used for
# routing/filtering. See docs/CONTRACTS.md (#2).
string species

# Confidence [0, 1] for `species`. 0.0 when species is empty.
float32 species_confidence

# True when the YOLO "bug" anomaly class was seen at this tag.
bool anomaly

# How many confident detections have been fused into this entry. Consumers
# can require sightings >= K before trusting the pose (debounce against a
# single noisy frame).
uint32 sightings
