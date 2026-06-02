# Snapshot of one greenhouse tag from the digital-twin node's perspective.
#
# Emitted as a repeated field inside TwinState — the twin publishes the
# whole table at ~1 Hz so a freshly-connected HMI gets the world state
# without a separate bootstrap round-trip.

string tag_id

# Map-frame pose snapshotted when the tag was first scanned. v1: the
# orchestrator's AMCL pose at scan time (i.e. *the robot's standoff pose*,
# not the AprilTag's actual location — they're offset by the standoff
# distance). v2 hardware: the AprilTag's pose from `apriltag_ros`. The
# twin caches it from the first OK observation and doesn't re-stamp it,
# so the HMI marker doesn't jitter on AMCL drift.
#
# Orientation.w == 0 means "never observed" and the rest of the pose is
# meaningless — the HMI should not place a pin.
geometry_msgs/Pose pose

# Latest reading for each sensor type the bridge supplied at this tag.
# Order is whatever the bridge returned; consumers should index by name.
SensorReading[] readings

# Absolute ROS time of the most recent observation. This is the durable
# field that persists into FloraNova exports / on-disk twin snapshots.
# A late HMI subscriber reading the latched TwinState should diff this
# against `header.stamp` (or wall-clock) to derive freshness, NOT trust
# stale_seconds in isolation — that's a publish-time delta and only
# updates at the twin's 1 Hz tick.
builtin_interfaces/Time last_observed

# Convenience for HMI staleness fade and the "LAST SEEN" column in the
# Greenhouse State table. Computed at publish time as
# (now - last_observed). Equivalent to `header.stamp - last_observed` on
# the consumer side, but precomputed here so simple consumers don't have
# to do the time math.
float32 stale_seconds

# ─── Flower classification (co-located with this tag) ────────────────────
# Populated when the twin has ingested a KIND_FLOWER observation for this
# tag id (perception_aggregator fuses the YOLO tulip classifier with the
# AprilTag). Empty/false until then, so the HMI can show tags before a
# flower is classified.

# YOLO dominant flower class: "tulip_red" | "tulip_white" | "tulip_pink".
# Empty string when no flower has been classified at this tag.
string species

# Confidence [0, 1] for `species`. 0.0 when species is empty.
float32 species_confidence

# True when the YOLO "bug" anomaly class was seen at this tag. Latest-wins,
# so it clears on the next clean monitoring pass.
bool anomaly
