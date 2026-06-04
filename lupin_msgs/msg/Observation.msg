# Polymorphic envelope for anything the mission emits about the world.
#
# Future-proofing: new observation kinds (flowers, anomalies, ...) are added
# by appending a constant + a typed field. Consumers MUST switch on `kind`
# and ignore unknown kinds rather than failing — that's how this stays
# non-breaking as the orchestrator grows.

std_msgs/Header header
string mission_id
string source                    # e.g. "InspectionMission", future: "MappingMission"

uint8 KIND_TAG_READING=0
uint8 KIND_FLOWER=1              # stubbed, not populated this MR
uint8 KIND_ANOMALY=2             # stubbed, not populated this MR
uint8 kind

uint8 STATUS_OK=0
uint8 STATUS_UNREACHABLE=1
uint8 STATUS_SCAN_FAILED=2
uint8 STATUS_SKIPPED=3
uint8 status
string status_detail             # free-text reason on non-OK; empty on OK

# At most one of these is populated per message — selected by `kind`.
# The others are default-zeroed.
TagReading tag_reading
FlowerObservation flower
AnomalyReport anomaly

# Map-frame pose of the tag at observation time. The orchestrator prefers the
# tag's OWN map pose (from the discovered-tags feed, which perception_aggregator
# resolves via TF) so consumers pin the tag where it physically is; it falls
# back to the latest AMCL snapshot (the robot's standoff pose, offset from the
# tag) only when the tag's own pose is unknown at scan time. Populated only for
# STATUS_OK. Consumers that don't need pose can ignore this field; consumers
# that do (the digital twin in particular) check the orientation.w field — when
# it's 0 the pose was never populated and the value should be treated as missing.
geometry_msgs/Pose tag_pose_in_map
