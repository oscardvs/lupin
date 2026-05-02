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

# Map-frame pose at observation time. Populated by the orchestrator from the
# latest AMCL snapshot at PUBLISHING — meaningful only when the orchestrator
# had a localized pose (the start-of-mission gate guarantees this for live
# observations; transient_local replay may carry the seed pose). Consumers
# that don't need pose can ignore this field; consumers that do (the digital
# twin in particular) check the orientation.w field — when it's 0 the pose
# was never populated and the value should be treated as missing.
geometry_msgs/Pose tag_pose_in_map
