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
