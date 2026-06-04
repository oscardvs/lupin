# Future anomaly observation (e.g. fallen plant, unexpected obstruction).
# Stub now to lock the contract; nothing emits these yet.
#
# CONTRACTS: deliberate dead stub — no producer emits KIND_ANOMALY, and the
# twin drops it. Pests ride DiscoveredTag.anomaly / FlowerObservation.anomaly
# during SCANNING instead. See docs/CONTRACTS.md (#1).

geometry_msgs/PoseStamped pose
string description
uint8 severity                   # 0=info, increasing → more urgent (left open-ended on purpose)
