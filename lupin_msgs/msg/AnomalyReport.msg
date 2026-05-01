# Future anomaly observation (e.g. fallen plant, unexpected obstruction).
# Stub now to lock the contract; nothing emits these yet.

geometry_msgs/PoseStamped pose
string description
uint8 severity                   # 0=info, increasing → more urgent (left open-ended on purpose)
