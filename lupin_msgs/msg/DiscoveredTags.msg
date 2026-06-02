# Latched snapshot of every AprilTag discovered so far during exploration.
#
# Published by perception_aggregator on /perception/discovered_tags with
# RELIABLE + TRANSIENT_LOCAL (depth 1) so the orchestrator — which subscribes
# only after a mission starts — immediately sees the current registry. The
# orchestrator counts distinct tags for the discovery goal and reads
# pose_in_map for the monitoring approach poses.

std_msgs/Header header
DiscoveredTag[] tags
