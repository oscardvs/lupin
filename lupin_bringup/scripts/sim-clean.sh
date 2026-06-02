#!/usr/bin/env bash
# sim-clean.sh — kill orphaned Gazebo/ROS sim processes and clear stale DDS
# shared memory before a fresh sim bringup. Gazebo classic notoriously leaves
# gzserver running after Ctrl+C, and a leftover rosbridge holds :9090 — both
# break the next launch (controller_manager never connects, "Address already in
# use"). Run this between runs.
set -u

echo "[sim-clean] killing sim processes…"
for pat in \
  gzserver gzclient \
  'robot_state_publisher' 'controller_manager' 'spawner' \
  'slam_toolbox' 'async_slam_toolbox_node' \
  'rosbridge_websocket' 'web_video_server' 'parameter_bridge' \
  'twist_mux' 'rviz2' \
  'mission_orchestrator' 'sim_flower_detector' 'tag_annotator' \
  'perception_aggregator' 'arm_sim_shim' 'arm_preset_server' \
  'gripper_action_bridge' 'greenhouse_bridge' 'twin_node' \
  'seed_amcl_pose' 'sim_battery_publisher' 'light_strip_bridge' ; do
  pkill -9 -f "$pat" 2>/dev/null
done

sleep 2

echo "[sim-clean] clearing stale FastDDS shared memory…"
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

# Report
alive=$(pgrep -x gzserver | wc -l)
port=$(ss -ltn 2>/dev/null | grep -c ':9090')
echo "[sim-clean] gzserver alive: ${alive} (want 0); :9090 listeners: ${port} (want 0)"
echo "[sim-clean] done."
