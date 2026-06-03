#!/usr/bin/env bash
# Run on the laptop right after the robot finishes booting (~90s after power-on).
# Syncs the robot's clock to the laptop's UTC (since the Orange Pi has no RTC
# and in AP mode has no NTP source), then sanity-checks the discovery server +
# Lupin systemd stack.
#
# Per-teammate setup (one-time):
#   mkdir -p ~/.config/lupin
#   ln -s "$(ros2 pkg prefix lupin_bringup)/share/lupin_bringup/scripts/post-boot-sync.sh" \
#         ~/.config/lupin/post-boot-sync.sh
# After that, the canonical demo-day path is `~/.config/lupin/post-boot-sync.sh`
# (referenced in DEMO_DAY.md). Symlinked so a future `colcon build` of
# lupin_bringup auto-updates the script without re-installing.
#
# Override ROBOT=mirte@<ip> to point at a different robot. The default matches
# Mirte-247264's AP (see project_robot_deployment_state).

set -e
ROBOT="${ROBOT:-mirte@192.168.42.1}"

echo "=== sync clock laptop → robot ==="
# CRITICAL: `date -s` on a LIVE ros2_control stack wedges the controllers. The
# step leaves stale FastDDS SHM (/dev/shm/fastrtps_*) that survives a plain
# `systemctl restart` and jams the new controller_manager (load/list service
# timeouts -> activation spawner SIGSEGV `exit code -11` -> controller_state
# silent -> wheels dead). See project_robot_clock_skew + lupin-fastdds-shm-
# contention. So only step the clock with the stack STOPPED, and only when the
# drift is real; for small drift leave the stack running (no gratuitous restart
# when this script is re-run mid-session as a health check).
ROBOT_UTC=$(ssh "$ROBOT" 'date -u +%s')
DRIFT=$(( $(date -u +%s) - ROBOT_UTC )); DRIFT=${DRIFT#-}
echo "  drift before sync: ${DRIFT}s"
if [ "$DRIFT" -le 3 ]; then
  echo "  within 3s -> no clock step needed (ros2_control left running)"
else
  echo "  stepping ${DRIFT}s with ros2_control STOPPED (avoids the live-stack wedge)..."
  ssh "$ROBOT" "sudo systemctl stop lupin-onboard lupin-cameras mirte-ros \
    && sudo date -s @$(date -u +%s) \
    && sudo rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* \
    && sudo systemctl start mirte-ros"
  echo "  set to $(date -u)  -- waiting 25s for controllers to re-activate..."
  sleep 25
  ssh "$ROBOT" "sudo systemctl start lupin-onboard lupin-cameras"
fi

sleep 1
LAPTOP=$(date +%s) ; ROBOT_T=$(ssh "$ROBOT" 'date +%s')
echo "  drift after sync: $((LAPTOP - ROBOT_T))s   (should be 0 or 1)"

echo
echo "=== discovery server alive? ==="
ssh "$ROBOT" 'sudo ss -lnup | grep -E "11811" || echo NOT-LISTENING'

echo
echo "=== robot service stack ==="
ssh "$ROBOT" 'for s in mirte-ros lupin-onboard lupin-cameras; do printf "  %-25s %s\n" "$s" "$(systemctl is-active $s)"; done'

echo
echo "=== ensure lupin-auto-home ran this session ==="
# A previous boot's failure leaves the oneshot in `failed`, and systemd
# does NOT auto-retry it on the next boot (and does not fire it at all on
# resume-from-suspend, where the boot ID doesn't change). Symptom: 3 of
# the 5 controllers stuck `unconfigured`, the wheel + gripper controllers
# never loaded, and the arm never moves to its low-load home pose.
# Idempotent: `reset-failed` is a no-op on non-failed units; `start` is a
# no-op on a oneshot already `active (exited)`.
ssh "$ROBOT" 'sudo systemctl reset-failed lupin-auto-home 2>/dev/null; sudo systemctl start lupin-auto-home'
echo "  $(ssh "$ROBOT" 'systemctl is-active lupin-auto-home')"

echo
echo "=== controllers (should show 5 active) ==="
# The "rcl node's context is invalid" error here is a ros2cli bug, not a real
# failure — controllers themselves work fine. See DEMO_DAY.md section 1.
ssh "$ROBOT" 'timeout 12 ros2 control list_controllers 2>&1 | head -8'

echo
echo "ready to launch: ros2 launch lupin_bringup hardware.launch.py"
