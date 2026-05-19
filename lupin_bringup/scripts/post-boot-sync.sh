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
LAPTOP_UTC=$(date -u +%s)
ssh "$ROBOT" "sudo date -s @$LAPTOP_UTC" >/dev/null
echo "  set to $(date -u -d @$LAPTOP_UTC)"

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
echo "=== controllers (should show 5 active) ==="
# The "rcl node's context is invalid" error here is a ros2cli bug, not a real
# failure — controllers themselves work fine. See DEMO_DAY.md section 1.
ssh "$ROBOT" 'timeout 12 ros2 control list_controllers 2>&1 | head -8'

echo
echo "ready to launch: ros2 launch lupin_bringup hardware.launch.py"
