#!/usr/bin/env bash
# Install the lupin-gripper-bridge systemd unit on the robot. Idempotent.
#
# Usage (on the robot):
#   sudo ./install-systemd.sh             # install + enable + start
#   sudo ./install-systemd.sh --uninstall # stop + disable + remove
#
# Prereq: lupin_hmi must already be colcon-built into ~/ros2_ws/install
# (so `ros2 run lupin_hmi gripper_action_bridge` resolves).
#
# After install:
#   sudo systemctl status lupin-gripper-bridge
#   journalctl -u lupin-gripper-bridge -f
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
UNIT_SRC="$PKG_DIR/systemd/lupin-gripper-bridge.service"
UNIT_DST="/etc/systemd/system/lupin-gripper-bridge.service"

if [[ "$EUID" -ne 0 ]]; then
  echo "error: must be run as root (try: sudo $0 $*)" >&2
  exit 1
fi

if [[ "${1:-}" == "--uninstall" ]]; then
  systemctl stop lupin-gripper-bridge 2>/dev/null || true
  systemctl disable lupin-gripper-bridge 2>/dev/null || true
  rm -f "$UNIT_DST"
  systemctl daemon-reload
  echo "lupin-gripper-bridge service removed."
  exit 0
fi

if [[ ! -f "$UNIT_SRC" ]]; then
  echo "error: $UNIT_SRC not found" >&2
  exit 1
fi

# Pre-flight: confirm the workspace overlay actually has the executable so
# the unit doesn't fail to start with a confusing "ros2 run: no executable"
# error. The unit sources /home/mirte/ros2_ws/install — if the overlay
# isn't built yet, ros2 won't find lupin_hmi.
WS_SETUP="/home/mirte/ros2_ws/install/setup.bash"
if [[ ! -f "$WS_SETUP" ]]; then
  echo "warning: $WS_SETUP missing — colcon-build the workspace first:" >&2
  echo "         cd /home/mirte/ros2_ws && colcon build --packages-select lupin_hmi" >&2
fi

install -m 0644 "$UNIT_SRC" "$UNIT_DST"
systemctl daemon-reload
systemctl enable lupin-gripper-bridge
systemctl restart lupin-gripper-bridge

echo
systemctl --no-pager --lines=0 status lupin-gripper-bridge || true
echo
echo "Tail logs with:  journalctl -u lupin-gripper-bridge -f"
