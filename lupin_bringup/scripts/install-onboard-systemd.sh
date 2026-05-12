#!/usr/bin/env bash
# Install the lupin-onboard systemd unit on the robot. Idempotent.
#
# This unit runs twist_mux + arm_preset_server + gripper_action_bridge so the
# HMI can drive the robot, move the arm, and operate the gripper as soon as
# the robot finishes booting — no laptop launch required.
#
# Replaces the older single-purpose lupin-gripper-bridge.service if it's
# present. The new onboard launch covers gripper plus the twist_mux and arm
# preset server that used to live in hardware.launch.py.
#
# Usage (on the robot):
#   sudo ./install-onboard-systemd.sh             # install + enable + start
#   sudo ./install-onboard-systemd.sh --uninstall # stop + disable + remove
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
UNIT_SRC="$PKG_DIR/systemd/lupin-onboard.service"
UNIT_DST="/etc/systemd/system/lupin-onboard.service"
SVC=lupin-onboard
LEGACY=lupin-gripper-bridge

if [[ "$EUID" -ne 0 ]]; then
  echo "error: must be run as root (try: sudo $0 $*)" >&2
  exit 1
fi

if [[ "${1:-}" == "--uninstall" ]]; then
  systemctl stop "$SVC" 2>/dev/null || true
  systemctl disable "$SVC" 2>/dev/null || true
  rm -f "$UNIT_DST"
  systemctl daemon-reload
  echo "$SVC service removed."
  exit 0
fi

if [[ ! -f "$UNIT_SRC" ]]; then
  echo "error: $UNIT_SRC not found" >&2
  exit 1
fi

# joy_node opens the Xbox controller via SDL2's evdev backend, which reads
# /dev/input/event* — those nodes are 0660 root:input on the stock Mirte
# image. Without group membership the controller pairs and /dev/input/js0
# even shows up, but joy_node can't open the matching event device and
# silently never publishes /joy. Idempotent (usermod no-ops if already a
# member); reflashed images need this rerun.
if id -nG mirte 2>/dev/null | grep -qw input; then
  echo "info: user 'mirte' already in 'input' group"
else
  echo "info: adding 'mirte' to 'input' group (Xbox controller via SDL2/evdev)"
  usermod -aG input mirte
  echo "      group change takes effect on next service start — done below."
fi

# Retire the legacy single-bridge unit if it's installed — onboard supersedes it.
if systemctl list-unit-files "${LEGACY}.service" 2>/dev/null | grep -q "$LEGACY"; then
  echo "info: superseding $LEGACY.service with $SVC.service"
  systemctl stop "$LEGACY" 2>/dev/null || true
  systemctl disable "$LEGACY" 2>/dev/null || true
  rm -f "/etc/systemd/system/${LEGACY}.service"
fi

# Pre-flight: confirm the lupin_hmi executables exist; otherwise the service
# will boot-loop with cryptic 'package not found' messages. Best-effort —
# sudo strips the user's environment so we have to source ROS ourselves to
# even reach `ros2`. If sourcing fails we just skip the check.
if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash 2>/dev/null || true
  source /home/mirte/ros2_ws/install/setup.bash 2>/dev/null || true
  set -u
fi
if command -v ros2 >/dev/null; then
  for exe in arm_preset_server gripper_action_bridge; do
    if ! ros2 pkg executables lupin_hmi 2>/dev/null | grep -q "lupin_hmi $exe"; then
      echo "warning: lupin_hmi $exe not found on this image." >&2
      echo "         Did you 'colcon build --packages-up-to lupin_hmi' on the robot?" >&2
    fi
  done
fi

install -m 0644 "$UNIT_SRC" "$UNIT_DST"
systemctl daemon-reload
systemctl enable "$SVC"
systemctl restart "$SVC"

echo
systemctl --no-pager --lines=0 status "$SVC" || true
echo
echo "Tail logs with:  journalctl -u $SVC -f"
echo "Verify topics:   ros2 topic list | grep -E '(cmd_vel|/lupin/(arm|gripper))'"
