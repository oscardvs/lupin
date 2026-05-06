#!/usr/bin/env bash
# Install the lupin-cameras-throttle systemd unit on the robot. Idempotent.
#
# Usage (on the robot):
#   sudo ./install-cameras-throttle-systemd.sh             # install + enable + start
#   sudo ./install-cameras-throttle-systemd.sh --uninstall # stop + disable + remove
#
# After install:
#   sudo systemctl status lupin-cameras-throttle
#   journalctl -u lupin-cameras-throttle -f
#   ros2 topic list | grep /lupin/camera
#
# To change rates: edit lupin_bringup/config/cameras.yaml in the source tree,
# `colcon build --packages-select lupin_bringup`, then
# `sudo systemctl restart lupin-cameras-throttle`.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
UNIT_SRC="$PKG_DIR/systemd/lupin-cameras-throttle.service"
UNIT_DST="/etc/systemd/system/lupin-cameras-throttle.service"
SVC=lupin-cameras-throttle

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

# Pre-flight: confirm topic_tools is actually installed (some Humble images
# strip it). Without this the launch comes up but every throttle node fails
# with "executable not found", which is annoying to debug from the journal.
if ! ros2 pkg executables topic_tools 2>/dev/null | grep -q '^topic_tools throttle$' \
     && ! [[ -x /opt/ros/humble/lib/topic_tools/throttle ]]; then
  echo "warning: topic_tools/throttle not found on this image." >&2
  echo "         apt install ros-humble-topic-tools, then rerun." >&2
fi

install -m 0644 "$UNIT_SRC" "$UNIT_DST"
systemctl daemon-reload
systemctl enable "$SVC"
systemctl restart "$SVC"

echo
systemctl --no-pager --lines=0 status "$SVC" || true
echo
echo "Tail logs with:  journalctl -u $SVC -f"
echo "Verify topics:   ros2 topic list | grep /lupin/camera"
