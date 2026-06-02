#!/usr/bin/env bash
# Install the lupin-cameras systemd unit on the robot. Idempotent.
#
# Replaces the v1 lupin-cameras-throttle service. Run with --uninstall first
# if upgrading from v1; this script will also stop+disable the legacy unit
# to avoid two services racing for the camera devices.
#
# Usage (on the robot):
#   sudo ./install-cameras-systemd.sh              # install + enable + start
#   sudo ./install-cameras-systemd.sh --uninstall  # stop + disable + remove
#
# After install:
#   sudo systemctl status lupin-cameras
#   journalctl -u lupin-cameras -f
#   ros2 topic list | grep -E '/camera/color|/gripper_camera'
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
UNIT_SRC="$PKG_DIR/systemd/lupin-cameras.service"
UNIT_DST="/etc/systemd/system/lupin-cameras.service"
SVC=lupin-cameras
LEGACY_SVC=lupin-cameras-throttle

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

# Sweep out the v1 service first. Leaving both running would have the
# legacy throttle subscribe to vendor topics that no longer exist (we kill
# them in ExecStartPre) — it'd respawn-loop and spam the journal.
if systemctl list-unit-files --no-legend --no-pager 2>/dev/null \
     | grep -q "^${LEGACY_SVC}\\.service "; then
  echo "Found legacy $LEGACY_SVC — disabling so v2 owns the cameras."
  systemctl stop  "$LEGACY_SVC" 2>/dev/null || true
  systemctl disable "$LEGACY_SVC" 2>/dev/null || true
  rm -f "/etc/systemd/system/${LEGACY_SVC}.service"
fi

# Pre-flight: usb_cam, orbbec_camera, and v4l-utils (for v4l2-ctl in our
# launch's device probe). usb_cam and orbbec_camera are vendor packages and
# should already be on the Mirte image; v4l-utils too. Warn but don't fail.
missing=()
for tool in v4l2-ctl pgrep pkill; do
  command -v "$tool" >/dev/null 2>&1 || missing+=("$tool")
done
if (( ${#missing[@]} > 0 )); then
  echo "warning: missing tools needed by lupin-cameras: ${missing[*]}" >&2
  echo "         apt install v4l-utils procps, then rerun." >&2
fi

# The kill helper needs to be executable on disk (we exec it from systemd).
chmod 0755 "$PKG_DIR/scripts/kill-vendor-cameras.sh" 2>/dev/null || true

install -m 0644 "$UNIT_SRC" "$UNIT_DST"
systemctl daemon-reload
systemctl enable "$SVC"
systemctl restart "$SVC"

echo
systemctl --no-pager --lines=0 status "$SVC" || true
echo
echo "Tail logs with:  journalctl -u $SVC -f"
echo "Verify topics:   ros2 topic list | grep -E '/camera/color|/gripper_camera'"
