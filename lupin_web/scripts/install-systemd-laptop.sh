#!/usr/bin/env bash
# Install the lupin-web user-mode systemd unit on the operator's laptop.
# Idempotent — safe to re-run. Does NOT need sudo (runs as the user).
#
# Usage (on the laptop):
#   ./install-systemd-laptop.sh             # install + enable + start at login
#   ./install-systemd-laptop.sh --linger    # also boot-time start (recommended)
#   ./install-systemd-laptop.sh --uninstall # stop + disable + remove
#
# After install:
#   systemctl --user status lupin-web
#   journalctl --user -u lupin-web -f
#
# This is the laptop peer of install-systemd.sh (which installs the same
# unit on the robot via system-wide systemd). Keeping them split avoids
# bind-conflicts when both hosts run at once: the robot side should be
# uninstalled before installing this one if you're moving the HMI off the
# Pi.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
UNIT_SRC="$PKG_DIR/systemd/lupin-web-laptop.service"
UNIT_DST_DIR="${XDG_CONFIG_HOME:-$HOME/.config}/systemd/user"
UNIT_DST="$UNIT_DST_DIR/lupin-web.service"

if [[ "${1:-}" == "--uninstall" ]]; then
  systemctl --user stop lupin-web 2>/dev/null || true
  systemctl --user disable lupin-web 2>/dev/null || true
  rm -f "$UNIT_DST"
  systemctl --user daemon-reload
  echo "lupin-web user unit removed."
  echo "To also drop boot-time start: loginctl disable-linger \$USER"
  exit 0
fi

if [[ ! -f "$UNIT_SRC" ]]; then
  echo "error: $UNIT_SRC not found" >&2
  exit 1
fi

# Cheap pre-flight: confirm the working dir referenced in the unit exists.
WD="$HOME/ros2_ws/src/lupin/lupin_web/web"
if [[ ! -d "$WD" ]]; then
  echo "warning: WorkingDirectory '$WD' does not exist." >&2
  echo "         The laptop unit expects ~/ros2_ws/src/lupin. Move the repo or" >&2
  echo "         edit the unit, then re-run." >&2
  exit 2
fi

if [[ ! -d "$WD/dist" ]]; then
  echo "warning: '$WD/dist' missing — service will refuse to start." >&2
  echo "         Build it: (cd \"$WD\" && npm install && npm run build)" >&2
fi

mkdir -p "$UNIT_DST_DIR"
install -m 0644 "$UNIT_SRC" "$UNIT_DST"
systemctl --user daemon-reload
systemctl --user enable lupin-web
systemctl --user restart lupin-web

if [[ "${1:-}" == "--linger" ]]; then
  if loginctl show-user "$USER" 2>/dev/null | grep -q '^Linger=yes$'; then
    echo "Linger already enabled for $USER."
  else
    echo "Enabling linger (boot-time start without login) for $USER..."
    sudo loginctl enable-linger "$USER"
  fi
else
  if ! loginctl show-user "$USER" 2>/dev/null | grep -q '^Linger=yes$'; then
    echo
    echo "NOTE: the service starts at LOGIN. To also start it at boot before"
    echo "      any GUI login, re-run with --linger or:"
    echo "        sudo loginctl enable-linger $USER"
  fi
fi

echo
systemctl --user --no-pager --lines=0 status lupin-web || true
echo
echo "Tail logs with:  journalctl --user -u lupin-web -f"
HOST_IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "Web UI:          https://${HOST_IP:-localhost}:8090/"
