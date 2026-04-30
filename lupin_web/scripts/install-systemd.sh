#!/usr/bin/env bash
# Install the lupin-web systemd unit on the robot. Idempotent — safe to re-run.
#
# Usage (on the robot):
#   sudo ./install-systemd.sh             # install + enable + start
#   sudo ./install-systemd.sh --uninstall # stop + disable + remove
#
# After install:
#   sudo systemctl status lupin-web
#   journalctl -u lupin-web -f
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
UNIT_SRC="$PKG_DIR/systemd/lupin-web.service"
UNIT_DST="/etc/systemd/system/lupin-web.service"

if [[ "$EUID" -ne 0 ]]; then
  echo "error: must be run as root (try: sudo $0 $*)" >&2
  exit 1
fi

if [[ "${1:-}" == "--uninstall" ]]; then
  systemctl stop lupin-web 2>/dev/null || true
  systemctl disable lupin-web 2>/dev/null || true
  rm -f "$UNIT_DST"
  systemctl daemon-reload
  echo "lupin-web service removed."
  exit 0
fi

if [[ ! -f "$UNIT_SRC" ]]; then
  echo "error: $UNIT_SRC not found" >&2
  exit 1
fi

# Cheap pre-flight: confirm the working dir referenced in the unit exists.
WD=$(awk -F= '/^WorkingDirectory=/{print $2}' "$UNIT_SRC")
if [[ ! -d "$WD" ]]; then
  echo "warning: WorkingDirectory '$WD' does not exist on this host." >&2
  echo "         Edit $UNIT_SRC if the repo lives elsewhere, then re-run." >&2
  exit 2
fi

if [[ ! -d "$WD/dist" ]]; then
  echo "warning: '$WD/dist' missing. Run 'npm run build' inside that directory" >&2
  echo "         before starting the service, or it will refuse to start." >&2
fi

install -m 0644 "$UNIT_SRC" "$UNIT_DST"
systemctl daemon-reload
systemctl enable lupin-web
systemctl restart lupin-web

echo
systemctl --no-pager --lines=0 status lupin-web || true
echo
echo "Tail logs with:  journalctl -u lupin-web -f"
echo "Web UI:          http://$(hostname -I | awk '{print $1}'):8090/"
