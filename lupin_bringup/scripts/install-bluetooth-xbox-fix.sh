#!/usr/bin/env bash
# Apply the BlueZ workaround for the Xbox Wireless Controller (045e:0b13)
# reconnect-loop on this Orange Pi image.
#
# Symptom without the fix: the controller pairs over BLE/HoG, BlueZ reads the
# device's suggested MinInterval=6 (7.5 ms), the GATT HID descriptor read
# fails with `Request attribute has encountered an unlikely error`, and the
# controller loops Connected: yes ↔ no every ~4 s without ever creating
# /dev/input/event*. joy_node sees no device and /joy stays silent.
#
# Fix: pin BLE connection intervals to 8.75-11.25 ms — close to the
# controller's internal 100 Hz protocol — so the GATT handshake completes.
#
# Refs:
#   https://github.com/bluez/bluez/issues/155
#   https://discourse.nixos.org/t/xbox-controller-stuck-in-a-disconnect-reconnect-loop/67845
#
# Modes:
#   ./install-bluetooth-xbox-fix.sh --robot
#       Push the fix to mirte@192.168.42.1 (override with ROBOT=mirte@<ip>),
#       restart bluetooth.service, drop any stale LE-only bond so the next
#       reconnect uses the new intervals. Run from the laptop.
#
#   sudo ./install-bluetooth-xbox-fix.sh --local
#       Apply the fix to the local machine's /etc/bluetooth/main.conf. Use
#       this when running the script on the robot directly.
#
#   ./install-bluetooth-xbox-fix.sh --verify
#       Check the fix is present on the robot.
#
#   ./install-bluetooth-xbox-fix.sh --uninstall-robot
#       Remove the [LE] block from the robot's /etc/bluetooth/main.conf.
#
# Idempotent: re-running --robot or --local detects the marker comment and
# skips the append, only restarts bluetoothd if the file was modified.

set -euo pipefail

ROBOT="${ROBOT:-mirte@192.168.42.1}"
MARKER="# Lupin Xbox-controller BLE reconnect-loop fix"
MAIN_CONF="/etc/bluetooth/main.conf"
# Bluetooth controller MAC + paired pad MAC — only used to nuke a stale
# LE-only bond after applying the fix so the next reconnect re-pairs with
# the new intervals. If your robot has different MACs, run the script and
# then `bluetoothctl remove <mac>` manually.
BT_CTRL_MAC="40:FD:F3:24:72:65"
XBOX_MAC="44:16:22:BA:14:22"

usage() {
  sed -n '3,32p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
}

# Heredoc payload appended to main.conf. Indent-free so the [LE] header
# starts at column 0 — required by BlueZ's config parser.
read -r -d '' PAYLOAD <<'EOF' || true

# Lupin Xbox-controller BLE reconnect-loop fix
# Pin BLE connection intervals to 8.75-11.25 ms (controller's 100 Hz
# protocol). Without this BlueZ uses the controller's suggested
# MinInterval=6 (7.5 ms) and the GATT HID descriptor read fails with
# "Request attribute has encountered an unlikely error", causing a
# connect/disconnect storm and no /dev/input/event* getting created.
# Ref: https://github.com/bluez/bluez/issues/155
[LE]
MinConnectionInterval=7
MaxConnectionInterval=9
ConnectionLatency=0
EOF

apply_local() {
  if [[ "$EUID" -ne 0 ]]; then
    echo "error: --local must be run as root (try: sudo $0 --local)" >&2
    exit 1
  fi

  if [[ ! -f "$MAIN_CONF" ]]; then
    echo "error: $MAIN_CONF not found — is bluez installed?" >&2
    exit 1
  fi

  if grep -qF "$MARKER" "$MAIN_CONF"; then
    echo "  $MAIN_CONF already contains the Lupin Xbox fix — skipping append."
    return 0
  fi

  echo "  append [LE] connection-interval block to $MAIN_CONF"
  printf '%s\n' "$PAYLOAD" >> "$MAIN_CONF"

  echo "  restart bluetooth.service"
  systemctl restart bluetooth.service

  # Stale LE-only bond keeps the old 7.5 ms interval until re-paired. Drop
  # it so the next pair uses the new defaults.
  local bond_dir="/var/lib/bluetooth/$BT_CTRL_MAC/$XBOX_MAC"
  if [[ -d "$bond_dir" ]]; then
    echo "  remove stale bond $bond_dir (re-pair after running this)"
    rm -rf "$bond_dir"
  fi

  echo "✓ local fix applied. Power-cycle the controller and re-pair."
}

install_robot() {
  echo "=== install Xbox BLE fix on robot ($ROBOT) ==="
  if ! ssh -o ConnectTimeout=4 "$ROBOT" 'true' 2>/dev/null; then
    echo "  ERROR: cannot ssh to $ROBOT" >&2
    exit 1
  fi

  scp -q "${BASH_SOURCE[0]}" "$ROBOT":/tmp/install-bluetooth-xbox-fix.sh
  ssh "$ROBOT" 'chmod 0755 /tmp/install-bluetooth-xbox-fix.sh && sudo /tmp/install-bluetooth-xbox-fix.sh --local && rm /tmp/install-bluetooth-xbox-fix.sh'

  echo
  echo "✓ robot done. Re-pair the controller:"
  echo "    1. power controller fully off (hold Xbox button ~6s)"
  echo "    2. hold pair button (top, next to USB-C) ~3s — fast flash"
  echo "    3. ssh $ROBOT 'bluetoothctl scan on'  (in another shell)"
  echo "    4. ssh $ROBOT 'bluetoothctl pair $XBOX_MAC && \\"
  echo "                    bluetoothctl trust $XBOX_MAC && \\"
  echo "                    bluetoothctl connect $XBOX_MAC'"
  echo "    5. verify: ssh $ROBOT 'ls /dev/input/' shows event1 + js0"
}

verify() {
  echo "=== check fix on robot ($ROBOT) ==="
  if ssh -o ConnectTimeout=4 "$ROBOT" "grep -qF '$MARKER' $MAIN_CONF" 2>/dev/null; then
    echo "  ✓ [LE] fix present"
    ssh "$ROBOT" "awk '/^\[LE\]/,/^$/' $MAIN_CONF | sed 's/^/    /'"
  else
    echo "  ✗ fix NOT present — run --robot to install"
    exit 1
  fi
}

uninstall_robot() {
  echo "=== remove Xbox BLE fix from robot ($ROBOT) ==="
  # Strip the marker line through end-of-file. Belt-and-braces: keep a
  # backup so the operator can recover if BlueZ misbehaves afterwards.
  ssh "$ROBOT" "sudo cp $MAIN_CONF ${MAIN_CONF}.lupin-bak && \
    sudo sed -i '/$MARKER/,\$d' $MAIN_CONF && \
    sudo systemctl restart bluetooth.service"
  echo "✓ removed (backup at ${MAIN_CONF}.lupin-bak on robot)"
}

case "${1:-}" in
  --robot)           install_robot ;;
  --local)           apply_local ;;
  --verify)          verify ;;
  --uninstall-robot) uninstall_robot ;;
  *) usage; exit 1 ;;
esac
