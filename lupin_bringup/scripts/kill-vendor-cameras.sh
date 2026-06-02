#!/usr/bin/env bash
# Kill the vendor camera nodes (usb_cam_node_exe in /gripper_camera and the
# orbbec component_container in /camera) so their /dev/video* and USB device
# handles are released. Used as ExecStartPre by lupin-cameras.service.
#
# The script waits up to WAIT_SEC for either vendor camera process to appear
# before signalling — mirte-ros.service spawns them several seconds after
# its own start, so racing it means our SIGTERM lands before they're even
# alive. After SIGTERM we poll for them to actually exit before returning,
# because cameras.launch.py will fail to open a busy V4L device.
#
# Idempotent: if no vendor cameras are running (someone already killed them,
# or the camera USB isn't plugged) the script returns cleanly.
set -euo pipefail

USB_PAT='usb_cam_node_exe.*/gripper_camera'
ORBBEC_PAT='component_container.*/camera'
WAIT_SEC="${LUPIN_KILL_VENDOR_WAIT_SEC:-30}"
DRAIN_SEC="${LUPIN_KILL_VENDOR_DRAIN_SEC:-5}"

log() { echo "[kill-vendor-cameras] $*" >&2; }

# Phase 1: wait up to WAIT_SEC for at least one vendor camera process to
# appear. We stop polling early once *both* are seen; otherwise we wait the
# full window and act on whatever we've got.
found_usb=0
found_orbbec=0
for _ in $(seq 1 "$WAIT_SEC"); do
  pgrep -f "$USB_PAT"    >/dev/null 2>&1 && found_usb=1    || true
  pgrep -f "$ORBBEC_PAT" >/dev/null 2>&1 && found_orbbec=1 || true
  if [ "$found_usb" = 1 ] && [ "$found_orbbec" = 1 ]; then
    break
  fi
  sleep 1
done
log "saw usb=$found_usb orbbec=$found_orbbec after wait"

# Phase 2: SIGTERM whatever's running. pkill returns nonzero when no match —
# that's fine, suppress it.
pkill -TERM -f "$USB_PAT"    2>/dev/null || true
pkill -TERM -f "$ORBBEC_PAT" 2>/dev/null || true

# Phase 3: drain — poll until both patterns have no live PIDs, up to
# DRAIN_SEC seconds. The orbbec container takes ~1–2 s to release its USB
# claim, which is the actual reason we wait.
for _ in $(seq 1 $((DRAIN_SEC * 4))); do
  if ! pgrep -f "$USB_PAT|$ORBBEC_PAT" >/dev/null 2>&1; then
    log "drained cleanly"
    exit 0
  fi
  sleep 0.25
done

# Phase 4: anything still alive gets SIGKILLed. The driver tends to comply
# with SIGTERM, so we rarely hit this — but if a USB hang leaves a process
# in D-state, SIGKILL frees the kernel device entry on the next /proc poll.
log "forcing SIGKILL on stragglers"
pkill -KILL -f "$USB_PAT"    2>/dev/null || true
pkill -KILL -f "$ORBBEC_PAT" 2>/dev/null || true
sleep 1
exit 0
