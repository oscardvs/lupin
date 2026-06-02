#!/usr/bin/env bash
# apply-sim-vendor-edits.sh — re-apply Lupin's edits to the (untracked) vendor
# mirte-ros-packages so the Gazebo sim works after a fresh vendor checkout.
#
# As of 2026-06-02 there is exactly ONE such edit:
#   * arm.xacro: add the gripper-camera Gazebo sensor (sim mirror of the real
#     Mirte usb_cam → /gripper_camera/image_raw) used by the per-pot arm 'inspect'
#     pose for the flower detector.
#
# What is NOT here (important): the sim drive design is the PRISTINE vendor one —
#   base_link <kinematic> + gazebo_ros_planar_move + frictionless wheels.
# An earlier attempt deleted planar_move (to chase real ros2_control wheels); that
# broke driving (hover) and triggered an Ogre AABB crash, so it was reverted. Do
# NOT re-delete planar_move without also making base_link dynamic AND giving the
# wheels mecanum friction — see lupin/docs and project_p3d_urdf_limitation.
#
# Idempotent: safe to run repeatedly. Reversible: `git -C <vendor> checkout -- <file>`.
set -euo pipefail

WS="${WS:-$HOME/ros2_ws}"
VENDOR="$WS/src/mirte-ros-packages"
ARM="$VENDOR/mirte_description/mirte_master_description/urdf/arm.xacro"
PATCH="$WS/src/lupin/lupin_bringup/scripts/patches/sim_gripper_camera_arm_xacro.patch"

echo "[vendor-edits] workspace: $WS"
[ -d "$VENDOR/.git" ] || { echo "[vendor-edits] ERROR: vendor repo not found at $VENDOR"; exit 1; }
[ -f "$PATCH" ]       || { echo "[vendor-edits] ERROR: patch not found: $PATCH"; exit 1; }
[ -f "$ARM" ]         || { echo "[vendor-edits] ERROR: arm.xacro not found: $ARM"; exit 1; }

if grep -q "gripper_camera_link" "$ARM"; then
  echo "[vendor-edits] gripper camera already present in arm.xacro — nothing to do."
  exit 0
fi

echo "[vendor-edits] applying gripper-camera patch to arm.xacro…"
if git -C "$VENDOR" apply --check "$PATCH" 2>/dev/null; then
  git -C "$VENDOR" apply "$PATCH"
else
  echo "[vendor-edits] git apply --check failed (vendor arm.xacro may have moved upstream)."
  echo "[vendor-edits] Re-create the patch from a known-good tree, or apply by hand. See $PATCH"
  exit 1
fi

grep -q "gripper_camera_link" "$ARM" \
  && echo "[vendor-edits] OK: gripper camera applied. Rebuild is NOT needed (install/ symlinks the vendor urdf)." \
  || { echo "[vendor-edits] ERROR: patch applied but gripper_camera_link still missing"; exit 1; }
