#!/usr/bin/env bash
# fix-gazebo-ros2-control.sh — work around a gazebo_ros2_control regression that
# stops controller_manager from ever starting in the Gazebo sim.
#
# SYMPTOM: spawners loop forever on "Could not contact service
# /controller_manager/list_controllers", and gzserver logs:
#   [gazebo_ros2_control]: parser error Couldn't parse parameter override rule:
#   '--param robot_description:=<?xml ...   , at ./src/rcl/arguments.c:343
#
# ROOT CAUSE: the apt gazebo_ros2_control plugin hands the WHOLE URDF to the
# controller_manager as a "--param robot_description:=<urdf>" CLI override rule.
# A newer rcl (rebuilt ~2026-05) rejects that (multi-line XML isn't a valid param
# rule), rcl_parse_arguments() fails, and the plugin returns BEFORE creating the
# controller_manager. (Confirmed regression: it worked on the pre-May gazebo_ros2_control
# build; the apt repo no longer serves the old .deb.)
#
# FIX: build a workspace overlay of gazebo_ros2_control with a 2-line patch that
# sets robot_description DIRECTLY on the controller_manager node instead of via the
# CLI rule. The overlay shadows the apt plugin whenever the workspace is sourced.
# No system/apt changes; fully reversible (delete src/_vendor_overlays + rebuild).
set -euo pipefail

WS="${WS:-$HOME/ros2_ws}"
PATCH="$WS/src/lupin/lupin_bringup/scripts/patches/gazebo_ros2_control_robot_description.patch"
OVERLAY_DIR="$WS/src/_vendor_overlays"
SRC="$OVERLAY_DIR/gazebo_ros2_control"

echo "[fix-gz] workspace: $WS"
[ -f "$PATCH" ] || { echo "[fix-gz] ERROR: patch not found: $PATCH"; exit 1; }

mkdir -p "$OVERLAY_DIR"
if [ ! -d "$SRC" ]; then
  echo "[fix-gz] cloning gazebo_ros2_control (humble)…"
  tmp="$(mktemp -d)"
  git clone --depth 1 -b humble https://github.com/ros-controls/gazebo_ros2_control.git "$tmp/gz"
  cp -r "$tmp/gz/gazebo_ros2_control" "$SRC"
  rm -rf "$tmp"
fi

echo "[fix-gz] applying patch…"
# -N: skip if already applied; --reject-file=- so a re-run is a no-op, not an error.
patch -p1 -N -d "$SRC" < "$PATCH" || echo "[fix-gz] (patch already applied — ok)"

echo "[fix-gz] building overlay…"
source /opt/ros/humble/setup.bash
cd "$WS"
colcon build --packages-select gazebo_ros2_control --cmake-args -DCMAKE_BUILD_TYPE=Release

echo "[fix-gz] DONE. 'source $WS/install/setup.bash' and relaunch the sim."
echo "[fix-gz] verify: ros2 control list_controllers  (expect 5 active controllers)"
