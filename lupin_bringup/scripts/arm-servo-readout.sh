#!/usr/bin/env bash
# arm-servo-readout.sh — snapshot the five MIRTE Master arm servos.
#
# Prints, per joint: the raw Hiwonder count, the angle the message reports, and
# degrees-from-home computed straight from the raw count. The two degree columns
# are a wrap detector: if they DISAGREE, the HW-interface/encoder is wrapping at
# that position (we saw shoulder_pan read 511° past its stop) — trust raw_deg
# until `raw` itself jumps, then you're at the encoder seam = the usable edge.
#
# USE
#   source ~/ros2_ws/install/setup.bash && source ~/.config/lupin/ros-env.sh
#   ./arm-servo-readout.sh                 # one snapshot of all five
#   watch -n2 ./arm-servo-readout.sh       # repeating (slowish: ~0.5 s/joint)
# For a fast LIVE readout while hand-moving ONE joint, echo it natively instead:
#   ros2 topic echo /io/servo/hiwonder/shoulder_pan/position
#
# Read-only — never commands the arm. Safe to run any time.
set -u

# home_out (raw count at 0°) per joint, from mirte_master_config.yaml; deg=(raw-home)/100.
declare -A HOME=( [shoulder_pan]=12000 [shoulder_lift]=11450 [elbow]=11750 [wrist]=12200 [gripper]=10524 )
# Current CONFIGURED command window (deg) — what we clamp to today (arm_limits.py).
declare -A CFG=( [shoulder_pan]="-86..90" [shoulder_lift]="-86..85" [elbow]="-90..90" [wrist]="-90..88" [gripper]="-30..30" )
# Wider per-SERVO software limits from the vendor config (the reach we may unlock).
declare -A SERVO=( [shoulder_pan]="-86..90" [shoulder_lift]="-86..86" [elbow]="-116..93" [wrist]="-111..88" [gripper]="-44..37" )

printf "%-14s %8s %9s %9s   %-10s %-10s\n" joint raw msg_deg raw_deg cfg_clamp servo_sw
printf '%.0s─' {1..68}; echo
for j in shoulder_pan shoulder_lift elbow wrist gripper; do
  out=$(timeout 6 ros2 topic echo "/io/servo/hiwonder/$j/position" --once 2>/dev/null | grep -E "angle:|raw:")
  ang=$(awk '/angle:/{print $2}' <<<"$out"); raw=$(awk '/raw:/{print $2}' <<<"$out")
  if [ -z "${raw:-}" ]; then printf "%-14s %8s  (no data — DDS env sourced? robot up?)\n" "$j" "FAIL"; continue; fi
  read -r msg_deg raw_deg <<<"$(python3 -c "
a='${ang:-0}' or '0'; r=int('$raw'); h=${HOME[$j]}
print(f'{float(a)*57.2958:.1f} {(r-h)/100:.1f}')" 2>/dev/null)"
  printf "%-14s %8s %8s° %8s°   %-10s %-10s\n" "$j" "$raw" "${msg_deg:-?}" "${raw_deg:-?}" "${CFG[$j]}" "${SERVO[$j]}"
done