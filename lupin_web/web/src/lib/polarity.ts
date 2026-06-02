import type { Twist } from '@/types/ros'

/**
 * Mirte-247264 has its mecanum drive wired with motor + encoder leads
 * physically reversed in pairs. The vendor stack is internally consistent
 * (cmd_vel ↔ odom ↔ lidar TF all sit in the same frame) but that internal
 * frame is rotated 180° from the physical chassis. Vendor-side fixes (negative
 * `wheels_radius`, `motor.inverted`) tripped firmware validation paths and
 * broke the boot sequence — see `project_hardware_axis_inversion`.
 *
 * Cleanest patch: invert at the human-facing surface only. Joystick output,
 * voice "drive forward" / "nav_forward", and the map-click → goal pipeline
 * each apply this transform when `polarityInvertHmi` is true. Nav2 / SLAM /
 * AMCL stay in their internally-consistent frame and converge normally.
 */
export function invertTwist(t: Twist, enable: boolean): Twist {
  if (!enable) return t
  return {
    linear: { x: -t.linear.x, y: -t.linear.y, z: t.linear.z },
    angular: { x: t.angular.x, y: t.angular.y, z: -t.angular.z },
  }
}

/** Same 180°-about-Z transform applied to (x, y) and yaw, used for map-frame
 *  goal poses sent from a rotated-display click. */
export function invertMapPose(
  x: number,
  y: number,
  yaw: number,
  enable: boolean,
): { x: number; y: number; yaw: number } {
  if (!enable) return { x, y, yaw }
  return { x: -x, y: -y, yaw: yaw + Math.PI }
}
