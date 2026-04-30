/**
 * Typed message contracts for the ROS topics this app subscribes to.
 * Names match the ROS 2 message field layout exactly so that JSON received
 * via rosbridge can be cast directly to these interfaces.
 */

export interface Time {
  sec: number
  nanosec: number
}

export interface Header {
  stamp: Time
  frame_id: string
}

export interface Vector3 {
  x: number
  y: number
  z: number
}

export interface Quaternion {
  x: number
  y: number
  z: number
  w: number
}

export interface Point {
  x: number
  y: number
  z: number
}

export interface Pose {
  position: Point
  orientation: Quaternion
}

export interface Twist {
  linear: Vector3
  angular: Vector3
}

export interface PoseWithCovariance {
  pose: Pose
  covariance: number[]
}

export interface TwistWithCovariance {
  twist: Twist
  covariance: number[]
}

export interface Odometry {
  header: Header
  child_frame_id: string
  pose: PoseWithCovariance
  twist: TwistWithCovariance
}

export interface Imu {
  header: Header
  orientation: Quaternion
  orientation_covariance: number[]
  angular_velocity: Vector3
  angular_velocity_covariance: number[]
  linear_acceleration: Vector3
  linear_acceleration_covariance: number[]
}

export interface LaserScan {
  header: Header
  angle_min: number
  angle_max: number
  angle_increment: number
  time_increment: number
  scan_time: number
  range_min: number
  range_max: number
  ranges: number[]
  intensities: number[]
}

export interface JointState {
  header: Header
  name: string[]
  position: number[]
  velocity: number[]
  effort: number[]
}

export interface BatteryState {
  header: Header
  voltage: number
  temperature: number
  current: number
  charge: number
  capacity: number
  design_capacity: number
  percentage: number
  power_supply_status: number
  power_supply_health: number
  power_supply_technology: number
  present: boolean
  cell_voltage: number[]
  cell_temperature: number[]
  location: string
  serial_number: string
}

export interface PoseStamped {
  header: Header
  pose: Pose
}

export interface Path {
  header: Header
  poses: PoseStamped[]
}

export interface MapMetaData {
  map_load_time: Time
  resolution: number
  width: number
  height: number
  origin: Pose
}

export interface OccupancyGrid {
  header: Header
  info: MapMetaData
  /**
   * Row-major occupancy values. -1 = unknown, 0 = free, 100 = occupied,
   * intermediate values = probability * 100.
   */
  data: number[]
}

export type RosoutLevel = 10 | 20 | 30 | 40 | 50

export interface Log {
  stamp: Time
  level: RosoutLevel
  name: string
  msg: string
  file: string
  function: string
  line: number
}

export const ROSOUT_LEVEL_NAMES: Record<RosoutLevel, string> = {
  10: 'DEBUG',
  20: 'INFO',
  30: 'WARN',
  40: 'ERROR',
  50: 'FATAL',
}

/** Hiwonder serial-bus servo position feedback, published per servo by mirte_telemetrix_cpp. */
export interface ServoPosition {
  header: Header
  /** Angle in radians. */
  angle: number
  /** Raw 12-bit servo count. */
  raw: number
}

/**
 * Mapping from a topic-default-name to its ROS 2 message type string,
 * for places where the type is the only thing that can disambiguate
 * subscriptions across mock/real.
 */
export const ROS_TYPE = {
  Twist: 'geometry_msgs/msg/Twist',
  Imu: 'sensor_msgs/msg/Imu',
  LaserScan: 'sensor_msgs/msg/LaserScan',
  Odometry: 'nav_msgs/msg/Odometry',
  JointState: 'sensor_msgs/msg/JointState',
  BatteryState: 'sensor_msgs/msg/BatteryState',
  Log: 'rcl_interfaces/msg/Log',
  OccupancyGrid: 'nav_msgs/msg/OccupancyGrid',
  Path: 'nav_msgs/msg/Path',
  PoseStamped: 'geometry_msgs/msg/PoseStamped',
  ServoPosition: 'mirte_msgs/msg/ServoPosition',
} as const

/** mirte_msgs service type strings. */
export const MIRTE_SRV = {
  SetServoAngleWithSpeed: 'mirte_msgs/srv/SetServoAngleWithSpeed',
  SetBool: 'std_srvs/srv/SetBool',
} as const

export interface SetServoAngleWithSpeedRequest {
  /** Target angle, interpreted in degrees when `degrees: true`. */
  angle: number
  /** Slew rate, deg/s when `degrees: true`. */
  rate: number
  degrees: boolean
}
export type RosTypeName = keyof typeof ROS_TYPE

/** Time helpers (Time → seconds, etc). */
export const timeToSec = (t: Time): number => t.sec + t.nanosec * 1e-9

export function quatToEuler(q: Quaternion): { roll: number; pitch: number; yaw: number } {
  // ZYX intrinsic — same convention as tf2.
  const sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
  const cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
  const roll = Math.atan2(sinr_cosp, cosr_cosp)

  const sinp = 2 * (q.w * q.y - q.z * q.x)
  const pitch = Math.abs(sinp) >= 1 ? Math.sign(sinp) * (Math.PI / 2) : Math.asin(sinp)

  const siny_cosp = 2 * (q.w * q.z + q.x * q.y)
  const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
  const yaw = Math.atan2(siny_cosp, cosy_cosp)

  return { roll, pitch, yaw }
}
