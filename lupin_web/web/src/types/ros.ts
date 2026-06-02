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
  MissionState: 'lupin_msgs/msg/MissionState',
  Observation: 'lupin_msgs/msg/Observation',
  TwinState: 'lupin_msgs/msg/TwinState',
} as const

/** mirte_msgs service type strings. */
export const MIRTE_SRV = {
  SetServoAngleWithSpeed: 'mirte_msgs/srv/SetServoAngleWithSpeed',
  SetBool: 'std_srvs/srv/SetBool',
  SetNeopixel: 'mirte_msgs/srv/SetNeopixel',
} as const

/** Status-light bridge service endpoints (lupin_hmi/light_strip_bridge).
 * `setManual` holds a manual whole-strip colour and stops the bridge
 * following mission state; `setAuto` hands colouring back to the FSM. */
export const LED_SERVICE = {
  setManual: '/lupin/leds/set',
  setAuto: '/lupin/leds/auto',
} as const

/** Mirror of `mirte_msgs/msg/NeopixelColor` — whole-strip RGB, 0–255/channel. */
export interface NeopixelColor {
  r: number
  g: number
  b: number
}

/** Mirror of `mirte_msgs/srv/SetNeopixel`. */
export interface SetNeopixelRequest {
  color: NeopixelColor
}

export interface SetNeopixelResponse {
  status: boolean
}

/** lupin_msgs service type strings. */
export const LUPIN_SRV = {
  StartMission: 'lupin_msgs/srv/StartMission',
  Trigger: 'std_srvs/srv/Trigger',
  GetField: 'lupin_msgs/srv/GetField',
  CalibrateArm: 'lupin_msgs/srv/CalibrateArm',
  SetArmPreset: 'lupin_msgs/srv/SetArmPreset',
} as const

/** Mirror of `lupin_msgs/srv/CalibrateArm`. The single-srv-with-action
 * shape (vs three separate srvs) mirrors the operator wizard 1:1: start
 * disables servos, operator hand-poses the arm, commit samples positions
 * and writes Hiwonder zero offsets, cancel re-enables without writing. */
export type CalibrateArmAction = 'start' | 'commit' | 'cancel' | 'status'
export type CalibrateArmState = 'IDLE' | 'AWAITING_POSE'

export interface CalibrateArmRequest {
  action: CalibrateArmAction
}

export interface CalibrateArmResponse {
  success: boolean
  state: CalibrateArmState | ''
  message: string
  /** Servo names in server-side order; populated only on commit. */
  joint_names: string[]
  /** Centidegrees actually written via _set_offset (per joint). */
  offsets_applied: number[]
  /** Raw-tick diff (position - home + curr_offset). */
  diffs_observed: number[]
}

/** Sensor channels the digital twin understands. Order is the canonical
 * sensor-pill order in the HMI header. */
export const TWIN_SENSORS = ['temperature', 'humidity', 'co2', 'light', 'soil_moisture'] as const
export type TwinSensor = (typeof TWIN_SENSORS)[number]

/** Mirror of `lupin_msgs/msg/TwinTagState`. orientation.w === 0 means
 * "no pose observed yet" — the HMI must not render a pin for that tag.
 *
 * Two timestamps for two consumers:
 *   - `last_observed` is the absolute ROS time of the last observation,
 *     durable across export/replay; use this for any persistence.
 *   - `stale_seconds` is a publish-time-relative convenience for the HMI
 *     fade animation and the LAST SEEN column. */
export interface TwinTagState {
  tag_id: string
  pose: Pose
  readings: SensorReading[]
  last_observed: Time
  stale_seconds: number
  /** YOLO flower class co-located with this tag ("tulip_red"|"tulip_white"|
   * "tulip_pink"), "" until perception classifies one. */
  species: string
  /** Confidence [0,1] for `species`; 0 when species is "". */
  species_confidence: number
  /** True when the YOLO "bug" anomaly was seen at this tag. */
  anomaly: boolean
}

/** Mirror of `lupin_msgs/msg/TwinState`. */
export interface TwinState {
  header: Header
  tags: TwinTagState[]
}

/** Mirror of `lupin_msgs/srv/GetField` request/response. */
export interface GetFieldRequest {
  sensor_type: TwinSensor | string
  resolution: number
  bbox_min_x: number
  bbox_min_y: number
  bbox_max_x: number
  bbox_max_y: number
}

export interface GetFieldResponse {
  ok: boolean
  error_message: string
  /** Row-major width*height. NaN cells are encoded as nulls by rosbridge —
   * the hook normalises those back to NaN before consumers see them. */
  values: (number | null)[]
  width: number
  height: number
  origin_x: number
  origin_y: number
  resolution_used: number
  value_min: number
  value_max: number
  sample_count: number
}

/**
 * Mirror of `lupin_msgs/msg/MissionState`. Lifecycle is the top-level FSM the
 * orchestrator advertises; `mission_phase` is the sub-state inside INSPECTING
 * and PREPARE (empty otherwise).
 */
export type MissionLifecycleState =
  | 'BOOT' | 'READY' | 'PREPARE' | 'EXPLORING' | 'INSPECTING' | 'MONITORING'
  | 'RETURNING' | 'DONE' | 'FAULT'

export interface MissionState {
  header: Header
  mission_id: string
  mission_type: string
  lifecycle_state: MissionLifecycleState
  mission_phase: string                  // "" | "LOCALIZING" | "NAVIGATING" | "SCANNING" | "PUBLISHING"
  current_target: string                 // tag_id, "" otherwise
  targets_total: number
  targets_completed: number
  targets_failed: number
  targets_unreachable: number
  targets_skipped: number
  /** ExplorationMission: distinct tags discovered so far, and the goal N.
   * Both 0 for an InspectionMission. */
  tags_discovered: number
  discovery_goal: number
  last_error: string
  estop_engaged: boolean
  paused: boolean
  started_at: Time                       // zero when no mission has run
}

/** Mirror of `lupin_msgs/msg/SensorReading`. */
export interface SensorReading {
  name: string
  value: number
}

/** Mirror of `lupin_msgs/msg/TagReading`. */
export interface TagReading {
  tag_id: string
  stamp: Time
  sim_time_of_day_seconds: number
  readings: SensorReading[]
}

/** Status enum from `Observation.msg`. Numeric on the wire. */
export const OBSERVATION_STATUS = {
  OK: 0,
  UNREACHABLE: 1,
  SCAN_FAILED: 2,
  SKIPPED: 3,
} as const
export type ObservationStatus = (typeof OBSERVATION_STATUS)[keyof typeof OBSERVATION_STATUS]

/** Kind enum from `Observation.msg`. */
export const OBSERVATION_KIND = {
  TAG_READING: 0,
  FLOWER: 1,
  ANOMALY: 2,
} as const
export type ObservationKind = (typeof OBSERVATION_KIND)[keyof typeof OBSERVATION_KIND]

/** Mirror of `lupin_msgs/msg/FlowerObservation`. Produced by the perception
 * aggregator and carried inside an Observation with `kind === FLOWER`. */
export interface FlowerObservation {
  tag_id: string
  pose: PoseStamped
  species: string
  confidence: number
  anomaly: boolean
}

/**
 * Mirror of `lupin_msgs/msg/Observation`. `tag_reading` is meaningful only when
 * `kind === OBSERVATION_KIND.TAG_READING`; `flower` only when
 * `kind === OBSERVATION_KIND.FLOWER`. `anomaly` (standalone KIND_ANOMALY) is
 * still unused — typed `unknown` to avoid pretending it exists.
 */
export interface Observation {
  header: Header
  mission_id: string
  source: string
  kind: ObservationKind
  status: ObservationStatus
  status_detail: string
  tag_reading: TagReading
  flower: FlowerObservation
  anomaly: unknown
}

/** Flower species → display palette. Keep in sync with best.pt classes. */
export const FLOWER_SPECIES = ['tulip_red', 'tulip_white', 'tulip_pink'] as const
export type FlowerSpecies = (typeof FLOWER_SPECIES)[number]

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

// ── AprilTag overlay (HMI Cameras view) ────────────────────────────────
/** `std_msgs/String`. Only `data` is meaningful for our consumer; the
 * AprilTag node packs a JSON array into it (see `TagDetection`). */
export interface StdMsgsString {
  data: string
}

/** One tag in the JSON payload published by `lupin_perception/tag_annotator`
 * on `/camera/tag_detections_json`. Pixel coordinates are in the original
 * (un-resized) image frame; the HMI's overlay canvas is sized to match. */
export interface TagDetection {
  id: number
  corners: [[number, number], [number, number], [number, number], [number, number]]
  dist: number
}
