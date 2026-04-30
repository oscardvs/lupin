import {
  type BatteryState,
  type Imu,
  type JointState,
  type LaserScan,
  type Log,
  type Odometry,
  type RosoutLevel,
} from '@/types/ros'

/** Synthetic data generators for offline / demo development. */

const t0 = Date.now()
const elapsed = () => (Date.now() - t0) / 1000

function makeStamp() {
  const now = Date.now()
  return { sec: Math.floor(now / 1000), nanosec: (now % 1000) * 1e6 }
}
function makeHeader(frame_id = 'base_link') {
  return { stamp: makeStamp(), frame_id }
}

export function mockImu(): Imu {
  const t = elapsed()
  // gentle yaw oscillation
  const yaw = Math.sin(t * 0.4) * 0.3
  const half = yaw / 2
  return {
    header: makeHeader('imu_link'),
    orientation: { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) },
    orientation_covariance: new Array(9).fill(0),
    angular_velocity: { x: 0, y: 0, z: Math.cos(t * 0.4) * 0.4 * 0.3 },
    angular_velocity_covariance: new Array(9).fill(0),
    linear_acceleration: { x: 0, y: 0, z: 9.81 + Math.sin(t * 2) * 0.05 },
    linear_acceleration_covariance: new Array(9).fill(0),
  }
}

export function mockScan(): LaserScan {
  const N = 360
  const angle_min = -Math.PI
  const angle_max = Math.PI
  const angle_increment = (angle_max - angle_min) / N
  const t = elapsed()
  const ranges = new Array<number>(N)
  for (let i = 0; i < N; i++) {
    const ang = angle_min + i * angle_increment
    // a "room" 4m on the long axis, 2.5m on the short, with a moving "person" blob
    const wallX = 2 / Math.max(0.05, Math.abs(Math.cos(ang)))
    const wallY = 1.25 / Math.max(0.05, Math.abs(Math.sin(ang)))
    let r = Math.min(wallX, wallY)
    const blobAng = (t * 0.2) % (2 * Math.PI) - Math.PI
    const dAng = Math.atan2(Math.sin(ang - blobAng), Math.cos(ang - blobAng))
    if (Math.abs(dAng) < 0.15) r = Math.min(r, 1.2 + Math.sin(t * 5) * 0.05)
    if (Math.random() < 0.01) r = Infinity
    ranges[i] = r
  }
  return {
    header: makeHeader('laser_link'),
    angle_min,
    angle_max,
    angle_increment,
    time_increment: 0,
    scan_time: 0.1,
    range_min: 0.12,
    range_max: 12,
    ranges,
    intensities: [],
  }
}

export function mockOdometry(): Odometry {
  const t = elapsed()
  const x = 0.2 * Math.sin(t * 0.1)
  const y = 0.1 * Math.cos(t * 0.07)
  const yaw = 0.05 * Math.sin(t * 0.2)
  const half = yaw / 2
  return {
    header: makeHeader('odom'),
    child_frame_id: 'base_link',
    pose: {
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) },
      },
      covariance: new Array(36).fill(0),
    },
    twist: {
      twist: {
        linear: { x: 0.02 * Math.cos(t * 0.1), y: 0, z: 0 },
        angular: { x: 0, y: 0, z: 0.01 * Math.cos(t * 0.2) },
      },
      covariance: new Array(36).fill(0),
    },
  }
}

const ARM_JOINTS = ['arm_joint_0', 'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4']
const WHEEL_JOINTS = ['front_left_wheel_joint', 'front_right_wheel_joint', 'rear_left_wheel_joint', 'rear_right_wheel_joint']

export function mockJointStates(): JointState {
  const t = elapsed()
  const armPositions = ARM_JOINTS.map((_, i) => 0.4 * Math.sin(t * (0.3 + i * 0.05) + i))
  const wheelPositions = WHEEL_JOINTS.map((_, i) => (t * (1 + 0.05 * i)) % (2 * Math.PI))
  return {
    header: makeHeader(),
    name: [...ARM_JOINTS, ...WHEEL_JOINTS],
    position: [...armPositions, ...wheelPositions],
    velocity: [...armPositions.map(() => 0), ...wheelPositions.map((_, i) => 1 + 0.05 * i)],
    effort: [],
  }
}

export function mockBattery(): BatteryState {
  const t = elapsed()
  // gentle "discharge" — drops 0.5% per minute
  const pct = Math.max(0.05, 0.78 - t / 12000)
  return {
    header: makeHeader(),
    voltage: 11.5 + pct * 1.2 + Math.sin(t * 0.5) * 0.02,
    temperature: 28 + Math.sin(t * 0.05) * 1.5,
    current: -1.5 + Math.sin(t * 1) * 0.3,
    charge: pct * 5.0,
    capacity: 5.0,
    design_capacity: 5.2,
    percentage: pct,
    power_supply_status: 2, // discharging
    power_supply_health: 1, // good
    power_supply_technology: 3, // li-ion
    present: true,
    cell_voltage: [],
    cell_temperature: [],
    location: 'main',
    serial_number: 'MOCK-0001',
  }
}

const LOG_TEMPLATES: Array<{ name: string; level: RosoutLevel; msg: string }> = [
  { name: 'mirte_base_controller', level: 20, msg: 'Wheel command published OK' },
  { name: 'rosbridge_websocket', level: 20, msg: 'Subscribed to /joint_states' },
  { name: 'rplidar_node', level: 20, msg: 'Scan published, 360 points, range 0.12-11.8' },
  { name: 'orbbec_camera', level: 20, msg: 'Streaming color 640x480 @ 30 fps' },
  { name: 'mirte_telemetrix_cpp', level: 30, msg: 'IMU calibration drift detected, recalibrating' },
  { name: 'tf2_listener', level: 40, msg: 'Lookup would require extrapolation into the future' },
  { name: 'controller_manager', level: 20, msg: 'Loaded controller mirte_base_controller' },
]

export function mockLog(): Log {
  const t = LOG_TEMPLATES[Math.floor(Math.random() * LOG_TEMPLATES.length)]
  return {
    stamp: makeStamp(),
    level: t.level,
    name: t.name,
    msg: t.msg,
    file: '',
    function: '',
    line: 0,
  }
}
