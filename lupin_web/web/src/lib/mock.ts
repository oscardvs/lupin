import {
  type BatteryState,
  type GetFieldRequest,
  type GetFieldResponse,
  type Imu,
  type JointState,
  type LaserScan,
  type Log,
  type MissionState,
  type Observation,
  type OccupancyGrid,
  type Odometry,
  type Path,
  type PoseStamped,
  type RosoutLevel,
  type SensorReading,
  type TwinSensor,
  type TwinState,
  type TwinTagState,
  OBSERVATION_KIND,
  OBSERVATION_STATUS,
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

/* ----------------------------------------------------------------------- */
/* Map / Nav2 mock — a synthetic greenhouse aisle map, a robot tracing a   */
/* figure-eight path through it, and a fake plan from current pose to goal */
/* ----------------------------------------------------------------------- */

const MAP_RES = 0.05 // 5 cm per cell
const MAP_W = 200 // 10 m wide
const MAP_H = 160 // 8 m tall
const MAP_ORIGIN_X = -5.0 // map cell [0,0] is at world (-5, -4)
const MAP_ORIGIN_Y = -4.0

let cachedMap: OccupancyGrid | null = null

/** Build a static greenhouse-aisle occupancy grid once and cache it. */
export function mockMap(): OccupancyGrid {
  if (cachedMap) return cachedMap

  const data = new Array<number>(MAP_W * MAP_H).fill(-1)
  const set = (cx: number, cy: number, v: number) => {
    if (cx < 0 || cy < 0 || cx >= MAP_W || cy >= MAP_H) return
    data[cy * MAP_W + cx] = v
  }

  // Free the interior — clear corridor, walls inset by 1 cell
  for (let y = 1; y < MAP_H - 1; y++) {
    for (let x = 1; x < MAP_W - 1; x++) set(x, y, 0)
  }

  // Outer walls
  for (let x = 0; x < MAP_W; x++) {
    set(x, 0, 100)
    set(x, MAP_H - 1, 100)
  }
  for (let y = 0; y < MAP_H; y++) {
    set(0, y, 100)
    set(MAP_W - 1, y, 100)
  }

  // Three planting rows running left-right, each ~3 cells thick
  const rowYs = [40, 80, 120]
  for (const ry of rowYs) {
    for (let x = 20; x < MAP_W - 20; x++) {
      for (let dy = 0; dy < 4; dy++) set(x, ry + dy, 100)
    }
  }

  // A few "pots" (occupied dots) scattered along the rows
  for (const ry of rowYs) {
    for (let x = 25; x < MAP_W - 25; x += 14) {
      for (let dx = 0; dx < 3; dx++)
        for (let dy = -2; dy < 6; dy++) set(x + dx, ry + dy, 100)
    }
  }

  cachedMap = {
    header: makeHeader('map'),
    info: {
      map_load_time: makeStamp(),
      resolution: MAP_RES,
      width: MAP_W,
      height: MAP_H,
      origin: {
        position: { x: MAP_ORIGIN_X, y: MAP_ORIGIN_Y, z: 0 },
        orientation: { x: 0, y: 0, z: 0, w: 1 },
      },
    },
    data,
  }
  return cachedMap
}

/** Robot's pose in map frame for the mock — figure-eight on the centre aisle. */
export function mockMapPose(): { x: number; y: number; yaw: number } {
  const t = elapsed() * 0.15
  // Figure-eight in the corridor between the planting rows
  const x = Math.sin(t) * 3.0
  const y = Math.sin(t * 2) * 0.8
  // Heading is the tangent of the path
  const dx = Math.cos(t) * 3.0
  const dy = Math.cos(t * 2) * 1.6
  const yaw = Math.atan2(dy, dx)
  return { x, y, yaw }
}

/* ----------------------------------------------------------------------- */
/* Mission orchestrator mock — drives the HMI's mission-aware surfaces      */
/* (topbar strip, Map/Nav controls, Observations panel) without a robot.    */
/* ----------------------------------------------------------------------- */

const MOCK_TAGS = ['tag-1', 'tag-2', 'tag-3', 'tag-4'] as const

// Phase durations in seconds. Sum determines the full mission length.
const MOCK_TIMING = {
  ready: 4,
  prepareLocalizing: 3,
  navigating: 5,
  scanning: 3,
  publishing: 1,
  returning: 4,
  done: 3,
}

interface MockMissionPhase {
  lifecycle: MissionState['lifecycle_state']
  phase: string
  targetIdx: number          // -1 means no target
  /** Position within the phase, [0, 1). Useful for animated UI. */
  progress: number
  /** Offset (s) from cycle start at which this phase began. */
  phaseStart: number
}

/**
 * Compute the current mock mission phase from the elapsed clock. The cycle
 * walks the full HSM once and loops:
 *
 *   READY → PREPARE/LOCALIZING → INSPECTING(NAV→SCAN→PUB)*N → RETURNING → DONE → READY
 *
 * Independent callers (state publisher + observation publisher) hit the
 * same cycle and stay phase-aligned.
 */
function mockMissionPhase(): MockMissionPhase {
  const T = MOCK_TIMING
  const perTag = T.navigating + T.scanning + T.publishing
  const total = T.ready + T.prepareLocalizing + perTag * MOCK_TAGS.length + T.returning + T.done
  const t = elapsed() % total

  let acc = 0
  if (t < (acc += T.ready))
    return { lifecycle: 'READY', phase: '', targetIdx: -1, progress: t / T.ready, phaseStart: 0 }
  if (t < (acc += T.prepareLocalizing))
    return {
      lifecycle: 'PREPARE',
      phase: 'LOCALIZING',
      targetIdx: -1,
      progress: (t - (acc - T.prepareLocalizing)) / T.prepareLocalizing,
      phaseStart: acc - T.prepareLocalizing,
    }
  for (let i = 0; i < MOCK_TAGS.length; i++) {
    const tagStart = acc
    if (t < (acc += T.navigating))
      return {
        lifecycle: 'INSPECTING',
        phase: 'NAVIGATING',
        targetIdx: i,
        progress: (t - tagStart) / T.navigating,
        phaseStart: tagStart,
      }
    const navEnd = acc
    if (t < (acc += T.scanning))
      return {
        lifecycle: 'INSPECTING',
        phase: 'SCANNING',
        targetIdx: i,
        progress: (t - navEnd) / T.scanning,
        phaseStart: navEnd,
      }
    const scanEnd = acc
    if (t < (acc += T.publishing))
      return {
        lifecycle: 'INSPECTING',
        phase: 'PUBLISHING',
        targetIdx: i,
        progress: (t - scanEnd) / T.publishing,
        phaseStart: scanEnd,
      }
  }
  if (t < (acc += T.returning))
    return {
      lifecycle: 'RETURNING',
      phase: '',
      targetIdx: -1,
      progress: (t - (acc - T.returning)) / T.returning,
      phaseStart: acc - T.returning,
    }
  return {
    lifecycle: 'DONE',
    phase: '',
    targetIdx: -1,
    progress: (t - (acc - T.done)) / T.done,
    phaseStart: acc - T.done,
  }
}

const MOCK_MISSION_ID = 'mock-' + Math.random().toString(16).slice(2, 10)
const MOCK_STARTED_AT = makeStamp()

/**
 * Synthetic `MissionState` snapshot mirroring what the orchestrator publishes
 * at 5 Hz. Counters advance as the cycle visits PUBLISHING for each tag so the
 * "k of M" UX has something to render. E-stop and pause are always false in
 * mock mode — testing those interactions belongs to integration tests, not
 * the demo loop.
 */
export function mockMissionState(): MissionState {
  const ph = mockMissionPhase()
  const N = MOCK_TAGS.length
  const completed = (() => {
    if (ph.lifecycle === 'INSPECTING' && ph.targetIdx >= 0) {
      // Tags fully published before this one are completed; the current tag
      // counts only once we've left PUBLISHING.
      return ph.targetIdx + (ph.phase === '' ? 0 : 0)
    }
    if (ph.lifecycle === 'RETURNING' || ph.lifecycle === 'DONE') return N
    return 0
  })()

  return {
    header: { stamp: makeStamp(), frame_id: '' },
    mission_id: ph.lifecycle === 'READY' ? '' : MOCK_MISSION_ID,
    mission_type: ph.lifecycle === 'READY' ? '' : 'InspectionMission',
    lifecycle_state: ph.lifecycle,
    mission_phase: ph.phase,
    current_target: ph.targetIdx >= 0 ? MOCK_TAGS[ph.targetIdx] : '',
    targets_total: ph.lifecycle === 'READY' ? 0 : N,
    targets_completed: completed,
    targets_failed: 0,
    targets_unreachable: 0,
    targets_skipped: 0,
    tags_discovered: 0,   // mock cycle is an InspectionMission (a-priori tags)
    discovery_goal: 0,
    last_error: '',
    estop_engaged: false,
    paused: false,
    started_at: ph.lifecycle === 'READY'
      ? { sec: 0, nanosec: 0 }
      : MOCK_STARTED_AT,
  }
}

// One-shot guard so we emit at most one Observation per (cycle, tag).
const observedThisCycle = new Map<string, number>()
let lastCycleStart = 0

/**
 * Emit a synthetic Observation when the cycle has just left PUBLISHING for
 * a tag. Returns null on every other tick — the subscriber sees observations
 * arrive in the same one-per-tag rhythm the orchestrator produces.
 */
export function mockObservation(): Observation | null {
  const T = MOCK_TIMING
  const perTag = T.navigating + T.scanning + T.publishing
  const total = T.ready + T.prepareLocalizing + perTag * MOCK_TAGS.length + T.returning + T.done
  const cycleStart = Math.floor(elapsed() / total) * total
  if (cycleStart !== lastCycleStart) {
    observedThisCycle.clear()
    lastCycleStart = cycleStart
  }
  const ph = mockMissionPhase()
  if (ph.lifecycle !== 'INSPECTING' || ph.phase !== 'PUBLISHING' || ph.targetIdx < 0) return null
  const tagId = MOCK_TAGS[ph.targetIdx]
  if (observedThisCycle.has(tagId)) return null
  observedThisCycle.set(tagId, elapsed())

  // Vary readings per tag and slowly across cycles so the table isn't static.
  const t = elapsed()
  const tempBase = 19 + ph.targetIdx * 0.8
  const humBase = 55 + ph.targetIdx * 2
  const co2Base = 410 + ph.targetIdx * 7
  return {
    header: { stamp: makeStamp(), frame_id: '' },
    mission_id: MOCK_MISSION_ID,
    source: 'InspectionMission',
    kind: OBSERVATION_KIND.TAG_READING,
    status: OBSERVATION_STATUS.OK,
    status_detail: '',
    tag_reading: {
      tag_id: tagId,
      stamp: makeStamp(),
      sim_time_of_day_seconds: 36000 + (t % 86400),
      readings: [
        { name: 'temperature', value: tempBase + Math.sin(t * 0.05 + ph.targetIdx) * 0.6 },
        { name: 'humidity', value: humBase + Math.cos(t * 0.03 + ph.targetIdx) * 1.5 },
        { name: 'co2', value: co2Base + Math.sin(t * 0.02) * 12 },
      ],
    },
    flower: {
      tag_id: '',
      pose: { header: makeHeader('map'), pose: { position: { x: 0, y: 0, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 0 } } },
      species: '',
      confidence: 0,
      anomaly: false,
    },
    anomaly: null,
  }
}

/** A synthetic Nav2 plan that moves with the robot — leading by ~2 s along the path. */
export function mockPlan(): Path {
  const t = elapsed() * 0.15
  const poses: PoseStamped[] = []
  const N = 40
  for (let i = 0; i < N; i++) {
    const dt = (i / N) * 1.2 // look ahead ~1.2 phase units
    const u = t + dt
    const x = Math.sin(u) * 3.0
    const y = Math.sin(u * 2) * 0.8
    const dx = Math.cos(u) * 3.0
    const dy = Math.cos(u * 2) * 1.6
    const yaw = Math.atan2(dy, dx)
    const half = yaw / 2
    poses.push({
      header: makeHeader('map'),
      pose: {
        position: { x, y, z: 0 },
        orientation: { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) },
      },
    })
  }
  return {
    header: makeHeader('map'),
    poses,
  }
}

/* ----------------------------------------------------------------------- */
/* Digital-twin mock — fake TwinState + GetField responses so the HMI's    */
/* heat map / Greenhouse State table / tulip work without a robot.         */
/* ----------------------------------------------------------------------- */

interface MockTwinTagSeed {
  tag_id: string
  x: number
  y: number
  /** Drift centre per sensor; readings drift around these over time. */
  base: Record<TwinSensor, number>
  /** Phase offset so each tag oscillates differently. */
  phase: number
  /** YOLO flower class co-located with this tag (demo the flower layer). */
  species: string
  /** Whether the YOLO "bug" anomaly is present at this tag. */
  anomaly: boolean
}

// 7 tags scattered across a synthetic 10×10 m greenhouse. Two of them have
// values intentionally near the edge of their ideal ranges so the tulip
// has something to react to in mock mode. Species + one anomaly demo the
// flower map layer / TYPE column / tooltip without a robot.
const MOCK_TWIN_TAGS: MockTwinTagSeed[] = [
  { tag_id: '1', x: 1.5, y: 1.5, base: { temperature: 22, humidity: 55, co2: 480, light: 500, soil_moisture: 45 }, phase: 0.0, species: 'tulip_red', anomaly: false },
  { tag_id: '2', x: 4.5, y: 1.5, base: { temperature: 24, humidity: 60, co2: 520, light: 600, soil_moisture: 50 }, phase: 0.7, species: 'tulip_white', anomaly: false },
  { tag_id: '3', x: 1.5, y: 4.5, base: { temperature: 21, humidity: 65, co2: 460, light: 450, soil_moisture: 40 }, phase: 1.4, species: 'tulip_pink', anomaly: false },
  { tag_id: '4', x: 4.5, y: 4.5, base: { temperature: 26, humidity: 70, co2: 700, light: 700, soil_moisture: 55 }, phase: 2.1, species: 'tulip_red', anomaly: false },
  { tag_id: '5', x: 8.0, y: 2.5, base: { temperature: 19, humidity: 50, co2: 420, light: 350, soil_moisture: 35 }, phase: 2.8, species: 'tulip_white', anomaly: false },
  { tag_id: '6', x: 8.0, y: 6.0, base: { temperature: 27, humidity: 78, co2: 950, light: 250, soil_moisture: 25 }, phase: 3.5, species: 'tulip_pink', anomaly: true },  // stressed + pest
  { tag_id: '7', x: 5.5, y: 8.0, base: { temperature: 23, humidity: 58, co2: 540, light: 550, soil_moisture: 48 }, phase: 4.2, species: '', anomaly: false },  // unclassified yet
]

const SENSOR_DRIFT_AMPLITUDES: Record<TwinSensor, number> = {
  temperature: 1.2,
  humidity: 4.0,
  co2: 35,
  light: 50,
  soil_moisture: 3.0,
}

function mockTagReadings(tag: MockTwinTagSeed, t: number): SensorReading[] {
  const out: SensorReading[] = []
  for (const sensor of ['temperature', 'humidity', 'co2', 'light', 'soil_moisture'] as const) {
    const amp = SENSOR_DRIFT_AMPLITUDES[sensor]
    const v = tag.base[sensor] + amp * Math.sin(t * 0.05 + tag.phase)
    out.push({ name: sensor, value: v })
  }
  return out
}

function mockTagPose(tag: MockTwinTagSeed) {
  return {
    position: { x: tag.x, y: tag.y, z: 0 },
    // Identity rotation; orientation.w==1 so the HMI treats this as a
    // valid pose (zero would mean "no pose observed").
    orientation: { x: 0, y: 0, z: 0, w: 1 },
  }
}

/**
 * Stagger when each tag becomes visible so the HMI's "fills in as the robot
 * patrols" narrative reads correctly under ?mock=1. The first tag appears
 * after 4 s, the next after 8 s, etc., so a fresh page load watches the
 * Greenhouse State table populate and the heat map grow over ~half a minute.
 */
function mockVisibleTagCount(t: number): number {
  const interval = 4
  return Math.min(MOCK_TWIN_TAGS.length, Math.floor(t / interval) + 1)
}

export function mockTwinState(): TwinState {
  const t = elapsed()
  const visible = mockVisibleTagCount(t)
  const tags: TwinTagState[] = []
  for (let i = 0; i < visible; i++) {
    const seed = MOCK_TWIN_TAGS[i]
    const stale = (i === visible - 1)
      ? (t % 4)               // newly-arrived tag is fresh
      : ((t * 0.8) % 90) + 1  // older tags age slightly faster than wall clock
                              // so the "X seconds ago" badge actually moves
    // Build a builtin_interfaces/Time for last_observed by subtracting
    // stale from "now" — keeps the durable timestamp consistent with the
    // relative stale_seconds.
    const observedAtSec = Date.now() / 1000 - stale
    tags.push({
      tag_id: seed.tag_id,
      pose: mockTagPose(seed),
      readings: mockTagReadings(seed, t),
      last_observed: {
        sec: Math.floor(observedAtSec),
        nanosec: Math.floor((observedAtSec - Math.floor(observedAtSec)) * 1e9),
      },
      stale_seconds: stale,
      species: seed.species,
      species_confidence: seed.species ? 0.82 + 0.1 * Math.sin(t * 0.1 + seed.phase) : 0,
      anomaly: seed.anomaly,
    })
  }
  return {
    header: makeHeader('map'),
    tags,
  }
}

/**
 * Synthetic GetField response — same shape the twin node returns, computed
 * from the current mock tag set with a tiny IDW so the heat map looks
 * consistent with the Greenhouse State table and the tulip.
 *
 * Mirrors the Python idw.py: nearest-tag NaN gating beyond max_distance,
 * weighted sum within falloff_radius, exact value at coincident cells.
 */
export function mockTwinField(req: GetFieldRequest): GetFieldResponse {
  const sensor = req.sensor_type as TwinSensor
  const t = elapsed()
  const visible = mockVisibleTagCount(t)
  const samples: Array<{ x: number; y: number; v: number }> = []
  for (let i = 0; i < visible; i++) {
    const seed = MOCK_TWIN_TAGS[i]
    const reading = mockTagReadings(seed, t).find((r) => r.name === sensor)
    if (!reading) continue
    samples.push({ x: seed.x, y: seed.y, v: reading.value })
  }

  if (req.resolution <= 0 || req.bbox_max_x < req.bbox_min_x || req.bbox_max_y < req.bbox_min_y) {
    return {
      ok: false, error_message: 'invalid bbox/resolution',
      values: [], width: 0, height: 0,
      origin_x: 0, origin_y: 0, resolution_used: 0,
      value_min: 0, value_max: 0, sample_count: 0,
    }
  }

  const width = Math.max(1, Math.ceil((req.bbox_max_x - req.bbox_min_x) / req.resolution))
  const height = Math.max(1, Math.ceil((req.bbox_max_y - req.bbox_min_y) / req.resolution))
  const values: (number | null)[] = new Array(width * height).fill(null)
  // Mirror the Python idw.py defaults — keep these in sync if the
  // backend tunings ever change. Reviewer flagged the duplication; for
  // now a comment is the cheapest contract since the values almost
  // never change and a /twin/get_params service would be over-engineered.
  const POWER = 2
  const FALLOFF = 1.5
  const MAX_DIST = 1.5
  const FALLOFF_SQ = FALLOFF * FALLOFF
  const MAX_DIST_SQ = MAX_DIST * MAX_DIST

  let vmin = Infinity, vmax = -Infinity

  for (let j = 0; j < height; j++) {
    const cy = req.bbox_min_y + (j + 0.5) * req.resolution
    for (let i = 0; i < width; i++) {
      const cx = req.bbox_min_x + (i + 0.5) * req.resolution
      const idx = j * width + i

      let nearestDsq = Infinity
      let nearestVal = 0
      for (const s of samples) {
        const dx = cx - s.x, dy = cy - s.y
        const d = dx * dx + dy * dy
        if (d < nearestDsq) { nearestDsq = d; nearestVal = s.v }
      }
      if (samples.length === 0 || nearestDsq > MAX_DIST_SQ) continue

      if (nearestDsq < 1e-6) {
        values[idx] = nearestVal
        if (nearestVal < vmin) vmin = nearestVal
        if (nearestVal > vmax) vmax = nearestVal
        continue
      }

      let num = 0, den = 0
      for (const s of samples) {
        const dx = cx - s.x, dy = cy - s.y
        const dsq = dx * dx + dy * dy
        if (dsq > FALLOFF_SQ) continue
        const w = 1 / Math.pow(Math.sqrt(dsq), POWER)
        num += w * s.v
        den += w
      }
      const v = den > 0 ? num / den : nearestVal
      values[idx] = v
      if (v < vmin) vmin = v
      if (v > vmax) vmax = v
    }
  }
  if (!Number.isFinite(vmin)) { vmin = 0; vmax = 0 }

  return {
    ok: true,
    error_message: '',
    values,
    width,
    height,
    origin_x: req.bbox_min_x,
    origin_y: req.bbox_min_y,
    resolution_used: req.resolution,
    value_min: vmin,
    value_max: vmax,
    sample_count: samples.length,
  }
}
