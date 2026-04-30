import { useCallback, useEffect, useSyncExternalStore } from 'react'

// Bumped from v1 → v2 when default topics changed (IMU + battery on /io/*,
// web_video_server on :8091). Old persisted v1 blobs override the new
// defaults silently, so we ignore them on load and start fresh.
const STORAGE_KEY = 'lupin-hmi-settings/v2'

export interface Settings {
  rosUrl: string
  cmdVelTopic: string
  cmdVelType: 'geometry_msgs/msg/Twist' | 'geometry_msgs/msg/TwistStamped'
  imuTopic: string
  scanTopic: string
  odomTopic: string
  jointStatesTopic: string
  batteryTopic: string
  rosoutTopic: string
  cameraTopic: string
  webVideoServerUrl: string
  // Map / Nav2
  mapTopic: string
  planTopic: string
  goalPoseTopic: string
  mapFrame: string
  baseFrame: string
  speedScale: number
  // Arm
  /** Service / topic prefix for the Hiwonder serial-bus servos (no trailing slash). */
  armServoNamespace: string
  /** Default angular rate sent with `set_angle_with_speed`, in degrees/second. */
  armRateDegPerSec: number
  theme: 'dark' | 'light'
  debugPublish: boolean
}

export const DEFAULT_SETTINGS: Settings = {
  rosUrl: '',
  // sim publishes via the controller's unstamped Twist input
  cmdVelTopic: '/mirte_base_controller/cmd_vel_unstamped',
  cmdVelType: 'geometry_msgs/msg/Twist',
  // The MIRTE telemetrix node publishes IMU on /io/imu/movement/data;
  // the canonical /imu/data has no publisher on the real robot.
  imuTopic: '/io/imu/movement/data',
  scanTopic: '/scan',
  odomTopic: '/mirte_base_controller/odom',
  jointStatesTopic: '/joint_states',
  // Same story for battery — /battery_state is advertised but unpublished;
  // /io/power/power_watcher is what the telemetrix node actually publishes.
  batteryTopic: '/io/power/power_watcher',
  rosoutTopic: '/rosout',
  cameraTopic: '/camera/color/image_raw',
  webVideoServerUrl: '',
  mapTopic: '/map',
  planTopic: '/plan',
  goalPoseTopic: '/goal_pose',
  mapFrame: 'map',
  baseFrame: 'base_link',
  speedScale: 0.5,
  armServoNamespace: '/io/servo/hiwonder',
  armRateDegPerSec: 60,
  theme: 'dark',
  debugPublish: false,
}

function defaultRosUrl(): string {
  if (typeof window === 'undefined') return 'ws://localhost:9090'
  const params = new URLSearchParams(window.location.search)
  const override = params.get('ros')
  if (override) return override
  const host = window.location.hostname || 'localhost'
  return `ws://${host}:9090`
}

function defaultWebVideoUrl(): string {
  // The vendor MIRTE setup runs web_video_server on [::1]:8181 (localhost-only),
  // and :8080 conflicts with the wifi-connect AP captive portal. We launch a
  // second web_video_server bound to 0.0.0.0:8091 alongside our Vite UI so
  // browsers on the LAN can reach the MJPEG stream.
  if (typeof window === 'undefined') return 'http://localhost:8091'
  const host = window.location.hostname || 'localhost'
  return `http://${host}:8091`
}

function readStored(): Partial<Settings> {
  try {
    const raw = localStorage.getItem(STORAGE_KEY)
    return raw ? (JSON.parse(raw) as Partial<Settings>) : {}
  } catch {
    return {}
  }
}

let cached: Settings = (() => {
  const stored = readStored()
  return {
    ...DEFAULT_SETTINGS,
    rosUrl: stored.rosUrl || defaultRosUrl(),
    webVideoServerUrl: stored.webVideoServerUrl || defaultWebVideoUrl(),
    ...stored,
  }
})()

const listeners = new Set<() => void>()

function notify() {
  listeners.forEach((l) => l())
}

function persist() {
  try {
    localStorage.setItem(STORAGE_KEY, JSON.stringify(cached))
  } catch {
    /* localStorage may be disabled — settings still work in-memory */
  }
}

export function getSettings(): Settings {
  return cached
}

export function updateSettings(patch: Partial<Settings>) {
  cached = { ...cached, ...patch }
  persist()
  notify()
}

export function resetSettings() {
  cached = {
    ...DEFAULT_SETTINGS,
    rosUrl: defaultRosUrl(),
    webVideoServerUrl: defaultWebVideoUrl(),
  }
  persist()
  notify()
}

function subscribe(cb: () => void) {
  listeners.add(cb)
  return () => {
    listeners.delete(cb)
  }
}

export function useSettings(): readonly [Settings, (patch: Partial<Settings>) => void, () => void] {
  const settings = useSyncExternalStore(subscribe, getSettings, getSettings)
  const update = useCallback((patch: Partial<Settings>) => updateSettings(patch), [])
  const reset = useCallback(() => resetSettings(), [])
  return [settings, update, reset] as const
}

export function useApplyTheme() {
  const [{ theme }] = useSettings()
  useEffect(() => {
    const root = document.documentElement
    root.classList.remove('dark', 'light')
    root.classList.add(theme)
    root.style.colorScheme = theme
  }, [theme])
}

export function isMockMode(): boolean {
  if (typeof window === 'undefined') return false
  const params = new URLSearchParams(window.location.search)
  return params.has('mock')
}
