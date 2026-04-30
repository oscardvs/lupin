import { useCallback, useEffect, useSyncExternalStore } from 'react'

const STORAGE_KEY = 'lupin-hmi-settings/v1'

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
  speedScale: number
  theme: 'dark' | 'light'
  debugPublish: boolean
}

export const DEFAULT_SETTINGS: Settings = {
  rosUrl: '',
  // sim publishes via the controller's unstamped Twist input
  cmdVelTopic: '/mirte_base_controller/cmd_vel_unstamped',
  cmdVelType: 'geometry_msgs/msg/Twist',
  imuTopic: '/imu/data',
  scanTopic: '/scan',
  odomTopic: '/mirte_base_controller/odom',
  jointStatesTopic: '/joint_states',
  batteryTopic: '/battery_state',
  rosoutTopic: '/rosout',
  cameraTopic: '/camera/color/image_raw',
  webVideoServerUrl: '',
  speedScale: 0.5,
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
  if (typeof window === 'undefined') return 'http://localhost:8080'
  const host = window.location.hostname || 'localhost'
  return `http://${host}:8080`
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
