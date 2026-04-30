import { useCallback, useEffect, useSyncExternalStore } from 'react'

// v3 adds voice-assistant fields (Gemini Live). v2 keys are migrated forward —
// rosbridge URL and topic overrides are preserved; new voice fields fall back
// to defaults.
const STORAGE_KEY = 'lupin-hmi-settings/v3'
const LEGACY_STORAGE_KEYS = ['lupin-hmi-settings/v2']

export interface VoiceNamedLocation {
  x: number
  y: number
  yaw: number
}

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
  // Voice (Gemini Live)
  /** Google AI Studio API key. Stored in localStorage only — never committed. */
  geminiApiKey: string
  /** Live-API model id, full path form: `models/<id>`. */
  geminiModel: string
  /** BCP-47 language code hint sent to the model (e.g. 'en-US', 'nl-NL'). */
  voiceLanguage: string
  /** When true, mic only opens while the user holds the button. False = open mic toggle. */
  voicePushToTalk: boolean
  /** Hard cap on |linear.x| / |linear.y| the voice agent can request, m/s. */
  voiceMaxLinearMps: number
  /** Hard cap on |angular.z| the voice agent can request, rad/s. */
  voiceMaxAngularRps: number
  /** System instruction prepended to every session. */
  voiceSystemPrompt: string
  /** Named map poses the voice agent can navigate to via `nav_goto_named`. */
  voiceNamedLocations: Record<string, VoiceNamedLocation>
  theme: 'dark' | 'light'
  debugPublish: boolean
}

export const DEFAULT_SETTINGS: Settings = {
  rosUrl: '',
  // real Mirte's controller listens on the stamped Twist topic
  cmdVelTopic: '/mirte_base_controller/cmd_vel',
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
  geminiApiKey: '',
  geminiModel: 'models/gemini-3.1-flash-live-preview',
  voiceLanguage: 'en-US',
  voicePushToTalk: true,
  voiceMaxLinearMps: 0.3,
  voiceMaxAngularRps: 0.8,
  voiceSystemPrompt: [
    'You are Lupin, the on-board voice assistant of a MIRTE Master mobile robot.',
    'You can drive the base, send Nav2 goals, set arm presets, and report telemetry.',
    'Keep replies short — one or two sentences. Confirm motion commands before',
    'executing them and never move the robot if the user sounds unsure or asks a',
    'question. Refuse anything beyond your declared tools and explain why.',
  ].join(' '),
  voiceNamedLocations: {
    home: { x: 0, y: 0, yaw: 0 },
  },
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
    if (raw) return JSON.parse(raw) as Partial<Settings>
    for (const legacy of LEGACY_STORAGE_KEYS) {
      const old = localStorage.getItem(legacy)
      if (old) return JSON.parse(old) as Partial<Settings>
    }
    return {}
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
