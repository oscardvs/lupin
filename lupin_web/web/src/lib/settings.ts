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
  /** When true, mic only opens while the user engages the button. False = open mic toggle. */
  voicePushToTalk: boolean
  /**
   * When true, the speech endpointer auto-closes the mic on a sustained pause —
   * so the agent stops listening once the user finishes their sentence. Applies
   * in both push-to-talk and hands-free modes.
   */
  voiceVadEnabled: boolean
  /** Continuous silence (ms) below voiceVadEndThreshold that ends an utterance. */
  voiceVadEndHoldMs: number
  /** RMS in [0, 1] that must be sustained to count as speech-start. */
  voiceVadStartThreshold: number
  /** RMS in [0, 1] below which silence accumulates toward speech-end. */
  voiceVadEndThreshold: number
  /** Hard cap on a single utterance — closes the mic even if speech is still detected. */
  voiceVadMaxUtteranceMs: number
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
  /**
   * Auto-trigger E-stop on `visibilitychange` / window `blur`. Default true for
   * the demo robot; disable during dev when alt-tabbing fires it constantly.
   * The `beforeunload` and rosbridge-disconnect triggers remain active either
   * way — those are real safety events, not focus changes.
   */
  estopAutoOnFocusLoss: boolean
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
  // Sim greenhouse_sim publishes the Astra Pro Plus plugin on /camera/image_raw.
  // Real Mirte: override via Settings → Topics if your camera node uses a
  // different name (e.g. /camera/color/image_raw on a stock Orbbec stack).
  cameraTopic: '/camera/image_raw',
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
  voiceVadEnabled: true,
  voiceVadEndHoldMs: 800,
  voiceVadStartThreshold: 0.02,
  voiceVadEndThreshold: 0.012,
  voiceVadMaxUtteranceMs: 15000,
  voiceMaxLinearMps: 0.3,
  voiceMaxAngularRps: 0.8,
  voiceSystemPrompt: [
    'You are Lupin, the on-board voice assistant of a MIRTE Master mobile robot.',
    'You can drive the base, send Nav2 goals, move the arm to named presets, open',
    'and close the gripper, and report telemetry.',
    'For "go N metres forward / back / sideways" use nav_forward — it takes a',
    'body-frame offset and the HMI computes the absolute goal for you. Use',
    'nav_goto ONLY when the user gives explicit map-frame coordinates. For "turn',
    'N degrees" use rotate. Prefer nav_* (Nav2-mediated) over drive bursts for',
    'any non-trivial displacement.',
    'If a motion tool returns blocked: true with an e-stop reason, do not retry —',
    'tell the user the e-stop is engaged and ask them to press Reset E-stop in',
    'the UI before trying again.',
    'Keep replies short — one or two sentences. Confirm motion commands before',
    'executing them and never move the robot if the user sounds unsure or asks a',
    'question. Refuse anything beyond your declared tools and explain why.',
  ].join(' '),
  voiceNamedLocations: {
    home: { x: 0, y: 0, yaw: 0 },
  },
  theme: 'dark',
  debugPublish: false,
  estopAutoOnFocusLoss: true,
}

function defaultRosUrl(): string {
  if (typeof window === 'undefined') return 'ws://localhost:9090'
  const params = new URLSearchParams(window.location.search)
  const override = params.get('ros')
  if (override) return override
  // Vite preview proxies /_ros (with WS upgrade) to ws://localhost:9090. Using
  // a same-origin path lets one TLS termination cover both the page and the
  // rosbridge socket — no mixed-content blocks when the HMI is served over
  // https on the robot, and no extra cert prompts for a separate :9090 origin.
  const wsScheme = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
  const host = window.location.host || 'localhost:8090'
  return `${wsScheme}//${host}/_ros`
}

function defaultWebVideoUrl(): string {
  // The vendor MIRTE setup runs web_video_server on [::1]:8181 (localhost-only),
  // and :8080 conflicts with the wifi-connect AP captive portal. We launch a
  // second web_video_server bound to 0.0.0.0:8091 alongside our Vite UI so
  // browsers on the LAN can reach the MJPEG stream — but go through the
  // same-origin /_video proxy so https pages don't get mixed-content-blocked
  // on the long-lived MJPEG stream.
  if (typeof window === 'undefined') return 'http://localhost:8091'
  const { protocol, host } = window.location
  return `${protocol}//${host || 'localhost:8090'}/_video`
}

/**
 * If the page is loaded over https but the stored URL is plain ws:// or http://,
 * the browser will block it as mixed content. Drop the stale stored value so
 * the protocol-aware default kicks in instead. Users who explicitly want a
 * remote rosbridge over wss:// keep their override.
 */
function dropMixedContentUrls(stored: Partial<Settings>): Partial<Settings> {
  if (typeof window === 'undefined' || window.location.protocol !== 'https:') return stored
  const out: Partial<Settings> = { ...stored }
  if (typeof out.rosUrl === 'string' && out.rosUrl.startsWith('ws://')) delete out.rosUrl
  if (typeof out.webVideoServerUrl === 'string' && out.webVideoServerUrl.startsWith('http://')) {
    delete out.webVideoServerUrl
  }
  return out
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
  const stored = dropMixedContentUrls(readStored())
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
