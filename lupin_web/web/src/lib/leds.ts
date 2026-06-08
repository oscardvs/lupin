/**
 * Status-light palette — the single source of truth shared by the Map-tab
 * LightControl widget and the voice `set_light` tool, so colour names and RGB
 * values can never drift between the UI, the agent, and the docs. Values mirror
 * lupin_hmi/light_strip_bridge's mission-state palette (see web-hmi docs).
 *
 * `name` is the canonical lowercase key the voice tool matches on; `label` is
 * the Titlecase display string the widget renders. The strip is whole-strip
 * RGB, 0–255 per channel.
 */

export interface Rgb {
  r: number
  g: number
  b: number
}

export interface LedPreset {
  /** Canonical lowercase key used by the voice tool and the system prompt. */
  name: string
  /** Titlecase display label used by the LightControl widget. */
  label: string
  rgb: Rgb
}

export const LED_PRESETS: LedPreset[] = [
  { name: 'red', label: 'Red', rgb: { r: 255, g: 0, b: 0 } },
  { name: 'orange', label: 'Orange', rgb: { r: 255, g: 128, b: 0 } },
  { name: 'amber', label: 'Amber', rgb: { r: 255, g: 191, b: 0 } },
  { name: 'yellow', label: 'Yellow', rgb: { r: 255, g: 255, b: 0 } },
  { name: 'green', label: 'Green', rgb: { r: 0, g: 255, b: 0 } },
  { name: 'spring', label: 'Spring', rgb: { r: 0, g: 255, b: 128 } },
  { name: 'teal', label: 'Teal', rgb: { r: 0, g: 180, b: 140 } },
  { name: 'cyan', label: 'Cyan', rgb: { r: 0, g: 255, b: 255 } },
  { name: 'blue', label: 'Blue', rgb: { r: 0, g: 0, b: 255 } },
  { name: 'purple', label: 'Purple', rgb: { r: 180, g: 0, b: 255 } },
  { name: 'white', label: 'White', rgb: { r: 255, g: 255, b: 255 } },
  { name: 'off', label: 'Off', rgb: { r: 0, g: 0, b: 0 } },
]

/** Canonical names, for the voice tool's error message and the system prompt. */
export const LED_PRESET_NAMES: string[] = LED_PRESETS.map((p) => p.name)

/** Resolve a (case-insensitive, trimmed) colour name to its preset, or undefined. */
export function ledPresetByName(name: string): LedPreset | undefined {
  const key = name.trim().toLowerCase()
  return LED_PRESETS.find((p) => p.name === key)
}

export function rgbToCss({ r, g, b }: Rgb): string {
  return `rgb(${r}, ${g}, ${b})`
}

export function rgbToHex({ r, g, b }: Rgb): string {
  const h = (n: number) => n.toString(16).padStart(2, '0')
  return `#${h(r)}${h(g)}${h(b)}`
}

export function hexToRgb(hex: string): Rgb {
  const m = /^#?([0-9a-f]{2})([0-9a-f]{2})([0-9a-f]{2})$/i.exec(hex.trim())
  if (!m) return { r: 0, g: 0, b: 0 }
  return {
    r: parseInt(m[1], 16),
    g: parseInt(m[2], 16),
    b: parseInt(m[3], 16),
  }
}
