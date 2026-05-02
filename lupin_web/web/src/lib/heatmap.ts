import type { GetFieldResponse, TwinSensor } from '@/types/ros'

/**
 * Single-hue color ramps per sensor — terminal aesthetic, no rainbows.
 * Each ramp goes from transparent at value_min to ~60% opacity at
 * value_max so the SLAM floor plan stays legible underneath.
 *
 * Hues:
 *   - temperature → warm amber  (HSL ~30°)
 *   - humidity    → cyan        (HSL ~190°)
 *   - co2         → red         (HSL ~5°)
 *   - light       → yellow      (HSL ~50°)
 *   - soil_moisture → teal      (HSL ~165°)
 *
 * The HMI also uses the ramp's mid-saturation value to color the tag
 * pins themselves, so a tag's pin matches the field colour around it.
 */
export interface RampSpec {
  hue: number          // HSL hue, 0–360
  /** Lightness anchors at the high end of the ramp; the low end uses
   * the same hue with high lightness + zero alpha so it fades to nothing. */
  lowL: number
  highL: number
  /** Sensor unit suffix shown in the legend. */
  unit: string
}

export const SENSOR_RAMPS: Record<TwinSensor, RampSpec> = {
  temperature:    { hue:  30, lowL: 80, highL: 40, unit: '°C' },
  humidity:       { hue: 190, lowL: 75, highL: 45, unit: '%' },
  co2:            { hue:   5, lowL: 78, highL: 42, unit: 'ppm' },
  light:          { hue:  50, lowL: 80, highL: 50, unit: 'lux' },
  soil_moisture:  { hue: 165, lowL: 75, highL: 42, unit: '%' },
}

const MAX_ALPHA = 0.6

/**
 * Map a field value to an [r, g, b, a] tuple in 0–255. Returns alpha=0
 * for null/NaN (the renderer should leave the underlying pixels alone).
 */
export function valueToRgba(
  v: number | null | undefined,
  vMin: number, vMax: number,
  ramp: RampSpec,
): [number, number, number, number] {
  if (v == null || !Number.isFinite(v)) return [0, 0, 0, 0]
  const span = vMax - vMin
  // When the whole field collapses to a single value, paint at full
  // opacity using the high-end colour — operator still wants to see
  // *something* over the support disc.
  const t = span > 1e-9 ? Math.min(1, Math.max(0, (v - vMin) / span)) : 1.0
  const lightness = ramp.lowL + (ramp.highL - ramp.lowL) * t
  const alpha = MAX_ALPHA * t
  const [r, g, b] = hslToRgb(ramp.hue / 360, 0.7, lightness / 100)
  return [r, g, b, Math.round(alpha * 255)]
}

/**
 * Render a GetField response into an offscreen canvas at native pixel
 * resolution. The caller drawImage()s it onto the main map canvas with
 * the appropriate scale (1 metre → projection.s pixels).
 *
 * width/height of the returned canvas equals field.width/field.height.
 * y is flipped: ROS map row 0 is at min_y (canvas y grows downward, so
 * we flip during paint).
 */
export function paintFieldToCanvas(
  field: GetFieldResponse,
  ramp: RampSpec,
): HTMLCanvasElement | null {
  if (field.width <= 0 || field.height <= 0) return null
  const canvas = document.createElement('canvas')
  canvas.width = field.width
  canvas.height = field.height
  const ctx = canvas.getContext('2d')
  if (!ctx) return null
  const img = ctx.createImageData(field.width, field.height)
  for (let j = 0; j < field.height; j++) {
    for (let i = 0; i < field.width; i++) {
      const srcIdx = j * field.width + i
      // Flip y for canvas: ROS row 0 is bottom, canvas row 0 is top.
      const dstRow = field.height - 1 - j
      const di = (dstRow * field.width + i) * 4
      const v = field.values[srcIdx]
      const [r, g, b, a] = valueToRgba(v, field.value_min, field.value_max, ramp)
      img.data[di + 0] = r
      img.data[di + 1] = g
      img.data[di + 2] = b
      img.data[di + 3] = a
    }
  }
  ctx.putImageData(img, 0, 0)
  return canvas
}

/** Standard HSL → RGB. Inputs in [0,1]; outputs in [0,255]. */
function hslToRgb(h: number, s: number, l: number): [number, number, number] {
  if (s === 0) {
    const c = Math.round(l * 255)
    return [c, c, c]
  }
  const q = l < 0.5 ? l * (1 + s) : l + s - l * s
  const p = 2 * l - q
  const f = (t: number) => {
    if (t < 0) t += 1
    if (t > 1) t -= 1
    if (t < 1 / 6) return p + (q - p) * 6 * t
    if (t < 1 / 2) return q
    if (t < 2 / 3) return p + (q - p) * (2 / 3 - t) * 6
    return p
  }
  return [
    Math.round(f(h + 1 / 3) * 255),
    Math.round(f(h) * 255),
    Math.round(f(h - 1 / 3) * 255),
  ]
}

/** CSS-friendly HSL string for the ramp's high-saturation colour. Used
 * for tag pin fills + the legend gradient endpoints. */
export function rampCssColor(ramp: RampSpec, t: number, alpha = 1): string {
  const tt = Math.min(1, Math.max(0, t))
  const lightness = ramp.lowL + (ramp.highL - ramp.lowL) * tt
  return `hsla(${ramp.hue}, 70%, ${lightness}%, ${alpha})`
}

/** A 1×N gradient strip suitable as the legend's background-image. */
export function rampGradientCss(ramp: RampSpec): string {
  const stops: string[] = []
  for (let i = 0; i <= 8; i++) {
    const t = i / 8
    stops.push(`${rampCssColor(ramp, t, MAX_ALPHA * t)} ${(t * 100).toFixed(0)}%`)
  }
  return `linear-gradient(to right, ${stops.join(', ')})`
}
