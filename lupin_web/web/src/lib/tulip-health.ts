import type { TwinSensor, TwinTagState } from '@/types/ros'

/**
 * Ideal-range model for the greenhouse-health tulip. Mirrors
 * lupin_twin/config/ideal_ranges.yaml so the HMI doesn't depend on a
 * remote config fetch — they should be kept in sync, but a small drift
 * never breaks operations.
 */
export const IDEAL_RANGES: Record<TwinSensor, { min: number; max: number }> = {
  temperature:    { min: 18, max: 26 },
  humidity:       { min: 40, max: 70 },
  co2:            { min: 400, max: 1000 },
  light:          { min: 200, max: 800 },
  soil_moisture:  { min: 30, max: 60 },
}

const SENSOR_UNITS: Record<TwinSensor, string> = {
  temperature: '°C',
  humidity: '%',
  co2: 'ppm',
  light: 'lux',
  soil_moisture: '%',
}

const SENSOR_LABELS: Record<TwinSensor, string> = {
  temperature: 'temperature',
  humidity: 'humidity',
  co2: 'CO₂',
  light: 'light',
  soil_moisture: 'soil moisture',
}

export type TulipState = 'healthy' | 'stressed' | 'critical' | 'no_data'

export interface TulipHealth {
  state: TulipState
  /** Worst-deviation across all observed tags+sensors, [0, 2] clamped. */
  score: number
  /** Tag id + sensor + value + range driving the state, when state ≠ healthy/no_data. */
  driver: TulipDriver | null
}

export interface TulipDriver {
  tag_id: string
  sensor: TwinSensor
  value: number
  ideal_min: number
  ideal_max: number
  deviation: number
}

/**
 * Compute per-tag, per-sensor deviation, then take the worst across all
 * (tag × sensor) pairs. The brief's "max wins" rule is intentional — one
 * stressed plant should drive the indicator so the operator notices it,
 * not get averaged into health by 22 healthy tags.
 */
export function computeTulipHealth(tags: TwinTagState[]): TulipHealth {
  if (!tags || tags.length === 0) {
    return { state: 'no_data', score: 0, driver: null }
  }

  let worstDev = 0
  let worstDriver: TulipDriver | null = null
  let anyReadings = false

  for (const t of tags) {
    for (const r of t.readings) {
      const range = IDEAL_RANGES[r.name as TwinSensor]
      if (!range) continue
      anyReadings = true
      const dev = sensorDeviation(r.value, range.min, range.max)
      if (dev > worstDev) {
        worstDev = dev
        worstDriver = {
          tag_id: t.tag_id,
          sensor: r.name as TwinSensor,
          value: r.value,
          ideal_min: range.min,
          ideal_max: range.max,
          deviation: dev,
        }
      }
    }
  }

  if (!anyReadings) {
    return { state: 'no_data', score: 0, driver: null }
  }

  const state =
    worstDev >= 0.7 ? 'critical'
    : worstDev >= 0.3 ? 'stressed'
    : 'healthy'

  return { state, score: worstDev, driver: worstDriver }
}

/** Health state for a single tag (the same max-deviation rule, scoped to
 * one tag's readings). Used for the per-row chip in the Greenhouse table and
 * the per-tag map tooltip. */
export function tagHealthState(tag: TwinTagState): TulipState {
  return computeTulipHealth([tag]).state
}

export function sensorDeviation(value: number, min: number, max: number): number {
  if (!Number.isFinite(value)) return 0
  if (value >= min && value <= max) return 0
  const span = Math.max(1e-9, max - min)
  const distance = value < min ? (min - value) : (value - max)
  return Math.min(2, distance / span)
}

/** Human-readable worst-driver string for the hover tooltip. */
export function formatDriver(d: TulipDriver | null): string {
  if (!d) return ''
  const label = SENSOR_LABELS[d.sensor]
  const unit = SENSOR_UNITS[d.sensor]
  const v = formatValue(d.value)
  return `tag-${d.tag_id} ${label} (${v}${unit} vs ideal ${d.ideal_min}–${d.ideal_max}${unit})`
}

function formatValue(v: number): string {
  if (!Number.isFinite(v)) return '—'
  if (Math.abs(v) >= 100) return v.toFixed(0)
  if (Math.abs(v) >= 10) return v.toFixed(1)
  return v.toFixed(2)
}
