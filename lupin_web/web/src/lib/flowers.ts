/**
 * Flower-species presentation helpers — colours + labels for the YOLO tulip
 * classes the perception aggregator co-locates with tags. Kept separate from
 * the abiotic sensor ramps (heatmap.ts) and the greenhouse-health tulip
 * (tulip-health.ts): species is a categorical type, not a continuous field.
 */

import type { TulipState } from '@/lib/tulip-health'

/** Fill colour per known species; a neutral grey for anything unrecognised. */
export const SPECIES_COLORS: Record<string, string> = {
  tulip_red: '#e23a3a',
  tulip_white: '#e8e8ee',
  tulip_pink: '#ec6aa6',
}
const SPECIES_FALLBACK = '#9aa0aa'

export function speciesColor(species: string): string {
  return SPECIES_COLORS[species] ?? SPECIES_FALLBACK
}

/** Short human label, e.g. "tulip_red" → "Red tulip". */
export function speciesLabel(species: string): string {
  if (!species) return 'Unclassified'
  const m = /^tulip_(\w+)$/.exec(species)
  if (m) return `${m[1][0].toUpperCase()}${m[1].slice(1)} tulip`
  return species
}

/** Outline/badge colour per health state, shared by the map ring + table chip. */
export const HEALTH_COLORS: Record<TulipState, string> = {
  healthy: '#3ec46d',
  stressed: '#e0b341',
  critical: '#e23a3a',
  no_data: '#6b7280',
}

export function healthLabel(state: TulipState): string {
  switch (state) {
    case 'healthy': return 'Healthy'
    case 'stressed': return 'Stressed'
    case 'critical': return 'Critical'
    default: return 'No data'
  }
}
