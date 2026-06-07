import { useCallback, useEffect, useRef, useState } from 'react'

import { useService, useTopic } from '@/lib/ros'
import {
  LUPIN_SRV,
  ROS_TYPE,
  timeToSec,
  type GetFieldRequest,
  type GetFieldResponse,
  type TwinSensor,
  type TwinState,
  type TwinTagState,
} from '@/types/ros'

/** A latched /twin/state frame stops being trustworthy this long after the
 * last message: the twin node crashed or rosbridge wedged (a documented
 * failure mode) and the frozen frame's stale_seconds no longer advances. */
export const TWIN_STALE_AFTER_MS = 3000

export interface TwinSnapshot {
  /** Latest /twin/state, or null until the first message arrives. */
  state: TwinState | null
  /** True once no /twin/state message has arrived for TWIN_STALE_AFTER_MS.
   * Consumers MUST drop any "fresh/green" styling when this is set — the
   * latched frame is frozen and would otherwise read as live. False before
   * the first message (render the empty state, not a stale one). */
  stale: boolean
}

/**
 * Live snapshot of every tag the digital-twin node has seen. Updates at
 * 1 Hz (matching the twin's TwinState publish rate). The wire format
 * survives an HMI page reload because the twin publishes with
 * RELIABLE+TRANSIENT_LOCAL depth 1.
 *
 * `state` is null until the first message arrives — the HMI should render an
 * empty-state placeholder ("Awaiting first observation") in that case. A
 * watchdog flips `stale` true when /twin/state stops flowing so consumers can
 * grey out instead of presenting the frozen frame as current.
 */
export function useTwinState(): TwinSnapshot {
  const [state, setState] = useState<TwinState | null>(null)
  const [stale, setStale] = useState(false)
  const receivedAtRef = useRef<number | null>(null)
  useTopic<TwinState>('/twin/state', ROS_TYPE.TwinState, {
    onMessage: (msg) => {
      receivedAtRef.current = performance.now()
      setStale(false)
      setState(msg)
    },
  })
  // Watchdog: flip `stale` when messages stop. 1 Hz tick lands the flip within
  // ~1 s of the threshold; setState bails on an unchanged value, so this is a
  // no-op render while the twin is healthy.
  useEffect(() => {
    const id = window.setInterval(() => {
      const at = receivedAtRef.current
      setStale(at != null && performance.now() - at > TWIN_STALE_AFTER_MS)
    }, 1000)
    return () => window.clearInterval(id)
  }, [])
  return { state, stale }
}

/**
 * Live age of a tag's most recent reading, in seconds. Prefers the absolute
 * `last_observed` stamp — which keeps advancing even after the twin stops
 * publishing — over the publish-frozen `stale_seconds`. BUT `last_observed` is
 * only a real wall-clock time on hardware (node clock == wall); under
 * `use_sim_time` the twin stamps it with the SIM clock (a few hundred seconds
 * since sim start), so `Date.now()/1000 - last_observed` would read ~56 years
 * (the "494680 h ago" bug). Guard: trust `last_observed` only when it's a
 * plausible wall timestamp (after ~2001); otherwise fall back to
 * `stale_seconds`, which the twin computes entirely in the ROS clock domain
 * and is therefore correct in sim AND hardware.
 */
export function tagAgeSeconds(t: TwinTagState, nowSec: number = Date.now() / 1000): number {
  const observed = timeToSec(t.last_observed)
  // Unix seconds after ~2001-09; sim-clock stamps (hundreds of seconds) fall
  // below this, so they route to the stale_seconds fallback.
  const WALL_EPOCH_FLOOR = 1_000_000_000
  if (observed > WALL_EPOCH_FLOOR) return Math.max(0, nowSec - observed)
  return t.stale_seconds
}

/** Re-renders the caller ~every `periodMs` with the current wall-clock time in
 * seconds, so "X seconds ago" displays keep advancing even when no new twin
 * message arrives (e.g. while the twin is wedged). */
export function useNowSeconds(periodMs = 1000): number {
  const [now, setNow] = useState(() => Date.now() / 1000)
  useEffect(() => {
    const id = window.setInterval(() => setNow(Date.now() / 1000), periodMs)
    return () => window.clearInterval(id)
  }, [periodMs])
  return now
}

/**
 * Lookup helper for one tag. Returns null when the tag isn't in the latest
 * twin snapshot or when the snapshot itself hasn't arrived. Components that
 * iterate every tag should consume the array directly; this is for hover
 * tooltips and the per-tag selector flow.
 */
export function useTwinTag(tagId: string | null | undefined): TwinTagState | null {
  const { state } = useTwinState()
  if (!state || !tagId) return null
  return state.tags.find((t) => t.tag_id === tagId) ?? null
}

/** Treat orientation.w === 0 as "no pose observed" — see TwinTagState.msg. */
export function tagHasPose(t: TwinTagState | null | undefined): boolean {
  if (!t) return false
  return t.pose.orientation.w !== 0
}

interface FieldRequestArgs {
  /** Active sensor channel; must match what the bridge supplies. */
  sensor: TwinSensor
  /** Map-frame extent. Caller usually derives this from the current
   * OccupancyGrid (so the field paints the same area the SLAM map
   * covers). When undefined the hook stays in a quiescent state. */
  bbox: { minX: number; minY: number; maxX: number; maxY: number } | null
  /** Cell size, metres. 0.25 is the brief's default. */
  resolution: number
  /** When false, the hook holds the last-good response and stops polling.
   * Use this to pause field re-fetching when the heat-map layer is off. */
  enabled?: boolean
  /** Re-poll period. Default 5 s — fields change slowly, no point burning
   * service-call bandwidth at 1 Hz. */
  refreshIntervalMs?: number
}

interface FieldHookResult {
  field: GetFieldResponse | null
  loading: boolean
  error: string | null
  refresh: () => void
}

/**
 * On-demand IDW field for a sensor over a bbox. Calls /twin/get_field on
 * mount, on argument change, and on a refresh interval (default 5 s);
 * exposes a manual refresh() too for the rare case the operator wants
 * to force a re-fetch.
 *
 * NaN-encoding contract: rosbridge serialises NaN floats as JSON `null`
 * (no other choice — JSON has no NaN literal). Consumers that compute on
 * field.values therefore treat null as "no data" — the renderer paints
 * nothing for those cells, which is what keeps the field honest.
 */
export function useTwinField(args: FieldRequestArgs): FieldHookResult {
  const callGetField = useService<GetFieldRequest, GetFieldResponse>(
    '/twin/get_field', LUPIN_SRV.GetField,
  )
  const [field, setField] = useState<GetFieldResponse | null>(null)
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)

  // Stash the latest args in a ref so the polling timer can fetch with
  // up-to-date params without restarting on every arg change.
  const argsRef = useRef(args)
  argsRef.current = args

  const fetchOnce = useCallback(async () => {
    const cur = argsRef.current
    if (!cur.bbox || cur.enabled === false) return
    setLoading(true)
    try {
      const res = await callGetField({
        sensor_type: cur.sensor,
        resolution: cur.resolution,
        bbox_min_x: cur.bbox.minX,
        bbox_min_y: cur.bbox.minY,
        bbox_max_x: cur.bbox.maxX,
        bbox_max_y: cur.bbox.maxY,
      })
      if (!res.ok) {
        setError(res.error_message || 'twin/get_field rejected')
      } else {
        setError(null)
        setField(res)
      }
    } catch (e: unknown) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setLoading(false)
    }
  }, [callGetField])

  // Re-fetch when the request shape changes meaningfully.
  const sig = sigOfArgs(args)
  useEffect(() => {
    void fetchOnce()
  }, [fetchOnce, sig])

  // Periodic refresh while enabled.
  useEffect(() => {
    if (args.enabled === false || !args.bbox) return
    const interval = Math.max(500, args.refreshIntervalMs ?? 5000)
    const id = window.setInterval(() => { void fetchOnce() }, interval)
    return () => window.clearInterval(id)
  }, [args.enabled, args.bbox, args.refreshIntervalMs, fetchOnce])

  return { field, loading, error, refresh: fetchOnce }
}

function sigOfArgs(a: FieldRequestArgs): string {
  if (!a.bbox) return `${a.sensor}|${a.resolution}|none|${a.enabled ?? true}`
  return [
    a.sensor,
    a.resolution.toFixed(4),
    a.bbox.minX.toFixed(3), a.bbox.minY.toFixed(3),
    a.bbox.maxX.toFixed(3), a.bbox.maxY.toFixed(3),
    a.enabled ?? true,
  ].join('|')
}
