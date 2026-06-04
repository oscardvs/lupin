import { useCallback, useRef, useState } from 'react'

import { useService, useTopic } from '@/lib/ros'
import {
  LUPIN_SRV,
  OBSERVATION_STATUS,
  type MissionState,
  type Observation,
  ROS_TYPE,
} from '@/types/ros'

/**
 * Live snapshot of the mission orchestrator. Updates at 5 Hz when connected
 * (matches `lupin_mission/node.py`'s `_state_pub`). `null` means we haven't
 * received a state yet — UI components should render an "—" placeholder
 * instead of guessing IDLE.
 *
 * The orchestrator is always alive when running, even between missions —
 * `lifecycle_state` cycles READY → ... → READY rather than the topic going
 * silent. So a missing snapshot is genuinely "no rosbridge connection",
 * not "no mission running".
 */
export function useMissionState(): MissionState | null {
  const [state, setState] = useState<MissionState | null>(null)
  useTopic<MissionState>('/mission/state', ROS_TYPE.MissionState, {
    onMessage: setState,
  })
  return state
}

export interface MissionServices {
  start: (tagSequence?: string[]) => Promise<{ accepted: boolean; mission_id: string; error_message: string }>
  /** Start an autonomous ExplorationMission: explore until `goal` tags are
   * discovered, then continuously monitor them. */
  startExploration: (goal: number) => Promise<{ accepted: boolean; mission_id: string; error_message: string }>
  pause: () => Promise<{ success: boolean; message: string }>
  resume: () => Promise<{ success: boolean; message: string }>
  abort: () => Promise<{ success: boolean; message: string }>
  skipCurrent: () => Promise<{ success: boolean; message: string }>
}

interface StartMissionRequest {
  mission_type: string
  tag_sequence: string[]
  discovery_goal: number
}

interface StartMissionResponse {
  accepted: boolean
  mission_id: string
  error_message: string
}

interface TriggerResponse {
  success: boolean
  message: string
}

/**
 * Bound service callers for the orchestrator's operator surface. v1 only
 * supports `mission_type: "InspectionMission"` — the orchestrator rejects
 * anything else, so this is hard-coded rather than threaded through the UI.
 */
export function useMissionServices(): MissionServices {
  const startSrv = useService<StartMissionRequest, StartMissionResponse>(
    '/mission/start',
    LUPIN_SRV.StartMission,
  )
  const pauseSrv = useService<Record<string, never>, TriggerResponse>(
    '/mission/pause',
    LUPIN_SRV.Trigger,
  )
  const resumeSrv = useService<Record<string, never>, TriggerResponse>(
    '/mission/resume',
    LUPIN_SRV.Trigger,
  )
  const abortSrv = useService<Record<string, never>, TriggerResponse>(
    '/mission/abort',
    LUPIN_SRV.Trigger,
  )
  const skipSrv = useService<Record<string, never>, TriggerResponse>(
    '/mission/skip_current',
    LUPIN_SRV.Trigger,
  )

  const start = useCallback(
    (tagSequence: string[] = []) =>
      startSrv({ mission_type: 'InspectionMission', tag_sequence: tagSequence, discovery_goal: 0 }),
    [startSrv],
  )
  const startExploration = useCallback(
    (goal: number) =>
      startSrv({ mission_type: 'ExplorationMission', tag_sequence: [], discovery_goal: goal }),
    [startSrv],
  )
  const pause = useCallback(() => pauseSrv({}), [pauseSrv])
  const resume = useCallback(() => resumeSrv({}), [resumeSrv])
  const abort = useCallback(() => abortSrv({}), [abortSrv])
  const skipCurrent = useCallback(() => skipSrv({}), [skipSrv])

  return { start, startExploration, pause, resume, abort, skipCurrent }
}

/**
 * Latest `Observation` per `tag_reading.tag_id`, accumulated for the lifetime
 * of the page. Returned as a Map so components can render in tag-id order
 * (numeric_string_sort matches the orchestrator's default ordering).
 *
 * Non-tag observations (kind != TAG_READING) are ignored in v1 — the brief's
 * digital-twin sink only cares about tag readings, and the message stubs the
 * other kinds in advance to stay non-breaking. When a future mission emits
 * those, this hook gets a sibling `useFlowerObservations` etc. rather than
 * being widened.
 */
export function useTagObservations(): Map<string, Observation> {
  const [obs, setObs] = useState<Map<string, Observation>>(new Map())
  useTopic<Observation>('/floranova/observations', ROS_TYPE.Observation, {
    onMessage: (msg) => {
      const tagId = msg.tag_reading?.tag_id
      if (!tagId) return
      setObs((prev) => {
        const existing = prev.get(tagId)
        // Drop out-of-order republishes — the orchestrator stamps each
        // observation with the bridge poll time, so newer-stamp wins.
        if (existing) {
          const a = existing.tag_reading.stamp
          const b = msg.tag_reading.stamp
          const aSec = a.sec + a.nanosec * 1e-9
          const bSec = b.sec + b.nanosec * 1e-9
          if (bSec < aSec) return prev
        }
        const next = new Map(prev)
        next.set(tagId, msg)
        return next
      })
    },
  })
  return obs
}

/** Chronological event captured for the operator-facing mission log. */
export interface MissionEvent {
  /** Wall-clock ms (Date.now()) — used for relative-age formatting. */
  at: number
  kind: 'lifecycle' | 'phase' | 'target' | 'pause' | 'estop' | 'fault' | 'observation' | 'notice'
  /** Short headline shown in the log row. */
  label: string
  /** Optional secondary text (sensor readings, error detail, transitions). */
  detail?: string
  /** Tone hint for the renderer. */
  tone: 'info' | 'ok' | 'warn' | 'err'
}

const EVENT_BUFFER_CAP = 200

/**
 * Build a chronological log of mission events from `/mission/state` transitions
 * and `/floranova/observations` messages. Lives at the Shell level so the log
 * persists across tab navigation — clicking the topbar strip opens a panel
 * that reads this buffer.
 *
 * Buffer is capped at 200 events. The orchestrator's TRANSIENT_LOCAL replay
 * means a fresh page reload will replay the latest snapshot of state and the
 * most recent ~50 observations, so the panel is useful immediately on first
 * open without needing to start a new mission.
 */
export function useMissionEventLog(): MissionEvent[] {
  const [events, setEvents] = useState<MissionEvent[]>([])
  const prevStateRef = useRef<MissionState | null>(null)

  const append = useCallback((evt: MissionEvent) => {
    setEvents((prev) => {
      const next = prev.length >= EVENT_BUFFER_CAP
        ? prev.slice(prev.length - EVENT_BUFFER_CAP + 1)
        : prev.slice()
      next.push(evt)
      return next
    })
  }, [])

  useTopic<MissionState>('/mission/state', ROS_TYPE.MissionState, {
    onMessage: (state) => {
      const prev = prevStateRef.current
      prevStateRef.current = state
      const at = Date.now()

      if (!prev) {
        append({
          at,
          kind: 'lifecycle',
          label: state.lifecycle_state,
          detail: state.mission_id ? `mission_id=${state.mission_id}` : 'first snapshot',
          tone: state.lifecycle_state === 'FAULT' ? 'err' : 'info',
        })
        return
      }
      if (state.lifecycle_state !== prev.lifecycle_state) {
        const tone =
          state.lifecycle_state === 'FAULT' ? 'err'
          : state.lifecycle_state === 'DONE' ? 'ok'
          : 'info'
        append({
          at,
          kind: 'lifecycle',
          label: `${prev.lifecycle_state} → ${state.lifecycle_state}`,
          detail: state.mission_id ? `mission_id=${state.mission_id}` : undefined,
          tone,
        })
      }
      if (state.mission_phase !== prev.mission_phase && state.mission_phase) {
        append({
          at,
          kind: 'phase',
          label: state.mission_phase,
          detail: state.current_target ? `tag=${state.current_target}` : undefined,
          tone: 'info',
        })
      }
      if (
        state.current_target !== prev.current_target &&
        state.current_target &&
        (state.lifecycle_state === 'INSPECTING' || state.lifecycle_state === 'MONITORING')
      ) {
        append({
          at,
          kind: 'target',
          label: `target ${state.current_target}`,
          detail: state.lifecycle_state === 'MONITORING'
            ? 'monitoring'
            : `${state.targets_completed + 1} / ${state.targets_total}`,
          tone: 'info',
        })
      }
      if (state.tags_discovered !== prev.tags_discovered && state.lifecycle_state === 'EXPLORING') {
        append({
          at,
          kind: 'target',
          label: `discovered tag (${state.tags_discovered}/${state.discovery_goal})`,
          tone: 'ok',
        })
      }
      if (state.paused !== prev.paused) {
        append({
          at,
          kind: 'pause',
          label: state.paused ? 'paused' : 'resumed',
          tone: 'warn',
        })
      }
      if (state.estop_engaged !== prev.estop_engaged) {
        append({
          at,
          kind: 'estop',
          label: state.estop_engaged ? 'e-stop engaged' : 'e-stop cleared',
          tone: state.estop_engaged ? 'err' : 'ok',
        })
      }
      if (state.last_error && state.last_error !== prev.last_error) {
        const sev = state.last_event_severity ?? 1
        const isFault = state.lifecycle_state === 'FAULT' || sev >= 2
        append({
          at,
          kind: isFault ? 'fault' : 'notice',
          label: state.last_event || state.last_error,
          detail: state.last_error,
          tone: isFault ? 'err' : sev === 0 ? 'info' : 'warn',
        })
      }
    },
  })

  useTopic<Observation>('/floranova/observations', ROS_TYPE.Observation, {
    onMessage: (msg) => {
      const tagId = msg.tag_reading?.tag_id || '?'
      const status = msg.status
      const tone =
        status === OBSERVATION_STATUS.OK ? 'ok'
        : status === OBSERVATION_STATUS.SCAN_FAILED ? 'warn'
        : status === OBSERVATION_STATUS.SKIPPED ? 'warn'
        : status === OBSERVATION_STATUS.UNREACHABLE ? 'err'
        : 'info'
      const readings = msg.tag_reading?.readings ?? []
      const detail = readings.length > 0
        ? readings.map((r) => `${r.name}=${formatReadingValue(r.value)}`).join(' · ')
        : msg.status_detail || undefined
      append({
        at: Date.now(),
        kind: 'observation',
        label: `tag ${tagId}`,
        detail,
        tone,
      })
    },
  })

  return events
}

function formatReadingValue(v: number): string {
  if (!Number.isFinite(v)) return '—'
  if (Math.abs(v) >= 100) return v.toFixed(0)
  if (Math.abs(v) >= 10) return v.toFixed(1)
  return v.toFixed(2)
}

/** A tag we have personally watched the robot scan during this session. */
export interface TagSighting {
  tagId: string
  /** Map-frame (x, y) snapshot of the robot pose when the Observation arrived. */
  x: number
  y: number
  /** Sensor-poll wall-clock time as advertised in the Observation. */
  stampSec: number
  /** Status from the Observation; lets the renderer colour OK vs failed. */
  status: number
}

/**
 * Map-frame positions of tags the robot has scanned. Prefers each Observation's
 * `tag_pose_in_map` — the tag's OWN map pose, which the orchestrator fills from
 * the discovered-tags feed — so the marker sits where the tag physically is and
 * matches the twin map. Falls back to a snapshot of the live robot pose only
 * when the tag pose is unset (orientation.w === 0, e.g. an older orchestrator or
 * a tag not yet localized) — that's offset from the tag by the standoff
 * distance, but better than no marker.
 *
 * The orchestrator publishes Observations with `RELIABLE + TRANSIENT_LOCAL`
 * depth=50, so a late subscriber gets the backlog. We filter that out by stamp
 * age — if the Observation is older than 5 s when we receive it, the robot-pose
 * fallback would be misleading, so we skip rather than place a stale marker.
 * Only fresh, live observations land on the map.
 */
export function useTagSightings(
  getPose: () => { x: number; y: number; yaw: number } | null,
): TagSighting[] {
  const STALE_OBSERVATION_AGE_MS = 5000
  const [sightings, setSightings] = useState<TagSighting[]>([])
  useTopic<Observation>('/floranova/observations', ROS_TYPE.Observation, {
    onMessage: (msg) => {
      const tagId = msg.tag_reading?.tag_id
      if (!tagId) return
      // Only mark OK scans; UNREACHABLE/SKIPPED have no meaningful "position
      // where we saw it" — the robot may not have reached the station at all.
      if (msg.status !== OBSERVATION_STATUS.OK) return
      const stamp = msg.tag_reading.stamp
      const stampSec = stamp.sec + stamp.nanosec * 1e-9
      if (Math.abs(Date.now() - stampSec * 1000) > STALE_OBSERVATION_AGE_MS) return
      // Prefer the tag's own map pose (orchestrator fills it from the
      // discovered-tags feed) so the marker matches the twin map; fall back to
      // the live robot pose only when it's unset (orientation.w === 0).
      const tagPose = msg.tag_pose_in_map
      let x: number
      let y: number
      if (tagPose && tagPose.orientation.w !== 0) {
        x = tagPose.position.x
        y = tagPose.position.y
      } else {
        const p = getPose()
        if (!p) return
        x = p.x
        y = p.y
      }
      setSightings((prev) => {
        const idx = prev.findIndex((s) => s.tagId === tagId)
        const entry: TagSighting = {
          tagId, x, y, stampSec, status: msg.status,
        }
        if (idx >= 0) {
          const next = prev.slice()
          next[idx] = entry
          return next
        }
        return [...prev, entry]
      })
    },
  })
  return sightings
}

/** Pretty-print helper used by both the topbar strip and the Map/Nav header. */
export function formatMissionPhase(state: MissionState | null): string {
  if (!state) return '—'
  const { lifecycle_state, mission_phase, current_target, targets_completed, targets_total } = state
  switch (lifecycle_state) {
    case 'BOOT': return 'BOOT'
    case 'READY': return 'IDLE · ready'
    case 'PREPARE':
      return mission_phase ? `PREPARE · ${mission_phase.toLowerCase()}` : 'PREPARE'
    case 'EXPLORING': {
      const k = state.tags_discovered
      const N = state.discovery_goal || 1
      return `EXPLORE · ${k}/${N} found`
    }
    case 'INSPECTING': {
      const phase = mission_phase || 'NAV'
      const tag = current_target || '?'
      const k = targets_completed + 1
      const M = targets_total || 1
      return `${phase} · ${tag} (${k}/${M})`
    }
    case 'MONITORING': {
      const phase = mission_phase || 'NAV'
      const tag = current_target || '?'
      return `MONITOR · ${phase} · ${tag}`
    }
    case 'RETURNING': return 'RETURNING'
    case 'DONE': return 'DONE'
    case 'FAULT': return 'FAULT'
    default: return lifecycle_state
  }
}

/** True when the orchestrator is in a state where Start should be enabled. */
export function canStartMission(state: MissionState | null): boolean {
  if (!state) return false
  if (state.estop_engaged) return false
  return state.lifecycle_state === 'READY' || state.lifecycle_state === 'DONE'
}

/** True when the operator should be able to take manual control. */
export function isMissionActive(state: MissionState | null): boolean {
  if (!state) return false
  return (
    state.lifecycle_state === 'PREPARE' ||
    state.lifecycle_state === 'EXPLORING' ||
    state.lifecycle_state === 'INSPECTING' ||
    state.lifecycle_state === 'MONITORING' ||
    state.lifecycle_state === 'RETURNING'
  )
}
