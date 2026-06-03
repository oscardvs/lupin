import {
  AlertOctagon, AlertTriangle, ChevronRight, CircleDot, Flag, Hand, Inbox,
  Loader2, Pause, Play, Power, Radio,
} from 'lucide-react'
import { useState } from 'react'

import { Button } from '@/components/ui/button'
import {
  Sheet,
  SheetContent,
  SheetDescription,
  SheetHeader,
  SheetTitle,
} from '@/components/ui/sheet'
import { Tooltip, TooltipContent, TooltipTrigger } from '@/components/ui/tooltip'
import {
  isMissionActive,
  useMissionEventLog,
  useMissionServices,
  useMissionState,
  type MissionEvent,
} from '@/lib/mission'
import { gotoTab } from '@/lib/navigation'
import { cn } from '@/lib/utils'
import type { MissionState } from '@/types/ros'

/**
 * Compact mission status pill rendered in the topbar. Shows the current
 * lifecycle / phase at a glance and exposes the **single-click intervention**
 * affordance from the brief — Take Control pauses the orchestrator AND routes
 * the operator to Teleop. Resume is the inverse on the same button when paused.
 *
 * The Tab-level Teleop gating remains as belt-and-braces, but this is the
 * primary path for "stuck leaf, give me the joystick now" interactions.
 */
export function MissionStrip({ className }: { className?: string }) {
  const state = useMissionState()
  const services = useMissionServices()
  // Drive the event log here at the topbar level so the buffer outlives any
  // tab switch / view re-mount. The log panel just consumes this list.
  const events = useMissionEventLog()
  const [logOpen, setLogOpen] = useState(false)
  const [busy, setBusy] = useState<'pause' | 'resume' | null>(null)
  const [errorMsg, setErrorMsg] = useState<string | null>(null)

  const lifecycle = state?.lifecycle_state
  const active = isMissionActive(state)
  const paused = state?.paused ?? false
  const estopped = state?.estop_engaged ?? false

  const onTakeControl = async () => {
    if (busy) return
    setBusy('pause')
    setErrorMsg(null)
    try {
      const res = await services.pause()
      if (res.success) {
        gotoTab('teleop')
      } else {
        setErrorMsg(res.message || 'pause rejected')
      }
    } catch (e: unknown) {
      setErrorMsg(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(null)
    }
  }

  const onResume = async () => {
    if (busy) return
    setBusy('resume')
    setErrorMsg(null)
    try {
      const res = await services.resume()
      if (!res.success) setErrorMsg(res.message || 'resume rejected')
    } catch (e: unknown) {
      setErrorMsg(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(null)
    }
  }

  // BOOT or no telemetry yet — render nothing rather than a half-truth pill.
  if (!state || lifecycle === 'BOOT') {
    return null
  }

  return (
    <div className={cn('hidden items-center gap-2 self-center md:flex', className)}>
      <Tooltip>
        <TooltipTrigger asChild>
          <button
            type="button"
            onClick={() => setLogOpen(true)}
            aria-label="Open mission event log"
            className="flex items-center"
          >
            <PhaseBadge state={state} interactive />
          </button>
        </TooltipTrigger>
        <TooltipContent side="bottom">
          {events.length > 0
            ? `Mission log · ${events.length} event${events.length === 1 ? '' : 's'}`
            : 'Open mission log'}
        </TooltipContent>
      </Tooltip>

      {active && !paused && !estopped && (
        <Tooltip>
          <TooltipTrigger asChild>
            <Button
              size="sm"
              variant="outline"
              onClick={onTakeControl}
              disabled={busy !== null}
              className="h-7 px-2.5 text-[11px]"
            >
              {busy === 'pause'
                ? <Loader2 className="h-3.5 w-3.5 animate-spin" />
                : <Hand className="h-3.5 w-3.5" />}
              Take Control
            </Button>
          </TooltipTrigger>
          <TooltipContent side="bottom">
            Pauses mission, routes you to Teleop
          </TooltipContent>
        </Tooltip>
      )}

      {active && paused && (
        <Tooltip>
          <TooltipTrigger asChild>
            <Button
              size="sm"
              variant="default"
              onClick={onResume}
              disabled={busy !== null}
              className="h-7 px-2.5 text-[11px]"
            >
              {busy === 'resume'
                ? <Loader2 className="h-3.5 w-3.5 animate-spin" />
                : <Play className="h-3.5 w-3.5" />}
              Resume mission
            </Button>
          </TooltipTrigger>
          <TooltipContent side="bottom">
            Resumes the orchestrator from where it paused
          </TooltipContent>
        </Tooltip>
      )}

      {errorMsg && (
        <span className="font-mono text-[10px] text-destructive" role="alert">
          {errorMsg}
        </span>
      )}

      <MissionLogSheet
        open={logOpen}
        onOpenChange={setLogOpen}
        events={events}
        state={state}
      />
    </div>
  )
}

function PhaseBadge({ state, interactive }: { state: MissionState; interactive?: boolean }) {
  const {
    lifecycle_state,
    mission_phase,
    current_target,
    targets_completed,
    targets_total,
    tags_discovered,
    discovery_goal,
    paused,
    estop_engaged,
  } = state

  const tone = (() => {
    if (estop_engaged || lifecycle_state === 'FAULT') return 'err'
    if (paused) return 'warn'
    if (lifecycle_state === 'INSPECTING' || lifecycle_state === 'PREPARE') return 'live'
    if (lifecycle_state === 'EXPLORING' || lifecycle_state === 'MONITORING') return 'live'
    if (lifecycle_state === 'RETURNING') return 'live'
    if (lifecycle_state === 'DONE') return 'ok'
    return 'idle'
  })()

  const lifeColor =
    tone === 'err' ? 'text-destructive-foreground'
    : tone === 'warn' ? 'text-warning'
    : tone === 'live' ? 'text-primary'
    : tone === 'ok' ? 'text-foreground'
    : 'text-muted-foreground'

  const dotColor =
    tone === 'err' ? 'bg-destructive'
    : tone === 'warn' ? 'bg-warning'
    : tone === 'live' ? 'bg-primary animate-pulse'
    : tone === 'ok' ? 'bg-foreground'
    : 'bg-muted-foreground'

  return (
    <div
      className={cn(
        'flex h-7 items-center gap-2 rounded-sm border border-hairline bg-card/40 px-2.5',
        interactive && 'cursor-pointer transition-colors hover:border-primary/50 hover:bg-card/60',
      )}
    >
      <span className={cn('inline-block h-1.5 w-1.5 rounded-full', dotColor)} aria-hidden />
      <span className="tag">mission</span>
      <span className={cn('font-display text-[11px] uppercase tracking-wider', lifeColor)}>
        {paused ? 'PAUSED' : lifecycle_state === 'READY' ? 'IDLE' : lifecycle_state}
      </span>
      {lifecycle_state === 'INSPECTING' && (
        <>
          <span className="h-3 w-px bg-hairline" aria-hidden />
          <span className="tag">{mission_phase || 'NAV'}</span>
          <span className="font-mono text-[11px] text-foreground">
            {current_target || '—'}
          </span>
          {targets_total > 0 && (
            <span className="font-mono text-[11px] text-muted-foreground">
              ({targets_completed + 1}/{targets_total})
            </span>
          )}
        </>
      )}
      {lifecycle_state === 'EXPLORING' && (
        <>
          <span className="h-3 w-px bg-hairline" aria-hidden />
          <span className="tag">discover</span>
          <span className="font-mono text-[11px] text-foreground">
            {tags_discovered}/{discovery_goal || '?'}
          </span>
        </>
      )}
      {lifecycle_state === 'MONITORING' && (
        <>
          <span className="h-3 w-px bg-hairline" aria-hidden />
          <span className="tag">{mission_phase || 'NAV'}</span>
          <span className="font-mono text-[11px] text-foreground">
            {current_target || '—'}
          </span>
        </>
      )}
      {lifecycle_state === 'PREPARE' && mission_phase && (
        <>
          <span className="h-3 w-px bg-hairline" aria-hidden />
          <span className="tag">{mission_phase}</span>
        </>
      )}
      {interactive && (
        <ChevronRight className="ml-1 h-3 w-3 text-muted-foreground/70" aria-hidden />
      )}
    </div>
  )
}

interface MissionLogSheetProps {
  open: boolean
  onOpenChange: (open: boolean) => void
  events: MissionEvent[]
  state: MissionState
}

type LogFilter = 'all' | 'transitions' | 'observations' | 'errors'

function MissionLogSheet({ open, onOpenChange, events, state }: MissionLogSheetProps) {
  const [filter, setFilter] = useState<LogFilter>('all')

  const visible = events.filter((e) => {
    if (filter === 'all') return true
    if (filter === 'observations') return e.kind === 'observation'
    if (filter === 'errors') return e.tone === 'err' || e.kind === 'fault' || e.kind === 'estop'
    // transitions = lifecycle, phase, target, pause
    return e.kind === 'lifecycle' || e.kind === 'phase' || e.kind === 'target' || e.kind === 'pause'
  })

  // Newest first.
  const ordered = [...visible].reverse()

  const counts = {
    total: events.length,
    obs: events.filter((e) => e.kind === 'observation').length,
    err: events.filter((e) => e.tone === 'err').length,
  }

  return (
    <Sheet open={open} onOpenChange={onOpenChange}>
      <SheetContent
        side="right"
        className="flex w-full flex-col gap-0 p-0 sm:max-w-md"
      >
        <SheetHeader className="space-y-1 border-b border-hairline p-4">
          <div className="flex items-baseline gap-2">
            <SheetTitle className="font-display text-lg leading-none">
              Mission log
            </SheetTitle>
            <span className="tag tag-accent">PNL-LOG-01</span>
          </div>
          <SheetDescription className="font-mono text-[11px]">
            mission_id={state.mission_id || '—'} · {state.lifecycle_state}
            {state.mission_phase && <> · {state.mission_phase}</>}
          </SheetDescription>
          <div className="flex flex-wrap gap-1.5 pt-1">
            <FilterChip current={filter} value="all" onClick={setFilter}>
              all <span className="opacity-60">{counts.total}</span>
            </FilterChip>
            <FilterChip current={filter} value="transitions" onClick={setFilter}>
              transitions
            </FilterChip>
            <FilterChip current={filter} value="observations" onClick={setFilter}>
              observations <span className="opacity-60">{counts.obs}</span>
            </FilterChip>
            <FilterChip current={filter} value="errors" onClick={setFilter}>
              errors {counts.err > 0 && <span className="text-destructive">{counts.err}</span>}
            </FilterChip>
          </div>
        </SheetHeader>

        <div className="flex-1 min-h-0 overflow-y-auto">
          {ordered.length === 0 ? (
            <EmptyLog filter={filter} />
          ) : (
            <ol className="divide-hairline divide-y">
              {ordered.map((evt, i) => (
                <EventRow key={`${evt.at}-${i}`} evt={evt} />
              ))}
            </ol>
          )}
        </div>

        <div className="border-t border-hairline p-3 text-[10px] text-muted-foreground">
          <span className="tag">buffer</span>{' '}
          <span className="font-mono">
            {events.length} / {EVENT_BUFFER_DISPLAY_CAP}
          </span>
          {' · '}
          <span>oldest events drop off as new ones arrive</span>
        </div>
      </SheetContent>
    </Sheet>
  )
}

const EVENT_BUFFER_DISPLAY_CAP = 200

function FilterChip<V extends string>({
  current, value, onClick, children,
}: {
  current: V
  value: V
  onClick: (v: V) => void
  children: React.ReactNode
}) {
  const active = current === value
  return (
    <button
      type="button"
      onClick={() => onClick(value)}
      className={cn(
        'tag rounded-sm border px-1.5 py-0.5 transition-colors',
        active
          ? 'border-primary/50 bg-primary/15 text-primary'
          : 'border-hairline bg-background/40 text-muted-foreground hover:text-foreground',
      )}
    >
      {children}
    </button>
  )
}

function EmptyLog({ filter }: { filter: LogFilter }) {
  return (
    <div className="m-6 flex flex-col items-center gap-2 rounded-sm border border-dashed border-hairline p-6 text-center text-muted-foreground">
      <Inbox className="h-5 w-5 opacity-60" />
      <div className="text-[12px]">No events {filter !== 'all' && `(${filter})`}</div>
      <div className="text-[10px] opacity-70">
        Start a patrol from <span className="font-mono">Map / Nav</span> to populate the log.
      </div>
    </div>
  )
}

function EventRow({ evt }: { evt: MissionEvent }) {
  const Icon = iconForKind(evt.kind)
  const toneClass =
    evt.tone === 'ok' ? 'text-primary'
    : evt.tone === 'warn' ? 'text-warning'
    : evt.tone === 'err' ? 'text-destructive'
    : 'text-foreground'
  return (
    <li className="flex gap-3 px-4 py-2.5 text-[12px]">
      <span className={cn('mt-0.5 shrink-0', toneClass)}>
        <Icon className="h-3.5 w-3.5" />
      </span>
      <div className="flex-1 min-w-0">
        <div className="flex items-baseline gap-2">
          <span className={cn('font-mono', toneClass)}>{evt.label}</span>
          <span className="ml-auto shrink-0 font-mono text-[10px] text-muted-foreground">
            {formatClock(evt.at)}
          </span>
        </div>
        {evt.detail && (
          <div className="mt-0.5 truncate font-mono text-[11px] text-muted-foreground">
            {evt.detail}
          </div>
        )}
      </div>
    </li>
  )
}

function iconForKind(kind: MissionEvent['kind']) {
  switch (kind) {
    case 'lifecycle': return Flag
    case 'phase': return Radio
    case 'target': return CircleDot
    case 'pause': return Pause
    case 'estop': return Power
    case 'fault': return AlertOctagon
    case 'notice': return AlertTriangle
    case 'observation': return Inbox
  }
}

function formatClock(ms: number): string {
  const d = new Date(ms)
  const pad = (n: number) => n.toString().padStart(2, '0')
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`
}
