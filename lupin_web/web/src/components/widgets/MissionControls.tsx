import {
  AlertOctagon,
  Loader2,
  Pause,
  Play,
  Power,
  Radar,
  SkipForward,
} from 'lucide-react'
import { useState } from 'react'

import { Button } from '@/components/ui/button'
import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from '@/components/ui/card'
import {
  canStartMission,
  formatMissionPhase,
  isMissionActive,
  useMissionServices,
  useMissionState,
} from '@/lib/mission'
import { cn } from '@/lib/utils'

interface MissionControlsProps {
  className?: string
}

/**
 * Operator surface for the mission orchestrator. v1 ships a single Start
 * (no station picker) per the brief — `tag_sequence` defaults to the
 * orchestrator's `tag_locations.json`. Pause / Resume / Skip / Abort map
 * 1:1 to the four `/mission/*` Trigger services.
 *
 * All state-derived enabling/disabling lives here, not in the orchestrator,
 * so the user can still hammer a button on a stale state — the service
 * itself is the source of truth and rejection messages surface inline.
 */
export function MissionControls({ className }: MissionControlsProps) {
  const state = useMissionState()
  const services = useMissionServices()
  const [busy, setBusy] = useState<string | null>(null)
  const [feedback, setFeedback] = useState<{ tone: 'ok' | 'err'; msg: string } | null>(null)
  const [discoveryGoal, setDiscoveryGoal] = useState(5)

  const lifecycle = state?.lifecycle_state ?? 'BOOT'
  const phaseLabel = formatMissionPhase(state)
  const active = isMissionActive(state)
  const paused = state?.paused ?? false
  const estopped = state?.estop_engaged ?? false
  const inspecting = lifecycle === 'INSPECTING'

  const run = async (
    name: string,
    fn: () => Promise<{ success?: boolean; accepted?: boolean; error_message?: string; message?: string }>,
  ) => {
    if (busy) return
    setBusy(name)
    setFeedback(null)
    try {
      const res = await fn()
      const ok = res.accepted ?? res.success ?? false
      const err = res.error_message || res.message || ''
      if (ok) {
        setFeedback({ tone: 'ok', msg: err || `${name} accepted` })
      } else {
        setFeedback({ tone: 'err', msg: err || `${name} rejected` })
      }
    } catch (e: unknown) {
      const msg = e instanceof Error ? e.message : String(e)
      setFeedback({ tone: 'err', msg: `${name} call failed: ${msg}` })
    } finally {
      setBusy(null)
    }
  }

  const startDisabled = !canStartMission(state) || busy !== null
  const pauseDisabled = !active || paused || busy !== null
  const resumeDisabled = !active || (!paused && !estopped) || busy !== null
  const skipDisabled = !inspecting || busy !== null
  const abortDisabled = !active || busy !== null

  return (
    <Card className={cn('flex flex-col', className)}>
      <CardHeader>
        <CardTitle>
          Mission Control
          <span className="tag tag-accent ml-auto">PNL-MIS-01</span>
        </CardTitle>
        <CardDescription className="font-mono text-xs">
          {state ? phaseLabel : 'awaiting /mission/state…'}
        </CardDescription>
      </CardHeader>
      <CardContent className="flex flex-col gap-3">
        <StatusRow state={state} />

        <div className="flex flex-wrap items-center gap-2">
          {/* PRIMARY — the full autonomous run: discover N tags, then monitor each.
              This is the demo button, so it gets the filled/accent variant. */}
          <div className="inline-flex items-center gap-1">
            <Button
              size="sm"
              variant="default"
              disabled={startDisabled}
              onClick={() => run('explore', () => services.startExploration(discoveryGoal))}
              className="h-9 px-4"
              title={`Autonomous run: explore the greenhouse until ${discoveryGoal} tags are found, then monitor each one. No prior map needed — this is the full demo run.`}
            >
              {busy === 'explore'
                ? <Loader2 className="h-4 w-4 animate-spin" />
                : <Radar className="h-4 w-4" />}
              Explore &amp; monitor
            </Button>
            <input
              type="number"
              min={1}
              max={50}
              value={discoveryGoal}
              disabled={startDisabled}
              onChange={(e) => setDiscoveryGoal(Math.max(1, Math.min(50, Number(e.target.value) || 1)))}
              className="h-9 w-14 rounded-sm border border-hairline bg-background/40 px-2 text-center font-mono text-[12px] disabled:opacity-50"
              title="number of tags to discover before switching to monitoring"
              aria-label="discovery goal (tags to find)"
            />
          </div>

          {/* SECONDARY — revisit already-known tags, no discovery phase. Muted so it
              doesn't read as the primary action (it is NOT the demo run). */}
          <Button
            size="sm"
            variant="secondary"
            disabled={startDisabled}
            onClick={() => run('start', () => services.start())}
            className="h-9 px-4"
            title="Patrol the already-known tags in order (InspectionMission). It does NOT explore — use only when the tags are already on the map."
          >
            {busy === 'start'
              ? <Loader2 className="h-4 w-4 animate-spin" />
              : <Play className="h-4 w-4" />}
            Patrol known tags
          </Button>

          {paused ? (
            <Button
              size="sm"
              variant="secondary"
              disabled={resumeDisabled}
              onClick={() => run('resume', services.resume)}
              className="h-9"
            >
              {busy === 'resume'
                ? <Loader2 className="h-4 w-4 animate-spin" />
                : <Play className="h-4 w-4" />}
              Resume
            </Button>
          ) : (
            <Button
              size="sm"
              variant="secondary"
              disabled={pauseDisabled}
              onClick={() => run('pause', services.pause)}
              className="h-9"
            >
              {busy === 'pause'
                ? <Loader2 className="h-4 w-4 animate-spin" />
                : <Pause className="h-4 w-4" />}
              Pause
            </Button>
          )}

          <Button
            size="sm"
            variant="outline"
            disabled={skipDisabled}
            onClick={() => run('skip', services.skipCurrent)}
            className="h-9"
          >
            {busy === 'skip'
              ? <Loader2 className="h-4 w-4 animate-spin" />
              : <SkipForward className="h-4 w-4" />}
            Skip
          </Button>

          <Button
            size="sm"
            variant="destructive"
            disabled={abortDisabled}
            onClick={() => run('abort', services.abort)}
            className="ml-auto h-9"
          >
            {busy === 'abort'
              ? <Loader2 className="h-4 w-4 animate-spin" />
              : <Power className="h-4 w-4" />}
            Abort
          </Button>
        </div>
        <p className="text-[11px] leading-snug text-muted-foreground">
          <span className="font-medium text-foreground">Explore &amp; monitor</span>{' '}
          runs the full autonomous mission (discover N tags, then monitor each) — use this
          for the demo.{' '}
          <span className="font-medium text-foreground">Patrol known tags</span>{' '}
          only revisits tags already on the map.
        </p>

        {feedback && (
          <div
            className={cn(
              'flex items-start gap-2 rounded-sm border px-2.5 py-1.5 text-[11px]',
              feedback.tone === 'err'
                ? 'border-destructive/50 bg-destructive/10 text-destructive-foreground'
                : 'border-primary/30 bg-primary/10 text-foreground',
            )}
          >
            {feedback.tone === 'err' && <AlertOctagon className="mt-px h-3.5 w-3.5 shrink-0" />}
            <span className="font-mono">{feedback.msg}</span>
          </div>
        )}

        {state?.last_error && lifecycle !== 'FAULT' && (
          <div className="rounded-sm border border-warning/30 bg-warning/5 px-2.5 py-1 text-[11px] text-warning">
            <span className="tag mr-2">last error</span>
            <span className="font-mono">{state.last_error}</span>
          </div>
        )}
      </CardContent>
    </Card>
  )
}

function StatusRow({ state }: { state: ReturnType<typeof useMissionState> }) {
  if (!state) {
    return (
      <div className="grid grid-cols-2 gap-2 sm:grid-cols-4">
        {(['lifecycle', 'phase', 'target', 'progress'] as const).map((k) => (
          <Cell key={k} label={k} value="—" />
        ))}
      </div>
    )
  }
  const {
    lifecycle_state,
    mission_phase,
    current_target,
    targets_completed,
    targets_total,
    targets_failed,
    targets_unreachable,
    targets_skipped,
    tags_discovered,
    discovery_goal,
    paused,
    estop_engaged,
  } = state
  const exploring = discovery_goal > 0
  // During an exploration mission the meaningful progress metric is
  // tags-discovered/N; otherwise it's the inspection completed/total.
  const progress = exploring
    ? `${tags_discovered} / ${discovery_goal} found`
    : targets_total > 0
      ? `${targets_completed} / ${targets_total}`
      : '— / —'
  const issues = targets_failed + targets_unreachable + targets_skipped

  return (
    <div className="space-y-2">
      <div className="grid grid-cols-2 gap-2 sm:grid-cols-4">
        <Cell label="lifecycle" value={lifecycle_state} accent />
        <Cell label="phase" value={mission_phase || '—'} />
        <Cell label="target" value={current_target || '—'} mono />
        <Cell label={exploring ? 'discovered' : 'progress'} value={progress} mono />
      </div>
      {(paused || estop_engaged || issues > 0) && (
        <div className="flex flex-wrap gap-2">
          {estop_engaged && <Pill tone="err">e-stop engaged</Pill>}
          {paused && <Pill tone="warn">paused</Pill>}
          {targets_failed > 0 && <Pill tone="warn">{targets_failed} failed</Pill>}
          {targets_unreachable > 0 && <Pill tone="warn">{targets_unreachable} unreachable</Pill>}
          {targets_skipped > 0 && <Pill tone="muted">{targets_skipped} skipped</Pill>}
        </div>
      )}
    </div>
  )
}

function Cell({
  label, value, mono, accent,
}: { label: string; value: string; mono?: boolean; accent?: boolean }) {
  return (
    <div className="rounded-sm border border-hairline bg-background/30 px-2 py-2">
      <div className="tag">{label}</div>
      <div
        className={cn(
          'mt-1 truncate text-[13px] leading-tight',
          mono && 'font-mono',
          accent ? 'text-primary' : 'text-foreground',
        )}
      >
        {value}
      </div>
    </div>
  )
}

function Pill({ tone, children }: { tone: 'err' | 'warn' | 'muted'; children: React.ReactNode }) {
  return (
    <span
      className={cn(
        'tag rounded-sm border px-1.5 py-0.5 text-[10px]',
        tone === 'err' && 'border-destructive/50 bg-destructive/10 text-destructive-foreground',
        tone === 'warn' && 'border-warning/40 bg-warning/10 text-warning',
        tone === 'muted' && 'border-hairline bg-background/40 text-muted-foreground',
      )}
    >
      {children}
    </span>
  )
}
