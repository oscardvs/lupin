import { Activity, Gauge, Hand, Lock, Play, RotateCcw, Square } from 'lucide-react'
import { useCallback, useEffect, useRef, useState } from 'react'

import { Joystick, type StickValue } from '@/components/widgets/Joystick'
import { TwistReadout } from '@/components/widgets/TwistReadout'
import { ViewShell } from '@/components/system/ViewShell'
import { Button } from '@/components/ui/button'
import { Slider } from '@/components/ui/slider'
import { ESTOP_REASON_LABELS, useCmdVel, useEStop } from '@/lib/estop'
import {
  isMissionActive,
  useMissionServices,
  useMissionState,
} from '@/lib/mission'
import { useIsLg, useIsSm, useIsXl } from '@/lib/responsive'
import { useSettings } from '@/lib/settings'
import { useThrottledRender } from '@/lib/throttle'
import { cn } from '@/lib/utils'
import type { Twist } from '@/types/ros'

const MAX_LINEAR = 0.4 // m/s at scale=1.0
const MAX_ANGULAR = 1.5 // rad/s at scale=1.0
const PUBLISH_HZ = 20

export function TeleopView() {
  const [{ speedScale, cmdVelTopic }, updateSettings] = useSettings()
  const {
    active: estopActive,
    reason: estopReason,
    reset: estopReset,
    trigger: estopTrigger,
  } = useEStop()
  const { publish, blocked } = useCmdVel()
  const missionState = useMissionState()
  const missionServices = useMissionServices()
  const missionLocked =
    isMissionActive(missionState) &&
    !missionState?.paused &&
    !missionState?.estop_engaged
  const [missionBusy, setMissionBusy] = useState<'pause' | 'resume' | null>(null)
  const [missionErr, setMissionErr] = useState<string | null>(null)
  const isSm = useIsSm()
  const isLg = useIsLg()
  const isXl = useIsXl()
  // Sticks grow with the viewport so the console fills desktop instead of
  // floating in a void; phone keeps two ~150px dials side-by-side in one row.
  const stickSize = isXl ? 240 : isLg ? 208 : isSm ? 184 : 150

  const leftRef = useRef<StickValue>({ x: 0, y: 0, active: false })
  const rightRef = useRef<StickValue>({ x: 0, y: 0, active: false })
  const [lastSent, setLastSent] = useState<Twist | null>(null)

  useThrottledRender(10)

  const computeTwist = useCallback((): Twist => {
    const L = leftRef.current
    const R = rightRef.current
    return {
      linear: { x: L.y * MAX_LINEAR * speedScale, y: -L.x * MAX_LINEAR * speedScale, z: 0 },
      angular: { x: 0, y: 0, z: -R.x * MAX_ANGULAR * speedScale },
    }
  }, [speedScale])

  // Held in a ref so the publish loop doesn't restart on every MissionState
  // tick (5 Hz). The loop checks the ref each iteration and bails before
  // sending — fighting the orchestrator's Nav2 cmd_vel during INSPECTING
  // would be a recipe for unpredictable motion on the floor.
  const missionLockedRef = useRef(missionLocked)
  missionLockedRef.current = missionLocked

  useEffect(() => {
    let zeroSentAfterIdle = true
    const id = setInterval(() => {
      if (missionLockedRef.current) {
        // Pin the readout at zero so the operator sees the lock is real.
        if (!zeroSentAfterIdle) {
          setLastSent({ linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } })
          zeroSentAfterIdle = true
        }
        return
      }
      const anyActive = leftRef.current.active || rightRef.current.active
      if (anyActive) {
        const t = computeTwist()
        publish(t)
        setLastSent(t)
        zeroSentAfterIdle = false
      } else if (!zeroSentAfterIdle) {
        const stop: Twist = { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } }
        publish(stop)
        setLastSent(stop)
        zeroSentAfterIdle = true
      }
    }, Math.round(1000 / PUBLISH_HZ))
    return () => clearInterval(id)
  }, [publish, computeTwist])

  const onTakeControl = async () => {
    if (missionBusy) return
    setMissionBusy('pause')
    setMissionErr(null)
    try {
      const res = await missionServices.pause()
      if (!res.success) setMissionErr(res.message || 'pause rejected')
    } catch (e: unknown) {
      setMissionErr(e instanceof Error ? e.message : String(e))
    } finally {
      setMissionBusy(null)
    }
  }

  const onResume = async () => {
    if (missionBusy) return
    setMissionBusy('resume')
    setMissionErr(null)
    try {
      const res = await missionServices.resume()
      if (!res.success) setMissionErr(res.message || 'resume rejected')
    } catch (e: unknown) {
      setMissionErr(e instanceof Error ? e.message : String(e))
    } finally {
      setMissionBusy(null)
    }
  }

  return (
    <ViewShell intent="fit">
      {estopActive ? (
        <div className="reticle relative flex flex-wrap items-center gap-x-3 gap-y-2 rounded-sm border-2 border-destructive bg-destructive/10 px-3 py-2.5 text-sm sm:px-4 sm:py-3">
          <span className="reticle-bl" aria-hidden />
          <span className="reticle-br" aria-hidden />
          <Square className="h-5 w-5 shrink-0 fill-destructive text-destructive" />
          <div className="flex-1 min-w-[12rem]">
            <div className="flex flex-wrap items-baseline gap-x-2 gap-y-1">
              <span className="font-semibold uppercase tracking-[0.16em] text-destructive">
                E-stop engaged
              </span>
              <span className="tag">PNL-EMG-01</span>
            </div>
            <div className="text-xs text-muted-foreground">
              {estopReason ? ESTOP_REASON_LABELS[estopReason] : 'Unknown reason'}
            </div>
          </div>
          <Button
            variant="default"
            size="sm"
            onClick={estopReset}
            className="w-full shrink-0 sm:w-auto"
          >
            <RotateCcw className="mr-2 h-4 w-4" />
            Reset E-stop
          </Button>
        </div>
      ) : null}

      {missionLocked && (
        <MissionLockBanner
          phase={missionState?.mission_phase || ''}
          target={missionState?.current_target || ''}
          busy={missionBusy === 'pause'}
          onTakeControl={onTakeControl}
          err={missionErr}
        />
      )}
      {missionState?.paused && !estopActive && (
        <MissionPausedBanner
          phase={missionState.mission_phase || ''}
          target={missionState.current_target || ''}
          busy={missionBusy === 'resume'}
          onResume={onResume}
          err={missionErr}
        />
      )}

      {/* primary console region — two stick stations + readout HUD. Flexes to
          fill the viewport so desktop has no void; opaque ink tile, not glass. */}
      <div
        className={cn(
          'reticle relative flex min-h-0 flex-1 flex-col rounded-sm border border-hairline bg-ink-2 p-3 sm:p-5 scanline',
          (estopActive || missionLocked) && 'pointer-events-none opacity-50',
        )}
      >
        <span className="reticle-bl" aria-hidden />
        <span className="reticle-br" aria-hidden />

        {/* Console header — flex-wrap so chrome lines break cleanly on narrow viewports
            instead of breaking mid-token (e.g. "PNL-DRV-" / "01"). */}
        <div className="mb-3 flex flex-wrap items-baseline gap-x-2 gap-y-1 sm:mb-4">
          <span className="tag tag-strong whitespace-nowrap">drive console</span>
          <span className="tag tag-accent whitespace-nowrap">PNL-DRV-01</span>
          <span className="ml-auto hidden whitespace-nowrap tag md:inline-block">
            2 axes · linear + angular
          </span>
        </div>

        {/* The two dials share a baseline; the readout is the OUTPUT column.
            On phone the dials sit 2-up and the readout spans the row below.
            content-center fills the panel height instead of leaving a void. */}
        <div className="grid min-h-0 flex-1 grid-cols-2 content-center items-center gap-4 lg:grid-cols-[1fr_1fr_minmax(13rem,17rem)] lg:gap-6">
          <Joystick
            label="Linear · vX / vY"
            hint="↑ forward · ↔ strafe"
            size={stickSize}
            axisTags={['+X', '−Y', '−X', '+Y']}
            serial="STK-01"
            className="justify-self-center"
            onChange={(v) => {
              leftRef.current = v
            }}
          />
          <Joystick
            label="Angular · ωZ"
            hint="↔ yaw"
            size={stickSize}
            axisTags={['—', 'ccw', '—', 'cw']}
            serial="STK-02"
            className="justify-self-center"
            onChange={(v) => {
              rightRef.current = v
            }}
          />
          <TwistReadout
            value={lastSent}
            className="col-span-2 w-full self-center lg:col-span-1"
          />
        </div>
      </div>

      {/* secondary controls — speed governor + STOP (one row, opaque tile) */}
      <div className="reticle relative flex shrink-0 flex-row flex-wrap items-center gap-3 rounded-sm border border-hairline bg-ink-2 p-3 sm:gap-5 sm:px-5 sm:py-4">
        <span className="reticle-bl" aria-hidden />
        <span className="reticle-br" aria-hidden />

        <div className="flex flex-1 items-center gap-3 min-w-0">
          <span className="flex items-center gap-1.5 shrink-0">
            <Gauge className="h-3.5 w-3.5 text-primary" />
            <span className="tag tag-strong">governor</span>
          </span>
          <Slider
            min={0.1}
            max={1.0}
            step={0.05}
            value={[speedScale]}
            onValueChange={(v) => updateSettings({ speedScale: v[0] })}
            className="flex-1"
          />
          <div className="flex w-20 shrink-0 items-baseline justify-end gap-0.5">
            <span className="ticker text-base text-foreground">{speedScale.toFixed(2)}</span>
            <span className="tag">×</span>
          </div>
        </div>

        <Button
          variant="destructive"
          size="lg"
          onClick={() => estopTrigger('user')}
          disabled={blocked}
          className="w-auto shrink-0 sm:px-6"
        >
          <Square className="mr-2 h-4 w-4 fill-current" />
          STOP
        </Button>
      </div>

      <div className="flex shrink-0 flex-wrap items-center gap-2 text-[11px] text-muted-foreground">
        <Activity className="h-3 w-3 text-primary/80" />
        <span className="tag">tx</span>
        <span className="font-mono text-foreground/80">{cmdVelTopic}</span>
        <span className="tag">·</span>
        <span className="ticker">{PUBLISH_HZ}Hz</span>
        <span className="tag">
          {missionLocked ? 'gated · mission active' : 'while stick · active'}
        </span>
      </div>
    </ViewShell>
  )
}

interface LockBannerProps {
  phase: string
  target: string
  busy: boolean
  onTakeControl: () => void
  err: string | null
}

function MissionLockBanner({ phase, target, busy, onTakeControl, err }: LockBannerProps) {
  const detail = [phase, target].filter(Boolean).join(' · ')
  return (
    <div
      className="flex flex-wrap items-center gap-x-3 gap-y-2 rounded-sm border border-primary/40 bg-primary/10 px-3 py-2.5 text-sm sm:px-4 sm:py-3"
      role="status"
    >
      <Lock className="h-5 w-5 shrink-0 text-primary" />
      <div className="flex-1 min-w-[12rem]">
        <div className="flex flex-wrap items-baseline gap-x-2 gap-y-1">
          <span className="font-semibold uppercase tracking-[0.16em] text-primary">
            Mission active
          </span>
          <span className="tag">PNL-MIS-LOCK</span>
        </div>
        <div className="text-xs text-muted-foreground">
          Joystick disabled while orchestrator drives the base
          {detail && <span className="ml-2 font-mono text-foreground/80">{detail}</span>}
          {err && <span className="ml-2 text-destructive">· {err}</span>}
        </div>
      </div>
      <Button
        variant="default"
        size="sm"
        onClick={onTakeControl}
        disabled={busy}
        className="w-full shrink-0 sm:w-auto"
      >
        <Hand className="mr-2 h-4 w-4" />
        {busy ? 'Pausing…' : 'Take Control'}
      </Button>
    </div>
  )
}

interface PausedBannerProps {
  phase: string
  target: string
  busy: boolean
  onResume: () => void
  err: string | null
}

function MissionPausedBanner({ phase, target, busy, onResume, err }: PausedBannerProps) {
  const detail = [phase, target].filter(Boolean).join(' · ')
  return (
    <div
      className="flex flex-wrap items-center gap-x-3 gap-y-2 rounded-sm border border-warning/40 bg-warning/10 px-3 py-2.5 text-sm sm:px-4 sm:py-3"
      role="status"
    >
      <Hand className="h-5 w-5 shrink-0 text-warning" />
      <div className="flex-1 min-w-[12rem]">
        <div className="flex flex-wrap items-baseline gap-x-2 gap-y-1">
          <span className="font-semibold uppercase tracking-[0.16em] text-warning">
            Mission paused — you have the joystick
          </span>
          <span className="tag">PNL-MIS-PAUSE</span>
        </div>
        <div className="text-xs text-muted-foreground">
          Drive freely. Resume when the obstacle is clear.
          {detail && <span className="ml-2 font-mono text-foreground/80">held at {detail}</span>}
          {err && <span className="ml-2 text-destructive">· {err}</span>}
        </div>
      </div>
      <Button
        variant="default"
        size="sm"
        onClick={onResume}
        disabled={busy}
        className="w-full shrink-0 sm:w-auto"
      >
        <Play className="mr-2 h-4 w-4" />
        {busy ? 'Resuming…' : 'Resume mission'}
      </Button>
    </div>
  )
}
