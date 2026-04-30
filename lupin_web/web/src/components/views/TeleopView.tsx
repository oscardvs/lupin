import { Activity, Gauge, RotateCcw, Square } from 'lucide-react'
import { useCallback, useEffect, useRef, useState } from 'react'

import { Joystick, type StickValue } from '@/components/widgets/Joystick'
import { TwistReadout } from '@/components/widgets/TwistReadout'
import { Button } from '@/components/ui/button'
import { Slider } from '@/components/ui/slider'
import { ESTOP_REASON_LABELS, useCmdVel, useEStop } from '@/lib/estop'
import { useIsSm } from '@/lib/responsive'
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
  const isSm = useIsSm()
  const stickSize = isSm ? 196 : 148

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

  useEffect(() => {
    let zeroSentAfterIdle = true
    const id = setInterval(() => {
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

  return (
    <div className="flex h-full min-h-0 w-full flex-col gap-3 p-3 sm:gap-4 sm:p-4">
      {estopActive ? (
        <div className="reticle relative flex flex-wrap items-center gap-3 rounded-sm border-2 border-destructive bg-destructive/10 px-3 py-2.5 text-sm sm:px-4 sm:py-3">
          <span className="reticle-bl" aria-hidden />
          <span className="reticle-br" aria-hidden />
          <Square className="h-5 w-5 shrink-0 fill-destructive text-destructive" />
          <div className="flex-1 min-w-0">
            <div className="flex items-baseline gap-2">
              <span className="font-semibold uppercase tracking-[0.16em] text-destructive">
                E-stop engaged
              </span>
              <span className="tag">PNL-EMG-01</span>
            </div>
            <div className="text-xs text-muted-foreground">
              {estopReason ? ESTOP_REASON_LABELS[estopReason] : 'Unknown reason'}
            </div>
          </div>
          <Button variant="default" size="sm" onClick={estopReset} className="shrink-0">
            <RotateCcw className="mr-2 h-4 w-4" />
            Reset E-stop
          </Button>
        </div>
      ) : null}

      {/* primary console region — two stick stations + readout HUD */}
      <div
        className={cn(
          'reticle relative flex flex-1 min-h-0 flex-col gap-4 rounded-sm border border-hairline bg-card/35 p-3 sm:gap-6 sm:p-5 scanline',
          'sm:flex-row sm:items-center sm:justify-around',
          estopActive && 'pointer-events-none opacity-50',
        )}
      >
        <span className="reticle-bl" aria-hidden />
        <span className="reticle-br" aria-hidden />

        {/* Floating section header (top-left of the console) */}
        <div className="absolute left-3 top-2 flex items-baseline gap-2 sm:left-5 sm:top-3">
          <span className="tag tag-strong">drive console</span>
          <span className="tag tag-accent">PNL-DRV-01</span>
        </div>

        {/* On phones the two joysticks sit side-by-side at the top
            so the twist readout below can use the available row width. */}
        <div className="flex w-full items-stretch justify-around gap-3 pt-6 sm:contents sm:pt-0">
          <Joystick
            label="Linear · vX / vY"
            hint="↑ forward · ↔ strafe"
            size={stickSize}
            axisTags={['+X', '−Y', '−X', '+Y']}
            serial="STK-01"
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
            onChange={(v) => {
              rightRef.current = v
            }}
          />
        </div>

        <TwistReadout
          value={lastSent}
          className="w-full max-w-xs sm:order-none sm:w-56"
        />
      </div>

      {/* secondary controls — speed governor + STOP */}
      <div className="reticle relative flex flex-col gap-3 rounded-sm border border-hairline bg-card/60 p-3 sm:flex-row sm:flex-wrap sm:items-center sm:gap-5 sm:px-5 sm:py-4">
        <span className="reticle-bl" aria-hidden />
        <span className="reticle-br" aria-hidden />

        <div className="flex flex-1 items-center gap-3">
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
          <div className="flex w-20 items-baseline justify-end gap-0.5">
            <span className="ticker text-base text-foreground">{speedScale.toFixed(2)}</span>
            <span className="tag">×</span>
          </div>
        </div>

        <Button
          variant="destructive"
          size="lg"
          onClick={() => estopTrigger('user')}
          disabled={blocked}
          className="w-full sm:w-auto sm:px-6"
        >
          <Square className="mr-2 h-4 w-4 fill-current" />
          STOP
        </Button>
      </div>

      <div className="flex items-center gap-2 text-[11px] text-muted-foreground">
        <Activity className="h-3 w-3 text-primary/80" />
        <span className="tag">tx</span>
        <span className="font-mono text-foreground/80">{cmdVelTopic}</span>
        <span className="tag">·</span>
        <span className="ticker">{PUBLISH_HZ}Hz</span>
        <span className="tag">while stick · active</span>
      </div>
    </div>
  )
}
