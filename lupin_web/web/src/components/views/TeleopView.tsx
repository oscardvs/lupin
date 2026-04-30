import { RotateCcw, Square } from 'lucide-react'
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
  const { active: estopActive, reason: estopReason, reset: estopReset, trigger: estopTrigger } = useEStop()
  const { publish, blocked } = useCmdVel()
  const isSm = useIsSm()
  const stickSize = isSm ? 176 : 132

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
    <div className="flex h-full min-h-0 flex-col gap-3 p-3 sm:gap-4 sm:p-4">
      {estopActive ? (
        <div className="flex flex-wrap items-center gap-3 rounded-md border-2 border-destructive bg-destructive/10 px-3 py-2.5 text-sm sm:px-4 sm:py-3">
          <Square className="h-5 w-5 shrink-0 fill-destructive text-destructive" />
          <div className="flex-1 min-w-0">
            <div className="font-semibold text-destructive">E-STOP active</div>
            <div className="text-xs text-muted-foreground">
              {estopReason ? ESTOP_REASON_LABELS[estopReason] : 'Unknown reason'}
            </div>
          </div>
          <Button variant="default" size="sm" onClick={estopReset} className="shrink-0">
            <RotateCcw className="mr-2 h-4 w-4" />
            Reset E-STOP
          </Button>
        </div>
      ) : null}

      <div
        className={cn(
          'flex flex-1 min-h-0 flex-col items-center gap-4 rounded-md border bg-card/40 p-3 sm:gap-6 sm:p-4',
          'sm:flex-row sm:items-center sm:justify-around',
          estopActive && 'pointer-events-none opacity-50',
        )}
      >
        {/* On phones (portrait) the two joysticks sit side-by-side at the top
            so the twist readout below can use the available row width. */}
        <div className="flex w-full items-center justify-around gap-3 sm:contents">
          <Joystick
            label="Linear X / Y"
            hint="up = forward · sides = strafe"
            color="#22c55e"
            size={stickSize}
            onChange={(v) => {
              leftRef.current = v
            }}
          />
          <Joystick
            label="Angular Z"
            hint="left/right = yaw"
            color="#38bdf8"
            size={stickSize}
            onChange={(v) => {
              rightRef.current = v
            }}
          />
        </div>

        <TwistReadout value={lastSent} className="w-full max-w-xs sm:order-none sm:w-48" />
      </div>

      <div className="flex flex-col gap-3 rounded-md border bg-card p-3 sm:flex-row sm:flex-wrap sm:items-center sm:gap-4">
        <div className="flex flex-1 items-center gap-3">
          <span className="shrink-0 text-xs uppercase tracking-wider text-muted-foreground">Speed</span>
          <Slider
            min={0.1}
            max={1.0}
            step={0.05}
            value={[speedScale]}
            onValueChange={(v) => updateSettings({ speedScale: v[0] })}
            className="flex-1"
          />
          <span className="w-12 shrink-0 text-right font-mono text-sm tabular-nums">{speedScale.toFixed(2)}×</span>
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

      <div className="text-[11px] text-muted-foreground">
        Publishing on <span className="font-mono">{cmdVelTopic}</span> at {PUBLISH_HZ} Hz while a stick is active.
      </div>
    </div>
  )
}
