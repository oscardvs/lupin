import { Activity, Camera, Compass, Grip, Home, Power, RotateCcw, Square } from 'lucide-react'
import { useCallback, useEffect, useRef, useState } from 'react'

import { ArmCalibrateDialog } from '@/components/widgets/ArmCalibrateDialog'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Slider } from '@/components/ui/slider'
import { ESTOP_REASON_LABELS, useEStop } from '@/lib/estop'
import { useRos, useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender } from '@/lib/throttle'
import { cn } from '@/lib/utils'
import {
  LUPIN_SRV,
  MIRTE_SRV,
  ROS_TYPE,
  type ServoPosition,
  type SetServoAngleWithSpeedRequest,
} from '@/types/ros'

const RAD2DEG = 180 / Math.PI
const DEG2RAD = Math.PI / 180

interface ArmJointSpec {
  /** Servo name as it appears in `/io/servo/hiwonder/<id>/...`. */
  id: string
  label: string
  hint: string
  code: string
  Icon: typeof Camera
  /** Min / max angle in degrees. URDF declares ±π/2 for the four arm servos. */
  minDeg: number
  maxDeg: number
  /**
   * If true, render a loud magenta "RANGE UNVERIFIED" badge. Used for joints
   * (currently the gripper) where the safe servo-angle limits haven't been
   * confirmed against the real robot — see the comment block above
   * `ARM_JOINTS` for the verification recipe.
   */
  unverified?: boolean
}

// Gripper sends through /lupin/gripper/set_angle_with_speed (gripper_action_bridge),
// NOT the raw Hiwonder service. On hardware the vendor mirte_master_arm_control
// HW interface treats any external servo motion as "moved by hand / by gravity"
// and re-asserts its own commanded position on every 100 ms tick — so a direct
// Hiwonder call would visibly move the jaw and then snap it back to the stale
// GripperActionController setpoint. The bridge forwards as a GripperCommand
// action goal so the controller's commanded state matches the HMI request.
//
// HMI degrees are mapped linearly to the URDF gripper_joint range
// [-0.20, 0.25] rad inside the bridge — the ±30° HMI window is the
// "range unverified" guess pending a live tuning pass.
//
// To verify range on a live robot:
//   ros2 service call /lupin/gripper/set_angle_with_speed \
//     mirte_msgs/srv/SetServoAngleWithSpeed "{angle: 0, rate: 30, degrees: true}"
// Then jog by ±5° at a time until the jaw hits its mechanical stops; record
// those as the new minDeg / maxDeg here and drop the `unverified` flag.
const ARM_JOINTS: ArmJointSpec[] = [
  {
    id: 'shoulder_pan',
    label: 'Shoulder pan',
    hint: 'base yaw',
    code: 'J01',
    Icon: RotateCcw,
    minDeg: -90,
    maxDeg: 90,
  },
  {
    id: 'shoulder_lift',
    label: 'Shoulder lift',
    hint: 'lift segment 1',
    code: 'J02',
    Icon: RotateCcw,
    minDeg: -90,
    maxDeg: 90,
  },
  {
    id: 'elbow',
    label: 'Elbow',
    hint: 'bend segment 2',
    code: 'J03',
    Icon: RotateCcw,
    minDeg: -90,
    maxDeg: 90,
  },
  {
    id: 'wrist',
    label: 'Wrist · gripper-cam tilt',
    hint: 'tilts the gripper camera',
    code: 'J04',
    Icon: Camera,
    minDeg: -90,
    maxDeg: 90,
  },
  {
    id: 'gripper',
    label: 'Gripper · jaw',
    hint: 'open / close — range unverified, tune on robot',
    code: 'J05',
    Icon: Grip,
    minDeg: -30,
    maxDeg: 30,
    unverified: true,
  },
]

type CallStatus = 'idle' | 'sending' | 'ok' | 'error'

export function ArmView() {
  const [{ armServoNamespace, armRateDegPerSec }, updateSettings] = useSettings()
  const {
    active: estopActive,
    reason: estopReason,
    reset: estopReset,
  } = useEStop()
  const { callService, status: rosStatus } = useRos()

  const [enableStatus, setEnableStatus] = useState<CallStatus>('idle')
  const [enableError, setEnableError] = useState<string | null>(null)
  /** Bumped whenever Init/safe-pose is pressed so each ServoSlider snaps
   *  its target visual to 0. The actual motion is driven by ONE
   *  /lupin/arm/init service call (handled below) so the four joints move
   *  via a single JTC trajectory instead of four racing slider commands. */
  const [homeTick, setHomeTick] = useState(0)
  const [initStatus, setInitStatus] = useState<CallStatus>('idle')
  const [initError, setInitError] = useState<string | null>(null)
  const [calibOpen, setCalibOpen] = useState(false)

  const blocked = estopActive || rosStatus !== 'connected'

  const enableAll = useCallback(
    async (enable: boolean) => {
      setEnableStatus('sending')
      setEnableError(null)
      try {
        await callService<{ data: boolean }, { success: boolean; message?: string }>(
          `${armServoNamespace}/enable_all_servos`,
          MIRTE_SRV.SetBool,
          { data: enable },
        )
        setEnableStatus('ok')
      } catch (e) {
        setEnableStatus('error')
        setEnableError(e instanceof Error ? e.message : String(e))
      }
    },
    [callService, armServoNamespace],
  )

  const goToSafePose = useCallback(async () => {
    setInitStatus('sending')
    setInitError(null)
    // Snap slider visuals to 0 immediately so the UI tracks the move.
    setHomeTick((t) => t + 1)
    try {
      await callService<Record<string, never>, { success: boolean; message?: string }>(
        '/lupin/arm/init',
        LUPIN_SRV.Trigger,
        {},
      )
      setInitStatus('ok')
    } catch (e) {
      setInitStatus('error')
      setInitError(e instanceof Error ? e.message : String(e))
    }
  }, [callService])

  return (
    <div className="flex w-full flex-col gap-3 p-3 sm:gap-4 sm:p-4">
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
              <span className="tag">PNL-ARM-EMG</span>
            </div>
            <div className="text-xs text-muted-foreground">
              {estopReason ? ESTOP_REASON_LABELS[estopReason] : 'Unknown reason'}
              {' · '}arm commands disabled
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

      {/* arm console controls — enable / home / rate */}
      <div className="reticle relative flex flex-col gap-3 rounded-sm border border-hairline bg-card/60 p-3 sm:flex-row sm:flex-wrap sm:items-center sm:gap-5 sm:px-5 sm:py-4">
        <span className="reticle-bl" aria-hidden />
        <span className="reticle-br" aria-hidden />

        <div className="flex items-baseline gap-2">
          <span className="tag tag-strong">arm console</span>
          <span className="tag tag-accent">PNL-ARM-01</span>
        </div>

        <div className="flex flex-wrap items-center gap-2">
          <Button
            variant="outline"
            size="sm"
            onClick={() => enableAll(true)}
            disabled={blocked || enableStatus === 'sending'}
          >
            <Power className="mr-2 h-4 w-4 text-primary" />
            Enable
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={() => enableAll(false)}
            disabled={blocked || enableStatus === 'sending'}
          >
            <Power className="mr-2 h-4 w-4 text-muted-foreground" />
            Disable
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={goToSafePose}
            disabled={blocked || initStatus === 'sending'}
            title="Drive all 4 arm joints to (0,0,0,0) via JTC over 5s"
          >
            <Home className="mr-2 h-4 w-4" />
            Init (home, 5s)
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={() => setCalibOpen(true)}
            disabled={blocked}
            title="Hiwonder zero-offset calibration (operator-in-the-loop, hardware only)"
          >
            <Compass className="mr-2 h-4 w-4" />
            Calibrate…
          </Button>
          {enableStatus === 'error' && enableError ? (
            <span className="tag text-destructive">enable failed: {enableError}</span>
          ) : enableStatus === 'ok' ? (
            <span className="tag text-primary">enable ack</span>
          ) : null}
          {initStatus === 'sending' ? (
            <span className="tag">init sending…</span>
          ) : initStatus === 'error' && initError ? (
            <span className="tag text-destructive" title={initError}>init failed</span>
          ) : initStatus === 'ok' ? (
            <span className="tag text-primary">init ack</span>
          ) : null}
        </div>

        <div className="flex flex-1 items-center gap-3 sm:min-w-[18rem]">
          <span className="tag shrink-0">rate</span>
          <Slider
            min={5}
            max={180}
            step={5}
            value={[armRateDegPerSec]}
            onValueChange={(v) => updateSettings({ armRateDegPerSec: v[0] })}
            className="flex-1"
          />
          <div className="flex w-24 shrink-0 items-baseline justify-end gap-1">
            <span className="ticker text-base text-foreground">
              {armRateDegPerSec.toFixed(0)}
            </span>
            <span className="tag">°/s</span>
          </div>
        </div>
      </div>

      {/* per-joint sliders */}
      <div
        className={cn(
          'grid grid-cols-1 gap-3 sm:gap-4 lg:grid-cols-2',
          blocked && 'pointer-events-none opacity-50',
        )}
      >
        {ARM_JOINTS.map((joint) => (
          <ServoSlider
            key={joint.id}
            spec={joint}
            namespace={armServoNamespace}
            rateDegPerSec={armRateDegPerSec}
            homeTick={homeTick}
            disabled={blocked}
          />
        ))}
      </div>

      <div className="flex flex-wrap items-center gap-2 text-[11px] text-muted-foreground">
        <Activity className="h-3 w-3 text-primary/80" />
        <span className="tag">srv</span>
        <span className="font-mono text-foreground/80">
          {armServoNamespace}/&lt;joint&gt;/set_angle_with_speed
        </span>
        <span className="tag">·</span>
        <span className="ticker">on release</span>
      </div>

      <ArmCalibrateDialog open={calibOpen} onOpenChange={setCalibOpen} />
    </div>
  )
}

interface ServoSliderProps {
  spec: ArmJointSpec
  namespace: string
  rateDegPerSec: number
  homeTick: number
  disabled: boolean
}

function ServoSlider({ spec, namespace, rateDegPerSec, homeTick, disabled }: ServoSliderProps) {
  const positionTopic = `${namespace}/${spec.id}/position`
  // All slider commands go through the lupin command bridge so the
  // JointTrajectoryController / GripperActionController stay aligned with
  // the HMI. Routing arm joints to the raw Hiwonder service caused the
  // vendor HW interface to reassert its own commanded position on the
  // next 10 Hz tick, snapping the joint back mid-motion.
  const setAngleService =
    spec.id === 'gripper'
      ? '/lupin/gripper/set_angle_with_speed'
      : `/lupin/arm/${spec.id}/set_angle_with_speed`

  const positionRef = useTopic<ServoPosition>(positionTopic, ROS_TYPE.ServoPosition)
  useThrottledRender(8)

  const { callService } = useRos()

  const [target, setTarget] = useState(0)
  const [lastSent, setLastSent] = useState<number | null>(null)
  const [status, setStatus] = useState<CallStatus>('idle')
  const [errMsg, setErrMsg] = useState<string | null>(null)
  const inFlightRef = useRef(0)

  // Snap target visual to 0 whenever the parent fires Init. The actual
  // motion is driven by ONE /lupin/arm/init call upstream — we do NOT
  // call send() per slider here, because four racing single-joint
  // trajectories would each preempt the previous and could leave the
  // arm in an awkward intermediate pose.
  useEffect(() => {
    if (homeTick === 0) return
    setTarget(0)
  }, [homeTick])

  const send = useCallback(
    async (angleDeg: number) => {
      const id = ++inFlightRef.current
      setStatus('sending')
      setErrMsg(null)
      try {
        await callService<SetServoAngleWithSpeedRequest, { status: boolean }>(
          setAngleService,
          MIRTE_SRV.SetServoAngleWithSpeed,
          { angle: angleDeg, rate: rateDegPerSec, degrees: true },
        )
        if (id !== inFlightRef.current) return // a newer call superseded us
        setLastSent(angleDeg)
        setStatus('ok')
      } catch (e) {
        if (id !== inFlightRef.current) return
        setStatus('error')
        setErrMsg(e instanceof Error ? e.message : String(e))
      }
    },
    [callService, setAngleService, rateDegPerSec],
  )

  const currentRad = positionRef.current?.angle ?? null
  const currentDeg = currentRad === null ? null : currentRad * RAD2DEG
  const targetRad = target * DEG2RAD
  const range = spec.maxDeg - spec.minDeg

  const currentPct =
    currentDeg === null
      ? null
      : ((Math.max(spec.minDeg, Math.min(spec.maxDeg, currentDeg)) - spec.minDeg) / range) * 100

  return (
    <Card className="flex flex-col">
      <CardHeader>
        <CardTitle>
          <spec.Icon className="h-3.5 w-3.5 text-primary" />
          {spec.label}
          {spec.unverified ? (
            <span className="tag ml-2 border border-fuchsia-500/60 bg-fuchsia-500/15 text-fuchsia-300">
              range unverified
            </span>
          ) : null}
          <span className="tag tag-accent ml-auto">{spec.code}</span>
        </CardTitle>
        <CardDescription>
          {spec.hint} · {spec.id}
        </CardDescription>
      </CardHeader>
      <CardContent className="flex flex-col gap-3">
        <div className="flex items-baseline gap-3 text-sm">
          <span className="tag">target</span>
          <span className="ticker text-base text-foreground">
            {target >= 0 ? '+' : ''}
            {target.toFixed(1)}°
          </span>
          <span className="tag tag-accent">{targetRad.toFixed(2)} rad</span>
          <span className="ml-auto flex items-baseline gap-2">
            <span className="tag">current</span>
            <span className="ticker text-base text-foreground">
              {currentDeg === null ? '—' : `${currentDeg >= 0 ? '+' : ''}${currentDeg.toFixed(1)}°`}
            </span>
          </span>
        </div>

        {/* Slider with a faint ghost bar showing live position underneath. */}
        <div className="relative">
          {currentPct !== null ? (
            <div
              aria-hidden
              className="pointer-events-none absolute inset-x-0 top-1/2 -translate-y-1/2"
            >
              <div className="relative h-2 w-full">
                <span
                  className="absolute top-0 h-full w-px bg-primary/60"
                  style={{ left: `${currentPct}%` }}
                />
              </div>
            </div>
          ) : null}
          <Slider
            min={spec.minDeg}
            max={spec.maxDeg}
            step={1}
            value={[target]}
            onValueChange={(v) => setTarget(v[0])}
            onValueCommit={(v) => {
              setTarget(v[0])
              void send(v[0])
            }}
            disabled={disabled}
          />
        </div>

        <div className="flex items-center justify-between text-[11px] text-muted-foreground">
          <span className="tag">{spec.minDeg}°</span>
          <div className="flex items-center gap-2">
            {status === 'sending' ? (
              <span className="tag">sending…</span>
            ) : status === 'ok' && lastSent !== null ? (
              <span className="tag text-primary">
                ack · {lastSent >= 0 ? '+' : ''}
                {lastSent.toFixed(1)}°
              </span>
            ) : status === 'error' ? (
              <span className="tag text-destructive" title={errMsg ?? undefined}>
                send failed
              </span>
            ) : (
              <span className="tag">idle</span>
            )}
          </div>
          <span className="tag">+{spec.maxDeg}°</span>
        </div>
      </CardContent>
    </Card>
  )
}
