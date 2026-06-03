import { Activity, Camera, Compass, Grip, Home, Minus, Plus, Power, RotateCcw, Square } from 'lucide-react'
import { useCallback, useRef, useState } from 'react'

import { RobotTwin } from '@/components/system/RobotTwin'
import { ArmCalibrateDialog } from '@/components/widgets/ArmCalibrateDialog'
import { PoseLibraryCard } from '@/components/widgets/PoseLibraryCard'
import { SequenceRecorderCard } from '@/components/widgets/SequenceRecorderCard'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Slider } from '@/components/ui/slider'
import {
  ARM_JOINT_LIMITS,
  ARM_SETTLE_TOL_DEG,
  GRIPPER_HMI_MAX_DEG,
  GRIPPER_HMI_MIN_DEG,
  GRIPPER_JOINT_STATE_NAME,
  GRIPPER_RANGE_UNVERIFIED,
  GRIPPER_SETTLE_TOL_DEG,
  type ArmJointId,
  armJointStateName,
  hmiDegToTargetRad,
  jointStateRadToHmiDeg,
} from '@/lib/arm'
import { ESTOP_REASON_LABELS, useEStop } from '@/lib/estop'
import { useRos, useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender } from '@/lib/throttle'
import { cn } from '@/lib/utils'
import {
  LUPIN_SRV,
  MIRTE_SRV,
  ROS_TYPE,
  type JointState,
  type SetServoAngleWithSpeedRequest,
} from '@/types/ros'

/** The arm power switch — orchestrates servo torque + the HW-interface command
 * gate in the backend (gripper_action_bridge `/lupin/arm/set_torque`). NOT the
 * raw `enable_all_servos`: that only toggles torque and leaves the HW interface
 * re-asserting its setpoint, which is the enable/disable lurch we fixed. */
const ARM_TORQUE_SERVICE = '/lupin/arm/set_torque'

type JointPiece = ArmJointId | 'gripper'

interface ArmJointSpec {
  /** Joint id as it appears in `/lupin/arm/<id>/set_angle_with_speed`. */
  id: JointPiece
  label: string
  hint: string
  code: string
  Icon: typeof Camera
  /** Command window in degrees — from the shared single-source-of-truth lib/arm. */
  minDeg: number
  maxDeg: number
  /** Loud magenta badge for a joint whose safe range isn't confirmed on the
   * real robot yet (currently the gripper). */
  unverified?: boolean
}

// Built from the shared limit table (lib/arm) so the slider windows can never
// drift from the backend clamp + the voice tools. The arm joint windows are the
// real asymmetric servo limits intersected with the ±90° envelope; the gripper
// keeps its conservative ±30° window pending the live tuning pass.
const ARM_JOINT_SPECS: ArmJointSpec[] = [
  { id: 'shoulder_pan', label: 'Shoulder pan', hint: 'base yaw', code: 'J01', Icon: RotateCcw, ...ARM_JOINT_LIMITS.shoulder_pan },
  { id: 'shoulder_lift', label: 'Shoulder lift', hint: 'lift segment 1', code: 'J02', Icon: RotateCcw, ...ARM_JOINT_LIMITS.shoulder_lift },
  { id: 'elbow', label: 'Elbow', hint: 'bend segment 2', code: 'J03', Icon: RotateCcw, ...ARM_JOINT_LIMITS.elbow },
  { id: 'wrist', label: 'Wrist · gripper-cam tilt', hint: 'tilts the gripper camera', code: 'J04', Icon: Camera, ...ARM_JOINT_LIMITS.wrist },
  {
    id: 'gripper',
    label: 'Gripper · jaw',
    hint: 'open (−) / close (+) — range unverified, tune on robot',
    code: 'J05',
    Icon: Grip,
    minDeg: GRIPPER_HMI_MIN_DEG,
    maxDeg: GRIPPER_HMI_MAX_DEG,
    unverified: GRIPPER_RANGE_UNVERIFIED,
  },
]

function jointStateName(id: JointPiece): string {
  return id === 'gripper' ? GRIPPER_JOINT_STATE_NAME : armJointStateName(id)
}

function settleTol(id: JointPiece): number {
  return id === 'gripper' ? GRIPPER_SETTLE_TOL_DEG : ARM_SETTLE_TOL_DEG
}

function humanizeError(e: unknown): string {
  const msg = e instanceof Error ? e.message : String(e)
  if (/not connected/i.test(msg)) return 'rosbridge disconnected'
  if (/not ready/i.test(msg)) return 'rosbridge not ready'
  return msg
}

type CallStatus = 'idle' | 'sending' | 'ok' | 'error'
type TorqueState = boolean | 'unknown'

export function ArmView() {
  const [{ armServoNamespace: _ns, armRateDegPerSec, jointStatesTopic }, updateSettings] = useSettings()
  void _ns // namespace no longer used for commands — kept in settings for the calibrate dialog
  const {
    active: estopActive,
    reason: estopReason,
    reset: estopReset,
  } = useEStop()
  const { callService, status: rosStatus } = useRos()

  // Torque is the arm power state AND the slider arming gate. It is 'unknown' on
  // every mount — including the in-app tab-switch remount (Radix unmounts the
  // inactive tab). We deliberately do NOT assume the arm is ready: 'unknown'
  // BLOCKS the sliders + Home until the operator presses Enable (set_torque
  // {true}). That makes Enable a real arming switch and stops a remount from
  // silently re-arming a freshly-seeded slider. (Earlier this treated 'unknown'
  // as live for boot convenience; that let a tab-return command the arm with no
  // Enable press — the bug this gate closes.)
  const [torque, setTorque] = useState<TorqueState>('unknown')
  const [enableStatus, setEnableStatus] = useState<CallStatus>('idle')
  const [enableError, setEnableError] = useState<string | null>(null)
  const [initStatus, setInitStatus] = useState<CallStatus>('idle')
  const [initError, setInitError] = useState<string | null>(null)
  const [calibOpen, setCalibOpen] = useState(false)
  // Rate slider: track locally during drag, persist only on release so we don't
  // write localStorage on every drag tick.
  const [rateLocal, setRateLocal] = useState(armRateDegPerSec)

  // Freeze the per-joint sliders + the torque toggle while a sequence recording
  // or replay is in flight (spec §11) so manual input can't fight a running
  // record/play. Driven by arm_library_server's /lupin/arm/library/state topic.
  const [libraryBusy, setLibraryBusy] = useState(false)
  useTopic<{ data: string }>('/lupin/arm/library/state', ROS_TYPE.String, {
    onMessage: (msg) => {
      try {
        const s = JSON.parse(msg.data)
        setLibraryBusy(!!s.recording || !!s.playing)
      } catch { /* ignore */ }
    },
  })

  const connBlocked = estopActive || rosStatus !== 'connected'
  // Strict arming gate: open ONLY on an explicit Enable. 'unknown' (fresh mount
  // / tab-return) and 'false' (explicit Disable) both keep the sliders + Home
  // locked, so commands never reach the arm without a deliberate arming press.
  const armDisabled = torque !== true
  // libraryBusy folds in so the per-joint sliders + Home freeze during a
  // record/replay (spec §11). The torque Enable/Disable toggle gates on
  // `connBlocked || libraryBusy` directly (it must stay live when armDisabled,
  // since Enable is how you arm, but must NOT fight a running record/play).
  const controlsBlocked = connBlocked || armDisabled || libraryBusy
  const blockReason = estopActive
    ? 'E-stop engaged — arm commands disabled'
    : rosStatus !== 'connected'
      ? 'rosbridge disconnected — arm commands disabled'
      : torque === false
        ? 'arm disabled (limp) — press Enable to energise'
        : torque !== true
          ? 'arm controls locked — press Enable to arm'
          : null

  const setTorqueCmd = useCallback(
    async (enable: boolean) => {
      setEnableStatus('sending')
      setEnableError(null)
      try {
        const res = await callService<{ data: boolean }, { success: boolean; message?: string }>(
          ARM_TORQUE_SERVICE,
          MIRTE_SRV.SetBool,
          { data: enable },
        )
        if (res?.success === false) throw new Error(res.message || 'torque command rejected')
        setTorque(enable)
        setEnableStatus('ok')
      } catch (e) {
        setEnableStatus('error')
        setEnableError(humanizeError(e))
        // Leave torque state unknown — we genuinely don't know what happened.
        setTorque('unknown')
      }
    },
    [callService],
  )

  const goToSafePose = useCallback(async () => {
    setInitStatus('sending')
    setInitError(null)
    try {
      // Drive to the SAME measured safe rest pose the robot uses on boot
      // (auto_home → /lupin/arm/preset {home}) and that the voice agent uses.
      const res = await callService<{ name: string }, { success: boolean; message?: string }>(
        '/lupin/arm/preset',
        LUPIN_SRV.SetArmPreset,
        { name: 'home' },
      )
      if (res.success === false) throw new Error(res.message || 'preset rejected')
      setInitStatus('ok')
    } catch (e) {
      setInitStatus('error')
      setInitError(humanizeError(e))
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

      {/* Live digital twin — mirrors joint_states + odom in real time. */}
      <div
        className="relative h-[240px] w-full overflow-hidden rounded-sm border border-hairline sm:h-[300px] lg:h-[360px]"
        style={{ background: 'radial-gradient(130% 100% at 50% -10%, hsl(var(--ink-2)), hsl(var(--ink-0)) 72%)' }}
      >
        <RobotTwin className="absolute inset-0" />
      </div>

      {/* arm console controls — torque toggle / home / rate */}
      <div className="reticle relative flex flex-col gap-3 rounded-sm border border-hairline bg-card/60 p-3 sm:flex-row sm:flex-wrap sm:items-center sm:gap-5 sm:px-5 sm:py-4">
        <span className="reticle-bl" aria-hidden />
        <span className="reticle-br" aria-hidden />

        <div className="flex items-baseline gap-2">
          <span className="tag tag-strong">arm console</span>
          <span className="tag tag-accent">PNL-ARM-01</span>
          {torque === false ? (
            <span className="tag border border-destructive/60 bg-destructive/15 font-semibold text-destructive">
              ARM LIMP
            </span>
          ) : torque === true ? (
            <span className="tag text-primary">torque on</span>
          ) : (
            <span className="tag text-warning" title="Arm controls are locked until you press Enable to arm.">
              not armed
            </span>
          )}
        </div>

        {/* Enable / Disable as a segmented toggle so the active state is legible. */}
        <div className="flex flex-wrap items-center gap-2">
          <div className="inline-flex overflow-hidden rounded-sm border border-hairline" role="group" aria-label="Arm torque">
            <Button
              variant={torque === true ? 'default' : 'outline'}
              size="sm"
              className="rounded-none border-0"
              onClick={() => setTorqueCmd(true)}
              disabled={connBlocked || libraryBusy || enableStatus === 'sending'}
              aria-pressed={torque === true}
            >
              <Power className={cn('mr-2 h-4 w-4', torque === true ? '' : 'text-primary')} />
              Enable
            </Button>
            <Button
              variant={torque === false ? 'default' : 'outline'}
              size="sm"
              className="rounded-none border-0 border-l border-hairline"
              onClick={() => setTorqueCmd(false)}
              disabled={connBlocked || libraryBusy || enableStatus === 'sending'}
              aria-pressed={torque === false}
            >
              <Power className={cn('mr-2 h-4 w-4', torque === false ? '' : 'text-muted-foreground')} />
              Disable
            </Button>
          </div>
          <Button
            variant="outline"
            size="sm"
            onClick={goToSafePose}
            disabled={controlsBlocked || initStatus === 'sending'}
            title="Drive the arm to the measured safe rest pose (same as boot auto-home) via JTC over ~3s"
          >
            <Home className="mr-2 h-4 w-4" />
            Home (safe · 3s)
          </Button>
          <Button
            variant="outline"
            size="sm"
            onClick={() => setCalibOpen(true)}
            disabled={connBlocked}
            title="Hiwonder zero-offset calibration (operator-in-the-loop, hardware only)"
          >
            <Compass className="mr-2 h-4 w-4" />
            Calibrate…
          </Button>
          {enableStatus === 'error' && enableError ? (
            <span className="tag text-destructive" title={enableError}>torque failed: {enableError}</span>
          ) : enableStatus === 'sending' ? (
            <span className="tag">torque…</span>
          ) : null}
          {initStatus === 'sending' ? (
            <span className="tag">home sending…</span>
          ) : initStatus === 'error' && initError ? (
            <span className="tag text-destructive" title={initError}>home failed</span>
          ) : initStatus === 'ok' ? (
            <span className="tag text-primary">home ack</span>
          ) : null}
        </div>

        <div className="flex flex-1 items-center gap-3 sm:min-w-[18rem]">
          <span className="tag shrink-0" title="Slew rate for arm joints. Ignored by the gripper (it uses a force-limited grasp).">rate</span>
          <Slider
            min={5}
            max={180}
            step={5}
            value={[rateLocal]}
            onValueChange={(v) => setRateLocal(v[0])}
            onValueCommit={(v) => updateSettings({ armRateDegPerSec: v[0] })}
            className="flex-1"
            aria-label="Arm slew rate, degrees per second"
          />
          <div className="flex w-24 shrink-0 items-baseline justify-end gap-1">
            <span className="ticker text-base text-foreground">{rateLocal.toFixed(0)}</span>
            <span className="tag">°/s</span>
          </div>
        </div>
      </div>

      {/* block-reason status line (screen-reader live + visible) */}
      {blockReason ? (
        <div
          className="rounded-sm border border-warning/40 bg-warning/10 px-3 py-1.5 text-xs text-warning"
          role="status"
          aria-live="polite"
        >
          {blockReason}
        </div>
      ) : null}

      {/* per-joint sliders */}
      <div
        className={cn(
          'grid grid-cols-1 gap-3 sm:gap-4 lg:grid-cols-2',
          controlsBlocked && 'opacity-50',
        )}
      >
        {ARM_JOINT_SPECS.map((joint) => (
          <ServoSlider
            key={joint.id}
            spec={joint}
            jointStatesTopic={jointStatesTopic}
            rateDegPerSec={armRateDegPerSec}
            disabled={controlsBlocked}
          />
        ))}
      </div>

      {/* Pose library + sequence recorder. Gated by connection/e-stop only —
          NOT armDisabled — because kinesthetic recording deliberately drops
          torque; a running record/replay freezes the sliders above via
          libraryBusy instead. */}
      <div className="grid grid-cols-1 gap-3 sm:gap-4 lg:grid-cols-2">
        <PoseLibraryCard disabled={connBlocked} />
        <SequenceRecorderCard disabled={connBlocked} />
      </div>

      <div className="flex flex-wrap items-center gap-2 text-[11px] text-muted-foreground">
        <Activity className="h-3 w-3 text-primary/80" />
        <span className="tag">srv</span>
        <span className="font-mono text-foreground/80">
          /lupin/arm/&lt;joint&gt;/set_angle_with_speed · /lupin/gripper/set_angle_with_speed
        </span>
        <span className="tag">·</span>
        <span className="ticker">commit on release · status from /joint_states</span>
      </div>

      <ArmCalibrateDialog open={calibOpen} onOpenChange={setCalibOpen} />
    </div>
  )
}

interface ServoSliderProps {
  spec: ArmJointSpec
  jointStatesTopic: string
  rateDegPerSec: number
  disabled: boolean
}

interface LastSend {
  target: number
  at: number
  /** Expected time-to-arrive (ms) used to flip "moving"→"stalled". */
  expectedMs: number
}

function ServoSlider({ spec, jointStatesTopic, rateDegPerSec, disabled }: ServoSliderProps) {
  // Command path: all slider commands go through the lupin command bridge so
  // the JTC / GripperActionController stay aligned with the HMI.
  const setAngleService =
    spec.id === 'gripper'
      ? '/lupin/gripper/set_angle_with_speed'
      : `/lupin/arm/${spec.id}/set_angle_with_speed`

  const jsName = jointStateName(spec.id)

  const { callService } = useRos()

  const [target, setTarget] = useState(0)
  const [lastSend, setLastSend] = useState<LastSend | null>(null)
  const [sendError, setSendError] = useState<string | null>(null)
  const inFlightRef = useRef(0)
  // Until the operator grabs the slider, the thumb mirrors the live pose (see
  // the joint_states handler below). Flipped true on first interaction so a
  // commanded setpoint is never yanked back to raw feedback.
  const touchedRef = useRef(false)

  // Readback comes from /joint_states — the SAME source the controllers and the
  // backend command path use. (Previously this subscribed to the raw vendor
  // /io/servo/hiwonder/<id>/position topic, a different frame that could read
  // "—" when the lazy publisher slept even though commands still worked.)
  const lastMsgRef = useRef(0)
  const angleRef = useRef<number | null>(null)
  const jsRef = useTopic<JointState>(jointStatesTopic, ROS_TYPE.JointState, {
    onMessage: (msg) => {
      const i = msg.name.indexOf(jsName)
      if (i >= 0 && i < msg.position.length) {
        const rad = msg.position[i]
        angleRef.current = rad
        lastMsgRef.current = performance.now()
        // Seed + track the target from live feedback until the operator takes
        // control. Without this, target is useState(0): on first mount AND
        // after a tab-switch remount the thumb snaps to 0° while the arm is
        // elsewhere, so the next touch lurches the arm from a phantom zero.
        // Clamp into the command window + round to the slider step so feedback
        // jitter can't fight the operator.
        if (!touchedRef.current) {
          const deg = Math.round(jointStateRadToHmiDeg(spec.id, rad))
          const clamped = Math.max(spec.minDeg, Math.min(spec.maxDeg, deg))
          setTarget((prev) => (prev === clamped ? prev : clamped))
        }
      }
    },
  })
  void jsRef
  useThrottledRender(8)

  const currentRad = angleRef.current
  const currentDeg = currentRad === null ? null : jointStateRadToHmiDeg(spec.id, currentRad)
  const currentDegRef = useRef<number | null>(null)
  currentDegRef.current = currentDeg
  // Stale once /joint_states stops; >2 s without an update means not live.
  const fresh = currentRad !== null && performance.now() - lastMsgRef.current < 2000

  const send = useCallback(
    async (angleDeg: number) => {
      const id = ++inFlightRef.current
      setSendError(null)
      const startDeg = currentDegRef.current ?? angleDeg
      const disp = Math.abs(angleDeg - startDeg)
      // Gripper time is force/effort driven (rate is ignored by GripperCommand),
      // so use a fixed generous window. Arm joints scale by the slew rate.
      const expectedMs =
        spec.id === 'gripper'
          ? 2500
          : Math.max(450, (disp / Math.max(1, rateDegPerSec)) * 1000 + 900)
      setLastSend({ target: angleDeg, at: performance.now(), expectedMs })
      const revertToLive = () => {
        // Only snap back to the readback when it's actually live — reverting to
        // a frozen value would move the thumb to a stale pose at the worst time.
        if (fresh && currentDegRef.current != null) setTarget(Math.round(currentDegRef.current))
      }
      try {
        const res = await callService<SetServoAngleWithSpeedRequest, { status: boolean }>(
          setAngleService,
          MIRTE_SRV.SetServoAngleWithSpeed,
          { angle: angleDeg, rate: rateDegPerSec, degrees: true },
        )
        if (id !== inFlightRef.current) return // superseded by a newer send
        if (res?.status === false) {
          setSendError('rejected by bridge — joint not seeded yet (no /joint_states)')
          revertToLive()
        }
      } catch (e) {
        if (id !== inFlightRef.current) return
        setSendError(humanizeError(e))
        revertToLive()
      }
    },
    [callService, setAngleService, rateDegPerSec, spec.id, fresh],
  )

  const commit = useCallback(
    (deg: number) => {
      touchedRef.current = true
      const clamped = Math.max(spec.minDeg, Math.min(spec.maxDeg, deg))
      setTarget(clamped)
      void send(clamped)
    },
    [send, spec.minDeg, spec.maxDeg],
  )

  const targetRad = hmiDegToTargetRad(spec.id, target)
  const range = spec.maxDeg - spec.minDeg
  const currentPct =
    currentDeg === null
      ? null
      : ((Math.max(spec.minDeg, Math.min(spec.maxDeg, currentDeg)) - spec.minDeg) / range) * 100

  // Status is DERIVED from /joint_states convergence — "ack" no longer means
  // merely "RPC dispatched". moving → settled when the joint arrives; stalled
  // if it never converges within the expected travel time (surfaces silent
  // servo rejects + Hiwonder thermal/effort stalls).
  const tol = settleTol(spec.id)
  let phase: 'idle' | 'moving' | 'settled' | 'stalled' | 'error' = 'idle'
  if (sendError) phase = 'error'
  else if (lastSend) {
    const converged = fresh && currentDeg !== null && Math.abs(currentDeg - lastSend.target) <= tol
    if (converged) phase = 'settled'
    else if (performance.now() - lastSend.at > lastSend.expectedMs) phase = 'stalled'
    else phase = 'moving'
  }

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
            <span className={cn('ticker text-base', fresh ? 'text-foreground' : 'text-muted-foreground/60')}>
              {currentDeg === null ? '—' : `${currentDeg >= 0 ? '+' : ''}${currentDeg.toFixed(1)}°`}
            </span>
            {currentDeg !== null && !fresh ? (
              <span className="tag text-warning" title="No /joint_states update in >2s — readout may be stale">
                ○ stale
              </span>
            ) : null}
          </span>
        </div>

        {/* Slider with a faint ghost bar showing live position underneath.
            Hidden when the feed is stale so a frozen position isn't shown
            as if it were live. */}
        <div className="relative">
          {currentPct !== null && fresh ? (
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
            onValueChange={(v) => {
              touchedRef.current = true
              setTarget(v[0])
            }}
            onValueCommit={(v) => commit(v[0])}
            disabled={disabled}
            aria-label={`${spec.label} target angle, degrees`}
          />
        </div>

        {/* Fine control: nudge ±1° and exact numeric entry (matters for the
            wrist camera-tilt onto an AprilTag — 1° over the slider's range is
            otherwise the finest reachable step). */}
        <div className="flex items-center gap-2">
          <Button
            variant="outline"
            size="icon"
            className="h-7 w-7"
            disabled={disabled || target <= spec.minDeg}
            onClick={() => commit(target - 1)}
            aria-label={`Nudge ${spec.label} down 1 degree`}
          >
            <Minus className="h-3.5 w-3.5" />
          </Button>
          <Input
            type="number"
            inputMode="decimal"
            step={1}
            min={spec.minDeg}
            max={spec.maxDeg}
            value={Number.isFinite(target) ? target : 0}
            disabled={disabled}
            onChange={(e) => {
              const v = Number(e.target.value)
              if (!Number.isNaN(v)) {
                touchedRef.current = true
                setTarget(v)
              }
            }}
            onKeyDown={(e) => {
              if (e.key === 'Enter') commit(target)
            }}
            onBlur={() => commit(target)}
            className="h-7 w-20 text-center text-xs"
            aria-label={`${spec.label} exact target, degrees`}
          />
          <Button
            variant="outline"
            size="icon"
            className="h-7 w-7"
            disabled={disabled || target >= spec.maxDeg}
            onClick={() => commit(target + 1)}
            aria-label={`Nudge ${spec.label} up 1 degree`}
          >
            <Plus className="h-3.5 w-3.5" />
          </Button>
          <span className="tag">°</span>
          <span className="ml-auto flex items-center gap-2">
            <span className="tag">{spec.minDeg}°</span>
            <StatusBadge phase={phase} target={lastSend?.target ?? null} errMsg={sendError} />
            <span className="tag">+{spec.maxDeg}°</span>
          </span>
        </div>
      </CardContent>
    </Card>
  )
}

function StatusBadge({
  phase,
  target,
  errMsg,
}: {
  phase: 'idle' | 'moving' | 'settled' | 'stalled' | 'error'
  target: number | null
  errMsg: string | null
}) {
  switch (phase) {
    case 'moving':
      return <span className="tag text-primary/90">moving…</span>
    case 'settled':
      return (
        <span className="tag text-primary">
          settled{target !== null ? ` · ${target >= 0 ? '+' : ''}${target.toFixed(0)}°` : ''}
        </span>
      )
    case 'stalled':
      return (
        <span className="tag text-warning" title="Commanded but the joint did not reach the target in time — servo reject, thermal/effort stall, or torque off">
          ⚠ stalled
        </span>
      )
    case 'error':
      return (
        <span className="tag text-destructive" title={errMsg ?? undefined}>
          send failed
        </span>
      )
    default:
      return <span className="tag">idle</span>
  }
}
