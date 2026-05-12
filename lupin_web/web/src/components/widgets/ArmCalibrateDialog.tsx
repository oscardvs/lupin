import { AlertTriangle, Compass, HandMetal, Loader2, RotateCcw } from 'lucide-react'
import { useCallback, useEffect, useRef, useState } from 'react'

import { Button } from '@/components/ui/button'
import { Sheet, SheetContent, SheetDescription, SheetHeader, SheetTitle } from '@/components/ui/sheet'
import { useEStop } from '@/lib/estop'
import { useRos } from '@/lib/ros'
import {
  LUPIN_SRV,
  type CalibrateArmRequest,
  type CalibrateArmResponse,
} from '@/types/ros'

/**
 * Hiwonder zero-offset calibration wizard.
 *
 * Wraps the `/lupin/arm/calibrate` service (lupin_msgs/srv/CalibrateArm).
 * The service is multi-action because the calibration is operator-in-the-
 * loop: between `start` and `commit` the operator physically hand-poses
 * the arm to its mechanical home. The dialog's local state machine maps
 * 1:1 to the server's:
 *
 *   idle            → press Start              → awaiting_pose
 *   awaiting_pose   → press Save calibration   → done   (commit)
 *   awaiting_pose   → press Cancel             → idle   (cancel)
 *   done            → press Close              → idle
 *
 * On open the dialog calls `{action: 'status'}` first — if the server
 * reports AWAITING_POSE (e.g. previous browser tab crashed mid-flow) we
 * jump straight to that step so the robot doesn't stay limp forever.
 *
 * Hardware-only. The Hiwonder `_set_offset` services do not exist in sim
 * — the dialog shows a warning when the connection is mocked / the server
 * returns an error early.
 */

type WizardState = 'idle' | 'awaiting_pose' | 'done' | 'error'

interface JointResult {
  name: string
  diff: number
  offset: number
}

interface ArmCalibrateDialogProps {
  open: boolean
  onOpenChange: (open: boolean) => void
}

const CALIBRATE_SRV = '/lupin/arm/calibrate'

export function ArmCalibrateDialog({ open, onOpenChange }: ArmCalibrateDialogProps) {
  const { callService, status: rosStatus, mode } = useRos()
  const { active: estopActive } = useEStop()

  const [wizard, setWizard] = useState<WizardState>('idle')
  const [busy, setBusy] = useState(false)
  const [message, setMessage] = useState<string | null>(null)
  const [cancelWarning, setCancelWarning] = useState<string | null>(null)
  const [results, setResults] = useState<JointResult[]>([])
  // Sequence guard so a late response from a cancelled action can't
  // resurrect the wizard.
  const seqRef = useRef(0)

  const blocked = estopActive || rosStatus !== 'connected' || busy

  const call = useCallback(
    async (action: CalibrateArmRequest['action']) => {
      const my = ++seqRef.current
      setBusy(true)
      try {
        const res = await callService<CalibrateArmRequest, CalibrateArmResponse>(
          CALIBRATE_SRV,
          LUPIN_SRV.CalibrateArm,
          { action },
        )
        if (my !== seqRef.current) return null
        setMessage(res.message ?? null)
        return res
      } catch (e) {
        if (my !== seqRef.current) return null
        setMessage(e instanceof Error ? e.message : String(e))
        setWizard('error')
        return null
      } finally {
        if (my === seqRef.current) setBusy(false)
      }
    },
    [callService],
  )

  // On open: probe the server state. Recovers from a stranded
  // AWAITING_POSE (browser refresh mid-flow).
  useEffect(() => {
    if (!open) return
    let cancelled = false
    setResults([])
    setMessage(null)
    setCancelWarning(null)
    setWizard('idle')
    // Clear any stale busy flag from a prior closed-while-in-flight call.
    setBusy(false)
    void (async () => {
      const res = await call('status')
      if (cancelled || !res) return
      if (res.state === 'AWAITING_POSE') {
        setWizard('awaiting_pose')
      } else {
        setWizard('idle')
      }
    })()
    return () => {
      cancelled = true
      // Bump seq so any in-flight response is ignored. Also clear busy
      // so reopening the sheet doesn't find every button disabled.
      seqRef.current += 1
      setBusy(false)
    }
  }, [open, call])

  const onStart = useCallback(async () => {
    const res = await call('start')
    if (!res) return
    if (res.success && res.state === 'AWAITING_POSE') {
      setWizard('awaiting_pose')
    } else {
      setWizard('error')
    }
  }, [call])

  const onCommit = useCallback(async () => {
    const res = await call('commit')
    if (!res) return
    if (res.success) {
      const joints: JointResult[] = res.joint_names.map((name, i) => ({
        name,
        diff: res.diffs_observed?.[i] ?? 0,
        offset: res.offsets_applied?.[i] ?? 0,
      }))
      setResults(joints)
      setWizard('done')
    } else {
      setWizard('error')
    }
  }, [call])

  const onCancel = useCallback(async () => {
    await call('cancel')
    setWizard('idle')
  }, [call])

  const onClose = useCallback(async () => {
    // If the operator dismisses the sheet while awaiting pose, cancel so
    // the robot doesn't stay limp. Await the cancel; if it fails (rosbridge
    // wedged), keep the sheet open and surface a warning — closing would
    // hide the only UI that can recover the robot.
    if (wizard === 'awaiting_pose') {
      const res = await call('cancel')
      if (!res || !res.success) {
        setCancelWarning(
          'Cancel did not confirm — the arm may still be limp. ' +
          'Reset the e-stop or restart mirte-ros to recover.',
        )
        return
      }
    }
    onOpenChange(false)
  }, [wizard, call, onOpenChange])

  return (
    <Sheet
      open={open}
      onOpenChange={(o) => {
        if (o) onOpenChange(true)
        else void onClose()
      }}
    >
      <SheetContent side="right" className="flex w-full flex-col gap-0 p-0 sm:max-w-md">
        <SheetHeader className="border-b p-4">
          <SheetTitle className="flex items-center gap-2">
            <Compass className="h-4 w-4 text-primary" />
            Arm calibration
            <span className="tag tag-accent ml-auto">PNL-ARM-CAL</span>
          </SheetTitle>
          <SheetDescription>
            Zero the Hiwonder servo offsets. The arm goes limp between Start and Save —
            be ready to support it.
          </SheetDescription>
        </SheetHeader>

        <div className="flex flex-1 flex-col gap-4 overflow-y-auto p-4">
          {mode === 'mock' ? (
            <Banner kind="warn">
              Mock mode — the service response is synthetic and the robot is not affected.
            </Banner>
          ) : null}

          {estopActive ? (
            <Banner kind="error">
              E-stop engaged. Reset the E-stop on the Arm or Teleop tab before calibrating.
            </Banner>
          ) : null}

          {cancelWarning ? (
            <Banner kind="error">{cancelWarning}</Banner>
          ) : null}

          {wizard === 'idle' ? (
            <IdleStep onStart={onStart} disabled={blocked || estopActive} />
          ) : null}

          {wizard === 'awaiting_pose' ? (
            <AwaitingPoseStep
              onCommit={onCommit}
              onCancel={onCancel}
              disabled={busy}
            />
          ) : null}

          {wizard === 'done' ? <DoneStep results={results} /> : null}

          {wizard === 'error' ? (
            <Banner kind="error">
              {message ?? 'Calibration failed. Check the robot logs for details.'}
            </Banner>
          ) : null}

          {message && wizard !== 'error' ? (
            <p className="text-xs text-muted-foreground">{message}</p>
          ) : null}
        </div>

        <div className="flex items-center justify-end gap-2 border-t p-4">
          {wizard === 'awaiting_pose' ? (
            <Button variant="outline" size="sm" onClick={onCancel} disabled={busy}>
              Cancel
            </Button>
          ) : null}
          <Button variant="outline" size="sm" onClick={() => void onClose()} disabled={busy}>
            {wizard === 'done' ? 'Close' : wizard === 'awaiting_pose' ? 'Dismiss' : 'Close'}
          </Button>
          {wizard === 'error' ? (
            <Button size="sm" onClick={() => setWizard('idle')} disabled={busy}>
              <RotateCcw className="mr-2 h-4 w-4" />
              Retry
            </Button>
          ) : null}
        </div>
      </SheetContent>
    </Sheet>
  )
}

function IdleStep({ onStart, disabled }: { onStart: () => void; disabled: boolean }) {
  return (
    <div className="flex flex-col gap-4">
      <Banner kind="warn">
        <strong>The arm will go limp.</strong> After you press Start, the servos are
        disabled so you can hand-pose them. Be ready to physically support the arm —
        gravity wins.
      </Banner>
      <ol className="space-y-2 text-sm text-muted-foreground">
        <li><span className="tag mr-2">1</span>Press <strong>Start calibration</strong>. The arm controller releases and the servos disable.</li>
        <li><span className="tag mr-2">2</span>Physically move the arm into its <strong>mechanical home pose</strong> (all four joints at their URDF zero).</li>
        <li><span className="tag mr-2">3</span>Press <strong>Save calibration</strong>. The current raw positions are written as new zero offsets.</li>
      </ol>
      <Button onClick={onStart} disabled={disabled} className="self-start">
        <Compass className="mr-2 h-4 w-4" />
        Start calibration
      </Button>
    </div>
  )
}

function AwaitingPoseStep({
  onCommit,
  onCancel,
  disabled,
}: {
  onCommit: () => void
  onCancel: () => void
  disabled: boolean
}) {
  return (
    <div className="flex flex-col gap-4">
      <div className="flex items-start gap-3 rounded-sm border border-primary/40 bg-primary/5 p-3">
        <HandMetal className="mt-0.5 h-5 w-5 shrink-0 text-primary" />
        <div className="text-sm">
          <p className="font-semibold text-foreground">Hand-pose the arm to home.</p>
          <p className="text-muted-foreground">
            Servos are disabled. Move every joint to its mechanical home position, then
            press Save. The server samples positions for ~2 s after you press Save.
          </p>
        </div>
      </div>
      <Button onClick={onCommit} disabled={disabled} className="self-start">
        {disabled ? (
          <Loader2 className="mr-2 h-4 w-4 animate-spin" />
        ) : (
          <Compass className="mr-2 h-4 w-4" />
        )}
        Save calibration
      </Button>
      <p className="text-xs text-muted-foreground">
        Cancel re-enables the servos and leaves existing offsets unchanged.
      </p>
      <Button variant="ghost" size="sm" onClick={onCancel} disabled={disabled} className="self-start">
        Cancel calibration
      </Button>
    </div>
  )
}

function DoneStep({ results }: { results: JointResult[] }) {
  return (
    <div className="flex flex-col gap-3">
      <Banner kind="ok">Calibration written. Arm controller re-enabled.</Banner>
      <div className="rounded-sm border border-hairline">
        <table className="w-full text-sm">
          <thead className="bg-card/40 text-xs uppercase tracking-wider text-muted-foreground">
            <tr>
              <th className="px-3 py-2 text-left">Joint</th>
              <th className="px-3 py-2 text-right">Diff (raw)</th>
              <th className="px-3 py-2 text-right">Offset (cdeg)</th>
            </tr>
          </thead>
          <tbody>
            {results.length === 0 ? (
              <tr>
                <td colSpan={3} className="px-3 py-3 text-center text-muted-foreground">
                  no per-joint data returned
                </td>
              </tr>
            ) : (
              results.map((r) => (
                <tr key={r.name} className="border-t border-hairline">
                  <td className="px-3 py-2 font-mono">{r.name}</td>
                  <td className="px-3 py-2 text-right font-mono">{r.diff >= 0 ? '+' : ''}{r.diff}</td>
                  <td className="px-3 py-2 text-right font-mono">{r.offset >= 0 ? '+' : ''}{r.offset}</td>
                </tr>
              ))
            )}
          </tbody>
        </table>
      </div>
    </div>
  )
}

function Banner({
  kind,
  children,
}: {
  kind: 'warn' | 'error' | 'ok'
  children: React.ReactNode
}) {
  const style =
    kind === 'error'
      ? 'border-destructive bg-destructive/10 text-destructive'
      : kind === 'ok'
        ? 'border-primary/40 bg-primary/10 text-primary'
        : 'border-fuchsia-500/40 bg-fuchsia-500/10 text-fuchsia-200'
  return (
    <div className={`flex items-start gap-2 rounded-sm border p-3 text-sm ${style}`}>
      <AlertTriangle className="mt-0.5 h-4 w-4 shrink-0" />
      <div className="leading-snug">{children}</div>
    </div>
  )
}
