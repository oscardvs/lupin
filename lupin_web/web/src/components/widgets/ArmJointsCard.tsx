import { useRef } from 'react'

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { RAD2DEG } from '@/lib/arm'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender } from '@/lib/throttle'
import { cn } from '@/lib/utils'
import { ROS_TYPE, type JointState } from '@/types/ros'

// MIRTE master arm = 5-DOF Hiwonder + gripper. Joint names come from the
// vendor URDF; they don't share a common prefix, so match by membership.
const ARM_JOINT_NAMES = new Set([
  'shoulder_pan_joint',
  'shoulder_lift_joint',
  'elbow_joint',
  'wrist_joint',
  'gripper_joint',
])

export function ArmJointsCard({ className }: { className?: string } = {}) {
  const [{ jointStatesTopic }] = useSettings()
  const lastMsgRef = useRef(0)
  const ref = useTopic<JointState>(jointStatesTopic, ROS_TYPE.JointState, {
    onMessage: () => {
      lastMsgRef.current = performance.now()
    },
  })
  useThrottledRender(10)
  const js = ref.current
  // Flag a frozen joint_states feed instead of rendering the last sample
  // as if it were live (8 Hz re-render re-evaluates this; no interval).
  const stale = js !== null && performance.now() - lastMsgRef.current > 2000

  const arm = js
    ? js.name
        .map((n, i) => ({ name: n, position: js.position[i] ?? 0 }))
        // Allow either the canonical MIRTE joint names OR a generic `arm_*`
        // prefix so this still works against vendor URDFs that namespace
        // their joints differently.
        .filter((j) => ARM_JOINT_NAMES.has(j.name) || j.name.startsWith('arm_'))
    : []

  return (
    <Card className={cn('flex flex-col', className)}>
      <CardHeader>
        <CardTitle>
          Arm joints
          {stale ? (
            <span className="tag ml-auto text-warning" title="No joint_states update in >2s — readout may be stale">
              ○ stale
            </span>
          ) : null}
          <span className={cn('tag tag-accent', stale ? 'ml-2' : 'ml-auto')}>PNL-ARM-01</span>
        </CardTitle>
        {/* Raw controller-frame diagnostic: degrees are a straight rad→deg of
            /joint_states (NOT the command scale the Arm page sliders use — the
            gripper especially reads differently because the slider maps a ±30°
            HMI window onto the URDF gripper_joint range). */}
        <CardDescription>raw /joint_states · controller frame · {jointStatesTopic}</CardDescription>
      </CardHeader>
      <CardContent className="flex flex-col gap-2 text-sm">
        {arm.length === 0 ? (
          <div className="tag">No arm joints reported.</div>
        ) : (
          arm.map((j) => (
            <div key={j.name} className="flex items-center gap-3">
              <span className="w-28 truncate font-mono text-[11px] text-muted-foreground">{j.name}</span>
              <div className="relative h-[3px] flex-1 overflow-hidden rounded-full bg-muted/60">
                <span aria-hidden className="absolute inset-y-0 left-1/2 w-px bg-hairline" />
                <div
                  className="absolute inset-y-0 left-1/2 rounded-full bg-primary"
                  style={{
                    transform: `translateX(${j.position >= 0 ? '0%' : '-100%'})`,
                    width: `${Math.min(50, (Math.abs(j.position) / Math.PI) * 50)}%`,
                  }}
                />
              </div>
              <span className="w-14 text-right ticker text-[11px]">
                {(j.position * RAD2DEG).toFixed(1)}°
              </span>
            </div>
          ))
        )}
      </CardContent>
    </Card>
  )
}
