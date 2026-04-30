import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender } from '@/lib/throttle'
import { ROS_TYPE, type JointState } from '@/types/ros'

const RAD2DEG = 180 / Math.PI

export function ArmJointsCard() {
  const [{ jointStatesTopic }] = useSettings()
  const ref = useTopic<JointState>(jointStatesTopic, ROS_TYPE.JointState)
  useThrottledRender(10)
  const js = ref.current

  const arm = js
    ? js.name
        .map((n, i) => ({ name: n, position: js.position[i] ?? 0 }))
        .filter((j) => j.name.startsWith('arm_'))
    : []

  return (
    <Card className="flex flex-col">
      <CardHeader className="pb-2">
        <CardTitle>Arm joints</CardTitle>
        <CardDescription className="font-mono">{jointStatesTopic}</CardDescription>
      </CardHeader>
      <CardContent className="flex flex-col gap-2 text-sm">
        {arm.length === 0 ? (
          <div className="text-xs text-muted-foreground">No arm joints reported.</div>
        ) : (
          arm.map((j) => (
            <div key={j.name} className="flex items-center gap-3">
              <span className="w-28 truncate font-mono text-xs text-muted-foreground">{j.name}</span>
              <div className="relative h-2 flex-1 overflow-hidden rounded-full bg-muted">
                <div
                  className="absolute inset-y-0 left-1/2 rounded-full bg-primary"
                  style={{
                    transform: `translateX(${j.position >= 0 ? 0 : '-100%'})`,
                    width: `${Math.min(50, (Math.abs(j.position) / Math.PI) * 50)}%`,
                  }}
                />
              </div>
              <span className="w-14 text-right font-mono text-xs tabular-nums">
                {(j.position * RAD2DEG).toFixed(1)}°
              </span>
            </div>
          ))
        )}
      </CardContent>
    </Card>
  )
}
