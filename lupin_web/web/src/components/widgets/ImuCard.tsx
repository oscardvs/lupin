import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender } from '@/lib/throttle'
import { ROS_TYPE, type Imu, quatToEuler } from '@/types/ros'
import { cn } from '@/lib/utils'

const RAD2DEG = 180 / Math.PI

export function ImuCard() {
  const [{ imuTopic }] = useSettings()
  const ref = useTopic<Imu>(imuTopic, ROS_TYPE.Imu)
  useThrottledRender(10)

  const imu = ref.current
  const euler = imu ? quatToEuler(imu.orientation) : null
  const wx = imu?.angular_velocity.x ?? 0
  const wy = imu?.angular_velocity.y ?? 0
  const wz = imu?.angular_velocity.z ?? 0

  return (
    <Card className="flex flex-col">
      <CardHeader className="pb-2">
        <CardTitle>IMU</CardTitle>
        <CardDescription className="font-mono">{imuTopic}</CardDescription>
      </CardHeader>
      <CardContent className="flex flex-col gap-4">
        <div className="grid grid-cols-3 gap-2 text-center">
          {(['roll', 'pitch', 'yaw'] as const).map((k) => {
            const v = euler ? euler[k] * RAD2DEG : 0
            return (
              <div key={k} className="rounded-md border bg-muted/30 px-2 py-2">
                <div className="text-[10px] uppercase tracking-wider text-muted-foreground">{k}</div>
                <div className="font-mono text-base tabular-nums">
                  {imu ? `${v >= 0 ? '+' : '−'}${Math.abs(v).toFixed(1)}°` : '—'}
                </div>
              </div>
            )
          })}
        </div>

        <div className="flex flex-col gap-2 text-xs">
          <div className="text-[10px] uppercase tracking-wider text-muted-foreground">
            angular velocity (rad/s)
          </div>
          <Bar label="ωx" value={wx} max={Math.PI} />
          <Bar label="ωy" value={wy} max={Math.PI} />
          <Bar label="ωz" value={wz} max={Math.PI} />
        </div>
      </CardContent>
    </Card>
  )
}

function Bar({ label, value, max }: { label: string; value: number; max: number }) {
  const clamped = Math.max(-max, Math.min(max, value))
  const pct = (clamped / max) * 50
  const positive = pct >= 0
  return (
    <div className="flex items-center gap-3">
      <span className="w-6 font-mono text-xs text-muted-foreground">{label}</span>
      <div className="relative h-2 flex-1 rounded-full bg-muted">
        <div className="absolute inset-y-0 left-1/2 w-px bg-border" />
        <div
          className={cn('absolute inset-y-0 rounded-full', positive ? 'bg-emerald-500' : 'bg-red-500')}
          style={{
            left: positive ? '50%' : `${50 + pct}%`,
            width: `${Math.abs(pct)}%`,
          }}
        />
      </div>
      <span className="w-14 text-right font-mono text-xs tabular-nums">
        {value >= 0 ? '+' : '−'}
        {Math.abs(value).toFixed(2)}
      </span>
    </div>
  )
}
