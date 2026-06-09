import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender } from '@/lib/throttle'
import { cn } from '@/lib/utils'
import { ROS_TYPE, type Odometry, quatToEuler } from '@/types/ros'

const RAD2DEG = 180 / Math.PI

export function OdometryCard({ className }: { className?: string } = {}) {
  const [{ odomTopic }] = useSettings()
  const ref = useTopic<Odometry>(odomTopic, ROS_TYPE.Odometry)
  useThrottledRender(5)
  const odom = ref.current
  const yawDeg = odom ? quatToEuler(odom.pose.pose.orientation).yaw * RAD2DEG : 0

  return (
    <Card className={cn('flex flex-col', className)}>
      <CardHeader>
        <CardTitle>
          Odometry
          <span className="tag tag-accent ml-auto">PNL-ODM-01</span>
        </CardTitle>
        <CardDescription>{odomTopic}</CardDescription>
      </CardHeader>
      <CardContent className="grid grid-cols-2 gap-2 text-sm">
        <Cell label="x" unit="m" value={odom?.pose.pose.position.x ?? null} />
        <Cell label="y" unit="m" value={odom?.pose.pose.position.y ?? null} />
        <Cell label="θ" unit="°" value={odom ? yawDeg : null} digits={1} />
        <Cell label="vx" unit="m/s" value={odom?.twist.twist.linear.x ?? null} />
        <Cell label="vy" unit="m/s" value={odom?.twist.twist.linear.y ?? null} />
        <Cell label="ωz" unit="rad/s" value={odom?.twist.twist.angular.z ?? null} />
      </CardContent>
    </Card>
  )
}

function Cell({
  label,
  value,
  unit,
  digits = 2,
}: {
  label: string
  unit: string
  value: number | null
  digits?: number
}) {
  return (
    <div className="rounded-sm border border-hairline bg-ink-3 px-2 py-1.5">
      <div className="flex items-baseline justify-between">
        <span className="tag tag-accent">{label}</span>
        <span className="tag">{unit}</span>
      </div>
      <div className="ticker text-base mt-0.5">
        {value == null
          ? '—'
          : `${value >= 0 ? '+' : '−'}${Math.abs(value).toFixed(digits)}`}
      </div>
    </div>
  )
}
