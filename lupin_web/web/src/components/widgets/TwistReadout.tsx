import type { Twist } from '@/types/ros'
import { cn } from '@/lib/utils'

interface TwistReadoutProps {
  value: Twist | null
  className?: string
}

export function TwistReadout({ value, className }: TwistReadoutProps) {
  const vx = value?.linear.x ?? 0
  const vy = value?.linear.y ?? 0
  const wz = value?.angular.z ?? 0

  return (
    <div className={cn('flex flex-col items-stretch gap-2 rounded-md border bg-card p-3 text-sm', className)}>
      <div className="text-xs uppercase tracking-wider text-muted-foreground">Publishing</div>
      <Row label="vx" unit="m/s" value={vx} />
      <Row label="vy" unit="m/s" value={vy} />
      <Row label="wz" unit="rad/s" value={wz} />
    </div>
  )
}

function Row({ label, unit, value }: { label: string; unit: string; value: number }) {
  const sign = value > 0 ? '+' : value < 0 ? '−' : ' '
  return (
    <div className="flex items-baseline justify-between gap-3">
      <span className="font-mono text-xs text-muted-foreground">{label}</span>
      <span className="flex-1 text-right font-mono tabular-nums">
        {sign}
        {Math.abs(value).toFixed(2)}
      </span>
      <span className="text-[10px] text-muted-foreground">{unit}</span>
    </div>
  )
}
