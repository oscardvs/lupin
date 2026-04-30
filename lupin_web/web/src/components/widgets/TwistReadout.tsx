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

  const moving =
    Math.abs(vx) + Math.abs(vy) + Math.abs(wz) > 0.001

  return (
    <div
      className={cn(
        'reticle relative flex flex-col gap-2 rounded-sm border border-hairline bg-card/70 p-3 text-sm',
        moving && 'glow-primary',
        className,
      )}
    >
      <span className="reticle-bl" aria-hidden />
      <span className="reticle-br" aria-hidden />

      <div className="flex items-center justify-between">
        <span className="tag">command · twist</span>
        <span
          className={cn(
            'flex items-center gap-1.5 text-[10px] uppercase tracking-[0.16em]',
            moving ? 'text-primary' : 'text-muted-foreground',
          )}
        >
          <span
            className={cn(
              'h-1.5 w-1.5 rounded-full',
              moving ? 'bg-primary animate-pulse' : 'bg-muted-foreground/40',
            )}
          />
          {moving ? 'tx' : 'idle'}
        </span>
      </div>

      <div className="flex flex-col gap-1">
        <Row label="vx" unit="m/s" value={vx} />
        <Row label="vy" unit="m/s" value={vy} />
        <Row label="ωz" unit="rad/s" value={wz} />
      </div>
    </div>
  )
}

function Row({ label, unit, value }: { label: string; unit: string; value: number }) {
  const sign = value > 0 ? '+' : value < 0 ? '−' : ' '
  const magnitude = Math.min(1, Math.abs(value) / (label === 'ωz' ? 1.5 : 0.4))
  return (
    <div className="grid grid-cols-[2.5rem_1fr_3.5rem_2.25rem] items-baseline gap-2">
      <span className="font-mono text-[11px] text-primary">{label}</span>
      <div className="relative h-[3px] w-full rounded-full bg-muted/60">
        <span
          aria-hidden
          className="absolute inset-y-0 left-1/2 w-px bg-hairline"
        />
        <span
          className={cn(
            'absolute inset-y-0 rounded-full',
            value >= 0 ? 'bg-primary' : 'bg-warning',
          )}
          style={{
            left: value >= 0 ? '50%' : `${50 - magnitude * 50}%`,
            width: `${magnitude * 50}%`,
          }}
        />
      </div>
      <span className="ticker text-right text-[12px]">
        {sign}
        {Math.abs(value).toFixed(2)}
      </span>
      <span className="text-[10px] text-muted-foreground">{unit}</span>
    </div>
  )
}
