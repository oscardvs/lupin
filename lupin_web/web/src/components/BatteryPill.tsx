import { Battery, BatteryCharging, BatteryLow, BatteryWarning } from 'lucide-react'

import { Ticker } from '@/components/widgets/Ticker'
import { Tooltip, TooltipContent, TooltipTrigger } from '@/components/ui/tooltip'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender } from '@/lib/throttle'
import { ROS_TYPE, type BatteryState } from '@/types/ros'
import { cn } from '@/lib/utils'

export function BatteryPill() {
  const [{ batteryTopic }] = useSettings()
  const ref = useTopic<BatteryState>(batteryTopic, ROS_TYPE.BatteryState)
  useThrottledRender(1)

  const battery = ref.current
  const pct = battery?.percentage ?? null
  const charging = battery?.power_supply_status === 1
  const voltageText = battery?.voltage == null ? '—' : `${battery.voltage.toFixed(2)} V`

  let Icon = Battery
  let color = 'text-primary'
  if (charging) {
    Icon = BatteryCharging
    color = 'text-signal'
  } else if (pct != null) {
    if (pct < 0.15) {
      Icon = BatteryWarning
      color = 'text-destructive'
    } else if (pct < 0.3) {
      Icon = BatteryLow
      color = 'text-warning'
    }
  }

  const meterPct = pct == null ? 0 : Math.max(0, Math.min(1, pct))
  const meterColor =
    pct != null && pct < 0.15
      ? 'bg-destructive'
      : pct != null && pct < 0.3
        ? 'bg-warning'
        : charging
          ? 'bg-signal'
          : 'bg-primary'

  return (
    <Tooltip>
      <TooltipTrigger asChild>
        <div
          className={cn(
            'relative flex h-9 shrink-0 items-center gap-2 overflow-hidden rounded-sm border border-hairline bg-card/40 px-2.5 text-[11px]',
            charging && 'glow-signal',
          )}
        >
          <Icon className={cn('h-[15px] w-[15px]', color)} strokeWidth={2.25} />
          {pct == null ? (
            <span className="ticker">—</span>
          ) : (
            <Ticker value={pct * 100} decimals={0} unit="%" reserve="100" className="text-[11px]" />
          )}
          {/* meter line at the bottom of the pill */}
          <span
            aria-hidden
            className={cn('absolute bottom-0 left-0 h-px transition-all duration-700', meterColor)}
            style={{ width: `${meterPct * 100}%` }}
          />
        </div>
      </TooltipTrigger>
      <TooltipContent side="bottom">
        <div>
          Voltage: <span className="font-mono">{voltageText}</span>
        </div>
        <div className="text-muted-foreground">Topic: {batteryTopic}</div>
      </TooltipContent>
    </Tooltip>
  )
}
