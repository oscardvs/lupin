import { Battery, BatteryCharging, BatteryLow, BatteryWarning } from 'lucide-react'

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
  const pctText = pct == null ? '—' : `${Math.round(pct * 100)}%`
  const voltageText = battery?.voltage == null ? '—' : `${battery.voltage.toFixed(2)} V`

  let Icon = Battery
  let color = 'text-emerald-400'
  if (charging) {
    Icon = BatteryCharging
    color = 'text-sky-400'
  } else if (pct != null) {
    if (pct < 0.15) {
      Icon = BatteryWarning
      color = 'text-red-500'
    } else if (pct < 0.3) {
      Icon = BatteryLow
      color = 'text-amber-400'
    }
  }

  return (
    <Tooltip>
      <TooltipTrigger asChild>
        <div className="flex shrink-0 items-center gap-1.5 rounded-full border bg-card/60 px-2.5 py-1 text-xs sm:px-3">
          <Icon className={cn('h-4 w-4', color)} strokeWidth={2.25} />
          <span className="font-mono tabular-nums">{pctText}</span>
        </div>
      </TooltipTrigger>
      <TooltipContent side="bottom">
        <div>Voltage: <span className="font-mono">{voltageText}</span></div>
        <div className="text-muted-foreground">Topic: {batteryTopic}</div>
      </TooltipContent>
    </Tooltip>
  )
}
