import { useEffect, useState } from 'react'

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { Sparkline } from '@/components/widgets/Sparkline'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender, Ring } from '@/lib/throttle'
import { ROS_TYPE, type BatteryState } from '@/types/ros'

const HISTORY_SECONDS = 60

export function BatteryCard() {
  const [{ batteryTopic }] = useSettings()
  const ref = useTopic<BatteryState>(batteryTopic, ROS_TYPE.BatteryState)
  const [history] = useState(() => new Ring<number>(HISTORY_SECONDS))
  useThrottledRender(1)

  // sample once per second into the ring
  useEffect(() => {
    const id = setInterval(() => {
      const v = ref.current?.voltage
      if (v != null && Number.isFinite(v)) history.push(v)
    }, 1000)
    return () => clearInterval(id)
  }, [history, ref])

  const battery = ref.current
  const series = history.snapshot()

  return (
    <Card className="flex flex-col">
      <CardHeader>
        <CardTitle>
          Battery
          <span className="tag tag-accent ml-auto">PNL-PWR-01</span>
        </CardTitle>
        <CardDescription>{batteryTopic}</CardDescription>
      </CardHeader>
      <CardContent className="flex flex-col gap-3 text-sm">
        <div className="grid grid-cols-3 gap-2 text-center">
          <Stat label="charge" value={battery?.percentage == null ? '—' : `${Math.round(battery.percentage * 100)}%`} />
          <Stat label="voltage" value={battery?.voltage == null ? '—' : `${battery.voltage.toFixed(2)} V`} />
          <Stat label="current" value={battery?.current == null ? '—' : `${battery.current.toFixed(2)} A`} />
        </div>
        <div className="flex items-end justify-between gap-3">
          <div className="tag">voltage · last {HISTORY_SECONDS}s</div>
          <Sparkline values={series} className="text-primary" width={160} height={32} />
        </div>
      </CardContent>
    </Card>
  )
}

function Stat({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-sm border border-hairline bg-background/40 px-2 py-1.5">
      <div className="tag">{label}</div>
      <div className="ticker text-sm mt-0.5">{value}</div>
    </div>
  )
}
