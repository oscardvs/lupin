import { useEffect, useState } from 'react'

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { RingGauge } from '@/components/widgets/RingGauge'
import { Sparkline } from '@/components/widgets/Sparkline'
import { Ticker } from '@/components/widgets/Ticker'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender, Ring } from '@/lib/throttle'
import { cn } from '@/lib/utils'
import { ROS_TYPE, type BatteryState } from '@/types/ros'

const HISTORY_SECONDS = 60

export function BatteryCard({ className }: { className?: string } = {}) {
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
  const pct = battery?.percentage ?? null
  const charging = battery?.power_supply_status === 1

  // Ring colour follows the status lanes (chartreuse → amber → red).
  const ringColor =
    pct == null
      ? 'hsl(var(--hairline))'
      : pct < 0.15
        ? 'hsl(var(--status-critical))'
        : pct < 0.3
          ? 'hsl(var(--status-caution))'
          : charging
            ? 'hsl(var(--status-data))'
            : 'hsl(var(--primary))'

  return (
    <Card className={cn('flex flex-col', className)}>
      <CardHeader>
        <CardTitle>
          Battery
          <span className="tag tag-accent ml-auto">PNL-PWR-01</span>
        </CardTitle>
        <CardDescription>{batteryTopic}</CardDescription>
      </CardHeader>
      <CardContent className="flex flex-col gap-3 text-sm">
        <div className="flex items-center gap-4">
          <RingGauge
            value={pct ?? 0}
            size={92}
            thickness={7}
            color={ringColor}
            glow={pct != null && pct < 0.3}
          >
            <div className="flex flex-col items-center leading-none">
              {pct == null ? (
                <span className="ticker text-xl">—</span>
              ) : (
                <Ticker value={pct * 100} decimals={0} reserve="100" className="text-xl text-foreground" />
              )}
              <span className="tag mt-1">charge</span>
            </div>
          </RingGauge>

          <div className="flex flex-1 flex-col gap-1.5">
            <Readout label="voltage" value={battery?.voltage ?? null} decimals={2} unit="V" />
            <Readout label="current" value={battery?.current ?? null} decimals={2} unit="A" />
            <Readout
              label="status"
              text={charging ? 'charging' : pct == null ? '—' : pct < 0.15 ? 'critical' : pct < 0.3 ? 'low' : 'nominal'}
            />
          </div>
        </div>

        <div className="flex items-end justify-between gap-3">
          <div className="tag">voltage · last {HISTORY_SECONDS}s</div>
          <Sparkline values={series} className="text-primary" width={160} height={32} />
        </div>
      </CardContent>
    </Card>
  )
}

function Readout({
  label,
  value,
  text,
  decimals = 2,
  unit,
}: {
  label: string
  value?: number | null
  text?: string
  decimals?: number
  unit?: string
}) {
  return (
    <div className="flex items-baseline justify-between gap-2 border-b border-hairline/60 pb-1">
      <span className="tag">{label}</span>
      {text != null ? (
        <span className="ticker text-sm text-foreground">{text}</span>
      ) : value == null ? (
        <span className="ticker text-sm text-muted-foreground">—</span>
      ) : (
        <Ticker value={value} decimals={decimals} unit={unit} className="text-sm text-foreground" />
      )}
    </div>
  )
}
