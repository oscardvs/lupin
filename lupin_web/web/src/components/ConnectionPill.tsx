import { useEffect, useRef, useState } from 'react'

import { Tooltip, TooltipContent, TooltipTrigger } from '@/components/ui/tooltip'
import { useRos, useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { cn } from '@/lib/utils'
import { ROS_TYPE, type BatteryState, type JointState } from '@/types/ros'

const dotColor: Record<ReturnType<typeof useRos>['status'], string> = {
  connecting: 'bg-warning',
  connected: 'bg-primary',
  closed: 'bg-muted-foreground',
  error: 'bg-destructive',
}

const labelText: Record<ReturnType<typeof useRos>['status'], string> = {
  connecting: 'LINK',
  connected: 'LIVE',
  closed: 'IDLE',
  error: 'FAULT',
}

// rosbridge can report "connected" while silently delivering nothing (the
// documented silent-wedge). Treat the link as STALE if no telemetry arrives on
// any watched topic within this window — surfaced as a distinct amber chip, NOT
// a happy LIVE.
const STALE_MS = 3500

export function ConnectionPill() {
  const { status, latencyMs, url, mode, lastError } = useRos()
  const [{ batteryTopic, jointStatesTopic }] = useSettings()
  const lastMsg = useRef(performance.now())
  const [stale, setStale] = useState(false)

  const bump = () => {
    lastMsg.current = performance.now()
  }
  useTopic<BatteryState>(batteryTopic, ROS_TYPE.BatteryState, { onMessage: bump })
  useTopic<JointState>(jointStatesTopic, ROS_TYPE.JointState, { onMessage: bump })

  useEffect(() => {
    const id = setInterval(() => {
      setStale(performance.now() - lastMsg.current > STALE_MS)
    }, 1000)
    return () => clearInterval(id)
  }, [])

  const animate = status === 'connecting' || status === 'connected'
  const isStale = mode !== 'mock' && status === 'connected' && stale

  const latencyText = status !== 'connected' || latencyMs == null ? '—' : `${latencyMs}ms`
  const label = mode === 'mock' ? 'MOCK' : isStale ? 'STALE' : labelText[status]
  const dot = isStale ? 'bg-warning' : dotColor[status]

  return (
    <Tooltip>
      <TooltipTrigger asChild>
        <div
          className={cn(
            'flex h-9 shrink-0 items-center gap-2 rounded-sm border border-hairline bg-card/40 px-2.5',
            'text-[11px] tracking-[0.16em]',
            isStale && 'border-warning/40',
          )}
        >
          <span className="relative inline-flex h-2 w-2">
            {animate && !isStale ? (
              <span className={cn('absolute inline-flex h-full w-full animate-ping rounded-full opacity-70', dot)} />
            ) : null}
            <span className={cn('relative inline-flex h-2 w-2 rounded-full', dot)} />
          </span>
          <span
            className={cn(
              'hidden font-semibold uppercase sm:inline',
              isStale ? 'text-warning' : 'text-foreground',
            )}
          >
            {label}
          </span>
          <span className="ticker text-[11px] text-muted-foreground">{latencyText}</span>
        </div>
      </TooltipTrigger>
      <TooltipContent side="bottom" className="max-w-sm">
        <div className="font-medium">{url}</div>
        {isStale ? (
          <div className="mt-1 text-warning">
            Link up but no telemetry for &gt;{Math.round(STALE_MS / 1000)}s — possible rosbridge wedge.
            Restart the rosbridge / mirte-ros service.
          </div>
        ) : null}
        {lastError ? <div className="mt-1 text-destructive">{lastError}</div> : null}
        {mode === 'mock' ? (
          <div className="mt-1 text-muted-foreground">
            Mock mode — synthetic data, no real rosbridge connection.
          </div>
        ) : null}
      </TooltipContent>
    </Tooltip>
  )
}
