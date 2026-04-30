import { Tooltip, TooltipContent, TooltipTrigger } from '@/components/ui/tooltip'
import { useRos } from '@/lib/ros'
import { cn } from '@/lib/utils'

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

export function ConnectionPill() {
  const { status, latencyMs, url, mode, lastError } = useRos()
  const animate = status === 'connecting' || status === 'connected'

  const latencyText =
    status !== 'connected' || latencyMs == null ? '—' : `${latencyMs}ms`

  const label = mode === 'mock' ? 'MOCK' : labelText[status]

  return (
    <Tooltip>
      <TooltipTrigger asChild>
        <div
          className={cn(
            'flex h-9 shrink-0 items-center gap-2 rounded-sm border border-hairline bg-card/40 px-2.5',
            'text-[11px] tracking-[0.16em]',
          )}
        >
          <span className="relative inline-flex h-2 w-2">
            {animate ? (
              <span
                className={cn(
                  'absolute inline-flex h-full w-full animate-ping rounded-full opacity-70',
                  dotColor[status],
                )}
              />
            ) : null}
            <span
              className={cn(
                'relative inline-flex h-2 w-2 rounded-full',
                dotColor[status],
              )}
            />
          </span>
          <span className="hidden font-semibold uppercase text-foreground sm:inline">
            {label}
          </span>
          <span className="ticker text-[11px] text-muted-foreground">{latencyText}</span>
        </div>
      </TooltipTrigger>
      <TooltipContent side="bottom" className="max-w-sm">
        <div className="font-medium">{url}</div>
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
