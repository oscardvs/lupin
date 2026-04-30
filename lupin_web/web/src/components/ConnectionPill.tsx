import { Tooltip, TooltipContent, TooltipTrigger } from '@/components/ui/tooltip'
import { useRos } from '@/lib/ros'
import { cn } from '@/lib/utils'

const dotColor: Record<ReturnType<typeof useRos>['status'], string> = {
  connecting: 'bg-amber-400',
  connected: 'bg-emerald-500',
  closed: 'bg-zinc-400',
  error: 'bg-red-500',
}

export function ConnectionPill() {
  const { status, latencyMs, url, mode, lastError } = useRos()
  const animate = status === 'connecting' || status === 'connected'

  const latencyText =
    status !== 'connected' || latencyMs == null ? '—' : `${latencyMs} ms`

  return (
    <Tooltip>
      <TooltipTrigger asChild>
        <div className="flex shrink-0 items-center gap-1.5 rounded-full border bg-card/60 px-2.5 py-1 text-xs sm:gap-2 sm:px-3">
          <span className="relative inline-flex h-2.5 w-2.5">
            {animate ? (
              <span className={cn('absolute inline-flex h-full w-full animate-ping rounded-full opacity-60', dotColor[status])} />
            ) : null}
            <span className={cn('relative inline-flex h-2.5 w-2.5 rounded-full', dotColor[status])} />
          </span>
          <span className="hidden font-medium uppercase tracking-wide sm:inline">
            {mode === 'mock' ? 'mock' : status}
          </span>
          <span className="font-mono text-muted-foreground tabular-nums">{latencyText}</span>
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
