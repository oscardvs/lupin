import type { RosStatus } from '@/lib/ros'
import { cn } from '@/lib/utils'

const statusMeta: Record<RosStatus, { label: string; dot: string; ring: string }> = {
  connecting: {
    label: 'Connecting…',
    dot: 'bg-amber-400',
    ring: 'ring-amber-300/40',
  },
  connected: {
    label: 'Connected',
    dot: 'bg-emerald-500',
    ring: 'ring-emerald-400/40',
  },
  closed: {
    label: 'Disconnected — reconnecting',
    dot: 'bg-zinc-400',
    ring: 'ring-zinc-300/40',
  },
  error: {
    label: 'Error',
    dot: 'bg-red-500',
    ring: 'ring-red-400/40',
  },
}

interface StatusDotProps {
  status: RosStatus
  url: string
  detail?: string | null
}

export function StatusDot({ status, url, detail }: StatusDotProps) {
  const meta = statusMeta[status]
  return (
    <div className="flex items-center gap-3 rounded-md border bg-card px-3 py-2 text-sm">
      <span className="relative inline-flex h-3 w-3">
        {status === 'connecting' || status === 'connected' ? (
          <span
            className={cn(
              'absolute inline-flex h-full w-full animate-ping rounded-full opacity-60',
              meta.dot,
            )}
          />
        ) : null}
        <span
          className={cn(
            'relative inline-flex h-3 w-3 rounded-full ring-2',
            meta.dot,
            meta.ring,
          )}
        />
      </span>
      <div className="flex min-w-0 flex-col">
        <span className="font-medium">{meta.label}</span>
        <span className="truncate text-xs text-muted-foreground">{url}</span>
        {detail ? (
          <span className="truncate text-xs text-red-500">{detail}</span>
        ) : null}
      </div>
    </div>
  )
}
