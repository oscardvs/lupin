import { Maximize2 } from 'lucide-react'

import { cn } from '@/lib/utils'

/**
 * The shared "maximize" affordance dropped in the top-right of every vision
 * panel (map, camera, twin, lidar). Tag-styled so it reads as instrument chrome;
 * a 32px hit area keeps it touch-reachable.
 */
export function ExpandButton({
  onClick,
  className,
  label = 'Maximize',
}: {
  onClick: () => void
  className?: string
  label?: string
}) {
  return (
    <button
      type="button"
      onClick={onClick}
      aria-label={label}
      title={label}
      className={cn(
        'inline-flex h-8 w-8 items-center justify-center rounded-sm border border-hairline',
        'bg-ink-2/80 text-muted-foreground backdrop-blur-sm transition-colors',
        'hover:border-primary/40 hover:bg-ink-3 hover:text-foreground',
        'focus-visible:ring-1 focus-visible:ring-primary',
        className,
      )}
    >
      <Maximize2 className="h-4 w-4" />
    </button>
  )
}
