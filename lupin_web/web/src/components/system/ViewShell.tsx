import type { ReactNode } from 'react'

import { cn } from '@/lib/utils'

/**
 * ViewShell — the height-budget layout primitive every tab view roots in.
 *
 * The app shell already provides ONE scroll region (App.tsx wraps each
 * TabsContent in `flex-1 min-h-0 overflow-y-auto`). ViewShell decides whether a
 * view scrolls the page or fits it:
 *
 *   intent="fit"  → the page never scrolls. Children distribute the remaining
 *                   height: give the ONE primary panel `flex-1 min-h-0` so it
 *                   grows to fill the viewport, and let secondary content
 *                   collapse or scroll internally. Used by control + vision
 *                   views (Teleop, Map/Nav, Cameras, Telemetry).
 *
 *   intent="flow" → a single internal scroll region for inherently-long feeds
 *                   (Logs stream, Voice transcript, Arm joint list).
 *
 * It owns the standard padding/gap density scale (`--pad`/`--gap`, which bump
 * up at sm and xl) so spacing retunes app-wide from one place.
 */
export function ViewShell({
  intent = 'fit',
  className,
  children,
}: {
  intent?: 'fit' | 'flow'
  className?: string
  children: ReactNode
}) {
  return (
    <div
      className={cn(
        'flex h-full min-h-0 w-full flex-col gap-[var(--gap)] p-[var(--pad)]',
        // `fit` is strict from sm up (desktop/tablet are tall enough to fill
        // without scroll). On phone it keeps a scroll safety-valve so a rare
        // overflow (e.g. stacked e-stop + mission banners) never clips a
        // primary control like STOP — it just scrolls instead.
        intent === 'fit' ? 'overflow-y-auto sm:overflow-hidden' : 'overflow-y-auto',
        className,
      )}
    >
      {children}
    </div>
  )
}
