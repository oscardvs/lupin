import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { cn } from '@/lib/utils'

/**
 * Placeholder card for system-health metrics (CPU/RAM/temp).
 * The MIRTE image doesn't currently expose these as ROS topics; once a
 * `lupin_system_metrics` node lands we'll wire the cards in here.
 */
export function SystemCard({ className }: { className?: string } = {}) {
  return (
    <Card className={cn('flex flex-col', className)}>
      <CardHeader>
        <CardTitle>
          System
          <span className="tag tag-accent ml-auto">PNL-SYS-01</span>
        </CardTitle>
        <CardDescription>awaiting lupin_system_metrics</CardDescription>
      </CardHeader>
      <CardContent className="grid grid-cols-3 gap-2 text-center">
        {(['cpu', 'ram', 'temp'] as const).map((k) => (
          <div key={k} className="rounded-sm border border-dashed border-hairline bg-background/30 px-2 py-3">
            <div className="tag">{k}</div>
            <div className="ticker text-muted-foreground mt-1">—</div>
          </div>
        ))}
      </CardContent>
    </Card>
  )
}
