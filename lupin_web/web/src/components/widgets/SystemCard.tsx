import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'

/**
 * Placeholder card for system-health metrics (CPU/RAM/temp).
 * The MIRTE image doesn't currently expose these as ROS topics; once a
 * `lupin_system_metrics` node lands we'll wire the cards in here.
 */
export function SystemCard() {
  return (
    <Card className="flex flex-col">
      <CardHeader className="pb-2">
        <CardTitle>System</CardTitle>
        <CardDescription>Awaiting <span className="font-mono">lupin_system_metrics</span> publisher</CardDescription>
      </CardHeader>
      <CardContent className="grid grid-cols-3 gap-2 text-center">
        {(['CPU', 'RAM', 'temp'] as const).map((k) => (
          <div key={k} className="rounded-md border border-dashed bg-muted/20 px-2 py-3">
            <div className="text-[10px] uppercase tracking-wider text-muted-foreground">{k}</div>
            <div className="font-mono text-sm text-muted-foreground">—</div>
          </div>
        ))}
      </CardContent>
    </Card>
  )
}
