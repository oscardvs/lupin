import { Map as MapIcon } from 'lucide-react'

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'

export function MapView() {
  return (
    <div className="flex min-h-full w-full flex-col gap-3 p-3 sm:p-4">
      <Card className="flex flex-1 flex-col">
        <CardHeader>
          <CardTitle>
            Map · navigation
            <span className="tag tag-accent ml-auto">PNL-NAV-01</span>
          </CardTitle>
          <CardDescription>SLAM map · pose · AprilTags · planned path</CardDescription>
        </CardHeader>
        <CardContent className="flex flex-1 items-center justify-center py-12">
          <div className="reticle relative flex max-w-md flex-col items-center gap-3 rounded-sm border border-dashed border-hairline p-8 text-center">
            <span className="reticle-bl" aria-hidden />
            <span className="reticle-br" aria-hidden />
            <MapIcon className="h-10 w-10 opacity-60 text-primary/70" />
            <div className="font-display text-xl text-foreground">Awaiting SLAM</div>
            <div className="tag">channel · idle</div>
            <div className="text-xs text-muted-foreground">
              This pane reserves the Nav2 + slam_toolbox surface. When{' '}
              <span className="font-mono text-primary">/map</span> and{' '}
              <span className="font-mono text-primary">/tf</span> are published, the
              greenhouse occupancy grid and AprilTag detections will render here without
              changes elsewhere in the console.
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
