import { Map as MapIcon } from 'lucide-react'

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'

export function MapView() {
  return (
    <div className="flex h-full min-h-0 w-full flex-col gap-3 p-3 sm:p-4">
      <Card className="flex flex-1 min-h-0 flex-col">
        <CardHeader className="pb-2">
          <CardTitle>Map / navigation</CardTitle>
          <CardDescription>SLAM map, robot pose, AprilTag detections, planned path</CardDescription>
        </CardHeader>
        <CardContent className="flex flex-1 items-center justify-center">
          <div className="flex max-w-md flex-col items-center gap-3 rounded-md border border-dashed p-8 text-center text-muted-foreground">
            <MapIcon className="h-10 w-10 opacity-60" />
            <div className="text-sm">Map will appear when SLAM is running.</div>
            <div className="text-xs">
              This pane is a placeholder for the upcoming Nav2 + slam_toolbox integration.
              When <span className="font-mono">/map</span> and <span className="font-mono">/tf</span> are
              published, they'll render here without changes elsewhere in the app.
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}
