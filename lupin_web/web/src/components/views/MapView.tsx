import { lazy, Suspense } from 'react'

import { GreenhouseStateCard } from '@/components/widgets/GreenhouseStateCard'
import { LightControl } from '@/components/widgets/LightControl'
import { MapCanvas } from '@/components/widgets/MapCanvas'
import { MissionControls } from '@/components/widgets/MissionControls'
import { pulseTag } from '@/lib/twin-events'

// The tulip pulls in three.js + r3f + drei + spring (~900 KB). Lazy-load
// it so the initial Map / Nav bundle stays light — user only pays the
// cost on this tab, not on Teleop / Cameras / Telemetry.
const TulipHealthIndicator = lazy(() =>
  import('@/components/widgets/TulipHealthIndicator')
    .then((m) => ({ default: m.TulipHealthIndicator })),
)

export function MapView() {
  return (
    <div className="flex min-h-full w-full flex-col gap-3 p-3 sm:p-4">
      <div className="flex flex-col gap-3 lg:flex-row lg:items-stretch">
        <MissionControls className="lg:flex-1" />
        <LightControl className="lg:w-[360px]" />
      </div>
      <div className="flex min-h-[24rem] flex-1">
        <MapCanvas />
      </div>
      {/* Bottom row: Greenhouse State table on the left (flex-1), tulip
          ambient indicator pinned to the right at md+. Stack on small
          screens so neither gets squashed. */}
      <div className="flex flex-col gap-3 md:flex-row">
        <GreenhouseStateCard className="flex-1" onSelectTag={pulseTag} />
        <Suspense fallback={
          <div className="flex aspect-square w-full items-center justify-center rounded-md border border-hairline bg-card/40 text-[11px] text-muted-foreground md:w-[260px]">
            loading tulip…
          </div>
        }>
          <TulipHealthIndicator className="w-full md:w-[260px]" />
        </Suspense>
      </div>
    </div>
  )
}
