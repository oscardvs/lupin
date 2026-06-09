import { lazy, Suspense } from 'react'

import { ViewShell } from '@/components/system/ViewShell'
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
    <ViewShell intent="fit">
      {/* Mission control + status light — proportional split so the light card
          tracks the width instead of a fixed 360px gap. */}
      <div className="grid shrink-0 grid-cols-1 gap-3 lg:grid-cols-[minmax(0,1fr)_minmax(320px,360px)] lg:items-stretch">
        <MissionControls />
        <LightControl />
      </div>

      {/* The map absorbs the remaining height (no fixed floor) so the bottom row
          always stays on-screen. */}
      <div className="flex min-h-[16rem] flex-1 sm:min-h-0">
        <MapCanvas />
      </div>

      {/* Bottom row: Greenhouse State table on the left (flex-1), tulip ambient
          indicator pinned to the right at md+. Stack on small screens. */}
      <div className="flex shrink-0 flex-col gap-3 md:flex-row">
        <GreenhouseStateCard className="flex-1" onSelectTag={pulseTag} />
        <Suspense fallback={
          <div className="flex aspect-square w-full items-center justify-center rounded-sm border border-hairline bg-ink-2 text-[11px] text-muted-foreground md:w-[240px]">
            loading tulip…
          </div>
        }>
          <TulipHealthIndicator className="w-full md:w-[240px]" />
        </Suspense>
      </div>
    </ViewShell>
  )
}
