import { GreenhouseStateCard } from '@/components/widgets/GreenhouseStateCard'
import { MapCanvas } from '@/components/widgets/MapCanvas'
import { MissionControls } from '@/components/widgets/MissionControls'
import { pulseTag } from '@/lib/twin-events'

export function MapView() {
  return (
    <div className="flex min-h-full w-full flex-col gap-3 p-3 sm:p-4">
      <MissionControls />
      <div className="flex min-h-[24rem] flex-1">
        <MapCanvas />
      </div>
      <GreenhouseStateCard onSelectTag={pulseTag} />
    </div>
  )
}
