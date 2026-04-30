import { MapCanvas } from '@/components/widgets/MapCanvas'

export function MapView() {
  return (
    <div className="flex min-h-full w-full flex-col gap-3 p-3 sm:p-4">
      <MapCanvas />
    </div>
  )
}
