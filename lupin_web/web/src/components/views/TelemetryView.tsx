import { ArmJointsCard } from '@/components/widgets/ArmJointsCard'
import { BatteryCard } from '@/components/widgets/BatteryCard'
import { ImuCard } from '@/components/widgets/ImuCard'
import { LidarCanvas } from '@/components/widgets/LidarCanvas'
import { ObservationsCard } from '@/components/widgets/ObservationsCard'
import { OdometryCard } from '@/components/widgets/OdometryCard'
import { SystemCard } from '@/components/widgets/SystemCard'
import { ViewShell } from '@/components/system/ViewShell'

export function TelemetryView() {
  return (
    <ViewShell intent="fit">
      <div className="flex shrink-0 items-baseline gap-2 px-1">
        <span className="font-display text-[20px] leading-none text-foreground">
          Instrumentation
        </span>
        <span className="tag">live channels &amp; mission readings</span>
      </div>
      {/* On lg the two sensor rows are `auto` (content-height) so each card track
          matches its tallest tile exactly — content can never overflow a 1fr
          track that's shorter than it and bleed over the row below. The
          Observations footer takes the flexible remainder (min 0) and scrolls
          internally. Cards stay h-full so the right rail stretches to the 2-row
          Lidar baseline (no void). Phone goes 2-up to halve the stack height. */}
      <div className="grid min-h-0 w-full flex-1 grid-cols-2 gap-3 sm:grid-cols-2 lg:grid-cols-12 lg:grid-rows-[auto_auto_minmax(0,1fr)]">
        <LidarCanvas className="col-span-2 lg:col-span-5 lg:row-span-2" />
        <ImuCard className="col-span-2 sm:col-span-1 lg:col-span-4 lg:h-full" />
        <OdometryCard className="lg:col-span-3 lg:h-full" />
        <BatteryCard className="lg:col-span-3 lg:h-full" />
        <ArmJointsCard className="lg:col-span-2 lg:h-full" />
        <SystemCard className="lg:col-span-2 lg:h-full" />
        <ObservationsCard className="col-span-2 lg:col-span-12" />
      </div>
    </ViewShell>
  )
}
