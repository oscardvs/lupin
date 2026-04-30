import { ArmJointsCard } from '@/components/widgets/ArmJointsCard'
import { BatteryCard } from '@/components/widgets/BatteryCard'
import { ImuCard } from '@/components/widgets/ImuCard'
import { LidarCanvas } from '@/components/widgets/LidarCanvas'
import { OdometryCard } from '@/components/widgets/OdometryCard'
import { SystemCard } from '@/components/widgets/SystemCard'

export function TelemetryView() {
  return (
    <div className="flex w-full flex-col gap-3 p-3 sm:p-4">
      <div className="flex items-baseline gap-2 px-1">
        <span className="font-display text-[20px] leading-none text-foreground">
          Instrumentation
        </span>
        <span className="tag">six channels · live</span>
      </div>
      <div className="grid w-full grid-cols-1 gap-3 sm:grid-cols-2 lg:grid-cols-12">
        <LidarCanvas className="lg:col-span-5 lg:row-span-2" />
        <ImuCard className="lg:col-span-4" />
        <OdometryCard className="lg:col-span-3" />
        <BatteryCard className="lg:col-span-3" />
        <ArmJointsCard className="lg:col-span-2" />
        <SystemCard className="lg:col-span-2" />
      </div>
    </div>
  )
}
