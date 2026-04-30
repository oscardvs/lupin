import { ArmJointsCard } from '@/components/widgets/ArmJointsCard'
import { BatteryCard } from '@/components/widgets/BatteryCard'
import { ImuCard } from '@/components/widgets/ImuCard'
import { LidarCanvas } from '@/components/widgets/LidarCanvas'
import { OdometryCard } from '@/components/widgets/OdometryCard'
import { SystemCard } from '@/components/widgets/SystemCard'

export function TelemetryView() {
  return (
    <div className="grid w-full grid-cols-1 gap-3 p-3 sm:grid-cols-2 sm:p-4 lg:grid-cols-3">
      <LidarCanvas />
      <ImuCard />
      <OdometryCard />
      <BatteryCard />
      <ArmJointsCard />
      <SystemCard />
    </div>
  )
}
