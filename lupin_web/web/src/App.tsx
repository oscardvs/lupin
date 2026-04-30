import { Camera, Gauge, Map as MapIcon, MessageSquare, Sliders } from 'lucide-react'
import { useState } from 'react'

import { SettingsDrawer } from '@/components/SettingsDrawer'
import { TopBar } from '@/components/TopBar'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { TooltipProvider } from '@/components/ui/tooltip'
import { CamerasView } from '@/components/views/CamerasView'
import { LogsView } from '@/components/views/LogsView'
import { MapView } from '@/components/views/MapView'
import { TeleopView } from '@/components/views/TeleopView'
import { TelemetryView } from '@/components/views/TelemetryView'
import { EStopProvider } from '@/lib/estop'
import { RosProvider } from '@/lib/ros'
import { useApplyTheme } from '@/lib/settings'

const TABS = [
  { id: 'teleop', label: 'Teleop', Icon: Sliders, View: TeleopView },
  { id: 'cameras', label: 'Cameras', Icon: Camera, View: CamerasView },
  { id: 'telemetry', label: 'Telemetry', Icon: Gauge, View: TelemetryView },
  { id: 'logs', label: 'Logs', Icon: MessageSquare, View: LogsView },
  { id: 'map', label: 'Map', Icon: MapIcon, View: MapView },
] as const

export default function App() {
  return (
    <TooltipProvider delayDuration={200}>
      <RosProvider>
        <EStopProvider>
          <Shell />
        </EStopProvider>
      </RosProvider>
    </TooltipProvider>
  )
}

function Shell() {
  useApplyTheme()
  const [settingsOpen, setSettingsOpen] = useState(false)
  const [tab, setTab] = useState<(typeof TABS)[number]['id']>('teleop')

  return (
    <div className="flex h-full min-h-0 flex-col bg-background">
      <TopBar onOpenSettings={() => setSettingsOpen(true)} />

      <Tabs
        value={tab}
        onValueChange={(v) => setTab(v as typeof tab)}
        className="flex flex-1 min-h-0 flex-col"
      >
        <div className="border-b bg-background/80">
          <TabsList className="m-2 flex h-11 w-fit max-w-full gap-0.5 overflow-x-auto px-1 sm:mx-3">
            {TABS.map(({ id, label, Icon }) => (
              <TabsTrigger
                key={id}
                value={id}
                className="gap-2 px-2.5 sm:px-3"
                aria-label={label}
              >
                <Icon className="h-4 w-4 shrink-0" />
                <span className="hidden sm:inline">{label}</span>
              </TabsTrigger>
            ))}
          </TabsList>
        </div>
        {TABS.map(({ id, View }) => (
          <TabsContent key={id} value={id} className="m-0 flex flex-1 min-h-0 overflow-auto">
            <div className="flex flex-1 min-h-0">
              <View />
            </div>
          </TabsContent>
        ))}
      </Tabs>

      <SettingsDrawer open={settingsOpen} onOpenChange={setSettingsOpen} />
    </div>
  )
}
