import { Bot, Camera, Gauge, Map as MapIcon, MessageSquare, Mic, Sliders } from 'lucide-react'
import { useEffect, useState } from 'react'

import { SettingsDrawer } from '@/components/SettingsDrawer'
import { TopBar } from '@/components/TopBar'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { TooltipProvider } from '@/components/ui/tooltip'
import { ArmView } from '@/components/views/ArmView'
import { CamerasView } from '@/components/views/CamerasView'
import { LogsView } from '@/components/views/LogsView'
import { MapView } from '@/components/views/MapView'
import { TeleopView } from '@/components/views/TeleopView'
import { TelemetryView } from '@/components/views/TelemetryView'
import { VoiceView } from '@/components/views/VoiceView'
import { EStopProvider } from '@/lib/estop'
import { onGotoTab } from '@/lib/navigation'
import { RosProvider } from '@/lib/ros'
import { useApplyTheme } from '@/lib/settings'

const TABS = [
  { id: 'teleop', code: '01', label: 'Teleop', Icon: Sliders, View: TeleopView },
  { id: 'arm', code: '02', label: 'Arm', Icon: Bot, View: ArmView },
  { id: 'voice', code: '03', label: 'Voice', Icon: Mic, View: VoiceView },
  { id: 'cameras', code: '04', label: 'Cameras', Icon: Camera, View: CamerasView },
  { id: 'telemetry', code: '05', label: 'Telemetry', Icon: Gauge, View: TelemetryView },
  { id: 'logs', code: '06', label: 'Logs', Icon: MessageSquare, View: LogsView },
  { id: 'map', code: '07', label: 'Map / Nav', Icon: MapIcon, View: MapView },
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

function initialTab(): (typeof TABS)[number]['id'] {
  if (typeof window === 'undefined') return 'teleop'
  const t = new URLSearchParams(window.location.search).get('tab')
  if (t && TABS.some((tab) => tab.id === t)) return t as (typeof TABS)[number]['id']
  return 'teleop'
}

function Shell() {
  useApplyTheme()
  const [settingsOpen, setSettingsOpen] = useState(false)
  const [tab, setTab] = useState<(typeof TABS)[number]['id']>(initialTab)
  const active = TABS.find((t) => t.id === tab) ?? TABS[0]

  // Cross-cut navigation requests (e.g. Take Control on the topbar mission
  // strip routes the operator to Teleop in a single click).
  useEffect(() => onGotoTab((next) => {
    if (TABS.some((t) => t.id === next)) setTab(next as typeof tab)
  }), [])

  return (
    <div className="flex h-full min-h-0 flex-col bg-background">
      <TopBar onOpenSettings={() => setSettingsOpen(true)} />

      <Tabs
        value={tab}
        onValueChange={(v) => setTab(v as typeof tab)}
        className="flex flex-1 min-h-0 flex-col"
      >
        <div className="relative flex shrink-0 items-center gap-3 border-b border-hairline bg-background/40 px-3 py-2 sm:px-4">
          <TabsList className="flex h-9 w-fit max-w-full overflow-x-auto">
            {TABS.map(({ id, label, code, Icon }) => (
              <TabsTrigger
                key={id}
                value={id}
                className="gap-2 px-2.5 sm:px-3"
                aria-label={label}
              >
                <span className="tag tag-accent hidden font-semibold opacity-80 sm:inline">
                  {code}
                </span>
                <Icon className="h-[14px] w-[14px] shrink-0" />
                <span className="hidden sm:inline">{label}</span>
              </TabsTrigger>
            ))}
          </TabsList>

          {/* Right-aligned section breadcrumb */}
          <div className="ml-auto hidden items-baseline gap-2 md:flex">
            <span className="tag">section</span>
            <span className="font-display text-[18px] leading-none text-foreground">
              {active.label}
            </span>
            <span className="tag tag-accent">·{active.code}</span>
          </div>
        </div>

        {TABS.map(({ id, View }) => (
          <TabsContent
            key={id}
            value={id}
            className="m-0 flex-1 min-h-0 overflow-y-auto"
          >
            <View />
          </TabsContent>
        ))}
      </Tabs>

      <SettingsDrawer open={settingsOpen} onOpenChange={setSettingsOpen} />
    </div>
  )
}
