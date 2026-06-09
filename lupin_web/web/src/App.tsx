import { MotionConfig, motion } from 'framer-motion'
import { Bot, Camera, Gauge, Map as MapIcon, MessageSquare, Mic, Sliders } from 'lucide-react'
import { useEffect, useState } from 'react'

import { SettingsDrawer } from '@/components/SettingsDrawer'
import { TopBar } from '@/components/TopBar'
import { AuroraBackground, type AuroraTone } from '@/components/system/AuroraBackground'
import { BootSequence } from '@/components/system/BootSequence'
import { FocusPanelProvider } from '@/components/system/FocusPanel'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { TooltipProvider } from '@/components/ui/tooltip'
import { ArmView } from '@/components/views/ArmView'
import { CamerasView } from '@/components/views/CamerasView'
import { LogsView } from '@/components/views/LogsView'
import { MapView } from '@/components/views/MapView'
import { TeleopView } from '@/components/views/TeleopView'
import { TelemetryView } from '@/components/views/TelemetryView'
import { VoiceView } from '@/components/views/VoiceView'
import { EStopProvider, useEStop } from '@/lib/estop'
import { isMissionActive, useMissionState } from '@/lib/mission'
import { transition, viewSwap } from '@/lib/motion'
import { onGotoTab } from '@/lib/navigation'
import { RosProvider, useRos } from '@/lib/ros'
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
    <MotionConfig reducedMotion="user">
      <TooltipProvider delayDuration={200}>
        <RosProvider>
          <EStopProvider>
            <FocusPanelProvider>
              <Shell />
            </FocusPanelProvider>
          </EStopProvider>
        </RosProvider>
      </TooltipProvider>
    </MotionConfig>
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
  const { active: estopActive } = useEStop()
  const { status } = useRos()
  const missionState = useMissionState()
  const [settingsOpen, setSettingsOpen] = useState(false)
  const [tab, setTab] = useState<(typeof TABS)[number]['id']>(initialTab)
  const active = TABS.find((t) => t.id === tab) ?? TABS[0]

  // The aurora reflects the robot's mood: red on e-stop, brighter while a
  // mission runs or the link is live, calm otherwise.
  const tone: AuroraTone = estopActive
    ? 'alert'
    : isMissionActive(missionState) || status === 'connected'
      ? 'active'
      : 'idle'

  // Cross-cut navigation requests (e.g. Take Control on the topbar mission
  // strip routes the operator to Teleop in a single click).
  useEffect(() => onGotoTab((next) => {
    if (TABS.some((t) => t.id === next)) setTab(next as typeof tab)
  }), [])

  return (
    <div className="relative flex h-full min-h-0 flex-col">
      <AuroraBackground tone={tone} />

      <div className="relative z-10 flex h-full min-h-0 flex-col">
        <TopBar onOpenSettings={() => setSettingsOpen(true)} />

        <Tabs
          value={tab}
          onValueChange={(v) => setTab(v as typeof tab)}
          className="flex flex-1 min-h-0 flex-col"
        >
          <div className="relative flex shrink-0 items-center gap-3 border-b border-hairline bg-background/30 px-3 py-2 backdrop-blur-sm sm:px-4">
            <TabsList className="flex h-9 w-fit max-w-full overflow-x-auto bg-card/30">
              {TABS.map(({ id, label, code, Icon }) => (
                <TabsTrigger
                  key={id}
                  value={id}
                  className="group gap-2 px-2.5 sm:px-3"
                  aria-label={label}
                >
                  {tab === id ? (
                    <motion.span
                      layoutId="tab-underline"
                      className="absolute inset-x-1 -bottom-px h-0.5 rounded-full bg-primary shadow-[0_0_8px_hsl(var(--primary)/0.7)]"
                      transition={transition.snappy}
                    />
                  ) : null}
                  <span className="tag tag-accent hidden font-semibold opacity-80 sm:inline">
                    {code}
                  </span>
                  <Icon className="h-[14px] w-[14px] shrink-0" />
                  {/* Keep the ACTIVE tab worded even on phone (others stay icon-only)
                      so there's always one text anchor in the strip. */}
                  <span className="hidden group-data-[state=active]:inline sm:inline">{label}</span>
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
              <motion.div variants={viewSwap} initial="hidden" animate="show" className="h-full min-h-0">
                <View />
              </motion.div>
            </TabsContent>
          ))}
        </Tabs>

        <SettingsDrawer open={settingsOpen} onOpenChange={setSettingsOpen} />
      </div>

      <BootSequence />
    </div>
  )
}
