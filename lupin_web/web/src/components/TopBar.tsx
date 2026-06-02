import { BookText, Settings as SettingsIcon } from 'lucide-react'
import { useEffect, useState, type ReactNode } from 'react'

import { BatteryPill } from '@/components/BatteryPill'
import { ClockPill } from '@/components/ClockPill'
import { ConnectionPill } from '@/components/ConnectionPill'
import { EStopButton } from '@/components/EStopButton'
import { MissionStrip } from '@/components/MissionStrip'
import { Button } from '@/components/ui/button'
import { Tooltip, TooltipContent, TooltipTrigger } from '@/components/ui/tooltip'

interface TopBarProps {
  onOpenSettings: () => void
}

/** Public operator handbook on the Vercel docs site. */
const DOCS_URL = 'https://lupin-robot.vercel.app/docs/web-hmi'

export function TopBar({ onOpenSettings }: TopBarProps) {
  return (
    <header className="relative flex h-16 shrink-0 items-stretch border-b border-hairline bg-background/80 backdrop-blur supports-[backdrop-filter]:bg-background/55">
      {/* primary chartreuse rail along the bottom of the chrome */}
      <span
        aria-hidden
        className="pointer-events-none absolute inset-x-0 bottom-0 h-px bg-gradient-to-r from-transparent via-primary/60 to-transparent"
      />

      <div className="flex flex-1 min-w-0 items-stretch gap-3 px-3 sm:gap-4 sm:px-4">
        <Logo />
        <Callsign />
        <MissionStrip />

        <div className="ml-auto flex flex-nowrap items-center gap-1.5 self-center overflow-x-auto sm:gap-2">
          <ConnectionPill />
          <BatteryPill />
          <ClockPill />
          <Tooltip>
            <TooltipTrigger asChild>
              <a
                href={DOCS_URL}
                target="_blank"
                rel="noreferrer noopener"
                aria-label="Documentation"
                className="grid h-9 w-9 shrink-0 place-items-center rounded-sm border border-transparent text-muted-foreground transition-colors hover:border-hairline hover:text-foreground focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring"
              >
                <BookText className="h-[18px] w-[18px]" />
              </a>
            </TooltipTrigger>
            <TooltipContent side="bottom">Docs ↗</TooltipContent>
          </Tooltip>
          <Tooltip>
            <TooltipTrigger asChild>
              <Button
                variant="ghost"
                size="icon"
                onClick={onOpenSettings}
                aria-label="Settings"
                className="h-9 w-9 shrink-0 rounded-sm border border-transparent hover:border-hairline"
              >
                <SettingsIcon className="h-[18px] w-[18px]" />
              </Button>
            </TooltipTrigger>
            <TooltipContent side="bottom">Settings</TooltipContent>
          </Tooltip>
        </div>
      </div>

      <EStopButton className="w-24 shrink-0 sm:w-28" />
    </header>
  )
}

function Logo(): ReactNode {
  return (
    <div className="flex shrink-0 items-center gap-3 self-center">
      <div className="relative grid h-9 w-9 place-items-center overflow-hidden rounded-sm border border-primary/40 bg-primary/10 text-primary">
        {/* small flower-ring monogram drawn in svg, rotates lazily */}
        <svg
          viewBox="0 0 24 24"
          className="absolute inset-0 h-full w-full opacity-50 [animation:spin_18s_linear_infinite]"
          aria-hidden
        >
          <g fill="none" stroke="currentColor" strokeWidth="0.6">
            {Array.from({ length: 12 }).map((_, i) => (
              <line
                key={i}
                x1="12"
                y1="2.5"
                x2="12"
                y2="5"
                transform={`rotate(${i * 30} 12 12)`}
              />
            ))}
          </g>
        </svg>
        <span className="font-display text-[20px] leading-none">L</span>
      </div>

      <div className="hidden flex-col leading-none sm:flex">
        <div className="flex items-baseline gap-2">
          <span className="font-display text-[22px] leading-none text-foreground">
            Lupin
          </span>
          <span className="tag tag-accent translate-y-[-1px]">v0.1</span>
        </div>
        <div className="tag mt-1">Greenhouse&nbsp;·&nbsp;Mission&nbsp;Console</div>
      </div>
    </div>
  )
}

function Callsign() {
  const [uptime, setUptime] = useState('00:00:00')
  useEffect(() => {
    const start = Date.now()
    const fmt = () => {
      const s = Math.floor((Date.now() - start) / 1000)
      const h = Math.floor(s / 3600)
        .toString()
        .padStart(2, '0')
      const m = Math.floor((s % 3600) / 60)
        .toString()
        .padStart(2, '0')
      const ss = (s % 60).toString().padStart(2, '0')
      setUptime(`${h}:${m}:${ss}`)
    }
    fmt()
    const id = setInterval(fmt, 1000)
    return () => clearInterval(id)
  }, [])

  return (
    <div className="hidden h-9 items-center gap-3 self-center rounded-sm border border-hairline bg-card/40 px-3 lg:flex">
      <div className="flex items-baseline gap-1.5">
        <span className="tag">callsign</span>
        <span className="ticker text-[12px] tracking-[0.14em] text-foreground">
          LUPIN-01
        </span>
      </div>
      <span className="h-4 w-px bg-hairline" aria-hidden />
      <div className="flex items-baseline gap-1.5">
        <span className="tag">uptime</span>
        <span className="ticker text-[12px] text-primary">{uptime}</span>
      </div>
    </div>
  )
}
