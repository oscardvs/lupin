import { Settings as SettingsIcon } from 'lucide-react'
import type { ReactNode } from 'react'

import { BatteryPill } from '@/components/BatteryPill'
import { ClockPill } from '@/components/ClockPill'
import { ConnectionPill } from '@/components/ConnectionPill'
import { EStopButton } from '@/components/EStopButton'
import { Button } from '@/components/ui/button'
import { Tooltip, TooltipContent, TooltipTrigger } from '@/components/ui/tooltip'

interface TopBarProps {
  onOpenSettings: () => void
}

export function TopBar({ onOpenSettings }: TopBarProps) {
  return (
    <header className="flex h-14 shrink-0 items-stretch border-b bg-background/95 backdrop-blur supports-[backdrop-filter]:bg-background/60">
      <div className="flex flex-1 min-w-0 items-center gap-2 px-2 sm:px-3">
        <Logo />
        <div className="ml-auto flex flex-nowrap items-center gap-1.5 overflow-x-auto sm:gap-2">
          <ConnectionPill />
          <BatteryPill />
          <ClockPill />
          <Tooltip>
            <TooltipTrigger asChild>
              <Button
                variant="ghost"
                size="icon"
                onClick={onOpenSettings}
                aria-label="Settings"
                className="shrink-0"
              >
                <SettingsIcon className="h-5 w-5" />
              </Button>
            </TooltipTrigger>
            <TooltipContent side="bottom">Settings</TooltipContent>
          </Tooltip>
        </div>
      </div>
      <EStopButton className="w-20 shrink-0 sm:w-24" />
    </header>
  )
}

function Logo(): ReactNode {
  return (
    <div className="flex shrink-0 items-center gap-2">
      <div className="grid h-8 w-8 place-items-center rounded-md bg-primary text-primary-foreground">
        <span className="text-sm font-black tracking-tight">L</span>
      </div>
      <div className="hidden leading-none sm:block">
        <div className="text-sm font-bold tracking-tight">LUPIN</div>
        <div className="text-[10px] uppercase tracking-[0.2em] text-muted-foreground">HMI</div>
      </div>
    </div>
  )
}
