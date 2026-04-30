import { OctagonAlert } from 'lucide-react'

import { useEStop } from '@/lib/estop'
import { cn } from '@/lib/utils'

interface EStopButtonProps {
  className?: string
}

export function EStopButton({ className }: EStopButtonProps) {
  const { active, trigger } = useEStop()

  return (
    <button
      type="button"
      onPointerDown={() => trigger('user')}
      onKeyDown={(e) => {
        if (e.key === ' ' || e.key === 'Enter') {
          e.preventDefault()
          trigger('user')
        }
      }}
      aria-label="Emergency stop"
      className={cn(
        'group relative flex h-full select-none flex-col items-center justify-center gap-0.5 px-3 transition-colors sm:px-4',
        'bg-red-600 text-white shadow-[inset_0_-3px_0_rgba(0,0,0,0.25)]',
        'hover:bg-red-500 active:bg-red-700',
        active && 'animate-pulse',
        className,
      )}
    >
      <OctagonAlert className="h-5 w-5 shrink-0 sm:h-6 sm:w-6" strokeWidth={2.5} />
      <span className="text-sm font-extrabold leading-none tracking-widest sm:text-base">STOP</span>
    </button>
  )
}
