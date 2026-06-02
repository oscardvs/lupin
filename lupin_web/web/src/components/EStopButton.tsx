import { OctagonAlert } from 'lucide-react'

import { useEStop } from '@/lib/estop'
import { cn } from '@/lib/utils'

interface EStopButtonProps {
  className?: string
}

export function EStopButton({ className }: EStopButtonProps) {
  const { active, trigger } = useEStop()
  const isActive = active

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
        'group relative flex h-full select-none flex-col items-center justify-center gap-1 px-4 transition-colors',
        // Diagonal hazard stripes on both edges; solid red core in the middle.
        'bg-[linear-gradient(135deg,_hsl(8_92%_55%)_0%,_hsl(8_92%_45%)_100%)] text-white',
        'shadow-[inset_0_-3px_0_rgba(0,0,0,0.35),inset_0_1px_0_rgba(255,255,255,0.18)]',
        'hover:brightness-110 active:brightness-90',
        'transition-[filter,box-shadow] duration-300',
        isActive && 'animate-pulse shadow-[0_0_28px_-4px_hsl(8_92%_55%/0.85)]',
        className,
      )}
    >
      {/* hazard chevron edge — left */}
      <span
        aria-hidden
        className="pointer-events-none absolute inset-y-0 left-0 w-2"
        style={{
          backgroundImage:
            'repeating-linear-gradient(135deg, rgba(0,0,0,0.85) 0 6px, transparent 6px 12px)',
        }}
      />
      {/* hazard chevron edge — right */}
      <span
        aria-hidden
        className="pointer-events-none absolute inset-y-0 right-0 w-2"
        style={{
          backgroundImage:
            'repeating-linear-gradient(135deg, rgba(0,0,0,0.85) 0 6px, transparent 6px 12px)',
        }}
      />

      <OctagonAlert className="h-[22px] w-[22px] shrink-0" strokeWidth={2.5} />
      <span className="text-[12px] font-extrabold leading-none tracking-[0.22em]">
        E·STOP
      </span>

      {/* status pip when armed */}
      <span
        aria-hidden
        className={cn(
          'absolute right-2 top-2 h-1.5 w-1.5 rounded-full',
          isActive ? 'bg-white' : 'bg-white/30',
        )}
      />
    </button>
  )
}
