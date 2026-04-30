import nipplejs, { type JoystickManager } from 'nipplejs'
import { useEffect, useRef } from 'react'

import { cn } from '@/lib/utils'

export interface StickValue {
  /** -1..1, right-positive */
  x: number
  /** -1..1, up-positive */
  y: number
  active: boolean
}

interface JoystickProps {
  label: string
  /** Hint text under the label, e.g. "linear x/y". */
  hint?: string
  /** Called on every move/end with normalised stick value. */
  onChange: (v: StickValue) => void
  className?: string
  /** Tint color for the rendered nipple. Hex or CSS color. */
  color?: string
}

export function Joystick({ label, hint, onChange, className, color = '#22c55e' }: JoystickProps) {
  const containerRef = useRef<HTMLDivElement | null>(null)
  const managerRef = useRef<JoystickManager | null>(null)
  const onChangeRef = useRef(onChange)
  onChangeRef.current = onChange

  useEffect(() => {
    if (!containerRef.current) return
    const manager = nipplejs.create({
      zone: containerRef.current,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color,
      size: 140,
      restOpacity: 0.7,
      lockX: false,
      lockY: false,
    })
    managerRef.current = manager

    manager.on('move', (_, data) => {
      // nipplejs returns vector with y pointing UP positive when stick is up.
      const { x, y } = data.vector
      onChangeRef.current({ x, y, active: true })
    })
    manager.on('end', () => {
      onChangeRef.current({ x: 0, y: 0, active: false })
    })

    return () => {
      manager.destroy()
      managerRef.current = null
    }
  }, [color])

  return (
    <div className={cn('relative flex flex-col items-center gap-2', className)}>
      <div className="text-xs font-medium uppercase tracking-wider text-muted-foreground">{label}</div>
      <div
        ref={containerRef}
        className="relative h-44 w-44 select-none rounded-full border bg-muted/40 touch-none"
      />
      {hint ? <div className="text-[11px] text-muted-foreground">{hint}</div> : null}
    </div>
  )
}
