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
  /** Outer container size (px). Nipple is sized at ~80% of this. */
  size?: number
  className?: string
  /** Tint color for the rendered nipple. Hex or CSS color. */
  color?: string
  /** Short tick labels around the dial: [up, right, down, left]. Defaults to compass-like X/Y axis tags. */
  axisTags?: [string, string, string, string]
  /** Optional small index code shown in the corner, e.g. "STK-01". */
  serial?: string
}

export function Joystick({
  label,
  hint,
  onChange,
  size = 176,
  className,
  color,
  axisTags = ['+Y', '+X', '−Y', '−X'],
  serial,
}: JoystickProps) {
  const containerRef = useRef<HTMLDivElement | null>(null)
  const managerRef = useRef<JoystickManager | null>(null)
  const onChangeRef = useRef(onChange)
  onChangeRef.current = onChange

  useEffect(() => {
    if (!containerRef.current) return
    // resolve the chartreuse primary at runtime so nipple matches the theme
    const tint =
      color ??
      (() => {
        const v = getComputedStyle(document.documentElement).getPropertyValue('--primary').trim()
        return v ? `hsl(${v})` : '#aef359'
      })()

    const nippleSize = Math.max(80, Math.round(size * 0.78))
    const manager = nipplejs.create({
      zone: containerRef.current,
      mode: 'static',
      position: { left: '50%', top: '50%' },
      color: tint,
      size: nippleSize,
      restOpacity: 0.6,
      lockX: false,
      lockY: false,
    })
    managerRef.current = manager

    manager.on('move', (_, data) => {
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
  }, [color, size])

  return (
    <div className={cn('reticle relative flex flex-col items-center gap-3 rounded-sm border border-hairline bg-card/40 p-4', className)}>
      <span className="reticle-bl" aria-hidden />
      <span className="reticle-br" aria-hidden />

      <div className="flex w-full items-center justify-between gap-3">
        <span className="tag tag-strong">{label}</span>
        {serial ? <span className="tag">{serial}</span> : null}
      </div>

      <div
        className="relative grid shrink-0 place-items-center"
        style={{ width: `${size}px`, height: `${size}px`, flex: '0 0 auto' }}
      >
        {/* concentric range rings */}
        <svg
          className="pointer-events-none absolute inset-0 h-full w-full text-primary/35"
          viewBox="0 0 100 100"
          aria-hidden
        >
          <circle cx="50" cy="50" r="49" fill="none" stroke="currentColor" strokeWidth="0.4" />
          <circle cx="50" cy="50" r="33" fill="none" stroke="currentColor" strokeWidth="0.3" strokeDasharray="0.6 1.2" />
          <circle cx="50" cy="50" r="16" fill="none" stroke="currentColor" strokeWidth="0.3" strokeDasharray="0.6 1.2" />
          {/* axis cross */}
          <line x1="2" y1="50" x2="98" y2="50" stroke="currentColor" strokeWidth="0.25" />
          <line x1="50" y1="2" x2="50" y2="98" stroke="currentColor" strokeWidth="0.25" />
          {/* tiny tick marks at 30° increments */}
          {Array.from({ length: 12 }).map((_, i) => {
            const a = (i * 30 * Math.PI) / 180
            const x1 = 50 + Math.cos(a) * 47
            const y1 = 50 + Math.sin(a) * 47
            const x2 = 50 + Math.cos(a) * 49
            const y2 = 50 + Math.sin(a) * 49
            return (
              <line
                key={i}
                x1={x1}
                y1={y1}
                x2={x2}
                y2={y2}
                stroke="currentColor"
                strokeWidth="0.5"
              />
            )
          })}
        </svg>

        {/* axis tick labels — pointer-events-none so taps reach the nipplejs zone */}
        <span className="tag pointer-events-none absolute -top-0.5 left-1/2 -translate-x-1/2 -translate-y-full pb-1">
          {axisTags[0]}
        </span>
        <span className="tag pointer-events-none absolute right-0 top-1/2 -translate-y-1/2 translate-x-full pl-1">
          {axisTags[1]}
        </span>
        <span className="tag pointer-events-none absolute -bottom-0.5 left-1/2 -translate-x-1/2 translate-y-full pt-1">
          {axisTags[2]}
        </span>
        <span className="tag pointer-events-none absolute left-0 top-1/2 -translate-y-1/2 -translate-x-full pr-1">
          {axisTags[3]}
        </span>

        {/* nipplejs target */}
        <div
          ref={containerRef}
          style={{ width: size, height: size }}
          className="relative select-none rounded-full touch-none"
        />
      </div>

      {hint ? <div className="tag">{hint}</div> : null}
    </div>
  )
}
