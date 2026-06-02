/**
 * Ticker — a spring-eased numeric readout.
 *
 * Live ROS scalars (battery %, speed, voltage, distance-to-goal) roll smoothly
 * to new values instead of snapping. Rendered in JetBrains Mono `tabular-nums`
 * inside a fixed-width slot so digits never reflow, with the unit in its own
 * fixed span. On a meaningful change it fires a one-shot direction-aware flash.
 *
 * Discipline (from the design spec):
 *   • Throttle inputs to ≤5 Hz before binding — never spring a high-rate stream.
 *   • Never smooth a SAFETY value (raw distance to obstacle) — pass `instant`.
 *   • Reduced motion → snap to value but still flash colour so change reads.
 */
import { motion, useMotionValue, useSpring, useTransform } from 'framer-motion'
import { useEffect } from 'react'

import { useChangeFlash } from '@/lib/reactive'
import { SPRING } from '@/lib/motion'
import { useReducedMotion } from '@/lib/motion'
import { cn } from '@/lib/utils'

interface TickerProps {
  value: number
  /** Decimal places. */
  decimals?: number
  /** Unit rendered in a fixed slot to the right (e.g. "%", "m/s", "V"). */
  unit?: string
  /** Longest string to reserve width for, e.g. "100" or "-0.00". Auto if unset. */
  reserve?: string
  /** Skip the spring (safety-relevant values that must show raw). */
  instant?: boolean
  /** Disable the change flash. */
  noFlash?: boolean
  className?: string
  unitClassName?: string
  'aria-label'?: string
}

export function Ticker({
  value,
  decimals = 2,
  unit,
  reserve,
  instant = false,
  noFlash = false,
  className,
  unitClassName,
  ...rest
}: TickerProps) {
  const reduced = useReducedMotion()
  const useSpringPath = !instant && !reduced
  const flash = useChangeFlash(value)

  const mv = useMotionValue(value)
  // tension/friction → framer stiffness/damping (close enough for a readout).
  const spring = useSpring(mv, { stiffness: SPRING.telemetry.tension, damping: SPRING.telemetry.friction })
  const text = useTransform(spring, (v) => format(v, decimals))

  useEffect(() => {
    mv.set(value)
  }, [value, mv])

  const flashColor =
    noFlash || !flash.on
      ? undefined
      : flash.dir === 'down'
        ? 'hsl(var(--status-caution))'
        : 'hsl(var(--status-data))'

  const slot = reserve ?? format(value, decimals)

  return (
    <span
      className={cn('ticker inline-flex items-baseline tabular-nums', className)}
      style={{ color: flashColor, transition: 'color 600ms var(--ease-out-expo)' }}
      aria-live="polite"
      {...rest}
    >
      <span className="relative inline-grid justify-items-end">
        {/* invisible width reservation so the value never reflows its row */}
        <span aria-hidden className="invisible col-start-1 row-start-1">
          {slot}
        </span>
        {useSpringPath ? (
          <motion.span className="col-start-1 row-start-1">{text}</motion.span>
        ) : (
          <span className="col-start-1 row-start-1">{format(value, decimals)}</span>
        )}
      </span>
      {unit ? (
        <span className={cn('tag ml-1 translate-y-[-1px]', unitClassName)}>{unit}</span>
      ) : null}
    </span>
  )
}

function format(v: number, decimals: number): string {
  if (Number.isNaN(v)) return '—'
  return v.toFixed(decimals)
}
