/**
 * RingGauge — an SVG arc that springs to a normalised value.
 *
 * Used for battery, governor, joint angles, mission progress. The arc length
 * is driven by a framer-motion spring (strokeDashoffset is GPU-cheap), with a
 * faint track ring behind. Colour follows the status lanes; under reduced
 * motion the spring is bypassed (instant set).
 */
import { motion, useMotionValue, useSpring, useTransform } from 'framer-motion'
import { useEffect } from 'react'

import { SPRING, useReducedMotion } from '@/lib/motion'
import { cn } from '@/lib/utils'

interface RingGaugeProps {
  /** 0..1 fill. */
  value: number
  size?: number
  thickness?: number
  /** CSS color for the arc. Defaults to chartreuse primary. */
  color?: string
  /** Track (unfilled) color. */
  trackColor?: string
  /** Sweep angle in degrees (default 270 — a ¾ dial). */
  sweep?: number
  /** Content rendered centered inside the ring. */
  children?: React.ReactNode
  className?: string
  glow?: boolean
}

export function RingGauge({
  value,
  size = 96,
  thickness = 6,
  color = 'hsl(var(--primary))',
  trackColor = 'hsl(var(--hairline))',
  sweep = 270,
  children,
  className,
  glow = false,
}: RingGaugeProps) {
  const reduced = useReducedMotion()
  const r = (size - thickness) / 2
  const cx = size / 2
  const cy = size / 2
  const arcLen = (sweep / 360) * (2 * Math.PI * r)
  const gap = 2 * Math.PI * r - arcLen
  const rotation = 90 + (360 - sweep) / 2 // center the gap at the bottom

  const clamped = Math.max(0, Math.min(1, value))
  const mv = useMotionValue(clamped)
  const spring = useSpring(mv, { stiffness: SPRING.gauge.tension, damping: SPRING.gauge.friction })
  const v = reduced ? mv : spring
  const offset = useTransform(v, (t) => arcLen * (1 - t))

  useEffect(() => {
    mv.set(clamped)
  }, [clamped, mv])

  return (
    <div className={cn('relative grid place-items-center', className)} style={{ width: size, height: size }}>
      <svg width={size} height={size} className="rotate-0" style={{ transform: `rotate(${rotation}deg)` }} aria-hidden>
        <circle
          cx={cx}
          cy={cy}
          r={r}
          fill="none"
          stroke={trackColor}
          strokeWidth={thickness}
          strokeDasharray={`${arcLen} ${gap}`}
          strokeLinecap="round"
        />
        <motion.circle
          cx={cx}
          cy={cy}
          r={r}
          fill="none"
          stroke={color}
          strokeWidth={thickness}
          strokeDasharray={`${arcLen} ${gap}`}
          strokeDashoffset={offset}
          strokeLinecap="round"
          style={glow ? { filter: `drop-shadow(0 0 6px ${color})` } : undefined}
        />
      </svg>
      {children ? <div className="absolute inset-0 grid place-items-center">{children}</div> : null}
    </div>
  )
}
