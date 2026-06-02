/**
 * motion.ts — the Lupin HMI motion vocabulary.
 *
 * One place for every easing curve, duration, and reusable framer-motion
 * variant so the whole console moves with one "hand". Curves are chosen to feel
 * mechanical-but-expensive: fast attack, long expo tail (the deceleration that
 * reads as precision hardware rather than a bouncy web toy).
 *
 * Reduced motion: the app is wrapped in <MotionConfig reducedMotion="user">, so
 * framer-motion automatically strips transform/layout animation for users who
 * ask for it and keeps only opacity. For our own CSS / canvas loops we also
 * expose `prefersReducedMotion()` and the `useReducedMotion` hook.
 */
import type { Transition, Variants } from 'framer-motion'

export { useReducedMotion } from 'framer-motion'

/** Signature easing curves (cubic-bezier control points). */
export const ease = {
  /** Long expo tail — the house deceleration. Use for entrances + most moves. */
  outExpo: [0.16, 1, 0.3, 1] as const,
  /** Slightly softer expo — content reveals. */
  outQuint: [0.22, 1, 0.36, 1] as const,
  /** Symmetric, snappy in-out for toggles + crossfades. */
  inOutCirc: [0.85, 0, 0.15, 1] as const,
  /** Gentle standard ease for hover/press color changes. */
  standard: [0.4, 0, 0.2, 1] as const,
}

/** Named durations (seconds — framer-motion units). */
export const dur = {
  /** 0.14s — micro feedback (press, tick flash). */
  xs: 0.14,
  /** 0.24s — hovers, small reveals. */
  sm: 0.24,
  /** 0.42s — panel entrances. */
  md: 0.42,
  /** 0.72s — hero / boot beats. */
  lg: 0.72,
}

/** Reusable transitions. */
export const transition = {
  snappy: { duration: dur.sm, ease: ease.outExpo } satisfies Transition,
  smooth: { duration: dur.md, ease: ease.outExpo } satisfies Transition,
  slow: { duration: dur.lg, ease: ease.outQuint } satisfies Transition,
  /** Crisp UI spring — value rolls, gauges, knobs settling. */
  spring: { type: 'spring', stiffness: 320, damping: 30, mass: 0.8 } satisfies Transition,
  /** Soft spring — larger elements, layout shifts. */
  springSoft: { type: 'spring', stiffness: 170, damping: 26 } satisfies Transition,
} as const

/** Fade + rise. The default panel/content entrance. */
export const fadeUp: Variants = {
  hidden: { opacity: 0, y: 14 },
  show: { opacity: 1, y: 0, transition: transition.smooth },
}

/** Pure fade. */
export const fadeIn: Variants = {
  hidden: { opacity: 0 },
  show: { opacity: 1, transition: transition.smooth },
}

/** Scale up from 96% — for cards / dialogs / the orb. */
export const scaleIn: Variants = {
  hidden: { opacity: 0, scale: 0.96 },
  show: { opacity: 1, scale: 1, transition: transition.smooth },
}

/** Reveal with a brief blur — premium "developing" feel for hero surfaces. */
export const blurIn: Variants = {
  hidden: { opacity: 0, y: 10, filter: 'blur(8px)' },
  show: { opacity: 1, y: 0, filter: 'blur(0px)', transition: transition.slow },
}

/** Container that staggers its children in. Pair with `staggerItem`. */
export const staggerContainer = (stagger = 0.06, delayChildren = 0.02): Variants => ({
  hidden: {},
  show: {
    transition: { staggerChildren: stagger, delayChildren },
  },
})

/** Child of a `staggerContainer`. */
export const staggerItem: Variants = {
  hidden: { opacity: 0, y: 12 },
  show: { opacity: 1, y: 0, transition: transition.smooth },
}

/** View/tab swap — content slides up + fades, exits down. */
export const viewSwap: Variants = {
  hidden: { opacity: 0, y: 10 },
  show: { opacity: 1, y: 0, transition: { duration: dur.md, ease: ease.outExpo } },
  exit: { opacity: 0, y: -6, transition: { duration: dur.xs, ease: ease.standard } },
}

/** True only when the user has requested reduced motion (SSR-safe). */
export function prefersReducedMotion(): boolean {
  if (typeof window === 'undefined' || !window.matchMedia) return false
  return window.matchMedia('(prefers-reduced-motion: reduce)').matches
}

/* ── CSS / @react-spring/three vocabulary ───────────────────────────────── */
/* Mirrors the framer-motion easings above as cubic-bezier strings + the
   spring presets used by the 3D twin and telemetry smoothing. Rule of thumb:
   exits faster than enters; never spring a >5 Hz stream (it never settles). */

/** Named cubic-bezier easings for CSS transitions / WAAPI. */
export const EASE = {
  reveal: 'cubic-bezier(0.16, 1, 0.3, 1)', // expo-out — the house deceleration
  standard: 'cubic-bezier(0.4, 0, 0.2, 1)',
  exit: 'cubic-bezier(0.4, 0, 1, 1)',
  micro: 'cubic-bezier(0.2, 0.8, 0.2, 1)',
} as const

/** Named durations (ms) for CSS / WAAPI. */
export const DUR = { fast: 150, base: 250, slow: 450, reveal: 550 } as const

/** @react-spring/three + telemetry spring presets. */
export const SPRING = {
  telemetry: { tension: 180, friction: 26 }, // settles ~400ms, no wobble
  gauge: { tension: 120, friction: 22 }, // heavier, mass feel
  snappy: { tension: 400, friction: 30 }, // controls / 3D entry
} as const

/**
 * Frame-rate-independent exponential smoothing (EMA). Call once per rAF tick
 * with the elapsed `dt` (seconds). `decay` ~8–16 = snappy, ~2–4 = syrupy.
 * The backbone of every data-reactive primitive — smooth FIRST, animate SECOND.
 */
export function damp(current: number, target: number, decay: number, dt: number): number {
  return target + (current - target) * Math.exp(-decay * dt)
}
