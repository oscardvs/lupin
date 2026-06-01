/**
 * reactive.ts — data-reactive motion helpers.
 *
 * The pipeline is always: smooth FIRST (frame-rate-independent EMA in one rAF
 * loop, written to a ref — never setState at stream rate), animate SECOND.
 * These power the tickers, gauges, glow chrome and the 3D twin so the whole
 * console visibly breathes with live ROS data.
 */
import { useEffect, useRef, useState } from 'react'

import { damp, prefersReducedMotion } from '@/lib/motion'

/**
 * Smooth a noisy target into a ref via EMA, ticking on rAF — zero React
 * re-renders. Consumers read `ref.current` from their own canvas/3D loop.
 * `decay`: ~8–16 snappy, ~2–4 syrupy.
 */
export function useSmoothedRef(target: number, decay = 10) {
  const value = useRef(target)
  const targetRef = useRef(target)
  targetRef.current = target

  useEffect(() => {
    if (prefersReducedMotion()) {
      value.current = targetRef.current
      return
    }
    let raf = 0
    let last = performance.now()
    const tick = () => {
      const now = performance.now()
      const dt = Math.min(0.05, (now - last) / 1000)
      last = now
      value.current = damp(value.current, targetRef.current, decay, dt)
      raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [decay])

  return value
}

/**
 * Returns a `flashing` flag that pulses true for `ms` whenever `value` changes
 * by more than `epsilon`. Direction (`up` / `down` / null) lets callers tint
 * the flash cyan-up / amber-down. Throttle inputs to ~5 Hz before binding.
 */
export function useChangeFlash(value: number, epsilon = 0.001, ms = 600) {
  const prev = useRef(value)
  const [state, setState] = useState<{ on: boolean; dir: 'up' | 'down' | null }>({
    on: false,
    dir: null,
  })

  useEffect(() => {
    const delta = value - prev.current
    if (Math.abs(delta) <= epsilon) return
    const dir = delta > 0 ? 'up' : 'down'
    prev.current = value
    setState({ on: true, dir })
    const id = setTimeout(() => setState((s) => ({ ...s, on: false })), ms)
    return () => clearTimeout(id)
  }, [value, epsilon, ms])

  return state
}

/**
 * Drive a CSS custom property on an element from a numeric signal, smoothed on
 * rAF. Used for the telemetry→glow chrome (`--glow`) and battery hue shift
 * (`--signal-hue`). Returns a ref to attach to the target element.
 */
export function useCssVarSignal(
  varName: string,
  target: number,
  { decay = 8, precision = 3 }: { decay?: number; precision?: number } = {},
) {
  const ref = useRef<HTMLElement>(null)
  const targetRef = useRef(target)
  targetRef.current = target

  useEffect(() => {
    const el = ref.current
    if (!el) return
    if (prefersReducedMotion()) {
      el.style.setProperty(varName, targetRef.current.toFixed(precision))
      return
    }
    let raf = 0
    let cur = targetRef.current
    let last = performance.now()
    const tick = () => {
      const now = performance.now()
      const dt = Math.min(0.05, (now - last) / 1000)
      last = now
      cur = damp(cur, targetRef.current, decay, dt)
      el.style.setProperty(varName, cur.toFixed(precision))
      raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [varName, decay, precision])

  return ref
}
