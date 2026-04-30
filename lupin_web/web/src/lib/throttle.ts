import { useEffect, useRef, useState } from 'react'

/**
 * Latest-wins ref for high-frequency message streams.
 * The ref always holds the most recent value; reads are cheap.
 * Use with `useThrottledRender` to drive React updates at a chosen rate.
 */
export function useLatestRef<T>(initial: T | null = null) {
  const ref = useRef<T | null>(initial)
  return ref
}

/**
 * Re-renders the calling component at the specified Hz. Combine with
 * `useLatestRef` to get throttled access to a high-frequency feed without
 * re-rendering on every message.
 */
export function useThrottledRender(hz = 5) {
  const [, setTick] = useState(0)
  useEffect(() => {
    const intervalMs = Math.max(16, Math.round(1000 / hz))
    const id = setInterval(() => setTick((t) => (t + 1) % 1000000), intervalMs)
    return () => clearInterval(id)
  }, [hz])
}

/**
 * For widgets that draw imperatively (canvas etc.), this hook calls `draw`
 * at requestAnimationFrame cadence as long as the component is mounted.
 * `draw` is given the latest value of the ref.
 */
export function useAnimationLoop<T>(
  ref: React.MutableRefObject<T | null>,
  draw: (value: T | null, now: number) => void,
  enabled = true,
) {
  const drawRef = useRef(draw)
  drawRef.current = draw

  useEffect(() => {
    if (!enabled) return
    let raf = 0
    const tick = (now: number) => {
      drawRef.current(ref.current, now)
      raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [ref, enabled])
}

/**
 * Fixed-size ring buffer for time-series widgets (sparklines, etc).
 * Cheap to push, snapshot returns a copy in chronological order.
 */
export class Ring<T> {
  private buf: T[] = []
  private head = 0
  constructor(public readonly capacity: number) {}

  push(v: T) {
    if (this.buf.length < this.capacity) {
      this.buf.push(v)
    } else {
      this.buf[this.head] = v
      this.head = (this.head + 1) % this.capacity
    }
  }

  snapshot(): T[] {
    if (this.buf.length < this.capacity) return this.buf.slice()
    return this.buf.slice(this.head).concat(this.buf.slice(0, this.head))
  }

  get length() {
    return this.buf.length
  }

  clear() {
    this.buf = []
    this.head = 0
  }
}
