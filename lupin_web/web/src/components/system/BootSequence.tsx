/**
 * BootSequence — the "system coming alive" moment on first load.
 *
 * A full-screen power-on overlay whose handshake log advances off REAL rosbridge
 * milestones (link → handshake → telemetry → live), so a slow link genuinely
 * reads slower. Honest percent = cleared / total; never a fake 99% hang.
 *
 * Delight, not annoyance:
 *   • Cinematic ONCE per session (sessionStorage, flag set on START so a mid-boot
 *     refresh doesn't replay). `?boot=1` forces a replay for demos.
 *   • Always skippable (click / any key / a focusable SKIP) and a 4.5s failsafe
 *     watchdog that dismisses to whatever state ROS actually reached — E-Stop is
 *     reachable the instant boot clears.
 *   • Reconnects do NOT replay the movie (handled by the ConnectionPill chip).
 *   • Reduced motion → all lines at once, counter snaps, ~400ms auto-dismiss.
 */
import { AnimatePresence, motion } from 'framer-motion'
import { useCallback, useEffect, useState } from 'react'

import { ease, useReducedMotion } from '@/lib/motion'
import { useRos, useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { ROS_TYPE, type BatteryState } from '@/types/ros'

const STEPS = [
  { code: 'LINK', label: 'Establishing rosbridge link' },
  { code: 'HSHK', label: 'Handshake · protocol negotiated' },
  { code: 'TLM', label: 'Telemetry stream acquired' },
  { code: 'LIVE', label: 'All systems nominal' },
] as const

const SEEN_KEY = 'lupin.boot.seen'
const WATCHDOG_MS = 4500

function shouldRun(): boolean {
  if (typeof window === 'undefined') return false
  const params = new URLSearchParams(window.location.search)
  if (params.has('boot')) return true // forced replay for demos
  try {
    if (sessionStorage.getItem(SEEN_KEY)) return false
    sessionStorage.setItem(SEEN_KEY, '1') // set on START, not end
  } catch {
    /* sessionStorage disabled — run once in-memory is fine */
  }
  return true
}

export function BootSequence() {
  const reduced = useReducedMotion()
  const [{ batteryTopic }] = useSettings()
  const { status } = useRos()
  const [visible, setVisible] = useState(shouldRun)
  const [gotTelemetry, setGotTelemetry] = useState(false)
  const [forced, setForced] = useState(false) // watchdog / skip → jump to LIVE
  const [elapsed, setElapsed] = useState(0)

  useTopic<BatteryState>(batteryTopic, ROS_TYPE.BatteryState, {
    onMessage: () => setGotTelemetry(true),
  })

  // Milestone clearing — real signals with timed fallbacks so it never hangs.
  const cleared = [
    forced || elapsed > 150,
    forced || status === 'connected' || elapsed > 700,
    forced || gotTelemetry || elapsed > 1400,
    forced || (gotTelemetry && status === 'connected'),
  ]
  const clearedCount = cleared.filter(Boolean).length
  const pct = Math.round((clearedCount / STEPS.length) * 100)
  const done = clearedCount >= STEPS.length

  const dismiss = useCallback(() => setVisible(false), [])

  // Tick elapsed so the timed fallbacks advance.
  useEffect(() => {
    if (!visible || reduced) return
    const t0 = performance.now()
    let raf = 0
    const tick = () => {
      setElapsed(performance.now() - t0)
      raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(raf)
  }, [visible, reduced])

  // Watchdog — never trap the operator behind a wedged link.
  useEffect(() => {
    if (!visible) return
    const id = setTimeout(() => setForced(true), reduced ? 250 : WATCHDOG_MS)
    return () => clearTimeout(id)
  }, [visible, reduced])

  // Once everything is cleared, hold a beat then reveal the console.
  useEffect(() => {
    if (!visible || !done) return
    const id = setTimeout(dismiss, reduced ? 200 : 600)
    return () => clearTimeout(id)
  }, [visible, done, reduced, dismiss])

  // Global skip affordances.
  useEffect(() => {
    if (!visible) return
    const onKey = () => setForced(true)
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [visible])

  return (
    <AnimatePresence>
      {visible ? (
        <motion.div
          key="boot"
          className="fixed inset-0 z-[100] grid place-items-center overflow-hidden"
          style={{ background: 'radial-gradient(120% 100% at 50% 35%, hsl(var(--ink-2)), hsl(var(--ink-0)) 75%)' }}
          initial={{ opacity: 1 }}
          exit={{ opacity: 0, filter: 'blur(6px)' }}
          transition={{ duration: 0.5, ease: ease.outExpo }}
          onClick={() => setForced(true)}
          role="status"
          aria-label="System starting"
        >
          {/* drifting grid + bloom */}
          <div
            className="pointer-events-none absolute inset-0 bg-grid opacity-40"
            style={{
              maskImage: 'radial-gradient(ellipse 70% 60% at 50% 45%, #000 20%, transparent 80%)',
              WebkitMaskImage: 'radial-gradient(ellipse 70% 60% at 50% 45%, #000 20%, transparent 80%)',
            }}
          />

          <div className="reticle relative mx-4 w-full max-w-md px-6 py-8 sm:px-10">
            <span className="reticle-bl" aria-hidden />
            <span className="reticle-br" aria-hidden />

            {/* monogram + wordmark */}
            <motion.div
              className="mb-6 flex items-center gap-4"
              initial={reduced ? false : { opacity: 0, y: 8 }}
              animate={{ opacity: 1, y: 0 }}
              transition={{ duration: 0.5, ease: ease.outExpo }}
            >
              <div className="relative grid h-12 w-12 place-items-center overflow-hidden rounded-sm border border-primary/40 bg-primary/10 text-primary">
                <svg viewBox="0 0 24 24" className="absolute inset-0 h-full w-full opacity-60 spin-slow" aria-hidden>
                  <g fill="none" stroke="currentColor" strokeWidth="0.6">
                    {Array.from({ length: 12 }).map((_, i) => (
                      <line key={i} x1="12" y1="2.5" x2="12" y2="5" transform={`rotate(${i * 30} 12 12)`} />
                    ))}
                  </g>
                </svg>
                <span className="font-display text-[26px] leading-none">L</span>
              </div>
              <div className="leading-none">
                <div className="font-display text-[30px] leading-none text-foreground">Lupin</div>
                <div className="tag mt-1.5">Greenhouse · Mission Console</div>
              </div>
            </motion.div>

            {/* handshake log */}
            <div className="space-y-2">
              {STEPS.map((step, i) => {
                const active = cleared[i]
                const pending = !active && (i === 0 || cleared[i - 1])
                return (
                  <motion.div
                    key={step.code}
                    className="flex items-center gap-3"
                    initial={reduced ? false : { opacity: 0, x: -8 }}
                    animate={{ opacity: active || pending ? 1 : 0.25, x: 0 }}
                    transition={{ duration: 0.3, ease: ease.outExpo }}
                  >
                    <span
                      className={`inline-block h-1.5 w-1.5 shrink-0 rounded-full ${
                        active ? 'bg-primary' : pending ? 'bg-warning animate-pulse' : 'bg-muted-foreground/40'
                      }`}
                    />
                    <span className="tag w-12 shrink-0 tag-accent">{step.code}</span>
                    <span className={`font-mono text-[12px] ${active ? 'text-foreground' : 'text-muted-foreground'}`}>
                      {step.label}
                    </span>
                    {active ? <span className="ml-auto tag text-primary">ok</span> : null}
                  </motion.div>
                )
              })}
            </div>

            {/* counter + progress rail */}
            <div className="mt-7 flex items-end justify-between">
              <div className="font-display text-[44px] leading-none text-foreground tabular-nums">
                {pct}
                <span className="tag tag-accent ml-1 align-top">%</span>
              </div>
              <button
                type="button"
                onClick={(e) => {
                  e.stopPropagation()
                  setForced(true)
                }}
                className="tag rounded-sm border border-hairline px-2 py-1 text-muted-foreground transition-colors hover:border-primary/50 hover:text-foreground focus-visible:outline-none focus-visible:ring-1 focus-visible:ring-ring"
              >
                [ skip ]
              </button>
            </div>
            <div className="mt-3 h-px w-full overflow-hidden bg-hairline">
              <motion.div
                className="h-full bg-gradient-to-r from-primary/50 via-primary to-signal"
                animate={{ width: `${pct}%` }}
                transition={{ duration: reduced ? 0 : 0.4, ease: ease.outExpo }}
              />
            </div>
          </div>
        </motion.div>
      ) : null}
    </AnimatePresence>
  )
}
