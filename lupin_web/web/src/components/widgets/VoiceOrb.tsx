/**
 * VoiceOrb — the centerpiece of the Voice tab.
 *
 * A canvas-driven, audio-reactive button. The visual layer reads real RMS from
 * the session's mic / speaker analysers (or synthesises a procedural level in
 * mock mode) and animates concentric rings + a center disc accordingly. It
 * also serves as the push-to-talk surface — touch / mouse handlers passthrough.
 *
 * Per state:
 *   idle       slow breath, dim
 *   connecting orbiting dot, faint scanline
 *   ready      slow breath, primary tint
 *   listening  rings expand with mic RMS, emerald
 *   thinking   ring sweeps + orbiting dots, amber
 *   speaking   rings expand with output RMS, sky-blue
 *   error      red flash + frozen
 */

import { useEffect, useRef } from 'react'

import { cn } from '@/lib/utils'
import type { VoiceStatus } from '@/lib/voice/types'

interface VoiceOrbProps {
  status: VoiceStatus
  /** Mic RMS in [0, 1] — read every frame. */
  getInputLevel: () => number
  /** Output RMS in [0, 1] — read every frame. */
  getOutputLevel: () => number
  /** Live (real Gemini) vs mock — controls procedural-level fallback. */
  isLive: boolean
  /** Whether button should ignore press. */
  disabled?: boolean
  micActive: boolean
  /** Push-to-talk handlers — same contract as a button. */
  onPress?: () => void
  onRelease?: () => void
  /** Optional click handler for mid-press tap (mock fires this for utterance-equivalent). */
  onTap?: () => void
  size?: number
  className?: string
  ariaLabel?: string
}

interface PaletteEntry {
  base: string
  ring: string
  glow: string
  rgb: [number, number, number]
}

// Tailwind-equivalent palettes resolved to hex. Stays in sync with the CSS
// tokens by sight; we don't pull from CSS variables so we can paint to canvas.
const PALETTES: Record<VoiceStatus, PaletteEntry> = {
  idle: {
    base: '#27272a',
    ring: '#52525b',
    glow: '#71717a',
    rgb: [113, 113, 122],
  },
  connecting: {
    base: '#1e293b',
    ring: '#38bdf8',
    glow: '#0ea5e9',
    rgb: [56, 189, 248],
  },
  ready: {
    base: '#1f2937',
    ring: '#a3e635',
    glow: '#84cc16',
    rgb: [163, 230, 53],
  },
  listening: {
    base: '#022c22',
    ring: '#10b981',
    glow: '#34d399',
    rgb: [52, 211, 153],
  },
  thinking: {
    base: '#1c1917',
    ring: '#f59e0b',
    glow: '#fbbf24',
    rgb: [251, 191, 36],
  },
  speaking: {
    base: '#082f49',
    ring: '#38bdf8',
    glow: '#7dd3fc',
    rgb: [125, 211, 252],
  },
  error: {
    base: '#450a0a',
    ring: '#ef4444',
    glow: '#f87171',
    rgb: [248, 113, 113],
  },
}

export function VoiceOrb({
  status,
  getInputLevel,
  getOutputLevel,
  isLive,
  disabled,
  micActive,
  onPress,
  onRelease,
  onTap,
  size = 240,
  className,
  ariaLabel,
}: VoiceOrbProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)

  // Live state read by the RAF loop. Refs so changes don't restart the loop.
  const statusRef = useRef<VoiceStatus>(status)
  const isLiveRef = useRef(isLive)
  const getInputLevelRef = useRef(getInputLevel)
  const getOutputLevelRef = useRef(getOutputLevel)
  statusRef.current = status
  isLiveRef.current = isLive
  getInputLevelRef.current = getInputLevel
  getOutputLevelRef.current = getOutputLevel

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return

    const dpr = Math.min(window.devicePixelRatio || 1, 2)
    const cssSize = size
    canvas.width = cssSize * dpr
    canvas.height = cssSize * dpr
    canvas.style.width = `${cssSize}px`
    canvas.style.height = `${cssSize}px`
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    ctx.scale(dpr, dpr)

    let raf = 0
    let smoothed = 0 // smoothed RMS
    let phase = 0 // breathing phase
    let orbit = 0 // angle for thinking dots
    let lastTs = performance.now()

    const draw = () => {
      const now = performance.now()
      const dt = Math.min(0.05, (now - lastTs) / 1000)
      lastTs = now
      phase += dt
      orbit += dt * 1.6

      const s = statusRef.current
      const live = isLiveRef.current
      const palette = PALETTES[s]

      // Pick the right level source per status. In mock mode we synthesise a
      // breathing waveform so the orb still feels alive in offline demos.
      let raw = 0
      if (s === 'listening') {
        raw = live
          ? clamp01(getInputLevelRef.current() * 5)
          : 0.3 + 0.5 * Math.abs(Math.sin(phase * 4) * Math.sin(phase * 0.7))
      } else if (s === 'speaking') {
        raw = live
          ? clamp01(getOutputLevelRef.current() * 5)
          : 0.4 + 0.4 * Math.abs(Math.sin(phase * 6) + 0.5 * Math.sin(phase * 11))
      } else if (s === 'thinking' || s === 'connecting') {
        raw = 0.18 + 0.05 * Math.sin(phase * 3)
      } else if (s === 'ready') {
        raw = 0.12 + 0.04 * Math.sin(phase * 1.4)
      } else if (s === 'error') {
        raw = 0.4 + 0.4 * Math.abs(Math.sin(phase * 8))
      } else {
        raw = 0.06 + 0.02 * Math.sin(phase * 0.9)
      }

      // Critically-damped smoothing — feels liquid, not jumpy.
      const tau = s === 'listening' || s === 'speaking' ? 0.06 : 0.18
      const alpha = 1 - Math.exp(-dt / tau)
      smoothed = smoothed + (raw - smoothed) * alpha

      // ── paint ──
      ctx.clearRect(0, 0, cssSize, cssSize)
      const cx = cssSize / 2
      const cy = cssSize / 2
      const baseR = cssSize * 0.22
      const maxR = cssSize * 0.46

      // Outermost halo — soft radial gradient that pulses with smoothed level.
      const haloR = baseR + (maxR - baseR) * (0.85 + 0.15 * smoothed)
      const halo = ctx.createRadialGradient(cx, cy, baseR * 0.6, cx, cy, haloR)
      halo.addColorStop(0, rgba(palette.rgb, 0.18 + 0.4 * smoothed))
      halo.addColorStop(0.6, rgba(palette.rgb, 0.06 + 0.18 * smoothed))
      halo.addColorStop(1, rgba(palette.rgb, 0))
      ctx.fillStyle = halo
      ctx.beginPath()
      ctx.arc(cx, cy, haloR, 0, Math.PI * 2)
      ctx.fill()

      // Three concentric rings, each with its own lag — looks alive, not flat.
      const rings = 3
      for (let i = 0; i < rings; i++) {
        const t = i / (rings - 1) // 0..1
        const lag = Math.max(0, smoothed - t * 0.25)
        const r = baseR + (maxR - baseR) * (0.55 + 0.35 * lag) - i * 6
        if (r < 4) continue
        ctx.strokeStyle = rgba(palette.rgb, 0.25 + 0.55 * (1 - t) * (0.5 + smoothed))
        ctx.lineWidth = 1.5 + (rings - i) * 0.4
        ctx.beginPath()
        ctx.arc(cx, cy, r, 0, Math.PI * 2)
        ctx.stroke()
      }

      // Thinking / connecting → orbiting dots overlaid on inner ring.
      if (s === 'thinking' || s === 'connecting') {
        const orbitR = baseR + 4
        for (let i = 0; i < 3; i++) {
          const a = orbit + i * ((Math.PI * 2) / 3)
          const dx = cx + Math.cos(a) * orbitR
          const dy = cy + Math.sin(a) * orbitR
          ctx.fillStyle = rgba(palette.rgb, 0.9 - i * 0.25)
          ctx.beginPath()
          ctx.arc(dx, dy, 3.2, 0, Math.PI * 2)
          ctx.fill()
        }
      }

      // Center disc — solid, with a subtle inner highlight + state ring.
      const innerR = baseR - 4
      const grad = ctx.createRadialGradient(cx, cy - innerR * 0.3, innerR * 0.1, cx, cy, innerR)
      grad.addColorStop(0, lighten(palette.base, 0.18))
      grad.addColorStop(1, palette.base)
      ctx.fillStyle = grad
      ctx.beginPath()
      ctx.arc(cx, cy, innerR, 0, Math.PI * 2)
      ctx.fill()

      // Outer rim of the center disc — primary state colour.
      ctx.strokeStyle = rgba(palette.rgb, 0.9)
      ctx.lineWidth = 2
      ctx.beginPath()
      ctx.arc(cx, cy, innerR, 0, Math.PI * 2)
      ctx.stroke()

      // Listening / speaking → vertical bar visualizer inside the disc.
      if (s === 'listening' || s === 'speaking') {
        const bars = 9
        const totalW = innerR * 1.2
        const startX = cx - totalW / 2
        const barW = totalW / (bars * 1.6)
        for (let i = 0; i < bars; i++) {
          // Bar heights derived from a per-bar noise signal modulated by RMS,
          // so the visualizer reacts to amplitude AND has independent motion.
          const seed = Math.sin(phase * 6 + i * 1.7) * 0.5 + 0.5
          const localLevel = smoothed * (0.3 + 0.7 * seed)
          const h = Math.max(2, innerR * 1.2 * localLevel)
          const x = startX + i * (barW * 1.6) + barW * 0.3
          const y = cy - h / 2
          ctx.fillStyle = rgba(palette.rgb, 0.85)
          roundRect(ctx, x, y, barW, h, barW * 0.5)
          ctx.fill()
        }
      } else if (s === 'thinking') {
        // Thinking — three pulsing dots in the disc.
        for (let i = 0; i < 3; i++) {
          const t = (phase * 2 + i * 0.4) % 1.2
          const a = Math.max(0, 1 - Math.abs(t - 0.6) / 0.6)
          ctx.fillStyle = rgba(palette.rgb, 0.4 + 0.6 * a)
          ctx.beginPath()
          ctx.arc(cx + (i - 1) * innerR * 0.35, cy, 5 + 2 * a, 0, Math.PI * 2)
          ctx.fill()
        }
      } else {
        // idle / ready / connecting / error → mic glyph.
        drawMicGlyph(ctx, cx, cy, innerR * 0.5, palette.rgb, micActive ? 1 : 0.65)
      }

      raf = requestAnimationFrame(draw)
    }

    raf = requestAnimationFrame(draw)
    return () => cancelAnimationFrame(raf)
  }, [size, micActive])

  return (
    <button
      type="button"
      disabled={disabled}
      onMouseDown={() => onPress?.()}
      onMouseUp={() => onRelease?.()}
      onMouseLeave={(e) => {
        if (e.buttons === 0) return
        onRelease?.()
      }}
      onTouchStart={(e) => {
        e.preventDefault()
        onPress?.()
      }}
      onTouchEnd={(e) => {
        e.preventDefault()
        onRelease?.()
      }}
      onClick={() => onTap?.()}
      aria-label={ariaLabel}
      className={cn(
        'relative inline-flex items-center justify-center rounded-full focus-visible:outline-none focus-visible:ring-2 focus-visible:ring-ring focus-visible:ring-offset-2',
        disabled ? 'opacity-50' : 'active:scale-[0.98]',
        className,
      )}
      style={{ width: size, height: size }}
    >
      <canvas ref={canvasRef} className="block" aria-hidden />
    </button>
  )
}

function clamp01(n: number): number {
  if (Number.isNaN(n)) return 0
  return Math.max(0, Math.min(1, n))
}

function rgba(rgb: [number, number, number], a: number): string {
  return `rgba(${rgb[0]},${rgb[1]},${rgb[2]},${a})`
}

function lighten(hex: string, amount: number): string {
  const m = /^#([0-9a-f]{6})$/i.exec(hex)
  if (!m) return hex
  const r = parseInt(m[1].slice(0, 2), 16)
  const g = parseInt(m[1].slice(2, 4), 16)
  const b = parseInt(m[1].slice(4, 6), 16)
  const lr = Math.round(r + (255 - r) * amount)
  const lg = Math.round(g + (255 - g) * amount)
  const lb = Math.round(b + (255 - b) * amount)
  return `rgb(${lr},${lg},${lb})`
}

function roundRect(
  ctx: CanvasRenderingContext2D,
  x: number,
  y: number,
  w: number,
  h: number,
  r: number,
): void {
  const rr = Math.min(r, w / 2, h / 2)
  ctx.beginPath()
  ctx.moveTo(x + rr, y)
  ctx.lineTo(x + w - rr, y)
  ctx.quadraticCurveTo(x + w, y, x + w, y + rr)
  ctx.lineTo(x + w, y + h - rr)
  ctx.quadraticCurveTo(x + w, y + h, x + w - rr, y + h)
  ctx.lineTo(x + rr, y + h)
  ctx.quadraticCurveTo(x, y + h, x, y + h - rr)
  ctx.lineTo(x, y + rr)
  ctx.quadraticCurveTo(x, y, x + rr, y)
  ctx.closePath()
}

function drawMicGlyph(
  ctx: CanvasRenderingContext2D,
  cx: number,
  cy: number,
  size: number,
  rgb: [number, number, number],
  alpha: number,
): void {
  const w = size * 0.55
  const h = size * 0.95
  const r = w / 2
  // Capsule
  ctx.fillStyle = rgba(rgb, alpha * 0.92)
  ctx.beginPath()
  ctx.moveTo(cx - r, cy - h / 2 + r)
  ctx.arc(cx, cy - h / 2 + r, r, Math.PI, 0, false)
  ctx.lineTo(cx + r, cy + h / 2 - r)
  ctx.arc(cx, cy + h / 2 - r, r, 0, Math.PI, false)
  ctx.closePath()
  ctx.fill()
  // U-cradle below
  ctx.strokeStyle = rgba(rgb, alpha * 0.7)
  ctx.lineWidth = Math.max(2, size * 0.08)
  ctx.beginPath()
  ctx.arc(cx, cy + size * 0.05, size * 0.55, 0.2 * Math.PI, 0.8 * Math.PI, false)
  ctx.stroke()
  // Stand
  ctx.beginPath()
  ctx.moveTo(cx, cy + size * 0.55)
  ctx.lineTo(cx, cy + size * 0.85)
  ctx.stroke()
  ctx.beginPath()
  ctx.moveTo(cx - size * 0.3, cy + size * 0.85)
  ctx.lineTo(cx + size * 0.3, cy + size * 0.85)
  ctx.stroke()
}
