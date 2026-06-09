import { RotateCcw } from 'lucide-react'
import { useEffect, useRef, useState } from 'react'

import { useFocusPanel } from '@/components/system/FocusPanel'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { ExpandButton } from '@/components/ui/ExpandButton'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useAnimationLoop } from '@/lib/throttle'
import { cn } from '@/lib/utils'
import { ROS_TYPE, type LaserScan } from '@/types/ros'

export function LidarCanvas({ className, interactive = false }: { className?: string; interactive?: boolean } = {}) {
  const [{ scanTopic }] = useSettings()
  const focus = useFocusPanel()
  const ref = useTopic<LaserScan>(scanTopic, ROS_TYPE.LaserScan)
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  // Display range (metres) the radar is scaled to. Wheel-zoom in the maximized
  // variant clamps it to 1–12 m; the rAF loop reads the ref each frame.
  const rangeRef = useRef(6)
  const [displayRange, setDisplayRange] = useState(6)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const resize = () => {
      const dpr = window.devicePixelRatio || 1
      // Backing store from the layout box, not getBoundingClientRect(): the
      // latter is shrunk by the FocusPanel's `zoom-in-95` open animation (a CSS
      // transform), which froze this buffer at 0.95× and stretched the scan.
      // clientW/H are transform-independent and match the draw loop's space.
      // Reset the transform before scaling so repeated resizes don't compound.
      const cw = canvas.clientWidth
      const ch = canvas.clientHeight
      canvas.width = Math.max(1, Math.floor(cw * dpr))
      canvas.height = Math.max(1, Math.floor(ch * dpr))
      const ctx = canvas.getContext('2d')
      if (ctx) {
        ctx.setTransform(1, 0, 0, 1, 0, 0)
        ctx.scale(dpr, dpr)
      }
    }
    resize()
    const ro = new ResizeObserver(resize)
    ro.observe(canvas)
    return () => ro.disconnect()
  }, [])

  useAnimationLoop(ref, (scan) => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    const w = canvas.clientWidth
    const h = canvas.clientHeight

    ctx.clearRect(0, 0, w, h)
    // dark botanical-ink fill
    ctx.fillStyle = 'hsl(120 12% 5%)'
    ctx.fillRect(0, 0, w, h)

    const cx = w / 2
    const cy = h / 2
    const cap = rangeRef.current
    const maxRange = scan?.range_max && scan.range_max > 0 ? Math.min(scan.range_max, cap) : cap
    const radius = Math.min(w, h) / 2 - 16
    const pxPerM = radius / maxRange

    // concentric range rings — dashed for 1m, solid every 5m equivalent
    ctx.strokeStyle = 'hsl(120 8% 22%)'
    ctx.lineWidth = 1
    for (let r = 1; r <= Math.floor(maxRange); r++) {
      ctx.setLineDash(r % 2 === 0 ? [] : [2, 4])
      ctx.beginPath()
      ctx.arc(cx, cy, r * pxPerM, 0, Math.PI * 2)
      ctx.stroke()
    }
    ctx.setLineDash([])
    // axis cross
    ctx.strokeStyle = 'hsl(120 8% 28%)'
    ctx.beginPath()
    ctx.moveTo(cx - radius, cy)
    ctx.lineTo(cx + radius, cy)
    ctx.moveTo(cx, cy - radius)
    ctx.lineTo(cx, cy + radius)
    ctx.stroke()

    // forward-bearing arc highlight (±30°)
    ctx.strokeStyle = 'hsl(78 90% 58% / 0.18)'
    ctx.lineWidth = 1.5
    ctx.beginPath()
    ctx.arc(cx, cy, radius - 4, -Math.PI / 2 - Math.PI / 6, -Math.PI / 2 + Math.PI / 6)
    ctx.stroke()

    // scan points — chartreuse to match the rest of the console
    if (scan && scan.ranges.length > 0) {
      ctx.fillStyle = 'hsl(78 90% 62%)'
      const N = scan.ranges.length
      for (let i = 0; i < N; i++) {
        const r = scan.ranges[i]
        if (!Number.isFinite(r) || r < scan.range_min || r > maxRange) continue
        const ang = scan.angle_min + i * scan.angle_increment
        // robot frame: x forward (up), y left → on-screen: x→up, y→left
        const px = cx - Math.sin(ang) * r * pxPerM
        const py = cy - Math.cos(ang) * r * pxPerM
        ctx.fillRect(px - 1, py - 1, 2, 2)
      }
    }

    // robot — small chevron pointing up (forward)
    ctx.fillStyle = 'hsl(60 18% 92%)'
    ctx.beginPath()
    ctx.moveTo(cx, cy - 8)
    ctx.lineTo(cx - 6, cy + 6)
    ctx.lineTo(cx, cy + 3)
    ctx.lineTo(cx + 6, cy + 6)
    ctx.closePath()
    ctx.fill()
  })

  const onLidarWheel = (e: React.WheelEvent) => {
    e.preventDefault()
    const next = Math.min(12, Math.max(1, rangeRef.current * Math.exp(e.deltaY * 0.0015)))
    rangeRef.current = next
    setDisplayRange(Math.round(next))
  }
  const resetRange = () => { rangeRef.current = 6; setDisplayRange(6) }
  const openMaximized = () =>
    focus.open({
      title: 'Lidar',
      subtitle: 'scroll to change range',
      render: () => <LidarCanvas interactive className="h-full w-full rounded-none border-0" />,
    })

  return (
    <Card className={cn('flex flex-col', className)}>
      <CardHeader>
        <CardTitle>
          Lidar
          <div className="ml-auto flex items-center gap-2">
            <span className="tag tag-accent">PNL-LDR-01</span>
            {!interactive && <ExpandButton onClick={openMaximized} className="h-7 w-7" label="Maximize lidar" />}
          </div>
        </CardTitle>
        <CardDescription>{scanTopic}</CardDescription>
      </CardHeader>
      <CardContent className="flex min-h-0 flex-1 flex-col">
        <div className="relative w-full flex-1 min-h-[180px] overflow-hidden rounded-sm border border-hairline bg-ink-1 sm:min-h-[260px]">
          <canvas
            ref={canvasRef}
            onWheel={interactive ? onLidarWheel : undefined}
            className={cn('h-full w-full', interactive && 'cursor-zoom-in')}
          />
          <span className="tag absolute left-2 top-2">N · forward</span>
          <span className="tag absolute right-2 bottom-2 tabular-nums">scale · {displayRange}m</span>
          {interactive && (
            <button
              type="button"
              onClick={resetRange}
              aria-label="Reset range"
              title="Reset range"
              className="absolute right-2 top-2 inline-flex h-8 w-8 items-center justify-center rounded-sm border border-hairline bg-ink-2/80 text-muted-foreground backdrop-blur-sm transition-colors hover:border-primary/40 hover:bg-ink-3 hover:text-foreground"
            >
              <RotateCcw className="h-4 w-4" />
            </button>
          )}
        </div>
      </CardContent>
    </Card>
  )
}
