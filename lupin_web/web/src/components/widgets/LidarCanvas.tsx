import { useEffect, useRef } from 'react'

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useAnimationLoop } from '@/lib/throttle'
import { ROS_TYPE, type LaserScan } from '@/types/ros'

export function LidarCanvas() {
  const [{ scanTopic }] = useSettings()
  const ref = useTopic<LaserScan>(scanTopic, ROS_TYPE.LaserScan)
  const canvasRef = useRef<HTMLCanvasElement | null>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const resize = () => {
      const dpr = window.devicePixelRatio || 1
      const rect = canvas.getBoundingClientRect()
      canvas.width = rect.width * dpr
      canvas.height = rect.height * dpr
      const ctx = canvas.getContext('2d')
      if (ctx) ctx.scale(dpr, dpr)
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
    const maxRange = scan?.range_max && scan.range_max > 0 ? Math.min(scan.range_max, 6) : 6
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

  return (
    <Card className="flex flex-col">
      <CardHeader>
        <CardTitle>
          Lidar
          <span className="tag tag-accent ml-auto">PNL-LDR-01</span>
        </CardTitle>
        <CardDescription>{scanTopic}</CardDescription>
      </CardHeader>
      <CardContent className="flex-1">
        <div className="relative aspect-square w-full overflow-hidden rounded-sm border border-hairline bg-background/60">
          <canvas ref={canvasRef} className="h-full w-full" />
          <span className="tag absolute left-2 top-2">N · forward</span>
          <span className="tag absolute right-2 bottom-2 tabular-nums">scale · 6m</span>
        </div>
      </CardContent>
    </Card>
  )
}
