import { Check, Crosshair, Target } from 'lucide-react'
import { useCallback, useEffect, useRef, useState } from 'react'

import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { useMapPose, useTopic, usePublisher } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useAnimationLoop, useThrottledRender } from '@/lib/throttle'
import { ROS_TYPE, type OccupancyGrid, type Path, type PoseStamped } from '@/types/ros'

/**
 * Renders the SLAM occupancy grid, the live robot pose (via TF), and the
 * current Nav2 plan. Click to set a goal — drag while clicking to set the
 * goal heading; releasing publishes a PoseStamped on `/goal_pose`.
 */
export function MapCanvas() {
  const [{ mapTopic, planTopic, goalPoseTopic, mapFrame, baseFrame }] = useSettings()
  const mapRef = useTopic<OccupancyGrid>(mapTopic, ROS_TYPE.OccupancyGrid)
  const planRef = useTopic<Path>(planTopic, ROS_TYPE.Path)
  const pose = useMapPose(mapFrame, baseFrame)
  const publishGoal = usePublisher<PoseStamped>(goalPoseTopic, ROS_TYPE.PoseStamped)
  // Map and plan are read via refs (imperatively mutated). Tick the React tree
  // a couple of times per second so the header status string + pose readout
  // pick up new data without having to re-render every frame.
  useThrottledRender(2)

  const canvasRef = useRef<HTMLCanvasElement | null>(null)

  // Pre-rasterised map bitmap, keyed by (width, height, data fingerprint)
  const mapBitmapRef = useRef<{ key: string; bitmap: HTMLCanvasElement } | null>(null)

  // Drag-for-yaw state. Stored in MAP-FRAME world coords.
  const [drag, setDrag] = useState<
    | null
    | {
        from: { x: number; y: number }
        to: { x: number; y: number }
      }
  >(null)
  const dragRef = useRef(drag)
  dragRef.current = drag

  // Latest committed goal preview (so we keep showing the published target until pose reaches it)
  const [committedGoal, setCommittedGoal] = useState<{ x: number; y: number; yaw: number } | null>(
    null,
  )

  // Goal staged after pointer-up but not yet published. The "navigate there?"
  // popup confirms; clicking outside the popup discards it.
  const [pendingGoal, setPendingGoal] = useState<{ x: number; y: number; yaw: number } | null>(
    null,
  )
  const popupRef = useRef<HTMLButtonElement | null>(null)

  /* ----- Sizing & DPR --- */
  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const resize = () => {
      const dpr = window.devicePixelRatio || 1
      const rect = canvas.getBoundingClientRect()
      canvas.width = Math.max(1, Math.floor(rect.width * dpr))
      canvas.height = Math.max(1, Math.floor(rect.height * dpr))
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

  /* ----- World <-> canvas projection -----
     We fit the whole map into the visible canvas, preserving aspect. When the
     canvas is landscape but the map is portrait (or vice versa), we rotate the
     world by 90° CCW so the map's long axis aligns with the canvas long axis —
     this fills laptops while still working unrotated on phones in portrait.
     Returned `s` is pixels per metre (canvas CSS pixels); `rot` is the world
     rotation in radians (CCW). */
  const projection = useCallback(() => {
    const canvas = canvasRef.current
    const map = mapRef.current
    if (!canvas || !map) return null
    const w = canvas.clientWidth
    const h = canvas.clientHeight
    const mw = map.info.width * map.info.resolution
    const mh = map.info.height * map.info.resolution
    const rot = (w > h) !== (mw > mh) ? Math.PI / 2 : 0
    const cosR = Math.cos(rot)
    const sinR = Math.sin(rot)
    const rmw = Math.abs(cosR) * mw + Math.abs(sinR) * mh
    const rmh = Math.abs(sinR) * mw + Math.abs(cosR) * mh
    const s = Math.max(0.0001, Math.min(w / rmw, h / rmh) * 0.95)
    const cx = w / 2
    const cy = h / 2
    const ox = map.info.origin.position.x + mw / 2
    const oy = map.info.origin.position.y + mh / 2
    // worldToCanvas: rotate (world - mapCenter) by `rot` CCW, scale, then flip Y
    // (canvas Y grows down) and translate to canvas centre.
    const worldToCanvas = (wx: number, wy: number) => {
      const dx = wx - ox
      const dy = wy - oy
      const rx = cosR * dx - sinR * dy
      const ry = sinR * dx + cosR * dy
      return { x: cx + rx * s, y: cy - ry * s }
    }
    const canvasToWorld = (px: number, py: number) => {
      const rx = (px - cx) / s
      const ry = -(py - cy) / s
      const dx = cosR * rx + sinR * ry
      const dy = -sinR * rx + cosR * ry
      return { x: ox + dx, y: oy + dy }
    }
    return { s, rot, worldToCanvas, canvasToWorld }
  }, [mapRef])

  /* ----- Pre-rasterise the OccupancyGrid into an offscreen canvas. ----- */
  const ensureMapBitmap = useCallback(() => {
    const map = mapRef.current
    if (!map) return null
    const W = map.info.width
    const H = map.info.height
    // Cheap fingerprint: dims + a sample of cells (full hash would be slow on
    // large maps and the map rarely changes between updates anyway).
    const sample =
      map.data.length > 0
        ? `${map.data[0]}-${map.data[Math.floor(map.data.length / 2)]}-${map.data[map.data.length - 1]}`
        : ''
    const key = `${W}x${H}/${map.info.resolution}/${sample}`
    if (mapBitmapRef.current?.key === key) return mapBitmapRef.current.bitmap

    const bmp = document.createElement('canvas')
    bmp.width = W
    bmp.height = H
    const bctx = bmp.getContext('2d')
    if (!bctx) return null
    const img = bctx.createImageData(W, H)
    for (let i = 0; i < W * H; i++) {
      const v = map.data[i]
      const di = i * 4
      // Flip Y: ROS map's row 0 is the bottom; canvas row 0 is the top.
      const row = Math.floor(i / W)
      const col = i - row * W
      const flipped = (H - 1 - row) * W + col
      const dj = flipped * 4
      if (v < 0) {
        // unknown — soft warm-grey
        img.data[dj + 0] = 28
        img.data[dj + 1] = 32
        img.data[dj + 2] = 30
        img.data[dj + 3] = 255
      } else if (v >= 65) {
        // occupied — bone, with some warmth
        img.data[dj + 0] = 232
        img.data[dj + 1] = 230
        img.data[dj + 2] = 215
        img.data[dj + 3] = 255
      } else if (v <= 20) {
        // free — deep ink, slightly green
        img.data[dj + 0] = 14
        img.data[dj + 1] = 18
        img.data[dj + 2] = 16
        img.data[dj + 3] = 255
      } else {
        // partial — interpolate
        const t = v / 100
        img.data[dj + 0] = Math.round(14 + (232 - 14) * t)
        img.data[dj + 1] = Math.round(18 + (230 - 18) * t)
        img.data[dj + 2] = Math.round(16 + (215 - 16) * t)
        img.data[dj + 3] = 255
      }
      void di
    }
    bctx.putImageData(img, 0, 0)
    mapBitmapRef.current = { key, bitmap: bmp }
    return bmp
  }, [mapRef])

  /* ----- Pointer interaction → goal_pose --- */
  const onPointerDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const proj = projection()
    if (!proj) return
    // Starting a new drag invalidates any pending confirmation.
    setPendingGoal(null)
    const rect = e.currentTarget.getBoundingClientRect()
    const px = e.clientX - rect.left
    const py = e.clientY - rect.top
    const w = proj.canvasToWorld(px, py)
    setDrag({ from: w, to: w })
    e.currentTarget.setPointerCapture(e.pointerId)
  }
  const onPointerMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (!dragRef.current) return
    const proj = projection()
    if (!proj) return
    const rect = e.currentTarget.getBoundingClientRect()
    const px = e.clientX - rect.left
    const py = e.clientY - rect.top
    const w = proj.canvasToWorld(px, py)
    setDrag({ from: dragRef.current.from, to: w })
  }
  const onPointerUp = (e: React.PointerEvent<HTMLCanvasElement>) => {
    const cur = dragRef.current
    setDrag(null)
    try {
      e.currentTarget.releasePointerCapture(e.pointerId)
    } catch {
      /* releasing a non-captured pointer is harmless */
    }
    if (!cur) return
    const dx = cur.to.x - cur.from.x
    const dy = cur.to.y - cur.from.y
    const yaw = dx * dx + dy * dy < 1e-4 ? (pose?.yaw ?? 0) : Math.atan2(dy, dx)
    setPendingGoal({ x: cur.from.x, y: cur.from.y, yaw })
  }

  const confirmPendingGoal = () => {
    const g = pendingGoal
    if (!g) return
    const half = g.yaw / 2
    const sec = Math.floor(Date.now() / 1000)
    const nanosec = (Date.now() % 1000) * 1e6
    const goal: PoseStamped = {
      header: { stamp: { sec, nanosec }, frame_id: mapFrame },
      pose: {
        position: { x: g.x, y: g.y, z: 0 },
        orientation: { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) },
      },
    }
    publishGoal(goal)
    setCommittedGoal({ x: g.x, y: g.y, yaw: g.yaw })
    setPendingGoal(null)
  }

  // Click outside the popup → cancel the pending goal. Use mousedown so it
  // fires before any click handler on the canvas, and check `popupRef` so the
  // confirm-button click below isn't swallowed.
  useEffect(() => {
    if (!pendingGoal) return
    const onDocDown = (ev: MouseEvent | PointerEvent) => {
      const t = ev.target as Node | null
      if (popupRef.current && t && popupRef.current.contains(t)) return
      setPendingGoal(null)
    }
    document.addEventListener('pointerdown', onDocDown, true)
    return () => document.removeEventListener('pointerdown', onDocDown, true)
  }, [pendingGoal])

  /* ----- Render loop --- */
  useAnimationLoop(mapRef, () => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    const w = canvas.clientWidth
    const h = canvas.clientHeight

    // Background
    ctx.fillStyle = 'hsl(120 12% 5%)'
    ctx.fillRect(0, 0, w, h)

    const proj = projection()
    if (!proj) {
      drawNoMap(ctx, w, h)
      return
    }
    const map = mapRef.current
    if (!map) return

    // Map bitmap drawn at scale, rotated about the map centre when the
    // projection rotates the world.
    const bmp = ensureMapBitmap()
    if (bmp) {
      const mw = map.info.width * map.info.resolution
      const mh = map.info.height * map.info.resolution
      const center = proj.worldToCanvas(
        map.info.origin.position.x + mw / 2,
        map.info.origin.position.y + mh / 2,
      )
      ctx.save()
      ctx.translate(center.x, center.y)
      // Canvas Y is flipped vs world; world CCW rotation = canvas CW.
      ctx.rotate(-proj.rot)
      ctx.imageSmoothingEnabled = false
      ctx.drawImage(bmp, (-mw * proj.s) / 2, (-mh * proj.s) / 2, mw * proj.s, mh * proj.s)
      ctx.imageSmoothingEnabled = true
      ctx.restore()
    }

    // 1m grid overlay
    drawMapGrid(ctx, proj, map)

    // Plan polyline
    const plan = planRef.current
    if (plan && plan.poses.length > 1) {
      ctx.beginPath()
      const p0 = proj.worldToCanvas(plan.poses[0].pose.position.x, plan.poses[0].pose.position.y)
      ctx.moveTo(p0.x, p0.y)
      for (let i = 1; i < plan.poses.length; i++) {
        const p = proj.worldToCanvas(plan.poses[i].pose.position.x, plan.poses[i].pose.position.y)
        ctx.lineTo(p.x, p.y)
      }
      ctx.lineWidth = 2
      ctx.strokeStyle = 'hsl(78 90% 62% / 0.85)'
      ctx.stroke()

      // small leading dot at the end of the plan
      const last = plan.poses[plan.poses.length - 1].pose.position
      const lp = proj.worldToCanvas(last.x, last.y)
      ctx.beginPath()
      ctx.arc(lp.x, lp.y, 3, 0, Math.PI * 2)
      ctx.fillStyle = 'hsl(78 90% 70%)'
      ctx.fill()
    }

    // Committed goal marker
    if (committedGoal) {
      drawGoal(ctx, proj, committedGoal, 'hsl(192 90% 60%)')
    }

    // Pending goal (awaiting user confirm in the popup)
    if (pendingGoal) {
      drawGoal(ctx, proj, pendingGoal, 'hsl(38 95% 60%)')
    }

    // Active drag preview
    if (drag) {
      const yaw =
        Math.hypot(drag.to.x - drag.from.x, drag.to.y - drag.from.y) < 0.01
          ? (pose?.yaw ?? 0)
          : Math.atan2(drag.to.y - drag.from.y, drag.to.x - drag.from.x)
      drawGoal(ctx, proj, { x: drag.from.x, y: drag.from.y, yaw }, 'hsl(38 95% 60%)')
    }

    // Robot chevron
    if (pose) {
      drawRobot(ctx, proj, pose)
    }

    // Frame & scale-bar
    drawScaleBar(ctx, proj, w, h)
  })

  /* ----- Status pill subtitle --- */
  const statusText = !mapRef.current
    ? 'awaiting map'
    : !pose
    ? 'awaiting TF'
    : `pose · ${pose.x.toFixed(2)} m, ${pose.y.toFixed(2)} m, ${((pose.yaw * 180) / Math.PI).toFixed(0)}°`

  /* ----- Popup screen position — anchored at the arrow tip of the pending
     goal, then nudged so it sits clear of the chevron. */
  const popupPos = (() => {
    if (!pendingGoal) return null
    const proj = projection()
    if (!proj) return null
    const c = proj.worldToCanvas(pendingGoal.x, pendingGoal.y)
    const visualYaw = pendingGoal.yaw + proj.rot
    const tipDx = Math.cos(visualYaw) * 26
    const tipDy = -Math.sin(visualYaw) * 26
    return { x: c.x + tipDx + 10, y: c.y + tipDy - 14 }
  })()

  return (
    <Card className="flex flex-1 flex-col">
      <CardHeader>
        <CardTitle>
          Map · navigation
          <span className="tag tag-accent ml-auto">PNL-NAV-01</span>
        </CardTitle>
        <CardDescription className="flex items-center gap-2">
          <span className="font-mono">{mapTopic}</span>
          <span>·</span>
          <span>{statusText}</span>
        </CardDescription>
      </CardHeader>
      <CardContent className="flex flex-1 min-h-[28rem] flex-col">
        <div className="relative flex-1 overflow-hidden rounded-sm border border-hairline bg-background/60">
          <canvas
            ref={canvasRef}
            onPointerDown={onPointerDown}
            onPointerMove={onPointerMove}
            onPointerUp={onPointerUp}
            onPointerCancel={onPointerUp}
            className="h-full w-full cursor-crosshair touch-none"
          />
          <div className="pointer-events-none absolute left-2 top-2 flex items-center gap-2">
            <span className="tag">frame · {mapFrame}</span>
          </div>
          <div className="pointer-events-none absolute right-2 top-2 flex items-center gap-2">
            {drag ? (
              <span className="tag tag-accent flex items-center gap-1">
                <Target className="h-3 w-3" /> drag · drop to nav
              </span>
            ) : pendingGoal ? (
              <span className="tag tag-accent flex items-center gap-1">
                <Target className="h-3 w-3" /> awaiting confirm
              </span>
            ) : (
              <span className="tag flex items-center gap-1">
                <Crosshair className="h-3 w-3" /> click + drag · set goal
              </span>
            )}
          </div>
          {pendingGoal && popupPos && (
            <button
              ref={popupRef}
              type="button"
              onClick={confirmPendingGoal}
              style={{ left: popupPos.x, top: popupPos.y }}
              className="absolute z-10 flex items-center gap-1.5 rounded-sm border border-primary/60 bg-primary/15 px-2 py-1 text-[10px] uppercase tracking-[0.18em] text-primary shadow-[0_0_18px_-6px_hsl(var(--primary)/0.7)] backdrop-blur transition-colors hover:bg-primary/25 focus:outline-none focus:ring-1 focus:ring-primary"
            >
              <Check className="h-3 w-3" />
              navigate there?
            </button>
          )}
        </div>
      </CardContent>
    </Card>
  )
}

/* ---------- canvas helpers ---------- */

function drawMapGrid(
  ctx: CanvasRenderingContext2D,
  proj: { worldToCanvas: (x: number, y: number) => { x: number; y: number } },
  map: OccupancyGrid,
) {
  const ox = map.info.origin.position.x
  const oy = map.info.origin.position.y
  const mw = map.info.width * map.info.resolution
  const mh = map.info.height * map.info.resolution
  ctx.save()
  ctx.strokeStyle = 'hsla(120, 8%, 30%, 0.18)'
  ctx.lineWidth = 0.5
  for (let xi = Math.ceil(ox); xi <= ox + mw; xi += 1) {
    const a = proj.worldToCanvas(xi, oy)
    const b = proj.worldToCanvas(xi, oy + mh)
    ctx.beginPath()
    ctx.moveTo(a.x, a.y)
    ctx.lineTo(b.x, b.y)
    ctx.stroke()
  }
  for (let yi = Math.ceil(oy); yi <= oy + mh; yi += 1) {
    const a = proj.worldToCanvas(ox, yi)
    const b = proj.worldToCanvas(ox + mw, yi)
    ctx.beginPath()
    ctx.moveTo(a.x, a.y)
    ctx.lineTo(b.x, b.y)
    ctx.stroke()
  }
  ctx.restore()
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  proj: { worldToCanvas: (x: number, y: number) => { x: number; y: number }; rot: number },
  pose: { x: number; y: number; yaw: number },
) {
  const c = proj.worldToCanvas(pose.x, pose.y)
  ctx.save()
  ctx.translate(c.x, c.y)
  // Canvas y is inverted vs world, so negate the (world-frame yaw + view rotation).
  ctx.rotate(-(pose.yaw + proj.rot))
  // Halo
  ctx.beginPath()
  ctx.arc(0, 0, 12, 0, Math.PI * 2)
  ctx.fillStyle = 'hsla(78, 90%, 58%, 0.18)'
  ctx.fill()
  // Chevron — points along +X (which after rotation = robot's forward)
  ctx.beginPath()
  ctx.moveTo(10, 0)
  ctx.lineTo(-6, 6)
  ctx.lineTo(-3, 0)
  ctx.lineTo(-6, -6)
  ctx.closePath()
  ctx.fillStyle = 'hsl(78 90% 65%)'
  ctx.fill()
  ctx.lineWidth = 1
  ctx.strokeStyle = 'hsl(120 25% 6%)'
  ctx.stroke()
  ctx.restore()
}

function drawGoal(
  ctx: CanvasRenderingContext2D,
  proj: { worldToCanvas: (x: number, y: number) => { x: number; y: number }; rot: number },
  goal: { x: number; y: number; yaw: number },
  color: string,
) {
  const c = proj.worldToCanvas(goal.x, goal.y)
  ctx.save()
  ctx.translate(c.x, c.y)
  ctx.rotate(-(goal.yaw + proj.rot))
  ctx.strokeStyle = color
  ctx.fillStyle = color
  ctx.lineWidth = 1.5
  // crosshair
  ctx.beginPath()
  ctx.arc(0, 0, 8, 0, Math.PI * 2)
  ctx.stroke()
  ctx.beginPath()
  ctx.moveTo(-12, 0)
  ctx.lineTo(-8, 0)
  ctx.moveTo(8, 0)
  ctx.lineTo(12, 0)
  ctx.moveTo(0, -12)
  ctx.lineTo(0, -8)
  ctx.moveTo(0, 8)
  ctx.lineTo(0, 12)
  ctx.stroke()
  // heading arrow
  ctx.beginPath()
  ctx.moveTo(8, 0)
  ctx.lineTo(20, 0)
  ctx.moveTo(20, 0)
  ctx.lineTo(15, -3)
  ctx.moveTo(20, 0)
  ctx.lineTo(15, 3)
  ctx.stroke()
  ctx.restore()
}

function drawScaleBar(
  ctx: CanvasRenderingContext2D,
  proj: { s: number },
  w: number,
  h: number,
) {
  const oneM = proj.s
  const x0 = w - oneM - 16
  const y0 = h - 18
  ctx.save()
  ctx.strokeStyle = 'hsl(60 18% 92% / 0.7)'
  ctx.fillStyle = 'hsl(60 18% 92% / 0.7)'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(x0, y0)
  ctx.lineTo(x0 + oneM, y0)
  ctx.moveTo(x0, y0 - 4)
  ctx.lineTo(x0, y0 + 4)
  ctx.moveTo(x0 + oneM, y0 - 4)
  ctx.lineTo(x0 + oneM, y0 + 4)
  ctx.stroke()
  ctx.font = '10px "JetBrains Mono", monospace'
  ctx.textAlign = 'right'
  ctx.fillText('1 m', x0 + oneM, y0 - 6)
  ctx.restore()
}

function drawNoMap(ctx: CanvasRenderingContext2D, w: number, h: number) {
  ctx.save()
  ctx.fillStyle = 'hsl(80 10% 60%)'
  ctx.font = '11px "JetBrains Mono", monospace'
  ctx.textAlign = 'center'
  ctx.fillText('awaiting /map …', w / 2, h / 2)
  ctx.restore()
}
