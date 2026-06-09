import {
  Check, Crosshair, Eraser, Eye, EyeOff, Loader2, Minus, Plus, RotateCcw, Target,
} from 'lucide-react'
import { useCallback, useEffect, useMemo, useRef, useState } from 'react'

import { useFocusPanel } from '@/components/system/FocusPanel'
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@/components/ui/card'
import { ExpandButton } from '@/components/ui/ExpandButton'
import { HEALTH_COLORS, healthLabel, speciesColor, speciesLabel } from '@/lib/flowers'
import {
  paintFieldToCanvas,
  rampGradientCss,
  rampCssColor,
  SENSOR_RAMPS,
} from '@/lib/heatmap'
import { tagHealthState } from '@/lib/tulip-health'
import { useMapPose, useTopic, usePublisher, useService } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useAnimationLoop, useThrottledRender } from '@/lib/throttle'
import { tagAgeSeconds, tagHasPose, useTwinField, useTwinState } from '@/lib/twin'
import { onPulseTag } from '@/lib/twin-events'
import { cn } from '@/lib/utils'
import {
  LUPIN_SRV,
  ROS_TYPE,
  TWIN_SENSORS,
  type OccupancyGrid,
  type Path,
  type PoseStamped,
  type TwinSensor,
  type TwinTagState,
} from '@/types/ros'

/**
 * Renders the SLAM occupancy grid, the live robot pose (via TF), and the
 * current Nav2 plan. Click to set a goal — drag while clicking to set the
 * goal heading; releasing publishes a PoseStamped on `/goal_pose`.
 */
export function MapCanvas({ interactive = false }: { interactive?: boolean } = {}) {
  const focus = useFocusPanel()
  // Maximized inspection: a view-scale multiplier on top of the fit scale and a
  // canvas-pixel pan offset, folded into the single projection. Identity inline
  // (so goal-setting is untouched); only the interactive/maximized variant zooms.
  const viewScaleRef = useRef(1)
  const viewOffsetRef = useRef({ x: 0, y: 0 })
  const mapPointers = useRef(new Map<number, { x: number; y: number }>())
  const mapGesture = useRef<{
    mode: 'pan' | 'pinch' | null
    sx?: number; sy?: number; ox?: number; oy?: number
    startDist?: number; startScale?: number; midX?: number; midY?: number
  }>({ mode: null })
  const mapRect = useRef<DOMRect | null>(null)
  const [{ mapTopic, planTopic, goalPoseTopic, mapFrame, baseFrame, polarityInvertHmi }] = useSettings()
  const mapRef = useTopic<OccupancyGrid>(mapTopic, ROS_TYPE.OccupancyGrid)
  const planRef = useTopic<Path>(planTopic, ROS_TYPE.Path)
  const pose = useMapPose(mapFrame, baseFrame)
  const publishGoal = usePublisher<PoseStamped>(goalPoseTopic, ROS_TYPE.PoseStamped)

  // Active sensor + per-layer toggles. Local-state, no settings persistence
  // — these are operator preferences for the current page session.
  const [sensor, setSensor] = useState<TwinSensor>('temperature')
  const [layers, setLayers] = useState({
    heatmap: true,
    trajectory: true,
    pins: true,
    flowers: true,
  })

  // Twin live snapshot — pin positions + readings + staleness.
  const { state: twin, stale: twinStale } = useTwinState()
  const tagsRef = useRef<TwinTagState[]>([])
  tagsRef.current = twin?.tags ?? []
  // Read inside the rAF draw loop: when the twin wedges, pins desaturate so a
  // frozen frame never reads as live.
  const twinStaleRef = useRef(false)
  twinStaleRef.current = twinStale

  // Heat-map field, derived lazily from /twin/get_field. We pass the
  // current map's bbox so the field paints the same area the SLAM map
  // covers; resolution is coarser than the SLAM grid (5–10× cells) since
  // the field is smooth and we want sub-100 ms render. Computed inline
  // each render — cheap, and the useTwinField sig check dedupes calls.
  const fieldBbox = useMemo(() => {
    const map = mapRef.current
    if (!map) return null
    const ox = map.info.origin.position.x
    const oy = map.info.origin.position.y
    const mw = map.info.width * map.info.resolution
    const mh = map.info.height * map.info.resolution
    return { minX: ox, minY: oy, maxX: ox + mw, maxY: oy + mh }
    // mapRef.current changes via the imperative subscribe path, but the
    // useThrottledRender ticker forces re-renders so this useMemo
    // re-evaluates on each tick — we depend on the live ref, not on a
    // stable React value.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [mapRef.current?.info.width, mapRef.current?.info.height])
  const { field } = useTwinField({
    sensor,
    bbox: fieldBbox,
    resolution: 0.25,
    enabled: layers.heatmap && fieldBbox != null,
    refreshIntervalMs: 5000,
  })
  // Keep the painted offscreen canvas around so the render loop only
  // rebuilds it when the field response actually changes.
  const fieldBitmapRef = useRef<{
    key: string
    bitmap: HTMLCanvasElement | null
  } | null>(null)

  // 60 s pose tail. Buffered as (x, y, t) tuples in a ref so the render
  // loop reads the latest without needing a re-render.
  const trajectoryRef = useRef<Array<{ x: number; y: number; t: number }>>([])
  useEffect(() => {
    if (!pose) return
    const now = performance.now() / 1000
    trajectoryRef.current.push({ x: pose.x, y: pose.y, t: now })
    // Prune older than 60 s. Keep at most ~600 samples to bound memory.
    while (
      trajectoryRef.current.length > 0 &&
      (now - trajectoryRef.current[0].t > 60 || trajectoryRef.current.length > 600)
    ) {
      trajectoryRef.current.shift()
    }
  }, [pose?.x, pose?.y])

  // Hover tooltip — set from canvas pointermove hit-testing against pins.
  const [hoveredTag, setHoveredTag] = useState<{
    tagId: string
    canvasX: number
    canvasY: number
  } | null>(null)

  // Pulse ring fired from the Greenhouse State table's row-click. Held in
  // a ref so the render loop reads it imperatively without retriggering
  // React renders during the animation.
  const pulseRef = useRef<{ tagId: string; startedAt: number } | null>(null)
  useEffect(() => onPulseTag((tagId) => {
    pulseRef.current = { tagId, startedAt: performance.now() }
  }), [])

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
    // 180° polarity flip stacks on top of the landscape/portrait fit so the
    // rendered map and click coords are in the operator's physical frame, not
    // the controller's flipped internal frame. See `lib/polarity.ts`.
    const rot =
      ((w > h) !== (mw > mh) ? Math.PI / 2 : 0) +
      (polarityInvertHmi ? Math.PI : 0)
    const cosR = Math.cos(rot)
    const sinR = Math.sin(rot)
    const rmw = Math.abs(cosR) * mw + Math.abs(sinR) * mh
    const rmh = Math.abs(sinR) * mw + Math.abs(cosR) * mh
    const sFit = Math.max(0.0001, Math.min(w / rmw, h / rmh) * 0.95)
    const s = sFit * viewScaleRef.current
    const cx = w / 2 + viewOffsetRef.current.x
    const cy = h / 2 + viewOffsetRef.current.y
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
  }, [mapRef, polarityInvertHmi])

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
    const rect = e.currentTarget.getBoundingClientRect()
    const px = e.clientX - rect.left
    const py = e.clientY - rect.top

    // Drag-for-yaw takes priority while a drag is active.
    if (dragRef.current) {
      const proj = projection()
      if (!proj) return
      const w = proj.canvasToWorld(px, py)
      setDrag({ from: dragRef.current.from, to: w })
      return
    }

    // Hit-test against tag pins (12 px radius around each pin centre,
    // generous so the tooltip is easy to land on). First hit wins.
    const proj = projection()
    if (!proj) {
      if (hoveredTag) setHoveredTag(null)
      return
    }
    const HIT_RADIUS_PX = 12
    let best: { tagId: string; cx: number; cy: number } | null = null
    let bestDsq = HIT_RADIUS_PX * HIT_RADIUS_PX + 1
    for (const t of tagsRef.current) {
      if (!tagHasPose(t)) continue
      const c = proj.worldToCanvas(t.pose.position.x, t.pose.position.y)
      const dx = px - c.x
      const dy = py - c.y
      const dsq = dx * dx + dy * dy
      if (dsq < bestDsq) {
        bestDsq = dsq
        best = { tagId: t.tag_id, cx: c.x, cy: c.y }
      }
    }
    if (best) {
      if (hoveredTag?.tagId !== best.tagId ||
          hoveredTag.canvasX !== best.cx ||
          hoveredTag.canvasY !== best.cy) {
        setHoveredTag({ tagId: best.tagId, canvasX: best.cx, canvasY: best.cy })
      }
    } else if (hoveredTag) {
      setHoveredTag(null)
    }
  }
  const onPointerLeave = () => {
    if (hoveredTag) setHoveredTag(null)
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

  // ---- Maximized inspection handlers: cursor-anchored wheel zoom + drag pan.
  // The rAF draw loop reads the refs each frame, so no state bump is needed.
  const onZoomWheel = (e: React.WheelEvent<HTMLCanvasElement>) => {
    e.preventDefault()
    const rect = e.currentTarget.getBoundingClientRect()
    const w = rect.width
    const h = rect.height
    const mx = e.clientX - rect.left
    const my = e.clientY - rect.top
    const cxBefore = w / 2 + viewOffsetRef.current.x
    const cyBefore = h / 2 + viewOffsetRef.current.y
    const target = Math.min(8, Math.max(1, viewScaleRef.current * Math.exp(-e.deltaY * 0.0015)))
    const k = target / viewScaleRef.current
    viewScaleRef.current = target
    if (target <= 1.001) {
      viewScaleRef.current = 1
      viewOffsetRef.current = { x: 0, y: 0 }
    } else {
      viewOffsetRef.current = { x: mx - k * (mx - cxBefore) - w / 2, y: my - k * (my - cyBefore) - h / 2 }
    }
  }
  const clampMapScale = (s: number) => Math.min(8, Math.max(1, s))
  const startMapGesture = () => {
    const pts = [...mapPointers.current.values()]
    if (pts.length === 1) {
      mapGesture.current = { mode: 'pan', sx: pts[0].x, sy: pts[0].y, ox: viewOffsetRef.current.x, oy: viewOffsetRef.current.y }
    } else if (pts.length >= 2) {
      const [a, b] = pts
      mapGesture.current = {
        mode: 'pinch', startDist: Math.hypot(b.x - a.x, b.y - a.y) || 1, startScale: viewScaleRef.current,
        midX: (a.x + b.x) / 2, midY: (a.y + b.y) / 2, ox: viewOffsetRef.current.x, oy: viewOffsetRef.current.y,
      }
    } else {
      mapGesture.current = { mode: null }
    }
  }
  const applyMapZoom = (nextScale: number, fromScale: number, fromOx: number, fromOy: number, cx: number, cy: number, rect: DOMRect) => {
    const ns = clampMapScale(nextScale)
    const px = cx - rect.left
    const py = cy - rect.top
    const cxBefore = rect.width / 2 + fromOx
    const cyBefore = rect.height / 2 + fromOy
    const k = ns / fromScale
    viewScaleRef.current = ns
    if (ns <= 1.001) {
      viewScaleRef.current = 1
      viewOffsetRef.current = { x: 0, y: 0 }
    } else {
      viewOffsetRef.current = { x: px - k * (px - cxBefore) - rect.width / 2, y: py - k * (py - cyBefore) - rect.height / 2 }
    }
  }
  const onPanDown = (e: React.PointerEvent<HTMLCanvasElement>) => {
    try { e.currentTarget.setPointerCapture(e.pointerId) } catch { /* */ }
    mapRect.current = e.currentTarget.getBoundingClientRect()
    mapPointers.current.set(e.pointerId, { x: e.clientX, y: e.clientY })
    startMapGesture()
  }
  const onPanMove = (e: React.PointerEvent<HTMLCanvasElement>) => {
    if (!mapPointers.current.has(e.pointerId)) return
    mapPointers.current.set(e.pointerId, { x: e.clientX, y: e.clientY })
    const g = mapGesture.current
    if (g.mode === 'pan') {
      viewOffsetRef.current = { x: g.ox! + (e.clientX - g.sx!), y: g.oy! + (e.clientY - g.sy!) }
    } else if (g.mode === 'pinch') {
      const pts = [...mapPointers.current.values()]
      if (pts.length < 2) return
      const [a, b] = pts
      const dist = Math.hypot(b.x - a.x, b.y - a.y)
      const rect = mapRect.current ?? e.currentTarget.getBoundingClientRect()
      applyMapZoom(g.startScale! * (dist / g.startDist!), g.startScale!, g.ox!, g.oy!, g.midX!, g.midY!, rect)
    }
  }
  const onPanUp = (e: React.PointerEvent<HTMLCanvasElement>) => {
    mapPointers.current.delete(e.pointerId)
    try { e.currentTarget.releasePointerCapture(e.pointerId) } catch { /* */ }
    startMapGesture()
  }
  const zoomByCenter = (factor: number) => {
    viewScaleRef.current = Math.min(8, Math.max(1, viewScaleRef.current * factor))
    if (viewScaleRef.current === 1) viewOffsetRef.current = { x: 0, y: 0 }
  }
  const resetView = () => {
    viewScaleRef.current = 1
    viewOffsetRef.current = { x: 0, y: 0 }
  }
  const openMaximized = () =>
    focus.open({ title: 'Map / Nav', subtitle: 'wheel zoom · drag pan', render: () => <MapCanvas interactive /> })

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

    // Heat-map layer (twin field). Painted over the SLAM bitmap but
    // *under* the 1 m grid so the map's structure stays legible. Updates
    // only when the field response or its dimensions change — render
    // loop is at 60 Hz, paintFieldToCanvas would be wasteful per frame.
    if (layers.heatmap && field && field.width > 0 && field.height > 0) {
      const fieldKey =
        `${field.width}x${field.height}|${field.value_min}|${field.value_max}|${sensor}|${field.sample_count}`
      const ramp = SENSOR_RAMPS[sensor]
      if (fieldBitmapRef.current?.key !== fieldKey) {
        fieldBitmapRef.current = {
          key: fieldKey,
          bitmap: paintFieldToCanvas(field, ramp),
        }
      }
      const fbmp = fieldBitmapRef.current?.bitmap
      if (fbmp) {
        const fw = field.width * field.resolution_used
        const fh = field.height * field.resolution_used
        const center = proj.worldToCanvas(
          field.origin_x + fw / 2,
          field.origin_y + fh / 2,
        )
        ctx.save()
        ctx.translate(center.x, center.y)
        ctx.rotate(-proj.rot)
        // Smoothing on so the cell-grid blurs into a continuous gradient.
        ctx.imageSmoothingEnabled = true
        ctx.drawImage(fbmp, (-fw * proj.s) / 2, (-fh * proj.s) / 2, fw * proj.s, fh * proj.s)
        ctx.restore()
      }
    }

    // 1m grid overlay
    drawMapGrid(ctx, proj, map)

    // Trajectory polyline — last 60 s of robot pose. Faint chartreuse
    // tail with the most recent points fully opaque, fading to ~20%
    // toward the oldest. Visually light so it doesn't compete with the
    // Nav2 plan polyline below.
    if (layers.trajectory && trajectoryRef.current.length > 1) {
      const traj = trajectoryRef.current
      const newest = traj[traj.length - 1].t
      ctx.save()
      ctx.lineWidth = 1.5
      for (let i = 1; i < traj.length; i++) {
        const a = traj[i - 1]
        const b = traj[i]
        const age = newest - b.t
        const alpha = Math.max(0.1, 0.6 - age / 60 * 0.5)
        const pa = proj.worldToCanvas(a.x, a.y)
        const pb = proj.worldToCanvas(b.x, b.y)
        ctx.beginPath()
        ctx.moveTo(pa.x, pa.y)
        ctx.lineTo(pb.x, pb.y)
        ctx.strokeStyle = `hsla(78, 70%, 60%, ${alpha.toFixed(2)})`
        ctx.stroke()
      }
      ctx.restore()
    }

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

    // Tag pins — twin-driven. Drawn under the robot chevron so the
    // chevron is on top when the robot is sitting on top of the most
    // recent tag. Pins fade as their stale_seconds climbs past 0; older
    // tags desaturate but stay visible.
    if (layers.pins) {
      const ramp = SENSOR_RAMPS[sensor]
      for (const t of tagsRef.current) {
        if (!tagHasPose(t)) continue  // never been seen with a pose
        const reading = t.readings.find((r) => r.name === sensor)?.value
        const hasReading = reading != null
        // Live age from last_observed (keeps advancing even when the twin
        // wedged), with an extra desaturation when the whole twin is offline.
        const age = tagAgeSeconds(t)
        const sat =
          (1 - Math.min(0.7, age / 600)) * (twinStaleRef.current ? 0.45 : 1)
        const c = proj.worldToCanvas(t.pose.position.x, t.pose.position.y)
        const r = 6
        ctx.save()
        // Diamond marker (distinct from round flower dots), sitting on the
        // box's front edge (the tag is mounted mid-front-face).
        ctx.beginPath()
        ctx.moveTo(c.x, c.y - r)
        ctx.lineTo(c.x + r, c.y)
        ctx.lineTo(c.x, c.y + r)
        ctx.lineTo(c.x - r, c.y)
        ctx.closePath()
        if (hasReading && field) {
          const span = Math.max(1e-9, field.value_max - field.value_min)
          const tNorm = Math.min(1, Math.max(0, (reading - field.value_min) / span))
          ctx.fillStyle = rampCssColor(ramp, tNorm, sat)
          ctx.fill()
          ctx.lineWidth = 1
          ctx.strokeStyle = 'hsl(120 25% 8%)'
          ctx.stroke()
        } else {
          // Honest "no sensor here" marker — hollow + hatched, never a
          // mid-ramp colour that fakes a reading.
          ctx.fillStyle = 'hsla(120, 6%, 28%, 0.22)'
          ctx.fill()
          ctx.setLineDash([2, 2])
          ctx.lineWidth = 1
          ctx.strokeStyle = 'hsl(120 8% 45%)'
          ctx.stroke()
          ctx.setLineDash([])
        }
        // Tag id label, dark backdrop pill (unchanged style).
        ctx.font = "10px 'JetBrains Mono', ui-monospace, monospace"
        const m = ctx.measureText(t.tag_id)
        const lx = c.x + 8
        const ly = c.y - 12
        ctx.fillStyle = 'hsla(120, 25%, 6%, 0.78)'
        ctx.fillRect(lx - 3, ly - 9.5, m.width + 6, 13)
        ctx.fillStyle = hasReading
          ? `hsla(${ramp.hue}, 70%, 80%, ${sat.toFixed(2)})`
          : 'hsla(120, 8%, 70%, 0.85)'
        ctx.textBaseline = 'alphabetic'
        ctx.fillText(t.tag_id, lx, ly)
        ctx.restore()
      }
    }

    // Flower markers — the localized box footprint (faint rectangle) plus a
    // species-coloured filled dot at each localized bloom, with a red dashed
    // alert ring on anomalous blooms. Drawn over the sensor pins so the
    // species reads at a glance.
    if (layers.flowers) {
      for (const t of tagsRef.current) {
        // Box footprint rectangle (faint), from the localized corners.
        const corners = t.box_footprint?.points ?? []
        if (corners.length >= 3) {
          ctx.save()
          ctx.beginPath()
          corners.forEach((p, i) => {
            const c = proj.worldToCanvas(p.x, p.y)
            if (i === 0) ctx.moveTo(c.x, c.y)
            else ctx.lineTo(c.x, c.y)
          })
          ctx.closePath()
          ctx.fillStyle = 'hsla(140, 35%, 50%, 0.06)'
          ctx.lineWidth = 1
          ctx.strokeStyle = 'hsla(140, 35%, 62%, 0.35)'
          ctx.fill()
          ctx.stroke()
          ctx.restore()
        }
        // Species-coloured filled dots at each localized bloom (NOT t.pose).
        for (const fp of t.flowers ?? []) {
          const c = proj.worldToCanvas(fp.position.x, fp.position.y)
          ctx.save()
          ctx.beginPath()
          ctx.arc(c.x, c.y, 4, 0, Math.PI * 2)
          ctx.fillStyle = speciesColor(fp.species)
          ctx.fill()
          ctx.lineWidth = 1
          ctx.strokeStyle = 'hsla(0, 0%, 0%, 0.45)'
          ctx.stroke()
          if (fp.anomaly) {
            ctx.beginPath()
            ctx.arc(c.x, c.y, 7, 0, Math.PI * 2)
            ctx.setLineDash([3, 3])
            ctx.lineWidth = 1.5
            ctx.strokeStyle = '#e23a3a'
            ctx.stroke()
            ctx.setLineDash([])
          }
          ctx.restore()
        }
      }
    }

    // Pulse ring — drawn over pins so the highlight sits on top, but
    // under the chevron so the robot itself is never obscured. Fades over
    // ~1.5 s and clears the ref when done.
    const pulse = pulseRef.current
    if (pulse) {
      const age = (performance.now() - pulse.startedAt) / 1000
      if (age > 1.5) {
        pulseRef.current = null
      } else {
        const target = tagsRef.current.find((t) => t.tag_id === pulse.tagId)
        // Highlight the FLOWER cluster the row represents, not the tag diamond:
        // centre on the bloom centroid, else the box-footprint centre, else the
        // tag pose. (A Greenhouse-State row is "this tag's flowers"; ringing the
        // tag pin missed what the user actually clicked.)
        let center: { x: number; y: number } | null = null
        if (target) {
          const blooms = target.flowers ?? []
          const box = target.box_footprint?.points ?? []
          if (blooms.length) {
            center = {
              x: blooms.reduce((s, f) => s + f.position.x, 0) / blooms.length,
              y: blooms.reduce((s, f) => s + f.position.y, 0) / blooms.length,
            }
          } else if (box.length >= 3) {
            center = {
              x: box.reduce((s, p) => s + p.x, 0) / box.length,
              y: box.reduce((s, p) => s + p.y, 0) / box.length,
            }
          } else if (tagHasPose(target)) {
            center = { x: target.pose.position.x, y: target.pose.position.y }
          }
        }
        if (center) {
          const c = proj.worldToCanvas(center.x, center.y)
          // Two concentric rings, each pulsing on a different phase, so
          // the highlight reads even on a busy heat map.
          for (let k = 0; k < 2; k++) {
            const phase = (age + k * 0.4) % 1.5
            const t = phase / 1.5
            const radius = 6 + t * 22
            const alpha = (1 - t) * 0.85
            ctx.beginPath()
            ctx.arc(c.x, c.y, radius, 0, Math.PI * 2)
            ctx.lineWidth = 2
            ctx.strokeStyle = `hsla(78, 90%, 65%, ${alpha.toFixed(2)})`
            ctx.stroke()
          }
        }
      }
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

  // Tags visible in the live twin snapshot — derived state for the
  // header status line + hover tooltip lookup.
  const visibleTags = (twin?.tags ?? []).filter(tagHasPose)
  const visibleTagCount = visibleTags.length
  const hoveredTagState = hoveredTag
    ? visibleTags.find((t) => t.tag_id === hoveredTag.tagId) ?? null
    : null

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
    <Card className="flex min-h-0 w-full flex-1 flex-col">
      <CardHeader>
        <CardTitle className="flex flex-wrap items-center gap-x-2 gap-y-1">
          <span>Map · navigation</span>
          <span className="tag tag-accent">PNL-NAV-01</span>
          <div className="ml-auto flex items-center gap-2">
            <SensorPills value={sensor} onChange={setSensor} />
            <span className="h-3 w-px bg-hairline" aria-hidden />
            <LayerToggles value={layers} onChange={setLayers} />
            <span className="h-3 w-px bg-hairline" aria-hidden />
            <EraseMapButton />
            {!interactive && (
              <>
                <span className="h-3 w-px bg-hairline" aria-hidden />
                <ExpandButton onClick={openMaximized} className="h-7 w-7" label="Maximize map" />
              </>
            )}
          </div>
        </CardTitle>
        <CardDescription className="flex items-center gap-2">
          <span className="font-mono">{mapTopic}</span>
          <span>·</span>
          <span>{statusText}</span>
          {visibleTagCount > 0 && (
            <>
              <span>·</span>
              <span className="font-mono">tags · {visibleTagCount}</span>
            </>
          )}
        </CardDescription>
      </CardHeader>
      <CardContent className="flex min-h-0 flex-1 flex-col">
        <div className="relative flex-1 overflow-hidden rounded-sm border border-hairline bg-ink-1">
          <canvas
            ref={canvasRef}
            onPointerDown={interactive ? onPanDown : onPointerDown}
            onPointerMove={interactive ? onPanMove : onPointerMove}
            onPointerUp={interactive ? onPanUp : onPointerUp}
            onPointerCancel={interactive ? onPanUp : onPointerUp}
            onPointerLeave={interactive ? onPanUp : onPointerLeave}
            onWheel={interactive ? onZoomWheel : undefined}
            className={cn(
              'h-full w-full touch-none',
              interactive ? 'cursor-grab active:cursor-grabbing' : 'cursor-crosshair',
            )}
          />
          {interactive && (
            <div className="absolute bottom-2 right-2 z-10 flex items-center gap-1">
              {[
                { k: 'out', label: 'Zoom out', icon: <Minus className="h-4 w-4" />, on: () => zoomByCenter(1 / 1.4) },
                { k: 'in', label: 'Zoom in', icon: <Plus className="h-4 w-4" />, on: () => zoomByCenter(1.4) },
                { k: 'fit', label: 'Fit', icon: <RotateCcw className="h-4 w-4" />, on: resetView },
              ].map((b) => (
                <button
                  key={b.k}
                  type="button"
                  onClick={b.on}
                  aria-label={b.label}
                  title={b.label}
                  className="inline-flex h-8 w-8 items-center justify-center rounded-sm border border-hairline bg-ink-2/80 text-muted-foreground backdrop-blur-sm transition-colors hover:border-primary/40 hover:bg-ink-3 hover:text-foreground"
                >
                  {b.icon}
                </button>
              ))}
            </div>
          )}
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

          {/* Bottom-left legend — single-hue ramp + min/max labels + unit. */}
          {layers.heatmap && field && field.sample_count > 0 && (
            <FieldLegend sensor={sensor} field={field} />
          )}

          {/* Hover tooltip. Positioned near the pin in canvas coords. */}
          {hoveredTagState && hoveredTag && (
            <TagTooltip
              tag={hoveredTagState}
              canvasX={hoveredTag.canvasX}
              canvasY={hoveredTag.canvasY}
              canvasWidth={canvasRef.current?.clientWidth ?? 0}
            />
          )}
        </div>
      </CardContent>
    </Card>
  )
}

/* ---------- header controls ---------- */

const SENSOR_LABELS: Record<TwinSensor, string> = {
  temperature: 'TEMP',
  humidity: 'HUM',
  co2: 'CO₂',
  light: 'LIGHT',
  soil_moisture: 'SOIL',
}

function SensorPills({
  value, onChange,
}: { value: TwinSensor; onChange: (v: TwinSensor) => void }) {
  return (
    <div className="flex items-center rounded-sm border border-hairline bg-card/40 p-0.5">
      {TWIN_SENSORS.map((s) => (
        <button
          key={s}
          type="button"
          onClick={() => onChange(s)}
          aria-pressed={value === s}
          className={cn(
            'tag rounded-sm px-1.5 py-0.5 transition-colors',
            value === s
              ? 'bg-primary/15 text-primary'
              : 'text-muted-foreground hover:text-foreground',
          )}
        >
          {SENSOR_LABELS[s]}
        </button>
      ))}
    </div>
  )
}

interface LayerState {
  heatmap: boolean
  trajectory: boolean
  pins: boolean
  flowers: boolean
}

function LayerToggles({
  value, onChange,
}: { value: LayerState; onChange: (v: LayerState) => void }) {
  const items: Array<{ key: keyof LayerState; label: string }> = [
    { key: 'heatmap',    label: 'heat' },
    { key: 'trajectory', label: 'tail' },
    { key: 'pins',       label: 'pins' },
    { key: 'flowers',    label: 'flowers' },
  ]
  return (
    <div className="flex items-center gap-1">
      {items.map(({ key, label }) => {
        const on = value[key]
        return (
          <button
            key={key}
            type="button"
            onClick={() => onChange({ ...value, [key]: !on })}
            aria-pressed={on}
            title={`Toggle ${label}`}
            className={cn(
              'tag flex items-center gap-1 rounded-sm border px-1.5 py-0.5 transition-colors',
              on
                ? 'border-primary/40 bg-primary/10 text-primary'
                : 'border-hairline bg-background/40 text-muted-foreground hover:text-foreground',
            )}
          >
            {on ? <Eye className="h-2.5 w-2.5" /> : <EyeOff className="h-2.5 w-2.5" />}
            {label}
          </button>
        )
      })}
    </div>
  )
}

/* ---------- erase-map button ---------- */

/**
 * Two-step destructive trigger for `/lupin/nav/clear_map`. First click
 * arms the button (label flips to "confirm?", styling shifts to
 * destructive). Second click within ARM_WINDOW_MS fires the service;
 * any other interaction or the timeout disarms.
 *
 * The backend SIGTERMs slam_toolbox; respawn brings it back with an
 * empty pose graph. /map drops out for ~5–10 s during the cycle —
 * the existing "awaiting map" placeholder covers the gap.
 */
const ARM_WINDOW_MS = 4000

function EraseMapButton() {
  const clearMap = useService<Record<string, never>, { success: boolean; message: string }>(
    '/lupin/nav/clear_map',
    LUPIN_SRV.Trigger,
  )
  const [armed, setArmed] = useState(false)
  const [busy, setBusy] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const armTimerRef = useRef<number | null>(null)

  const disarm = useCallback(() => {
    setArmed(false)
    if (armTimerRef.current != null) {
      window.clearTimeout(armTimerRef.current)
      armTimerRef.current = null
    }
  }, [])

  useEffect(() => () => {
    if (armTimerRef.current != null) window.clearTimeout(armTimerRef.current)
  }, [])

  const onClick = async () => {
    if (busy) return
    if (!armed) {
      setArmed(true)
      setError(null)
      armTimerRef.current = window.setTimeout(disarm, ARM_WINDOW_MS)
      return
    }
    disarm()
    setBusy(true)
    try {
      const res = await clearMap({})
      if (!res.success) {
        setError(res.message || 'erase rejected')
      }
    } catch (e: unknown) {
      setError(e instanceof Error ? e.message : String(e))
    } finally {
      setBusy(false)
    }
  }

  return (
    <div className="relative flex items-center">
      <button
        type="button"
        onClick={onClick}
        disabled={busy}
        title={armed ? 'Click again to wipe the SLAM map' : 'Erase the SLAM map'}
        aria-pressed={armed}
        className={cn(
          'tag flex items-center gap-1 rounded-sm border px-1.5 py-0.5 transition-colors',
          armed
            ? 'border-destructive/70 bg-destructive/20 text-destructive-foreground'
            : 'border-hairline bg-background/40 text-muted-foreground hover:border-destructive/50 hover:text-destructive',
          busy && 'opacity-60',
        )}
      >
        {busy
          ? <Loader2 className="h-2.5 w-2.5 animate-spin" />
          : <Eraser className="h-2.5 w-2.5" />}
        {busy ? 'erasing…' : armed ? 'confirm?' : 'erase'}
      </button>
      {error && (
        <span
          className="absolute right-0 top-full mt-1 max-w-[220px] rounded-sm border border-destructive/40 bg-destructive/10 px-1.5 py-0.5 font-mono text-[10px] text-destructive"
          role="alert"
        >
          {error}
        </span>
      )}
    </div>
  )
}

/* ---------- legend + tooltip ---------- */

function FieldLegend({
  sensor, field,
}: { sensor: TwinSensor; field: NonNullable<ReturnType<typeof useTwinField>['field']> }) {
  const ramp = SENSOR_RAMPS[sensor]
  return (
    <div className="pointer-events-none absolute bottom-2 left-2 flex flex-col gap-1 rounded-sm border border-hairline bg-card/70 px-2 py-1 backdrop-blur">
      <div className="flex items-baseline gap-2">
        <span className="tag tag-strong">{SENSOR_LABELS[sensor]}</span>
        <span className="font-mono text-[10px] text-muted-foreground">{ramp.unit}</span>
      </div>
      <div
        className="h-2 w-[120px] rounded-sm border border-hairline"
        style={{ backgroundImage: rampGradientCss(ramp) }}
      />
      <div className="flex items-baseline justify-between font-mono text-[10px] text-muted-foreground">
        <span>{formatLegendNumber(field.value_min)}</span>
        <span>{formatLegendNumber(field.value_max)}</span>
      </div>
    </div>
  )
}

function formatLegendNumber(v: number): string {
  if (!Number.isFinite(v)) return '—'
  if (Math.abs(v) >= 100) return v.toFixed(0)
  if (Math.abs(v) >= 10) return v.toFixed(1)
  return v.toFixed(2)
}

const TOOLTIP_OFFSET = 14
const TOOLTIP_W_EST = 200

function TagTooltip({
  tag, canvasX, canvasY, canvasWidth,
}: { tag: TwinTagState; canvasX: number; canvasY: number; canvasWidth: number }) {
  // Anchor up-and-right of the pin by default; flip left only when
  // there isn't room on the right. Comparing against canvas width
  // (rather than canvasX > 0, which is always true) ensures the
  // tooltip never escapes the right edge.
  const flipX = canvasX > canvasWidth - TOOLTIP_W_EST
  const left = flipX
    ? canvasX - TOOLTIP_W_EST + TOOLTIP_OFFSET
    : canvasX + TOOLTIP_OFFSET
  const top = canvasY - 8 - 80
  return (
    <div
      className="pointer-events-none absolute z-10 max-w-[220px] rounded-sm border border-hairline bg-card/90 px-2 py-1.5 text-[11px] shadow-[0_2px_12px_rgba(0,0,0,0.4)] backdrop-blur"
      style={{ left, top }}
    >
      <div className="flex items-baseline gap-2">
        <span className="tag tag-accent">tag</span>
        <span className="font-mono text-foreground">{tag.tag_id}</span>
        <span className="ml-auto font-mono text-[10px] text-muted-foreground">
          {formatStaleness(tagAgeSeconds(tag))}
        </span>
      </div>
      {tag.species && (
        <div className="mt-1 flex items-center gap-1.5">
          <span
            className="inline-block h-2.5 w-2.5 rounded-full"
            style={{ background: speciesColor(tag.species) }}
          />
          <span className="text-foreground">{speciesLabel(tag.species)}</span>
          {tag.species_confidence > 0 && (
            <span className="text-muted-foreground">
              {(tag.species_confidence * 100).toFixed(0)}%
            </span>
          )}
          <span
            className="ml-auto rounded-sm px-1 text-[10px]"
            style={{ color: HEALTH_COLORS[tagHealthState(tag)] }}
          >
            {healthLabel(tagHealthState(tag))}
          </span>
        </div>
      )}
      {tag.anomaly && (
        <div className="mt-1 font-mono text-[10px] text-destructive">⚠ pest detected (bug)</div>
      )}
      <div className="mt-1 space-y-0.5">
        {tag.readings.length === 0 ? (
          <div className="text-muted-foreground">no readings yet</div>
        ) : (
          tag.readings.map((r) => (
            <div key={r.name} className="flex items-baseline justify-between gap-2 font-mono">
              <span className="text-muted-foreground">{r.name}</span>
              <span className="text-foreground">
                {formatReadingValue(r.value)}
                <span className="ml-0.5 text-muted-foreground">{readingUnit(r.name)}</span>
              </span>
            </div>
          ))
        )}
      </div>
    </div>
  )
}

const READING_UNITS: Record<string, string> = {
  temperature: '°C',
  humidity: '%',
  co2: 'ppm',
  light: 'lux',
  soil_moisture: '%',
}
function readingUnit(name: string): string {
  return READING_UNITS[name] ?? ''
}
function formatReadingValue(v: number): string {
  if (!Number.isFinite(v)) return '—'
  if (Math.abs(v) >= 100) return v.toFixed(0)
  if (Math.abs(v) >= 10) return v.toFixed(1)
  return v.toFixed(2)
}
function formatStaleness(s: number): string {
  if (!Number.isFinite(s) || s < 0) return '—'
  if (s < 1) return 'now'
  if (s < 60) return `${Math.round(s)} s ago`
  if (s < 3600) return `${Math.round(s / 60)} min ago`
  return `${Math.round(s / 3600)} h ago`
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
