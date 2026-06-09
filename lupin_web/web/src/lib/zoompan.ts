import { useCallback, useRef, useState } from 'react'

/** A 2D zoom + pan transform applied as `scale()` then `translate()` (px). */
export interface Transform {
  scale: number
  x: number
  y: number
}

const IDENTITY: Transform = { scale: 1, x: 0, y: 0 }

interface Gesture {
  mode: 'pan' | 'pinch' | null
  // pan
  sx?: number
  sy?: number
  // pinch
  startDist?: number
  startScale?: number
  midX?: number
  midY?: number
  // shared origin offset
  ox?: number
  oy?: number
}

/**
 * useZoomPan — cursor-anchored wheel zoom, single-finger drag pan, and
 * two-finger pinch zoom, clamped to [min, max]. Spread `bind` onto the surface.
 * The transform is React state, so it suits CSS-transform surfaces (camera).
 */
export function useZoomPan(min = 1, max = 6) {
  const [transform, setTransform] = useState<Transform>(IDENTITY)
  const ref = useRef(transform)
  ref.current = transform

  const pointers = useRef(new Map<number, { x: number; y: number }>())
  const gesture = useRef<Gesture>({ mode: null })
  const rectRef = useRef<DOMRect | null>(null)

  const clamp = useCallback((s: number) => Math.min(max, Math.max(min, s)), [min, max])

  // (Re)derive the active gesture from the current pointer set.
  const startGesture = useCallback(() => {
    const pts = [...pointers.current.values()]
    if (pts.length === 1) {
      gesture.current = { mode: 'pan', sx: pts[0].x, sy: pts[0].y, ox: ref.current.x, oy: ref.current.y }
    } else if (pts.length >= 2) {
      const [a, b] = pts
      gesture.current = {
        mode: 'pinch',
        startDist: Math.hypot(b.x - a.x, b.y - a.y) || 1,
        startScale: ref.current.scale,
        midX: (a.x + b.x) / 2,
        midY: (a.y + b.y) / 2,
        ox: ref.current.x,
        oy: ref.current.y,
      }
    } else {
      gesture.current = { mode: null }
    }
  }, [])

  const zoomAbout = useCallback(
    (nextScale: number, fromScale: number, fromX: number, fromY: number, cx: number, cy: number, rect: DOMRect) => {
      const ns = clamp(nextScale)
      const px = cx - rect.left - rect.width / 2
      const py = cy - rect.top - rect.height / 2
      const k = ns / fromScale
      setTransform({ scale: ns, x: px - (px - fromX) * k, y: py - (py - fromY) * k })
    },
    [clamp],
  )

  const onWheel = useCallback(
    (e: React.WheelEvent) => {
      e.preventDefault()
      const rect = e.currentTarget.getBoundingClientRect()
      zoomAbout(ref.current.scale * Math.exp(-e.deltaY * 0.0015), ref.current.scale, ref.current.x, ref.current.y, e.clientX, e.clientY, rect)
    },
    [zoomAbout],
  )

  const onPointerDown = useCallback((e: React.PointerEvent) => {
    try { (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId) } catch { /* */ }
    rectRef.current = e.currentTarget.getBoundingClientRect()
    pointers.current.set(e.pointerId, { x: e.clientX, y: e.clientY })
    startGesture()
  }, [startGesture])

  const onPointerMove = useCallback((e: React.PointerEvent) => {
    if (!pointers.current.has(e.pointerId)) return
    pointers.current.set(e.pointerId, { x: e.clientX, y: e.clientY })
    const g = gesture.current
    if (g.mode === 'pan') {
      const nx = g.ox! + (e.clientX - g.sx!)
      const ny = g.oy! + (e.clientY - g.sy!)
      setTransform((t) => (t.x === nx && t.y === ny ? t : { ...t, x: nx, y: ny }))
    } else if (g.mode === 'pinch') {
      const pts = [...pointers.current.values()]
      if (pts.length < 2) return
      const [a, b] = pts
      const dist = Math.hypot(b.x - a.x, b.y - a.y)
      const rect = rectRef.current ?? e.currentTarget.getBoundingClientRect()
      zoomAbout(g.startScale! * (dist / g.startDist!), g.startScale!, g.ox!, g.oy!, g.midX!, g.midY!, rect)
    }
  }, [zoomAbout])

  const onPointerUp = useCallback((e: React.PointerEvent) => {
    pointers.current.delete(e.pointerId)
    try { (e.currentTarget as HTMLElement).releasePointerCapture(e.pointerId) } catch { /* */ }
    startGesture() // resume single-finger pan, or clear
  }, [startGesture])

  const zoomBy = useCallback(
    (factor: number) => setTransform((t) => ({ ...t, scale: clamp(t.scale * factor) })),
    [clamp],
  )

  const reset = useCallback(() => setTransform(IDENTITY), [])

  const bind = {
    onWheel,
    onPointerDown,
    onPointerMove,
    onPointerUp,
    onPointerLeave: onPointerUp,
    onPointerCancel: onPointerUp,
  }

  return { transform, ref, bind, zoomBy, reset, setTransform }
}

/**
 * CSS `transform` string. With `transform-origin: center`, the point under the
 * pointer stays fixed because the offset (x, y) is in parent pixel space:
 * a point p maps to `scale * p + (x, y)`, i.e. `translate(x,y) scale(s)`.
 */
export function toCss({ scale, x, y }: Transform): string {
  return `translate(${x}px, ${y}px) scale(${scale})`
}
