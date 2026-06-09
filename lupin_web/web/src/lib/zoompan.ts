import { useCallback, useRef, useState } from 'react'

/** A 2D zoom + pan transform applied as `scale()` then `translate()` (px). */
export interface Transform {
  scale: number
  x: number
  y: number
}

const IDENTITY: Transform = { scale: 1, x: 0, y: 0 }

/**
 * useZoomPan — cursor-anchored wheel zoom + drag pan + pinch, clamped to
 * [min, max]. Returns the live `transform`, a `bind` object to spread onto the
 * surface, and `zoomBy`/`reset` for toolbar buttons.
 *
 * The transform is React state (re-renders the consumer) — fine for CSS-transform
 * surfaces (camera stream). Canvas consumers that draw every frame should read
 * `ref.current` in their draw loop instead and avoid the re-render.
 */
export function useZoomPan(min = 1, max = 6) {
  const [transform, setTransform] = useState<Transform>(IDENTITY)
  const ref = useRef(transform)
  ref.current = transform

  const drag = useRef<{ px: number; py: number; ox: number; oy: number } | null>(null)
  const pinch = useRef<number | null>(null)

  const clamp = useCallback((s: number) => Math.min(max, Math.max(min, s)), [min, max])

  const zoomAbout = useCallback(
    (factor: number, clientX: number, clientY: number, rect: DOMRect) => {
      setTransform((t) => {
        const ns = clamp(t.scale * factor)
        if (ns === t.scale) return t
        // Keep the point under the cursor fixed while scaling.
        const px = clientX - rect.left - rect.width / 2
        const py = clientY - rect.top - rect.height / 2
        const k = ns / t.scale
        return { scale: ns, x: px - (px - t.x) * k, y: py - (py - t.y) * k }
      })
    },
    [clamp],
  )

  const onWheel = useCallback(
    (e: React.WheelEvent) => {
      e.preventDefault()
      const rect = e.currentTarget.getBoundingClientRect()
      zoomAbout(Math.exp(-e.deltaY * 0.0015), e.clientX, e.clientY, rect)
    },
    [zoomAbout],
  )

  const onPointerDown = useCallback((e: React.PointerEvent) => {
    if (e.button !== 0) return
    try { (e.currentTarget as HTMLElement).setPointerCapture(e.pointerId) } catch { /* */ }
    drag.current = { px: e.clientX, py: e.clientY, ox: ref.current.x, oy: ref.current.y }
  }, [])

  const onPointerMove = useCallback((e: React.PointerEvent) => {
    const d = drag.current
    if (!d) return
    const nx = d.ox + (e.clientX - d.px)
    const ny = d.oy + (e.clientY - d.py)
    setTransform((t) => (t.x === nx && t.y === ny ? t : { ...t, x: nx, y: ny }))
  }, [])

  const onPointerUp = useCallback((e: React.PointerEvent) => {
    drag.current = null
    try { (e.currentTarget as HTMLElement).releasePointerCapture(e.pointerId) } catch { /* */ }
  }, [])

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

  return { transform, ref, bind, zoomBy, reset, setTransform, pinch }
}

/**
 * CSS `transform` string. With `transform-origin: center`, the point under the
 * pointer stays fixed because zoomAbout keeps (x, y) as the parent-space offset:
 * a point p maps to `scale * p + (x, y)`, i.e. `translate(x,y) scale(s)`.
 */
export function toCss({ scale, x, y }: Transform): string {
  return `translate(${x}px, ${y}px) scale(${scale})`
}
