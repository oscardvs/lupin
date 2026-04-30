import { useEffect, useRef } from 'react'

interface SparklineProps {
  values: number[]
  min?: number
  max?: number
  width?: number
  height?: number
  color?: string
  className?: string
}

export function Sparkline({
  values,
  min,
  max,
  width = 200,
  height = 40,
  color = 'currentColor',
  className,
}: SparklineProps) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const dpr = window.devicePixelRatio || 1
    canvas.width = width * dpr
    canvas.height = height * dpr
    const ctx = canvas.getContext('2d')
    if (!ctx) return
    ctx.scale(dpr, dpr)
    ctx.clearRect(0, 0, width, height)
    if (values.length < 2) return
    const lo = min ?? Math.min(...values)
    const hi = max ?? Math.max(...values)
    const span = Math.max(1e-6, hi - lo)
    ctx.strokeStyle = color
    ctx.lineWidth = 1.5
    ctx.lineJoin = 'round'
    ctx.beginPath()
    values.forEach((v, i) => {
      const x = (i / (values.length - 1)) * width
      const y = height - ((v - lo) / span) * height
      if (i === 0) ctx.moveTo(x, y)
      else ctx.lineTo(x, y)
    })
    ctx.stroke()
  }, [values, min, max, width, height, color])

  return <canvas ref={canvasRef} style={{ width, height }} className={className} />
}
