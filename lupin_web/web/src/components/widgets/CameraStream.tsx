import { Camera, Maximize2, Minimize2, RefreshCcw } from 'lucide-react'
import { useEffect, useRef, useState } from 'react'

import { Button } from '@/components/ui/button'
import { isMockMode } from '@/lib/settings'
import { cn } from '@/lib/utils'

interface CameraStreamProps {
  topic: string
  baseUrl: string
  className?: string
  /** Display label, e.g. "RGB". */
  label?: string
}

export function CameraStream({ topic, baseUrl, className, label }: CameraStreamProps) {
  const mock = isMockMode()
  const [errored, setErrored] = useState(false)
  const [reloadKey, setReloadKey] = useState(0)
  const [fps, setFps] = useState<number | null>(null)
  const [fullscreen, setFullscreen] = useState(false)

  const wrapRef = useRef<HTMLDivElement | null>(null)
  const frameTimes = useRef<number[]>([])

  useEffect(() => {
    setErrored(false)
  }, [topic, baseUrl, reloadKey])

  // Mock FPS: jiggle around 24
  useEffect(() => {
    if (!mock) return
    const id = setInterval(() => setFps(Math.round(22 + Math.random() * 4)), 1000)
    return () => clearInterval(id)
  }, [mock])

  const url = `${baseUrl.replace(/\/$/, '')}/stream?topic=${encodeURIComponent(topic)}&type=mjpeg`

  const onLoad = () => {
    const now = performance.now()
    frameTimes.current.push(now)
    while (frameTimes.current.length > 30) frameTimes.current.shift()
    if (frameTimes.current.length >= 2) {
      const span = frameTimes.current[frameTimes.current.length - 1] - frameTimes.current[0]
      const f = ((frameTimes.current.length - 1) / span) * 1000
      setFps(Math.max(0, Math.min(120, Math.round(f))))
    }
  }

  const toggleFullscreen = async () => {
    const el = wrapRef.current
    if (!el) return
    try {
      if (!document.fullscreenElement) {
        await el.requestFullscreen()
        setFullscreen(true)
      } else {
        await document.exitFullscreen()
        setFullscreen(false)
      }
    } catch {
      /* fullscreen permission denied — ignore */
    }
  }

  return (
    <div
      ref={wrapRef}
      className={cn(
        'relative flex flex-col overflow-hidden rounded-md border bg-black',
        fullscreen && 'h-screen w-screen',
        className,
      )}
    >
      <div className="absolute left-2 top-2 z-10 flex items-center gap-2 rounded-md bg-black/60 px-2 py-1 text-xs text-white">
        <Camera className="h-3.5 w-3.5" />
        {label ? <span className="font-semibold">{label}</span> : null}
        <span className="font-mono opacity-80">{topic}</span>
        {fps != null ? <span className="font-mono opacity-80">{fps} fps</span> : null}
      </div>
      <div className="absolute right-2 top-2 z-10 flex gap-1">
        <Button
          variant="ghost"
          size="icon"
          onClick={() => setReloadKey((k) => k + 1)}
          aria-label="Reload"
          className="h-8 w-8 bg-black/60 text-white hover:bg-black/80"
        >
          <RefreshCcw className="h-4 w-4" />
        </Button>
        <Button
          variant="ghost"
          size="icon"
          onClick={toggleFullscreen}
          aria-label="Toggle fullscreen"
          className="h-8 w-8 bg-black/60 text-white hover:bg-black/80"
        >
          {fullscreen ? <Minimize2 className="h-4 w-4" /> : <Maximize2 className="h-4 w-4" />}
        </Button>
      </div>

      <div className="grid flex-1 place-items-center">
        {mock ? (
          <MockCamera />
        ) : errored ? (
          <NoStream topic={topic} url={url} onRetry={() => setReloadKey((k) => k + 1)} />
        ) : (
          <img
            key={reloadKey}
            src={url}
            alt={`Stream ${topic}`}
            onError={() => setErrored(true)}
            onLoad={onLoad}
            className="h-full w-full object-contain"
          />
        )}
      </div>
    </div>
  )
}

function MockCamera() {
  // animated gradient + grid that "moves" with time
  const t = (Date.now() / 50) % 360
  return (
    <div className="relative h-full w-full overflow-hidden">
      <div
        className="absolute inset-0 opacity-60"
        style={{
          background: `linear-gradient(${t}deg, #14532d, #0c4a6e)`,
        }}
      />
      <div
        className="absolute inset-0 opacity-30"
        style={{
          backgroundImage:
            'linear-gradient(rgba(255,255,255,0.15) 1px, transparent 1px), linear-gradient(90deg, rgba(255,255,255,0.15) 1px, transparent 1px)',
          backgroundSize: '24px 24px',
        }}
      />
      <div className="absolute inset-0 grid place-items-center text-white/80">
        <div className="rounded-md border border-white/30 bg-black/40 px-4 py-2 text-sm">
          MOCK CAMERA · {new Date().toLocaleTimeString()}
        </div>
      </div>
    </div>
  )
}

function NoStream({ topic, url, onRetry }: { topic: string; url: string; onRetry: () => void }) {
  return (
    <div className="m-6 flex max-w-md flex-col items-center gap-3 rounded-md border border-dashed border-white/20 p-6 text-center text-white/80">
      <Camera className="h-8 w-8 opacity-60" />
      <div className="text-sm">No stream from <span className="font-mono">{topic}</span></div>
      <div className="text-[11px] text-white/50">
        Tried <span className="font-mono">{url}</span>. The Mirte's <code>web_video_server</code> binds to
        localhost only by default — you may need a proxy to reach it from outside the robot.
      </div>
      <Button variant="secondary" size="sm" onClick={onRetry}>
        <RefreshCcw className="mr-2 h-3.5 w-3.5" /> Retry
      </Button>
    </div>
  )
}
