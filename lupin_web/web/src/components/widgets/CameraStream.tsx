// import { Camera, Maximize2, Minimize2, RefreshCcw } from 'lucide-react'
// import { useEffect, useRef, useState } from 'react'

// import { Button } from '@/components/ui/button'
// import { isMockMode } from '@/lib/settings'
// import { cn } from '@/lib/utils'

// import { useTopic } from '@/lib/ros'
// import { StdMsgsString, TagDetection } from '@/types/ros'
// import { Scan } from 'lucide-react'

// interface CameraStreamProps {
//   topic: string
//   baseUrl: string
//   className?: string
//   /** Display label, e.g. "RGB". */
//   label?: string
// }

// export function CameraStream({ topic, baseUrl, className, label }: CameraStreamProps) {
//   const mock = isMockMode()
//   const [errored, setErrored] = useState(false)
//   const [reloadKey, setReloadKey] = useState(0)
//   const [fps, setFps] = useState<number | null>(null)
//   const [fullscreen, setFullscreen] = useState(false)

//   const [showBoxes, setShowBoxes] = useState(false)
//   const canvasRef = useRef<HTMLCanvasElement | null>(null)
  
//   // Listen to the JSON topic
//   const tagMessage = useTopic<StdMsgsString>(
//     '/camera/tag_detections_json',
//     'std_msgs/String'
//   )
//   const wrapRef = useRef<HTMLDivElement | null>(null)
//   const frameTimes = useRef<number[]>([])

//   useEffect(() => {
//     setErrored(false)
//   }, [topic, baseUrl, reloadKey])

//   // Mock FPS: jiggle around 24
//   useEffect(() => {
//     if (!mock) return
//     const id = setInterval(() => setFps(Math.round(22 + Math.random() * 4)), 1000)
//     return () => clearInterval(id)
//   }, [mock])

//   // web_video_server takes the topic as a literal query-string value and
//   // (on the version we ship in Humble) does NOT URL-decode it before
//   // looking it up. encodeURIComponent escapes the leading `/` to `%2F`,
//   // which then gets rejected as "Invalid topic name". Slashes are
//   // permitted in the query component per RFC 3986 §3.4, so just send the
//   // raw topic. Whitespace shouldn't appear in valid ROS topic names but
//   // we still escape spaces defensively.
//   const url = `${baseUrl.replace(/\/$/, '')}/stream?topic=${topic.replace(/ /g, '%20')}&type=mjpeg`

//   const onLoad = () => {
//     const now = performance.now()
//     frameTimes.current.push(now)
//     while (frameTimes.current.length > 30) frameTimes.current.shift()
//     if (frameTimes.current.length >= 2) {
//       const span = frameTimes.current[frameTimes.current.length - 1] - frameTimes.current[0]
//       const f = ((frameTimes.current.length - 1) / span) * 1000
//       setFps(Math.max(0, Math.min(120, Math.round(f))))
//     }
//   }

//   const toggleFullscreen = async () => {
//     const el = wrapRef.current
//     if (!el) return
//     try {
//       if (!document.fullscreenElement) {
//         await el.requestFullscreen()
//         setFullscreen(true)
//       } else {
//         await document.exitFullscreen()
//         setFullscreen(false)
//       }
//     } catch {
//       /* fullscreen permission denied — ignore */
//     }
//   }

//   useEffect(() => {
//     // 1. If boxes are turned off, wipe the canvas and do nothing else
//     if (!showBoxes) {
//       const canvas = canvasRef.current
//       if (canvas) canvas.getContext('2d')?.clearRect(0, 0, canvas.width, canvas.height)
//       return
//     }

//     let animationId: number

//     // 2. Create a fast loop that runs outside of React's render cycle
//     const renderLoop = () => {
//       const canvas = canvasRef.current
//       if (!canvas) return
//       const ctx = canvas.getContext('2d')
//       if (!ctx) return

//       // Clear the previous frame
//       ctx.clearRect(0, 0, canvas.width, canvas.height)

//       // 3. READ FROM THE REF (.current)
//       const currentData = tagMessage.current?.data

//       if (currentData) {
//         try {
//           const detections: TagDetection[] = JSON.parse(currentData)
//           ctx.lineWidth = 6
//           ctx.font = "bold 40px monospace"

//           detections.forEach(tag => {
//             // Draw green box
//             ctx.beginPath()
//             ctx.moveTo(tag.corners[0][0], tag.corners[0][1])
//             ctx.lineTo(tag.corners[1][0], tag.corners[1][1])
//             ctx.lineTo(tag.corners[2][0], tag.corners[2][1])
//             ctx.lineTo(tag.corners[3][0], tag.corners[3][1])
//             ctx.closePath()
//             ctx.strokeStyle = "#00FF00"
//             ctx.stroke()

//             // Draw text background and label
//             const text = `ID: ${tag.id} | ${tag.dist.toFixed(2)}m`
//             const textX = tag.corners[0][0]
//             const textY = tag.corners[0][1] - 15

//             ctx.fillStyle = "rgba(0, 0, 0, 0.7)"
//             const textMetrics = ctx.measureText(text)
//             ctx.fillRect(textX, textY - 40, textMetrics.width + 10, 50)

//             ctx.fillStyle = "#FF3333"
//             ctx.fillText(text, textX + 5, textY)
//           })
//         } catch (err) {
//           // Silently ignore incomplete JSON packets so the video doesn't stutter
//         }
//       }

//       // 4. Ask the browser to run this loop again on the next video frame
//       animationId = requestAnimationFrame(renderLoop)
//     }

//     // Start the loop!
//     renderLoop()

//     // 5. Clean up the loop when the user turns the boxes off or leaves the page
//     return () => cancelAnimationFrame(animationId)
//   }, [showBoxes, tagMessage])

//   return (
//     <div
//       ref={wrapRef}
//       className={cn(
//         'relative flex flex-col overflow-hidden rounded-md border bg-black',
//         fullscreen && 'h-screen w-screen',
//         className,
//       )}
//     >
//       <div className="absolute left-2 top-2 z-10 flex items-center gap-2 rounded-md bg-black/60 px-2 py-1 text-xs text-white">
//         <Camera className="h-3.5 w-3.5" />
//         {label ? <span className="font-semibold">{label}</span> : null}
//         <span className="font-mono opacity-80">{topic}</span>
//         {fps != null ? <span className="font-mono opacity-80">{fps} fps</span> : null}
//       </div>
//       <div className="absolute right-2 top-2 z-10 flex gap-1">

//         <Button
//           variant="ghost"
//           size="icon"
//           onClick={() => setShowBoxes(!showBoxes)}
//           aria-label="Toggle AprilTags"
//           className={cn(
//             "h-8 w-8 text-white transition-colors", 
//             showBoxes ? "bg-red-600/80 hover:bg-red-600" : "bg-black/60 hover:bg-black/80"
//           )}
//         >
//           <Scan className="h-4 w-4" />
//         </Button>

//         <Button
//           variant="ghost"
//           size="icon"
//           onClick={() => setReloadKey((k) => k + 1)}
//           aria-label="Reload"
//           className="h-8 w-8 bg-black/60 text-white hover:bg-black/80"
//         >
//           <RefreshCcw className="h-4 w-4" />
//         </Button>
//         <Button
//           variant="ghost"
//           size="icon"
//           onClick={toggleFullscreen}
//           aria-label="Toggle fullscreen"
//           className="h-8 w-8 bg-black/60 text-white hover:bg-black/80"
//         >
//           {fullscreen ? <Minimize2 className="h-4 w-4" /> : <Maximize2 className="h-4 w-4" />}
//         </Button>
//       </div>

//       <div className="grid flex-1 place-items-center">
//         {mock ? (
//           <MockCamera />
//         ) : errored ? (
//           <NoStream topic={topic} url={url} onRetry={() => setReloadKey((k) => k + 1)} />
//         ) : (
//           <div className="relative h-full w-full">
//             <img
//               key={reloadKey}
//               src={url}
//               alt={`Stream ${topic}`}
//               onError={() => setErrored(true)}
//               onLoad={onLoad}
//               className="absolute inset-0 h-full w-full object-contain"
//             />
//             {/* The transparent drawing layer. Change 1920x1080 to match your Gazebo camera resolution if it differs! */}
//             <canvas
//               ref={canvasRef}
//               width={1920}
//               height={1080}
//               className="absolute inset-0 h-full w-full object-contain pointer-events-none"
//             />
//           </div>
//         )}
//       </div>
//     </div>
//   )
// }

// function MockCamera() {
//   // animated gradient + grid that "moves" with time
//   const t = (Date.now() / 50) % 360
//   return (
//     <div className="relative h-full w-full overflow-hidden">
//       <div
//         className="absolute inset-0 opacity-60"
//         style={{
//           background: `linear-gradient(${t}deg, #14532d, #0c4a6e)`,
//         }}
//       />
//       <div
//         className="absolute inset-0 opacity-30"
//         style={{
//           backgroundImage:
//             'linear-gradient(rgba(255,255,255,0.15) 1px, transparent 1px), linear-gradient(90deg, rgba(255,255,255,0.15) 1px, transparent 1px)',
//           backgroundSize: '24px 24px',
//         }}
//       />
//       <div className="absolute inset-0 grid place-items-center text-white/80">
//         <div className="rounded-md border border-white/30 bg-black/40 px-4 py-2 text-sm">
//           MOCK CAMERA · {new Date().toLocaleTimeString()}
//         </div>
//       </div>
//     </div>
//   )
// }

// function NoStream({ topic, url, onRetry }: { topic: string; url: string; onRetry: () => void }) {
//   return (
//     <div className="m-6 flex max-w-md flex-col items-center gap-3 rounded-md border border-dashed border-white/20 p-6 text-center text-white/80">
//       <Camera className="h-8 w-8 opacity-60" />
//       <div className="text-sm">No stream from <span className="font-mono">{topic}</span></div>
//       <div className="text-[11px] text-white/50">
//         Tried <span className="font-mono">{url}</span>. The Mirte's <code>web_video_server</code> binds to
//         localhost only by default — you may need a proxy to reach it from outside the robot.
//       </div>
//       <Button variant="secondary" size="sm" onClick={onRetry}>
//         <RefreshCcw className="mr-2 h-3.5 w-3.5" /> Retry
//       </Button>
//     </div>
//   )
// }
import { Camera, Minus, Plus, RefreshCcw, RotateCcw } from 'lucide-react'
import { useEffect, useRef, useState, type ReactNode } from 'react'

import { useFocusPanel } from '@/components/system/FocusPanel'
import { Button } from '@/components/ui/button'
import { ExpandButton } from '@/components/ui/ExpandButton'
import { isMockMode } from '@/lib/settings'
import { cn } from '@/lib/utils'
import { toCss, useZoomPan } from '@/lib/zoompan'

import { useTopic } from '@/lib/ros'
import { StdMsgsString, TagDetection, YoloDetection } from '@/types/ros'

interface CameraStreamProps {
  topic: string
  baseUrl: string
  className?: string
  label?: string
  /** When true, render the AprilTag overlay boxes. */
  showBoxes?: boolean
  /** When true, render YOLO detections from /yolo/detections. */
  showYoloBoxes?: boolean
  /** When true, enable wheel/drag zoom-pan + show zoom controls (maximized variant). */
  interactive?: boolean
}

/** Small instrument-chrome icon button used for the stream's overlay controls. */
function StreamIconButton({ onClick, label, children }: { onClick: () => void; label: string; children: ReactNode }) {
  return (
    <button
      type="button"
      onClick={onClick}
      aria-label={label}
      title={label}
      className="inline-flex h-8 w-8 items-center justify-center rounded-sm border border-hairline bg-ink-0/70 text-muted-foreground backdrop-blur-sm transition-colors hover:border-primary/40 hover:bg-ink-3 hover:text-foreground"
    >
      {children}
    </button>
  )
}

export function CameraStream({
  topic,
  baseUrl,
  className,
  label,
  showBoxes = false,
  showYoloBoxes = false,
  interactive = false,
}: CameraStreamProps) {
  const mock = isMockMode()
  const focus = useFocusPanel()
  const { transform, bind, zoomBy, reset } = useZoomPan(1, 6)
  const [errored, setErrored] = useState(false)
  const [reloadKey, setReloadKey] = useState(0)
  const [fps, setFps] = useState<number | null>(null)
  // Overlay canvas size derived from the real frame so AprilTag corners (in
  // stream pixel coords) stay registered at any resolution — not hardcoded.
  const [dims, setDims] = useState<{ w: number; h: number }>({ w: 640, h: 480 })

  const canvasRef = useRef<HTMLCanvasElement | null>(null)

  const tagMessage = useTopic<StdMsgsString>(
    '/camera/tag_detections_json',
    'std_msgs/String'
  )
  const yoloMessage = useTopic<StdMsgsString>(
    '/yolo/detections',
    'std_msgs/String'
  )

  const wrapRef = useRef<HTMLDivElement | null>(null)
  const frameTimes = useRef<number[]>([])

  useEffect(() => {
    setErrored(false)
  }, [topic, baseUrl, reloadKey])

  useEffect(() => {
    if (!mock) return
    const id = setInterval(() => setFps(Math.round(22 + Math.random() * 4)), 1000)
    return () => clearInterval(id)
  }, [mock])

  const url = `${baseUrl.replace(/\/$/, '')}/stream?topic=${topic.replace(/ /g, '%20')}&type=mjpeg`

  const onLoad = (e: React.SyntheticEvent<HTMLImageElement>) => {
    const img = e.currentTarget
    if (img.naturalWidth && (img.naturalWidth !== dims.w || img.naturalHeight !== dims.h)) {
      setDims({ w: img.naturalWidth, h: img.naturalHeight })
    }
    const now = performance.now()
    frameTimes.current.push(now)
    while (frameTimes.current.length > 30) frameTimes.current.shift()
    if (frameTimes.current.length >= 2) {
      const span = frameTimes.current[frameTimes.current.length - 1] - frameTimes.current[0]
      const f = ((frameTimes.current.length - 1) / span) * 1000
      setFps(Math.max(0, Math.min(120, Math.round(f))))
    }
  }

  // Open the same stream maximized in the shared in-app FocusPanel (stays in the
  // app shell so E-stop / Take-Control remain visible — unlike native fullscreen).
  const openMaximized = () =>
    focus.open({
      title: label ? `Camera · ${label}` : 'Camera',
      subtitle: topic,
      render: () => (
        <CameraStream
          topic={topic}
          baseUrl={baseUrl}
          label={label}
          showBoxes={showBoxes}
          showYoloBoxes={showYoloBoxes}
          interactive
          className="h-full w-full rounded-none border-0"
        />
      ),
    })

// --- The High-Performance Render Loop ---
  useEffect(() => {
    if (!showBoxes && !showYoloBoxes) {
      const canvas = canvasRef.current
      if (canvas) canvas.getContext('2d')?.clearRect(0, 0, canvas.width, canvas.height)
      return
    }

    let animationId: number

    const renderLoop = () => {
      const canvas = canvasRef.current
      if (!canvas) return
      const ctx = canvas.getContext('2d')
      if (!ctx) return

      ctx.clearRect(0, 0, canvas.width, canvas.height)

      if (showBoxes) {
        const currentData = tagMessage.current?.data
        if (currentData) {
          try {
            const detections: TagDetection[] = JSON.parse(currentData)
          
            ctx.lineWidth = 2 // Thinner box lines
            ctx.font = "bold 10px monospace" // Scaled down to ~0.6
            ctx.textBaseline = "top"

            detections.forEach(tag => {
              // NEW: Put a safety net INSIDE the loop.
              // If one tag fails, the others still draw perfectly!
              try {
                // 1. Draw the Green Box
                ctx.beginPath()
                ctx.moveTo(tag.corners[0][0], tag.corners[0][1])
                ctx.lineTo(tag.corners[1][0], tag.corners[1][1])
                ctx.lineTo(tag.corners[2][0], tag.corners[2][1])
                ctx.lineTo(tag.corners[3][0], tag.corners[3][1])
                ctx.closePath()
                ctx.strokeStyle = "#00FF00"
                ctx.stroke()

                // 2. Format the Text Safely (Force it to be a Number so it never crashes)
                const safeDist = Number(tag.dist) || 0
                const text = `ID: ${tag.id} | ${safeDist.toFixed(2)}m`

                // 3. Calculate Safe Positions
                const textX = tag.corners[0][0]
                let textY = tag.corners[0][1] - 16 // Try to put it 25px above the tag

                // If the tag is too close to the top of the video, push the text BELOW the tag!
                if (textY < 0) {
                  textY = tag.corners[0][1] + 6
                }

                // 4. Draw the dark background block
                ctx.fillStyle = "rgba(0, 0, 0, 0.8)"
                const textMetrics = ctx.measureText(text)
                ctx.fillRect(textX - 4, textY - 4, textMetrics.width + 8, 24)

                // 5. Draw the Red text
                ctx.fillStyle = "#FF3333"
                ctx.fillText(text, textX, textY)
              } catch (innerErr) {
                // Silently ignore a corrupted tag, but keep the loop alive!
              }
            })
          } catch (err) {
            // Ignore JSON parse errors from incomplete packets
          }
        }
      }

      if (showYoloBoxes) {
        const currentData = yoloMessage.current?.data
        if (currentData) {
          try {
            const detections: YoloDetection[] = JSON.parse(currentData)

            ctx.lineWidth = 2
            ctx.font = "bold 11px monospace"
            ctx.textBaseline = "top"

            detections.forEach(det => {
              try {
                if (!Array.isArray(det.bbox_xyxy) || det.bbox_xyxy.length !== 4) return
                const [x1, y1, x2, y2] = det.bbox_xyxy.map(Number)
                if (![x1, y1, x2, y2].every(Number.isFinite)) return

                const w = Math.max(0, x2 - x1)
                const h = Math.max(0, y2 - y1)
                const name = det.class_name || `class ${det.class}`
                const conf = Number(det.confidence) || 0
                const track = det.track_id == null ? '' : ` #${det.track_id}`
                const text = `${name}${track} ${(conf * 100).toFixed(0)}%`

                ctx.strokeStyle = "#38BDF8"
                ctx.strokeRect(x1, y1, w, h)

                const textMetrics = ctx.measureText(text)
                const textY = y1 < 20 ? y1 + 4 : y1 - 18
                ctx.fillStyle = "rgba(0, 0, 0, 0.8)"
                ctx.fillRect(x1 - 3, textY - 3, textMetrics.width + 8, 20)
                ctx.fillStyle = "#E0F2FE"
                ctx.fillText(text, x1 + 1, textY)
              } catch {
                // Keep one malformed detection from killing the overlay loop.
              }
            })
          } catch {
            // Ignore JSON parse errors from incomplete packets.
          }
        }
      }
      animationId = requestAnimationFrame(renderLoop)
    }

    renderLoop()
    return () => cancelAnimationFrame(animationId)
  }, [showBoxes, showYoloBoxes, tagMessage, yoloMessage])

  return (
    <div
      ref={wrapRef}
      className={cn(
        'edge-light relative flex flex-col overflow-hidden rounded-sm border border-hairline bg-ink-0',
        className,
      )}
    >
      <div className="absolute left-2 top-2 z-10 flex items-center gap-2 rounded-sm border border-hairline bg-ink-0/70 px-2 py-1 text-xs backdrop-blur-sm">
        <Camera className="h-3.5 w-3.5 text-primary" />
        {label ? <span className="font-semibold text-foreground">{label}</span> : null}
        <span className="font-mono text-muted-foreground">{topic}</span>
        {fps != null ? <span className="ticker text-muted-foreground">{fps} fps</span> : null}
      </div>
      <div className="absolute right-2 top-2 z-10 flex items-center gap-1">
        <StreamIconButton onClick={() => setReloadKey((k) => k + 1)} label="Reload">
          <RefreshCcw className="h-4 w-4" />
        </StreamIconButton>
        {interactive ? (
          <>
            <StreamIconButton onClick={() => zoomBy(1 / 1.4)} label="Zoom out"><Minus className="h-4 w-4" /></StreamIconButton>
            <StreamIconButton onClick={() => zoomBy(1.4)} label="Zoom in"><Plus className="h-4 w-4" /></StreamIconButton>
            <StreamIconButton onClick={reset} label="Reset view"><RotateCcw className="h-4 w-4" /></StreamIconButton>
          </>
        ) : (
          <ExpandButton onClick={openMaximized} />
        )}
      </div>

      <div className="grid flex-1 place-items-center overflow-hidden">
        {mock ? (
          <MockCamera />
        ) : errored ? (
          <NoStream topic={topic} url={url} onRetry={() => setReloadKey((k) => k + 1)} />
        ) : (
          <div
            className={cn(
              'relative grid h-full w-full place-items-center',
              interactive && 'cursor-grab touch-none active:cursor-grabbing',
            )}
            {...(interactive ? bind : {})}
          >
            {/* img + overlay-canvas share one transformed box so AprilTag boxes
                stay registered while zooming/panning. Both are absolutely
                positioned: an in-flow <img> with h-full/w-full feeds its
                aspect-transferred intrinsic height into the auto grid track,
                inflating it past the (overflow-hidden) panel and clipping the
                frame. Absolute positioning keeps the image from driving layout,
                so object-contain letterboxes the full frame as intended. */}
            <div
              className="relative h-full w-full"
              style={interactive ? { transform: toCss(transform), transformOrigin: 'center', willChange: 'transform' } : undefined}
            >
              <img
                key={reloadKey}
                src={url}
                alt={`Stream ${topic}`}
                onError={() => setErrored(true)}
                onLoad={onLoad}
                draggable={false}
                className="absolute inset-0 h-full w-full select-none object-contain"
              />
              <canvas
                ref={canvasRef}
                width={dims.w}
                height={dims.h}
                className="absolute inset-0 h-full w-full object-contain pointer-events-none"
              />
            </div>
          </div>
        )}
      </div>
    </div>
  )
}

function MockCamera() {
  const t = (Date.now() / 50) % 360
  return (
    <div className="relative h-full w-full overflow-hidden">
      <div className="absolute inset-0 opacity-60" style={{ background: `linear-gradient(${t}deg, #14532d, #0c4a6e)` }} />
      <div className="absolute inset-0 opacity-30" style={{ backgroundImage: 'linear-gradient(rgba(255,255,255,0.15) 1px, transparent 1px), linear-gradient(90deg, rgba(255,255,255,0.15) 1px, transparent 1px)', backgroundSize: '24px 24px' }} />
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
        localhost only by default.
      </div>
      <Button variant="secondary" size="sm" onClick={onRetry}>
        <RefreshCcw className="mr-2 h-3.5 w-3.5" /> Retry
      </Button>
    </div>
  )
}
