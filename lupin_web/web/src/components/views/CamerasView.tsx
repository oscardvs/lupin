// import { Camera, RefreshCcw } from 'lucide-react'
// import { useCallback, useEffect, useMemo, useState } from 'react'

// import { Button } from '@/components/ui/button'
// import { CameraStream } from '@/components/widgets/CameraStream'
// import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
// import { isMockMode, useSettings } from '@/lib/settings'

// interface DiscoveredCamera {
//   /** ROS topic, e.g. `/camera/image_raw`. */
//   topic: string
//   /** Short label shown on the tab. Derived from the topic (or pinned). */
//   label: string
// }

// // Friendly labels for the topics we expect on the MIRTE stack. Anything not in
// // this map gets a label derived from the topic's last segment (e.g.
// // `/foo/image_raw` → `image_raw`).
// const PINNED_LABELS: Record<string, string> = {
//   '/camera/image_raw': 'RGB',
//   '/camera/color/image_raw': 'RGB',
//   '/camera/depth/image_raw': 'Depth',
//   '/camera/gripper/image_raw': 'Gripper',
//   '/camera/ir/image_raw': 'IR',
// }

// function deriveLabel(topic: string): string {
//   if (PINNED_LABELS[topic]) return PINNED_LABELS[topic]
//   const tail = topic.split('/').filter(Boolean).slice(-2).join('/')
//   return tail || topic
// }

// /**
//  * Scrape web_video_server's index page for the list of streamable topics.
//  * The endpoint returns an HTML <ul> with links of the form
//  * `/stream?topic=<topic>` — the cheapest way to enumerate without adding a
//  * second discovery channel. Returns an empty list on any error so the UI
//  * collapses to a "no streams" state instead of throwing.
//  *
//  * Tries the Vite same-origin proxy at `/_video/` first (so we sidestep
//  * web_video_server's missing CORS headers). Falls back to a direct fetch on
//  * the configured baseUrl — useful when running the built bundle from a
//  * different host or when the proxy isn't wired in.
//  */
// async function fetchAvailableStreams(baseUrl: string): Promise<string[]> {
//   const parse = (html: string): string[] => {
//     const matches = html.matchAll(/href="\/stream\?topic=([^"&]+)/g)
//     const out = new Set<string>()
//     for (const m of matches) {
//       try { out.add(decodeURIComponent(m[1])) } catch { out.add(m[1]) }
//     }
//     return Array.from(out).sort()
//   }

//   // 1. Same-origin proxy — bypasses CORS.
//   try {
//     const res = await fetch('/_video/', { cache: 'no-store' })
//     if (res.ok) {
//       const ct = res.headers.get('content-type') || ''
//       if (ct.includes('text/html')) return parse(await res.text())
//     }
//   } catch { /* fall through to direct fetch */ }

//   // 2. Direct fetch as a fallback.
//   if (!baseUrl) return []
//   const root = baseUrl.replace(/\/$/, '') + '/'
//   const res = await fetch(root, { cache: 'no-store' })
//   if (!res.ok) throw new Error(`web_video_server responded ${res.status}`)
//   return parse(await res.text())
// }

// export function CamerasView() {
//   const [{ cameraTopic, webVideoServerUrl }] = useSettings()
//   const mock = isMockMode()
//   const [discovered, setDiscovered] = useState<string[]>([])
//   const [discoveryError, setDiscoveryError] = useState<string | null>(null)
//   const [discovering, setDiscovering] = useState(false)
//   const [refreshKey, setRefreshKey] = useState(0)

//   const refresh = useCallback(() => setRefreshKey((k) => k + 1), [])

//   useEffect(() => {
//     if (mock) return
//     let cancelled = false
//     setDiscovering(true)
//     setDiscoveryError(null)
//     fetchAvailableStreams(webVideoServerUrl)
//       .then((topics) => {
//         if (cancelled) return
//         setDiscovered(topics)
//       })
//       .catch((err: unknown) => {
//         if (cancelled) return
//         const msg = err instanceof Error ? err.message : String(err)
//         setDiscoveryError(msg)
//         setDiscovered([])
//       })
//       .finally(() => {
//         if (!cancelled) setDiscovering(false)
//       })
//     return () => { cancelled = true }
//   }, [webVideoServerUrl, refreshKey, mock])

//   const discoverySucceeded = !discoveryError && !discovering
//   const configuredTopicMissing =
//     !mock && discoverySucceeded && discovered.length > 0 &&
//     !!cameraTopic && !discovered.includes(cameraTopic)

//   const cameras = useMemo<DiscoveredCamera[]>(() => {
//     if (mock) {
//       // Keep mock mode honest — show a single fake RGB tab.
//       return [{ topic: cameraTopic || '/camera/image_raw', label: 'RGB' }]
//     }
//     const seen = new Set<string>()
//     const out: DiscoveredCamera[] = []
//     // Show the configured cameraTopic first IF discovery hasn't proven it
//     // wrong (either discovery hasn't succeeded yet, or the topic is in the
//     // discovered list). Otherwise fall through to discovered-only so a stale
//     // localStorage value doesn't permanently render a broken tab.
//     if (cameraTopic && !configuredTopicMissing) {
//       out.push({ topic: cameraTopic, label: deriveLabel(cameraTopic) })
//       seen.add(cameraTopic)
//     }
//     for (const topic of discovered) {
//       if (seen.has(topic)) continue
//       out.push({ topic, label: deriveLabel(topic) })
//       seen.add(topic)
//     }
//     return out
//   }, [cameraTopic, configuredTopicMissing, discovered, mock])

//   const defaultValue = cameras[0]?.topic ?? ''

//   return (
//     <div className="flex min-h-full w-full flex-col gap-3 p-3 sm:p-4">
//       <div className="flex flex-wrap items-center gap-2">
//         <span className="tag tag-strong">Streams</span>
//         <span className="tag font-mono">{webVideoServerUrl || '—'}</span>
//         {mock ? (
//           <span className="tag tag-accent">mock</span>
//         ) : discovering ? (
//           <span className="tag">discovering…</span>
//         ) : discoveryError ? (
//           <span className="tag text-destructive">unreachable · {discoveryError}</span>
//         ) : (
//           <span className="tag">{discovered.length} discovered</span>
//         )}
//         <Button
//           variant="outline"
//           size="sm"
//           onClick={refresh}
//           className="ml-auto h-7 text-[11px]"
//         >
//           <RefreshCcw className="mr-1.5 h-3 w-3" /> Rescan
//         </Button>
//       </div>

//       {configuredTopicMissing && (
//         <div className="rounded-md border border-amber-500/40 bg-amber-500/10 px-3 py-2 text-[11px] text-amber-200">
//           Configured camera topic <span className="font-mono">{cameraTopic}</span> isn't being
//           published — falling back to discovered streams. Update or clear it under Settings → Topics.
//         </div>
//       )}

//       {cameras.length === 0 ? (
//         <NoCameras hasError={!!discoveryError} />
//       ) : (
//         <Tabs key={defaultValue} defaultValue={defaultValue} className="flex flex-1 flex-col">
//           <TabsList className="self-start">
//             {cameras.map((c) => (
//               <TabsTrigger key={c.topic} value={c.topic} title={c.topic}>
//                 {c.label}
//               </TabsTrigger>
//             ))}
//           </TabsList>
//           {cameras.map((c) => (
//             <TabsContent key={c.topic} value={c.topic} className="m-0 mt-2 flex flex-1">
//               <CameraStream
//                 label={c.label}
//                 topic={c.topic}
//                 baseUrl={webVideoServerUrl}
//                 className="flex-1 min-h-[24rem]"
//               />
//             </TabsContent>
//           ))}
//         </Tabs>
//       )}
//     </div>
//   )
// }

// function NoCameras({ hasError }: { hasError: boolean }) {
//   return (
//     <div className="m-6 flex max-w-md flex-col items-center gap-3 self-center rounded-md border border-dashed border-hairline p-6 text-center text-muted-foreground">
//       <Camera className="h-8 w-8 opacity-60" />
//       <div className="text-sm">No camera streams found</div>
//       <div className="text-[11px] opacity-80">
//         {hasError
//           ? 'web_video_server is unreachable. Check that lupin_web brought it up on :8091 and that the URL above matches.'
//           : 'web_video_server is reachable but reports no image topics. Make sure the camera plugin is running and publishes a sensor_msgs/Image.'}
//       </div>
//     </div>
//   )
// }

import { Camera, RefreshCcw } from 'lucide-react' // --- NEW: Added Scan icon ---
import { useCallback, useEffect, useMemo, useState } from 'react'

import { Button } from '@/components/ui/button'
import { Switch } from '@/components/ui/switch'
import { CameraStream } from '@/components/widgets/CameraStream'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { isMockMode, useSettings } from '@/lib/settings'

interface DiscoveredCamera {
  topic: string
  label: string
}

const PINNED_LABELS: Record<string, string> = {
  '/camera/image_raw': 'RGB',
  '/camera/color/image_raw': 'RGB',
  '/camera/depth/image_raw': 'Depth',
  '/gripper_camera/image_raw': 'Gripper',
  '/camera/ir/image_raw': 'IR',
}

function deriveLabel(topic: string): string {
  if (PINNED_LABELS[topic]) return PINNED_LABELS[topic]
  const tail = topic.split('/').filter(Boolean).slice(-2).join('/')
  return tail || topic
}

async function fetchAvailableStreams(baseUrl: string): Promise<string[]> {
  const parse = (html: string): string[] => {
    const matches = html.matchAll(/href="\/stream\?topic=([^"&]+)/g)
    const out = new Set<string>()
    for (const m of matches) {
      try { out.add(decodeURIComponent(m[1])) } catch { out.add(m[1]) }
    }
    return Array.from(out).sort()
  }

  try {
    const res = await fetch('/_video/', { cache: 'no-store' })
    if (res.ok) {
      const ct = res.headers.get('content-type') || ''
      if (ct.includes('text/html')) return parse(await res.text())
    }
  } catch { }

  if (!baseUrl) return []
  const root = baseUrl.replace(/\/$/, '') + '/'
  const res = await fetch(root, { cache: 'no-store' })
  if (!res.ok) throw new Error(`web_video_server responded ${res.status}`)
  return parse(await res.text())
}

export function CamerasView() {
  const [{ cameraTopic, webVideoServerUrl }] = useSettings()
  const mock = isMockMode()
  const [discovered, setDiscovered] = useState<string[]>([])
  const [discoveryError, setDiscoveryError] = useState<string | null>(null)
  const [discovering, setDiscovering] = useState(false)
  const [refreshKey, setRefreshKey] = useState(0)

  // --- NEW: Global Master State for the AprilTag Toggle ---
  // Read the saved state from the browser, defaulting to false
  const [showBoxes, setShowBoxes] = useState(() => {
    return sessionStorage.getItem('showAprilTags') === 'true'
  })

  // Every time the switch is flipped, save it instantly
  useEffect(() => {
    sessionStorage.setItem('showAprilTags', String(showBoxes))
  }, [showBoxes])

  const refresh = useCallback(() => setRefreshKey((k) => k + 1), [])

  useEffect(() => {
    if (mock) return
    let cancelled = false
    setDiscovering(true)
    setDiscoveryError(null)
    fetchAvailableStreams(webVideoServerUrl)
      .then((topics) => {
        if (cancelled) return
        setDiscovered(topics)
      })
      .catch((err: unknown) => {
        if (cancelled) return
        const msg = err instanceof Error ? err.message : String(err)
        setDiscoveryError(msg)
        setDiscovered([])
      })
      .finally(() => {
        if (!cancelled) setDiscovering(false)
      })
    return () => { cancelled = true }
  }, [webVideoServerUrl, refreshKey, mock])

  const discoverySucceeded = !discoveryError && !discovering
  const configuredTopicMissing =
    !mock && discoverySucceeded && discovered.length > 0 &&
    !!cameraTopic && !discovered.includes(cameraTopic)

  const cameras = useMemo<DiscoveredCamera[]>(() => {
    if (mock) {
      return [{ topic: cameraTopic || '/camera/image_raw', label: 'RGB' }]
    }
    const seen = new Set<string>()
    const out: DiscoveredCamera[] = []
    if (cameraTopic && !configuredTopicMissing) {
      out.push({ topic: cameraTopic, label: deriveLabel(cameraTopic) })
      seen.add(cameraTopic)
    }
    for (const topic of discovered) {
      if (seen.has(topic)) continue
      out.push({ topic, label: deriveLabel(topic) })
      seen.add(topic)
    }
    return out
  }, [cameraTopic, configuredTopicMissing, discovered, mock])

  const defaultValue = cameras[0]?.topic ?? ''

  return (
    <div className="flex min-h-full w-full flex-col gap-3 p-3 sm:p-4">
      <div className="flex flex-wrap items-center gap-2">
        <span className="tag tag-strong">Streams</span>
        <span className="tag font-mono">{webVideoServerUrl || '—'}</span>
        {mock ? (
          <span className="tag tag-accent">mock</span>
        ) : discovering ? (
          <span className="tag">discovering…</span>
        ) : discoveryError ? (
          <span className="tag text-destructive">unreachable · {discoveryError}</span>
        ) : (
          <span className="tag">{discovered.length} discovered</span>
        )}
        <Button
          variant="outline"
          size="sm"
          onClick={refresh}
          className="ml-auto h-7 text-[11px]"
        >
          <RefreshCcw className="mr-1.5 h-3 w-3" /> Rescan
        </Button>
      </div>

      {configuredTopicMissing && (
        <div className="rounded-md border border-amber-500/40 bg-amber-500/10 px-3 py-2 text-[11px] text-amber-200">
          Configured camera topic <span className="font-mono">{cameraTopic}</span> isn't being
          published — falling back to discovered streams. Update or clear it under Settings → Topics.
        </div>
      )}

      {cameras.length === 0 ? (
        <NoCameras hasError={!!discoveryError} />
      ) : (
        <Tabs key={defaultValue} defaultValue={defaultValue} className="flex flex-1 flex-col">
          
          <div className="flex items-center gap-6 self-start">
            <TabsList>
              {cameras.map((c) => (
                <TabsTrigger key={c.topic} value={c.topic} title={c.topic}>
                  {c.label}
                </TabsTrigger>
              ))}
            </TabsList>

            {/* --- NEW: The Toggle Switch --- */}
            <div className="flex items-center space-x-2 rounded-md border border-border px-3 py-1.5">
              <Switch 
                id="apriltag-mode" 
                checked={showBoxes} 
                onCheckedChange={setShowBoxes} 
              />
              <label 
                htmlFor="apriltag-mode" 
                className="text-sm font-medium leading-none cursor-pointer text-muted-foreground"
              >
                Overlay AprilTags
              </label>
            </div>
          </div>

          {cameras.map((c) => (
            <TabsContent key={c.topic} value={c.topic} className="m-0 mt-2 flex flex-1">
              <CameraStream
                label={c.label}
                topic={c.topic}
                baseUrl={webVideoServerUrl}
                className="flex-1 min-h-[24rem]"
                showBoxes={showBoxes}
              />
            </TabsContent>
          ))}
        </Tabs>
      )}
    </div>
  )
}

function NoCameras({ hasError }: { hasError: boolean }) {
  return (
    <div className="m-6 flex max-w-md flex-col items-center gap-3 self-center rounded-md border border-dashed border-hairline p-6 text-center text-muted-foreground">
      <Camera className="h-8 w-8 opacity-60" />
      <div className="text-sm">No camera streams found</div>
      <div className="text-[11px] opacity-80">
        {hasError
          ? 'web_video_server is unreachable. Check that lupin_web brought it up on :8091 and that the URL above matches.'
          : 'web_video_server is reachable but reports no image topics. Make sure the camera plugin is running and publishes a sensor_msgs/Image.'}
      </div>
    </div>
  )
}
