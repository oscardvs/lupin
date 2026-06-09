import { Camera, RefreshCcw } from 'lucide-react'
import { useCallback, useEffect, useMemo, useState } from 'react'

import { ViewShell } from '@/components/system/ViewShell'
import { Button } from '@/components/ui/button'
import { Switch } from '@/components/ui/switch'
import { CameraStream } from '@/components/widgets/CameraStream'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { isMockMode, useSettings } from '@/lib/settings'
import { cn } from '@/lib/utils'

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

/**
 * Scrape web_video_server's index page for the list of streamable topics.
 * Tries the Vite same-origin proxy at `/_video/` first (sidesteps the missing
 * CORS headers), then falls back to a direct fetch on the configured baseUrl.
 * Returns an empty list on any error so the UI collapses to a "no streams"
 * state instead of throwing.
 */
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
  } catch { /* fall through to direct fetch */ }

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

  // AprilTag overlay toggle — persisted so it survives a tab switch / remount.
  const [showBoxes, setShowBoxes] = useState(
    () => sessionStorage.getItem('showAprilTags') === 'true',
  )
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
    <ViewShell intent="fit">
      {configuredTopicMissing && (
        <div className="shrink-0 rounded-sm border border-warning/40 bg-warning/10 px-3 py-2 text-[11px] text-warning">
          Configured camera topic <span className="font-mono">{cameraTopic}</span> isn't being
          published — falling back to discovered streams. Update or clear it under Settings → Topics.
        </div>
      )}

      {cameras.length === 0 ? (
        <NoCameras hasError={!!discoveryError} />
      ) : (
        <Tabs
          key={defaultValue}
          defaultValue={defaultValue}
          className="flex min-h-0 flex-1 flex-col gap-[var(--gap)]"
        >
          {/* Single instrument toolbar — tabs on the left, status + controls
              bookended to the right. */}
          <div className="flex shrink-0 flex-wrap items-center gap-2">
            <TabsList>
              {cameras.map((c) => (
                <TabsTrigger key={c.topic} value={c.topic} title={c.topic}>
                  {c.label}
                </TabsTrigger>
              ))}
            </TabsList>

            <span className="hidden max-w-[40vw] truncate font-mono tag sm:inline">
              {webVideoServerUrl || '—'}
            </span>
            {mock ? (
              <span className="tag tag-accent">mock</span>
            ) : discovering ? (
              <span className="tag">discovering…</span>
            ) : discoveryError ? (
              <span className="tag text-destructive">unreachable</span>
            ) : (
              <span className="tag">{discovered.length} found</span>
            )}

            <div className="ml-auto flex items-center gap-2">
              <label
                htmlFor="apriltag-mode"
                className="flex h-9 cursor-pointer items-center gap-2 rounded-sm border border-hairline bg-ink-2 px-3"
              >
                <Switch id="apriltag-mode" checked={showBoxes} onCheckedChange={setShowBoxes} />
                <span
                  className={cn(
                    'text-[12px] font-medium leading-none',
                    showBoxes ? 'text-foreground' : 'text-muted-foreground',
                  )}
                >
                  AprilTags
                </span>
              </label>
              <Button variant="outline" size="sm" onClick={refresh} className="h-9">
                <RefreshCcw className="h-3.5 w-3.5 sm:mr-1.5" />
                <span className="hidden sm:inline">Rescan</span>
              </Button>
            </div>
          </div>

          {cameras.map((c) => (
            <TabsContent key={c.topic} value={c.topic} className="m-0 flex min-h-0 flex-1">
              <CameraStream
                label={c.label}
                topic={c.topic}
                baseUrl={webVideoServerUrl}
                className="min-h-[14rem] flex-1 sm:min-h-[20rem]"
                showBoxes={showBoxes}
              />
            </TabsContent>
          ))}
        </Tabs>
      )}
    </ViewShell>
  )
}

function NoCameras({ hasError }: { hasError: boolean }) {
  return (
    <div className="m-6 flex max-w-md flex-col items-center gap-3 self-center rounded-sm border border-dashed border-hairline p-6 text-center text-muted-foreground">
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
