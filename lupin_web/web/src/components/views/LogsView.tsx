import { Pause, Play, Trash2 } from 'lucide-react'
import { useCallback, useEffect, useRef, useState } from 'react'

import { Button } from '@/components/ui/button'
import { Input } from '@/components/ui/input'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Select, SelectContent, SelectItem, SelectTrigger, SelectValue } from '@/components/ui/select'
import { Switch } from '@/components/ui/switch'
import { Label } from '@/components/ui/label'
import { useTopic } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import { useThrottledRender } from '@/lib/throttle'
import { ROS_TYPE, ROSOUT_LEVEL_NAMES, type Log, type RosoutLevel } from '@/types/ros'
import { cn } from '@/lib/utils'

const MAX_ROWS = 500

const levelClass: Record<RosoutLevel, string> = {
  10: 'text-zinc-400',
  20: 'text-foreground',
  30: 'text-amber-400',
  40: 'text-red-400',
  50: 'text-red-500 font-semibold',
}

const levelOrder: RosoutLevel[] = [10, 20, 30, 40, 50]

export function LogsView() {
  const [{ rosoutTopic }] = useSettings()
  const bufferRef = useRef<Log[]>([])
  const [paused, setPaused] = useState(false)
  const [autoScroll, setAutoScroll] = useState(true)
  const [minLevel, setMinLevel] = useState<RosoutLevel>(20)
  const [nodeFilter, setNodeFilter] = useState<string>('all')
  const [search, setSearch] = useState('')
  const seenNodes = useRef<Set<string>>(new Set())

  useTopic<Log>(rosoutTopic, ROS_TYPE.Log, {
    onMessage: (msg) => {
      seenNodes.current.add(msg.name)
      if (paused) return
      bufferRef.current.push(msg)
      if (bufferRef.current.length > MAX_ROWS) {
        bufferRef.current.splice(0, bufferRef.current.length - MAX_ROWS)
      }
    },
  })

  useThrottledRender(5)

  const visible = bufferRef.current.filter((l) => {
    if (l.level < minLevel) return false
    if (nodeFilter !== 'all' && l.name !== nodeFilter) return false
    if (search.trim() && !l.msg.toLowerCase().includes(search.toLowerCase())) return false
    return true
  })

  const scrollViewportRef = useRef<HTMLDivElement | null>(null)
  useEffect(() => {
    if (!autoScroll) return
    const el = scrollViewportRef.current
    if (!el) return
    el.scrollTop = el.scrollHeight
  }, [visible.length, autoScroll])

  const clear = useCallback(() => {
    bufferRef.current = []
  }, [])

  return (
    <div className="flex min-h-full w-full flex-col gap-3 p-3 sm:p-4">
      <div className="flex flex-col gap-2 sm:flex-row sm:flex-wrap sm:items-center">
        <div className="flex flex-wrap items-center gap-2">
          <Select value={String(minLevel)} onValueChange={(v) => setMinLevel(Number(v) as RosoutLevel)}>
            <SelectTrigger className="h-9 w-32"><SelectValue /></SelectTrigger>
            <SelectContent>
              {levelOrder.map((lv) => (
                <SelectItem key={lv} value={String(lv)}>
                  ≥ {ROSOUT_LEVEL_NAMES[lv]}
                </SelectItem>
              ))}
            </SelectContent>
          </Select>
          <Select value={nodeFilter} onValueChange={setNodeFilter}>
            <SelectTrigger className="h-9 w-44"><SelectValue placeholder="Node…" /></SelectTrigger>
            <SelectContent>
              <SelectItem value="all">all nodes</SelectItem>
              {Array.from(seenNodes.current).sort().map((n) => (
                <SelectItem key={n} value={n}>{n}</SelectItem>
              ))}
            </SelectContent>
          </Select>
        </div>
        <Input
          value={search}
          onChange={(e) => setSearch(e.target.value)}
          placeholder="Search messages…"
          className="h-9 sm:w-56"
        />
        <div className="ml-auto flex flex-wrap items-center gap-2">
          <div className="flex items-center gap-2">
            <Label htmlFor="autoscroll" className="text-xs text-muted-foreground">auto-scroll</Label>
            <Switch id="autoscroll" checked={autoScroll} onCheckedChange={setAutoScroll} />
          </div>
          <Button variant="outline" size="sm" onClick={() => setPaused((p) => !p)} aria-label={paused ? 'Resume' : 'Pause'}>
            {paused ? <Play className="h-3.5 w-3.5 sm:mr-2" /> : <Pause className="h-3.5 w-3.5 sm:mr-2" />}
            <span className="hidden sm:inline">{paused ? 'Resume' : 'Pause'}</span>
          </Button>
          <Button variant="outline" size="sm" onClick={clear} aria-label="Clear">
            <Trash2 className="h-3.5 w-3.5 sm:mr-2" />
            <span className="hidden sm:inline">Clear</span>
          </Button>
        </div>
      </div>

      <ScrollArea className="flex-1 rounded-sm border border-hairline bg-card font-mono text-xs">
        <div ref={scrollViewportRef} className="max-h-[60vh] overflow-y-auto">
          {visible.length === 0 ? (
            <div className="p-4 text-muted-foreground">No log lines match the current filter.</div>
          ) : (
            <div className="divide-y divide-border/50">
              {visible.map((l, i) => (
                <div
                  key={i}
                  className="grid gap-x-2 gap-y-0.5 px-3 py-1.5 sm:grid-cols-[auto_auto_auto_1fr]"
                >
                  <span className="hidden text-muted-foreground tabular-nums sm:inline">
                    {new Date(l.stamp.sec * 1000).toLocaleTimeString()}
                  </span>
                  <div className="flex items-center gap-2 sm:contents">
                    <span className="text-[10px] text-muted-foreground tabular-nums sm:hidden">
                      {new Date(l.stamp.sec * 1000).toLocaleTimeString()}
                    </span>
                    <span className={cn('text-[10px] uppercase tracking-wider sm:w-12', levelClass[l.level])}>
                      {ROSOUT_LEVEL_NAMES[l.level]}
                    </span>
                    <span className="truncate text-sky-300">{l.name}</span>
                  </div>
                  <span className={cn('break-words', levelClass[l.level])}>{l.msg}</span>
                </div>
              ))}
            </div>
          )}
        </div>
      </ScrollArea>

      <div className="text-[11px] text-muted-foreground">
        {visible.length} / {bufferRef.current.length} rows · capped at {MAX_ROWS} · topic <span className="font-mono">{rosoutTopic}</span>
      </div>
    </div>
  )
}
