import { Inbox } from 'lucide-react'
import { useMemo, useState } from 'react'

import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from '@/components/ui/card'
import { tagHasPose, useTwinState } from '@/lib/twin'
import { cn } from '@/lib/utils'
import { TWIN_SENSORS, type TwinSensor, type TwinTagState } from '@/types/ros'

interface GreenhouseStateCardProps {
  className?: string
  /** Fired when the operator clicks a tag row — Map / Nav reacts to centre
   * its view on the tag and pulse the pin briefly. Optional so the card
   * stands alone in tests / Storybook. */
  onSelectTag?: (tagId: string) => void
}

const SENSOR_LABELS: Record<TwinSensor, string> = {
  temperature: 'TEMP',
  humidity: 'HUM',
  co2: 'CO₂',
  light: 'LIGHT',
  soil_moisture: 'SOIL',
}

const SENSOR_UNITS: Record<TwinSensor, string> = {
  temperature: '°C',
  humidity: '%',
  co2: 'ppm',
  light: 'lux',
  soil_moisture: '%',
}

type SortKey = 'tag' | TwinSensor | 'last_seen'
interface SortState {
  key: SortKey
  dir: 'asc' | 'desc'
}

/**
 * Live Greenhouse State table — every tag the digital-twin has observed,
 * with its latest reading per sensor and a staleness-tinted "LAST SEEN".
 *
 * Sits below Mission Control on Map / Nav. Empty state is "Awaiting first
 * observation" — by design the table starts empty and fills row by row as
 * the robot patrols, mirroring the SLAM-incremental story.
 */
export function GreenhouseStateCard({
  className, onSelectTag,
}: GreenhouseStateCardProps) {
  const twin = useTwinState()
  const tags = twin?.tags ?? []
  const [sort, setSort] = useState<SortState>({ key: 'tag', dir: 'asc' })

  const visible = useMemo(() => {
    // We show every tag the twin knows about, including pose-less ones —
    // the operator wants to see "we got a reading from tag-7 even though
    // we haven't pinned it yet". Sort accordingly.
    const arr = [...tags]
    arr.sort((a, b) => compareTags(a, b, sort))
    return arr
  }, [tags, sort])

  const toggleSort = (key: SortKey) => {
    setSort((s) => {
      if (s.key !== key) return { key, dir: 'asc' }
      return { key, dir: s.dir === 'asc' ? 'desc' : 'asc' }
    })
  }

  return (
    <Card className={cn('flex flex-col', className)}>
      <CardHeader>
        <CardTitle>
          Greenhouse State
          <span className="tag tag-accent ml-auto">PNL-TWIN-01</span>
        </CardTitle>
        <CardDescription className="font-mono text-xs">
          {visible.length === 0
            ? 'awaiting first observation…'
            : `${visible.length} tag${visible.length === 1 ? '' : 's'} · live from /twin/state`}
        </CardDescription>
      </CardHeader>
      <CardContent className="flex-1">
        {visible.length === 0 ? <Empty /> : (
          <Table sort={sort} onSort={toggleSort} tags={visible} onSelectTag={onSelectTag} />
        )}
      </CardContent>
    </Card>
  )
}

function Empty() {
  return (
    <div className="flex h-full min-h-[6rem] flex-col items-center justify-center gap-2 rounded-sm border border-dashed border-hairline bg-background/30 px-4 py-6 text-center text-muted-foreground">
      <Inbox className="h-5 w-5 opacity-60" />
      <div className="text-[12px]">Awaiting first observation</div>
      <div className="text-[10px] opacity-80">
        Start a patrol from <span className="font-mono">Map / Nav</span> · the table fills as tags are scanned.
      </div>
    </div>
  )
}

function Table({
  sort, onSort, tags, onSelectTag,
}: {
  sort: SortState
  onSort: (k: SortKey) => void
  tags: TwinTagState[]
  onSelectTag?: (tagId: string) => void
}) {
  return (
    <div className="overflow-x-auto">
      <table className="w-full border-collapse text-[12px]">
        <thead>
          <tr className="border-b border-hairline text-left text-muted-foreground">
            <Th label="tag"      sortKey="tag"        sort={sort} onSort={onSort} />
            {TWIN_SENSORS.map((s) => (
              <Th
                key={s}
                label={`${SENSOR_LABELS[s]} (${SENSOR_UNITS[s]})`}
                sortKey={s}
                sort={sort}
                onSort={onSort}
                className="text-right"
              />
            ))}
            <Th
              label="last seen"
              sortKey="last_seen"
              sort={sort}
              onSort={onSort}
              className="text-right"
            />
          </tr>
        </thead>
        <tbody className="divide-hairline divide-y">
          {tags.map((t) => (
            <Row key={t.tag_id} tag={t} onSelect={onSelectTag} />
          ))}
        </tbody>
      </table>
    </div>
  )
}

function Row({
  tag, onSelect,
}: { tag: TwinTagState; onSelect?: (tagId: string) => void }) {
  const readingByName = new Map(tag.readings.map((r) => [r.name, r.value]))
  const stale = tag.stale_seconds
  const staleClass =
    stale < 60 ? 'text-primary'
    : stale < 300 ? 'text-warning'
    : 'text-destructive'
  return (
    <tr
      onClick={onSelect ? () => onSelect(tag.tag_id) : undefined}
      className={cn(
        'transition-colors',
        onSelect && 'cursor-pointer hover:bg-muted/20',
      )}
    >
      <td className="px-2 py-1.5 align-middle">
        <span className="font-mono text-foreground">{tag.tag_id}</span>
        {!tagHasPose(tag) && (
          <span className="ml-2 tag text-muted-foreground">unpinned</span>
        )}
      </td>
      {TWIN_SENSORS.map((s) => {
        const v = readingByName.get(s)
        return (
          <td key={s} className="px-2 py-1.5 align-middle text-right font-mono">
            {v == null ? <span className="text-muted-foreground">—</span> : (
              <span className="text-foreground">{formatNum(v)}</span>
            )}
          </td>
        )
      })}
      <td className={cn('px-2 py-1.5 align-middle text-right font-mono', staleClass)}>
        {formatStaleness(stale)}
      </td>
    </tr>
  )
}

function Th({
  label, sortKey, sort, onSort, className,
}: {
  label: string
  sortKey: SortKey
  sort: SortState
  onSort: (k: SortKey) => void
  className?: string
}) {
  const active = sort.key === sortKey
  const arrow = active ? (sort.dir === 'asc' ? '▲' : '▼') : ''
  // aria-sort is what screen readers expect on a sortable column header.
  // Set "none" on the inactive headers so the live region transitions
  // cleanly when the user changes sort.
  const ariaSort: 'ascending' | 'descending' | 'none' = active
    ? (sort.dir === 'asc' ? 'ascending' : 'descending')
    : 'none'
  return (
    <th
      aria-sort={ariaSort}
      className={cn('px-2 py-1.5 align-bottom', className)}
    >
      <button
        type="button"
        onClick={() => onSort(sortKey)}
        className={cn(
          'tag inline-flex items-baseline gap-1 transition-colors hover:text-foreground',
          active && 'text-foreground',
        )}
      >
        {label} {arrow && <span className="text-[8px]">{arrow}</span>}
      </button>
    </th>
  )
}

/* ---------- helpers ---------- */

function compareTags(a: TwinTagState, b: TwinTagState, sort: SortState): number {
  let cmp = 0
  if (sort.key === 'tag') {
    cmp = numericTagSort(a.tag_id, b.tag_id)
  } else if (sort.key === 'last_seen') {
    cmp = a.stale_seconds - b.stale_seconds
  } else {
    const av = a.readings.find((r) => r.name === sort.key)?.value
    const bv = b.readings.find((r) => r.name === sort.key)?.value
    if (av == null && bv == null) cmp = 0
    else if (av == null) cmp = 1   // missing values sort to the end ascending
    else if (bv == null) cmp = -1
    else cmp = av - bv
  }
  return sort.dir === 'asc' ? cmp : -cmp
}

function numericTagSort(a: string, b: string): number {
  const na = parseInt(a.replace(/\D+/g, ''), 10)
  const nb = parseInt(b.replace(/\D+/g, ''), 10)
  if (Number.isFinite(na) && Number.isFinite(nb) && na !== nb) return na - nb
  return a.localeCompare(b)
}

function formatNum(v: number): string {
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
