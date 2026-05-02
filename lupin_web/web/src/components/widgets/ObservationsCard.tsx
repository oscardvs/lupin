import { Inbox, Thermometer } from 'lucide-react'
import { useEffect, useState } from 'react'

import {
  Card,
  CardContent,
  CardDescription,
  CardHeader,
  CardTitle,
} from '@/components/ui/card'
import { useTagObservations } from '@/lib/mission'
import { cn } from '@/lib/utils'
import { OBSERVATION_STATUS, type Observation, type SensorReading } from '@/types/ros'

interface ObservationsCardProps {
  className?: string
}

const STATUS_LABEL: Record<number, { tone: 'ok' | 'warn' | 'err' | 'muted'; label: string }> = {
  [OBSERVATION_STATUS.OK]: { tone: 'ok', label: 'OK' },
  [OBSERVATION_STATUS.UNREACHABLE]: { tone: 'err', label: 'unreachable' },
  [OBSERVATION_STATUS.SCAN_FAILED]: { tone: 'warn', label: 'scan failed' },
  [OBSERVATION_STATUS.SKIPPED]: { tone: 'muted', label: 'skipped' },
}

const READING_UNITS: Record<string, string> = {
  temperature: '°C',
  humidity: '%',
  co2: 'ppm',
  light: 'lx',
  soil_moisture: '%',
}

/**
 * Latest tag observation per station, accumulated across the page lifetime.
 * This is the v1 stand-in for the digital-twin sink — `/mission/observations`
 * is the orchestrator's published stream and we render the latest reading
 * for each tag we've seen.
 *
 * The brief's "headline value" (FloraNova quote) is continuous insight into
 * plant conditions per station; this is the on-robot read of that. The actual
 * twin pipeline is downstream and not in scope for this MR.
 */
export function ObservationsCard({ className }: ObservationsCardProps) {
  const obs = useTagObservations()
  // Tick once per second so "age" relabels without spamming React renders.
  const [, setTick] = useState(0)
  useEffect(() => {
    const id = setInterval(() => setTick((t) => (t + 1) % 1_000_000), 1000)
    return () => clearInterval(id)
  }, [])

  const sorted = [...obs.values()].sort((a, b) =>
    numericTagSort(a.tag_reading.tag_id, b.tag_reading.tag_id),
  )

  return (
    <Card className={cn('flex flex-col', className)}>
      <CardHeader>
        <CardTitle>
          Observations
          <span className="tag tag-accent ml-auto">PNL-OBS-01</span>
        </CardTitle>
        <CardDescription className="font-mono text-xs">
          {sorted.length === 0
            ? 'awaiting /floranova/observations…'
            : `${sorted.length} station${sorted.length === 1 ? '' : 's'} · latest per tag`}
        </CardDescription>
      </CardHeader>
      <CardContent className="flex-1">
        {sorted.length === 0 ? <Empty /> : <Table rows={sorted} />}
      </CardContent>
    </Card>
  )
}

function Empty() {
  return (
    <div className="flex h-full min-h-[6rem] flex-col items-center justify-center gap-2 rounded-sm border border-dashed border-hairline bg-background/30 px-4 py-6 text-center text-muted-foreground">
      <Inbox className="h-5 w-5 opacity-60" />
      <div className="text-[12px]">No observations yet</div>
      <div className="text-[10px] opacity-80">
        Start a patrol from <span className="font-mono">Map / Nav</span> to populate this table.
      </div>
    </div>
  )
}

function Table({ rows }: { rows: Observation[] }) {
  return (
    <div className="overflow-x-auto">
      <table className="w-full border-collapse text-[12px]">
        <thead>
          <tr className="border-b border-hairline text-left text-muted-foreground">
            <Th>tag</Th>
            <Th>status</Th>
            <Th>readings</Th>
            <Th className="text-right">age</Th>
          </tr>
        </thead>
        <tbody className="divide-hairline divide-y">
          {rows.map((row) => (
            <Row key={row.tag_reading.tag_id} obs={row} />
          ))}
        </tbody>
      </table>
    </div>
  )
}

function Row({ obs }: { obs: Observation }) {
  const tag = obs.tag_reading.tag_id
  const status = STATUS_LABEL[obs.status] ?? { tone: 'muted' as const, label: `status ${obs.status}` }
  const stamp = obs.tag_reading.stamp.sec + obs.tag_reading.stamp.nanosec * 1e-9

  return (
    <tr className="hover:bg-muted/20">
      <Td className="font-mono text-foreground">{tag}</Td>
      <Td>
        <StatusPill tone={status.tone}>{status.label}</StatusPill>
        {obs.status_detail && (
          <span className="ml-2 truncate text-[11px] text-muted-foreground">
            {obs.status_detail}
          </span>
        )}
      </Td>
      <Td>
        <ReadingsList readings={obs.tag_reading.readings} />
      </Td>
      <Td className="whitespace-nowrap text-right font-mono text-muted-foreground">
        {formatAge(stamp)}
      </Td>
    </tr>
  )
}

function ReadingsList({ readings }: { readings: SensorReading[] }) {
  if (!readings || readings.length === 0) {
    return <span className="text-muted-foreground">—</span>
  }
  return (
    <div className="flex flex-wrap items-center gap-1.5">
      {readings.map((r) => (
        <span
          key={r.name}
          className="inline-flex items-center gap-1 rounded-sm border border-hairline bg-background/40 px-1.5 py-0.5 font-mono text-[11px]"
          title={`${r.name}: ${r.value}`}
        >
          {r.name === 'temperature' && <Thermometer className="h-3 w-3 opacity-60" />}
          <span className="text-muted-foreground">{r.name}</span>
          <span className="text-foreground">
            {formatReadingValue(r.value)}
            {READING_UNITS[r.name] && (
              <span className="ml-0.5 text-muted-foreground">{READING_UNITS[r.name]}</span>
            )}
          </span>
        </span>
      ))}
    </div>
  )
}

function StatusPill({
  tone, children,
}: { tone: 'ok' | 'warn' | 'err' | 'muted'; children: React.ReactNode }) {
  return (
    <span
      className={cn(
        'inline-block rounded-sm border px-1.5 py-0.5 font-mono text-[10px] uppercase tracking-wider',
        tone === 'ok' && 'border-primary/40 bg-primary/10 text-primary',
        tone === 'warn' && 'border-warning/40 bg-warning/10 text-warning',
        tone === 'err' && 'border-destructive/50 bg-destructive/10 text-destructive-foreground',
        tone === 'muted' && 'border-hairline bg-background/40 text-muted-foreground',
      )}
    >
      {children}
    </span>
  )
}

function Th({ children, className }: { children: React.ReactNode; className?: string }) {
  return (
    <th className={cn('px-2 py-1.5 align-bottom', className)}>
      <span className="tag">{children}</span>
    </th>
  )
}

function Td({ children, className }: { children: React.ReactNode; className?: string }) {
  return <td className={cn('px-2 py-2 align-middle', className)}>{children}</td>
}

function formatReadingValue(v: number): string {
  if (!Number.isFinite(v)) return '—'
  if (Math.abs(v) >= 100) return v.toFixed(0)
  if (Math.abs(v) >= 10) return v.toFixed(1)
  return v.toFixed(2)
}

function formatAge(stampSec: number): string {
  if (!Number.isFinite(stampSec) || stampSec <= 0) return '—'
  const age = Math.max(0, Date.now() / 1000 - stampSec)
  if (age < 1) return 'now'
  if (age < 60) return `${Math.round(age)}s ago`
  if (age < 3600) return `${Math.round(age / 60)}m ago`
  return `${Math.round(age / 3600)}h ago`
}

/**
 * Match the orchestrator's `numeric_string_sort_key`: sort numeric prefixes
 * numerically so "tag-2" < "tag-10". Fall back to lexicographic for ties.
 */
function numericTagSort(a: string, b: string): number {
  const na = parseInt(a.replace(/\D+/g, ''), 10)
  const nb = parseInt(b.replace(/\D+/g, ''), 10)
  if (Number.isFinite(na) && Number.isFinite(nb) && na !== nb) return na - nb
  return a.localeCompare(b)
}
