import { useCallback, useMemo, useRef, useState } from 'react'

import { StatusDot } from '@/components/StatusDot'
import { Button } from '@/components/ui/button'
import { makeTwistTopic, twist, useRos } from '@/lib/ros'

const DRIVE_TOPIC = '/mirte_base_controller/cmd_vel'
const NUDGE_LINEAR_X = 0.1
const NUDGE_DURATION_MS = 500
const NUDGE_RATE_HZ = 20

export default function App() {
  const { status, lastError, ros, url } = useRos()
  const [busy, setBusy] = useState(false)
  const [lastSent, setLastSent] = useState<string | null>(null)
  const intervalRef = useRef<ReturnType<typeof setInterval> | null>(null)
  const stopTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null)

  const topic = useMemo(
    () => (ros && status === 'connected' ? makeTwistTopic(ros, DRIVE_TOPIC) : null),
    [ros, status],
  )

  const nudgeForward = useCallback(() => {
    if (!topic || busy) return
    setBusy(true)

    const tickMs = Math.round(1000 / NUDGE_RATE_HZ)
    const moveMsg = twist(NUDGE_LINEAR_X, 0)
    topic.publish(moveMsg)
    intervalRef.current = setInterval(() => topic.publish(moveMsg), tickMs)

    stopTimerRef.current = setTimeout(() => {
      if (intervalRef.current) {
        clearInterval(intervalRef.current)
        intervalRef.current = null
      }
      topic.publish(twist(0, 0))
      setBusy(false)
      const ts = new Date().toLocaleTimeString()
      setLastSent(`linear.x=${NUDGE_LINEAR_X.toFixed(2)} for ${NUDGE_DURATION_MS}ms @ ${ts}`)
    }, NUDGE_DURATION_MS)
  }, [topic, busy])

  return (
    <main className="mx-auto flex h-full max-w-md flex-col gap-6 p-6">
      <header className="flex flex-col gap-1">
        <h1 className="text-2xl font-semibold tracking-tight">Lupin HMI</h1>
        <p className="text-sm text-muted-foreground">
          Browser teleop for the MIRTE Master.
        </p>
      </header>

      <StatusDot status={status} url={url} detail={lastError} />

      <section className="flex flex-col gap-3 rounded-md border bg-card p-4">
        <div className="flex flex-col gap-0.5">
          <span className="text-sm font-medium">Drive test</span>
          <span className="text-xs text-muted-foreground">
            Publishes <code>{DRIVE_TOPIC}</code> at {NUDGE_RATE_HZ} Hz for{' '}
            {NUDGE_DURATION_MS} ms, then zero.
          </span>
        </div>
        <Button
          size="lg"
          onClick={nudgeForward}
          disabled={status !== 'connected' || busy}
        >
          {busy ? 'Nudging…' : 'Nudge forward'}
        </Button>
        <div className="text-xs text-muted-foreground">
          Last command:{' '}
          <span className="font-mono">{lastSent ?? '—'}</span>
        </div>
      </section>
    </main>
  )
}
