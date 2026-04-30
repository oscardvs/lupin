import { useEffect, useState } from 'react'

export function ClockPill() {
  const [now, setNow] = useState(() => new Date())
  useEffect(() => {
    const id = setInterval(() => setNow(new Date()), 1000)
    return () => clearInterval(id)
  }, [])

  const time = now.toLocaleTimeString([], {
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    hour12: false,
  })
  const tz = now
    .toLocaleTimeString([], { timeZoneName: 'short' })
    .split(' ')
    .slice(-1)[0]

  return (
    <div className="hidden h-9 shrink-0 items-center gap-2 rounded-sm border border-hairline bg-card/40 px-2.5 text-[11px] md:flex">
      <span className="ticker text-foreground">{time}</span>
      <span className="tag">{tz}</span>
    </div>
  )
}
