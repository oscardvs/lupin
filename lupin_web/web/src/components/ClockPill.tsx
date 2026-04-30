import { useEffect, useState } from 'react'

export function ClockPill() {
  const [now, setNow] = useState(() => new Date())
  useEffect(() => {
    const id = setInterval(() => setNow(new Date()), 1000)
    return () => clearInterval(id)
  }, [])
  const text = now.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false })
  return (
    <div className="hidden shrink-0 items-center rounded-full border bg-card/60 px-3 py-1 font-mono text-xs tabular-nums md:flex">
      {text}
    </div>
  )
}
