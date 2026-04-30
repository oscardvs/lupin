import ROSLIB from 'roslib'
import { useEffect, useRef, useState } from 'react'

export type RosStatus = 'connecting' | 'connected' | 'closed' | 'error'

function defaultUrl(): string {
  const params = new URLSearchParams(window.location.search)
  const override = params.get('ros')
  if (override) return override
  const host = window.location.hostname || 'localhost'
  return `ws://${host}:9090`
}

export function useRos(url: string = defaultUrl()) {
  const [status, setStatus] = useState<RosStatus>('connecting')
  const [lastError, setLastError] = useState<string | null>(null)
  const rosRef = useRef<ROSLIB.Ros | null>(null)

  useEffect(() => {
    let cancelled = false
    let backoffMs = 500
    const MAX_BACKOFF = 5000
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null

    const connect = () => {
      if (cancelled) return
      setStatus('connecting')
      const ros = new ROSLIB.Ros({ url })
      rosRef.current = ros

      ros.on('connection', () => {
        if (cancelled) return
        backoffMs = 500
        setStatus('connected')
        setLastError(null)
      })
      ros.on('error', (err: unknown) => {
        if (cancelled) return
        setStatus('error')
        setLastError(err instanceof Error ? err.message : String(err))
      })
      ros.on('close', () => {
        if (cancelled) return
        setStatus('closed')
        reconnectTimer = setTimeout(connect, backoffMs)
        backoffMs = Math.min(backoffMs * 2, MAX_BACKOFF)
      })
    }

    connect()
    return () => {
      cancelled = true
      if (reconnectTimer) clearTimeout(reconnectTimer)
      rosRef.current?.close()
      rosRef.current = null
    }
  }, [url])

  return { status, lastError, ros: rosRef.current, url }
}

export function makeTwistTopic(ros: ROSLIB.Ros, name: string) {
  return new ROSLIB.Topic({
    ros,
    name,
    messageType: 'geometry_msgs/Twist',
  })
}

export function twist(linearX = 0, angularZ = 0): ROSLIB.Message {
  return new ROSLIB.Message({
    linear: { x: linearX, y: 0, z: 0 },
    angular: { x: 0, y: 0, z: angularZ },
  })
}
