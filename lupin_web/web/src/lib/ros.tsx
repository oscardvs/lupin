import ROSLIB from 'roslib'
import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useMemo,
  useRef,
  useState,
  type ReactNode,
} from 'react'

import { isMockMode, useSettings } from '@/lib/settings'
import {
  mockBattery,
  mockImu,
  mockJointStates,
  mockLog,
  mockOdometry,
  mockScan,
} from '@/lib/mock'

export type RosStatus = 'connecting' | 'connected' | 'closed' | 'error'

export interface PublishLogEntry {
  topic: string
  msg: unknown
  at: number
}

const PUBLISH_LOG_CAP = 50

interface Subscription {
  topicName: string
  msgType: string
  topic: ROSLIB.Topic | null
  callbacks: Set<(msg: unknown) => void>
  mockTimer: ReturnType<typeof setInterval> | null
}

interface RosCore {
  status: RosStatus
  url: string
  mode: 'real' | 'mock'
  latencyMs: number | null
  lastError: string | null
  subscribe: <T>(topicName: string, msgType: string, cb: (msg: T) => void) => () => void
  publish: <T>(topicName: string, msgType: string, msg: T) => void
  publishLog: PublishLogEntry[]
}

const noopCore: RosCore = {
  status: 'connecting',
  url: '',
  mode: 'real',
  latencyMs: null,
  lastError: null,
  subscribe: () => () => undefined,
  publish: () => undefined,
  publishLog: [],
}

const RosContext = createContext<RosCore>(noopCore)

const MOCK_RATES_HZ: Record<string, number> = {
  'sensor_msgs/msg/Imu': 50,
  'sensor_msgs/msg/LaserScan': 10,
  'nav_msgs/msg/Odometry': 30,
  'sensor_msgs/msg/JointState': 30,
  'sensor_msgs/msg/BatteryState': 1,
  'rcl_interfaces/msg/Log': 0.5,
}

function mockMessageFor(msgType: string): unknown {
  switch (msgType) {
    case 'sensor_msgs/msg/Imu':
      return mockImu()
    case 'sensor_msgs/msg/LaserScan':
      return mockScan()
    case 'nav_msgs/msg/Odometry':
      return mockOdometry()
    case 'sensor_msgs/msg/JointState':
      return mockJointStates()
    case 'sensor_msgs/msg/BatteryState':
      return mockBattery()
    case 'rcl_interfaces/msg/Log':
      return mockLog()
    default:
      return null
  }
}

export function RosProvider({ children }: { children: ReactNode }) {
  const [{ rosUrl, debugPublish }] = useSettings()
  const mock = useMemo(() => isMockMode(), [])
  const debugPublishRef = useRef(debugPublish)
  debugPublishRef.current = debugPublish

  const [status, setStatus] = useState<RosStatus>(mock ? 'connected' : 'connecting')
  const [latencyMs, setLatencyMs] = useState<number | null>(mock ? 6 : null)
  const [lastError, setLastError] = useState<string | null>(null)
  const [, forcePublishLogTick] = useState(0)

  const rosRef = useRef<ROSLIB.Ros | null>(null)
  const subsRef = useRef<Map<string, Subscription>>(new Map())
  const publishersRef = useRef<Map<string, ROSLIB.Topic>>(new Map())
  const publishLogRef = useRef<PublishLogEntry[]>([])

  const recordPublish = useCallback((topic: string, msg: unknown) => {
    const next = publishLogRef.current.slice(-(PUBLISH_LOG_CAP - 1))
    next.push({ topic, msg, at: Date.now() })
    publishLogRef.current = next
    forcePublishLogTick((t) => (t + 1) % 1000000)
    if (debugPublishRef.current) {
      // Surface the exact JSON payload going over the wire to the console.
      // Useful for diagnosing rosbridge serialisation issues end-to-end.
      // eslint-disable-next-line no-console
      console.log('[lupin-web publish]', topic, JSON.stringify(msg))
    }
  }, [])

  useEffect(() => {
    if (mock) return
    let cancelled = false
    let backoffMs = 500
    const MAX_BACKOFF = 5000
    let reconnectTimer: ReturnType<typeof setTimeout> | null = null
    let pingTimer: ReturnType<typeof setInterval> | null = null

    const setupSubscriptionsFor = (ros: ROSLIB.Ros) => {
      subsRef.current.forEach((sub) => {
        sub.topic = new ROSLIB.Topic({
          ros,
          name: sub.topicName,
          messageType: sub.msgType,
        })
        sub.topic.subscribe((msg) => {
          sub.callbacks.forEach((cb) => cb(msg))
        })
      })
    }

    const tearDownSubscriptions = () => {
      subsRef.current.forEach((sub) => {
        sub.topic?.unsubscribe()
        sub.topic = null
      })
    }

    const startLatencyPings = (ros: ROSLIB.Ros) => {
      const svc = new ROSLIB.Service({
        ros,
        name: '/rosapi/get_time',
        serviceType: 'rosapi/GetTime',
      })
      pingTimer = setInterval(() => {
        const t0 = performance.now()
        svc.callService(
          new ROSLIB.ServiceRequest({}),
          () => {
            if (cancelled) return
            setLatencyMs(Math.round(performance.now() - t0))
          },
          () => undefined,
        )
      }, 2000)
    }

    const connect = () => {
      if (cancelled) return
      setStatus('connecting')
      setLastError(null)
      const ros = new ROSLIB.Ros({ url: rosUrl })
      rosRef.current = ros

      ros.on('connection', () => {
        if (cancelled) return
        backoffMs = 500
        setStatus('connected')
        setupSubscriptionsFor(ros)
        startLatencyPings(ros)
      })
      ros.on('error', (err: unknown) => {
        if (cancelled) return
        setStatus('error')
        setLastError(err instanceof Error ? err.message : String(err))
      })
      ros.on('close', () => {
        if (cancelled) return
        setStatus('closed')
        setLatencyMs(null)
        if (pingTimer) clearInterval(pingTimer)
        pingTimer = null
        tearDownSubscriptions()
        publishersRef.current.clear()
        reconnectTimer = setTimeout(connect, backoffMs)
        backoffMs = Math.min(backoffMs * 2, MAX_BACKOFF)
      })
    }

    connect()
    return () => {
      cancelled = true
      if (reconnectTimer) clearTimeout(reconnectTimer)
      if (pingTimer) clearInterval(pingTimer)
      tearDownSubscriptions()
      publishersRef.current.clear()
      rosRef.current?.close()
      rosRef.current = null
    }
  }, [mock, rosUrl])

  const subscribe = useCallback(
    <T,>(topicName: string, msgType: string, cb: (msg: T) => void) => {
      const key = `${topicName}|${msgType}`
      let sub = subsRef.current.get(key)
      if (!sub) {
        sub = {
          topicName,
          msgType,
          topic: null,
          callbacks: new Set(),
          mockTimer: null,
        }
        subsRef.current.set(key, sub)

        if (mock) {
          const hz = MOCK_RATES_HZ[msgType] ?? 5
          const intervalMs = Math.max(20, Math.round(1000 / hz))
          sub.mockTimer = setInterval(() => {
            const m = mockMessageFor(msgType)
            if (m == null) return
            sub!.callbacks.forEach((c) => c(m))
          }, intervalMs)
        } else if (rosRef.current && status === 'connected') {
          sub.topic = new ROSLIB.Topic({
            ros: rosRef.current,
            name: topicName,
            messageType: msgType,
          })
          sub.topic.subscribe((msg) => {
            sub!.callbacks.forEach((c) => c(msg))
          })
        }
      }
      sub.callbacks.add(cb as (msg: unknown) => void)

      return () => {
        const s = subsRef.current.get(key)
        if (!s) return
        s.callbacks.delete(cb as (msg: unknown) => void)
        if (s.callbacks.size === 0) {
          s.topic?.unsubscribe()
          if (s.mockTimer) clearInterval(s.mockTimer)
          subsRef.current.delete(key)
        }
      }
    },
    [mock, status],
  )

  const publish = useCallback(
    <T,>(topicName: string, msgType: string, msg: T) => {
      recordPublish(topicName, msg)
      if (mock) return
      if (!rosRef.current || status !== 'connected') return
      const key = `${topicName}|${msgType}`
      let pub = publishersRef.current.get(key)
      if (!pub) {
        pub = new ROSLIB.Topic({
          ros: rosRef.current,
          name: topicName,
          messageType: msgType,
        })
        publishersRef.current.set(key, pub)
      }
      pub.publish(new ROSLIB.Message(msg as object))
    },
    [mock, status, recordPublish],
  )

  const value = useMemo<RosCore>(
    () => ({
      status,
      url: mock ? 'mock://synthetic' : rosUrl,
      mode: mock ? 'mock' : 'real',
      latencyMs,
      lastError,
      subscribe,
      publish,
      publishLog: publishLogRef.current,
    }),
    [status, mock, rosUrl, latencyMs, lastError, subscribe, publish],
  )

  return <RosContext.Provider value={value}>{children}</RosContext.Provider>
}

export function useRos() {
  return useContext(RosContext)
}

export function useTopic<T>(
  topicName: string,
  msgType: string,
  options: { onMessage?: (msg: T) => void } = {},
): React.MutableRefObject<T | null> {
  const { subscribe } = useRos()
  const ref = useRef<T | null>(null)
  const onMessageRef = useRef(options.onMessage)
  onMessageRef.current = options.onMessage

  useEffect(() => {
    return subscribe<T>(topicName, msgType, (msg) => {
      ref.current = msg
      onMessageRef.current?.(msg)
    })
  }, [subscribe, topicName, msgType])

  return ref
}

export function usePublisher<T>(topicName: string, msgType: string) {
  const { publish } = useRos()
  return useCallback((msg: T) => publish(topicName, msgType, msg), [publish, topicName, msgType])
}
