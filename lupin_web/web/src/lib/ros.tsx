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
  mockMap,
  mockMapPose,
  mockOdometry,
  mockPlan,
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
  callService: <Req, Res = unknown>(
    serviceName: string,
    serviceType: string,
    request: Req,
  ) => Promise<Res>
  publishLog: PublishLogEntry[]
  /** Underlying ROSLIB.Ros instance — null in mock mode and while disconnected. Used by TFClient consumers. */
  rosRef: React.MutableRefObject<ROSLIB.Ros | null>
}

const noopRosRef: React.MutableRefObject<ROSLIB.Ros | null> = { current: null }
const noopCore: RosCore = {
  status: 'connecting',
  url: '',
  mode: 'real',
  latencyMs: null,
  lastError: null,
  subscribe: () => () => undefined,
  publish: () => undefined,
  callService: () => Promise.reject(new Error('rosbridge not ready')),
  publishLog: [],
  rosRef: noopRosRef,
}

const RosContext = createContext<RosCore>(noopCore)

const MOCK_RATES_HZ: Record<string, number> = {
  'sensor_msgs/msg/Imu': 50,
  'sensor_msgs/msg/LaserScan': 10,
  'nav_msgs/msg/Odometry': 30,
  'sensor_msgs/msg/JointState': 30,
  'sensor_msgs/msg/BatteryState': 1,
  'rcl_interfaces/msg/Log': 0.5,
  'nav_msgs/msg/OccupancyGrid': 0.2, // map updates rarely
  'nav_msgs/msg/Path': 5,
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
    case 'nav_msgs/msg/OccupancyGrid':
      return mockMap()
    case 'nav_msgs/msg/Path':
      return mockPlan()
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

  const callService = useCallback(
    <Req, Res = unknown>(
      serviceName: string,
      serviceType: string,
      request: Req,
    ): Promise<Res> => {
      recordPublish(`srv:${serviceName}`, request)
      if (mock) {
        return Promise.resolve({ success: true } as unknown as Res)
      }
      if (!rosRef.current || status !== 'connected') {
        return Promise.reject(new Error('rosbridge not connected'))
      }
      const svc = new ROSLIB.Service({
        ros: rosRef.current,
        name: serviceName,
        serviceType,
      })
      return new Promise<Res>((resolve, reject) => {
        svc.callService(
          new ROSLIB.ServiceRequest(request as object),
          (res: Res) => resolve(res),
          (err: unknown) =>
            reject(err instanceof Error ? err : new Error(String(err))),
        )
      })
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
      callService,
      publishLog: publishLogRef.current,
      rosRef,
    }),
    [status, mock, rosUrl, latencyMs, lastError, subscribe, publish, callService],
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

export function useService<Req, Res = unknown>(serviceName: string, serviceType: string) {
  const { callService } = useRos()
  return useCallback(
    (req: Req) => callService<Req, Res>(serviceName, serviceType, req),
    [callService, serviceName, serviceType],
  )
}

export interface MapPose {
  x: number
  y: number
  yaw: number
}

/**
 * Live transform from `mapFrame` → `baseFrame` as a 2D pose. Returns null until
 * the first transform arrives. In mock mode, samples `mockMapPose()` at 10 Hz.
 */
export function useMapPose(mapFrame: string, baseFrame: string): MapPose | null {
  const { mode, status, rosRef } = useRos()
  const [pose, setPose] = useState<MapPose | null>(null)

  useEffect(() => {
    if (mode === 'mock') {
      const tick = () => setPose(mockMapPose())
      tick()
      const id = setInterval(tick, 100)
      return () => clearInterval(id)
    }
    if (status !== 'connected' || !rosRef.current) return

    const tf = new ROSLIB.TFClient({
      ros: rosRef.current,
      fixedFrame: mapFrame,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 10,
    })

    const handler = (t: { translation: { x: number; y: number; z: number }; rotation: { x: number; y: number; z: number; w: number } }) => {
      const q = t.rotation
      const siny_cosp = 2 * (q.w * q.z + q.x * q.y)
      const cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
      const yaw = Math.atan2(siny_cosp, cosy_cosp)
      setPose({ x: t.translation.x, y: t.translation.y, yaw })
    }

    tf.subscribe(baseFrame, handler)
    return () => {
      try {
        tf.unsubscribe(baseFrame, handler)
      } catch {
        /* TFClient.unsubscribe can throw if already torn down — swallow */
      }
      try {
        tf.dispose()
      } catch {
        /* dispose may not exist on older roslib builds — swallow */
      }
    }
  }, [mode, status, rosRef, mapFrame, baseFrame])

  return pose
}
