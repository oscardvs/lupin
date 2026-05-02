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
  mockMissionState,
  mockObservation,
  mockOdometry,
  mockPlan,
  mockScan,
  mockTwinField,
  mockTwinState,
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
  // Match the orchestrator: MissionState ticks at 5 Hz unconditionally.
  // Observation polling at 5 Hz is safe — mockObservation() returns null
  // on every tick except the rising edge of PUBLISHING, so the subscriber
  // only sees one message per fake tag.
  'lupin_msgs/msg/MissionState': 5,
  'lupin_msgs/msg/Observation': 5,
  // Match the twin node: TwinState ticks at 1 Hz unconditionally.
  'lupin_msgs/msg/TwinState': 1,
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
    case 'lupin_msgs/msg/MissionState':
      return mockMissionState()
    case 'lupin_msgs/msg/Observation':
      return mockObservation()
    case 'lupin_msgs/msg/TwinState':
      return mockTwinState()
    default:
      return null
  }
}

/**
 * Type-aware mock service responses. Returning {success: true} for every
 * service call works for std_srvs/Trigger but breaks consumers that
 * inspect typed fields on the response (e.g. GetField). Add cases here
 * as we add typed services to the HMI.
 */
function mockServiceResponse(serviceType: string, request: unknown): unknown {
  switch (serviceType) {
    case 'lupin_msgs/srv/GetField':
      return mockTwinField(request as Parameters<typeof mockTwinField>[0])
    default:
      return { success: true }
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
        return Promise.resolve(mockServiceResponse(serviceType, request) as Res)
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
 * the chain is resolvable. In mock mode, samples `mockMapPose()` at 10 Hz.
 *
 * We subscribe to /tf and /tf_static directly instead of using ROSLIB.TFClient,
 * which depends on the legacy `tf2_web_republisher` ROS package (not packaged
 * for ROS 2 Jazzy and not present in our stack — rosbridge logs the missing
 * import as the symptom). Doing the chain composition in the browser avoids
 * the extra service node entirely and matches how rviz2 resolves transforms.
 *
 * The full transform tree is cached in a ref; we evaluate the requested chain
 * at 10 Hz so the React tree only re-renders that often (TF traffic can hit
 * hundreds of Hz on a mecanum base).
 */
type TfEntry = {
  parent: string
  x: number
  y: number
  qx: number
  qy: number
  qz: number
  qw: number
}

interface TfMessage {
  transforms: Array<{
    header: { frame_id: string }
    child_frame_id: string
    transform: {
      translation: { x: number; y: number; z: number }
      rotation: { x: number; y: number; z: number; w: number }
    }
  }>
}

function quatYaw(qx: number, qy: number, qz: number, qw: number): number {
  // Standard Z-axis yaw extraction from a quaternion.
  const siny_cosp = 2 * (qw * qz + qx * qy)
  const cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
  return Math.atan2(siny_cosp, cosy_cosp)
}

function composeChain(
  tree: Map<string, TfEntry>,
  target: string,
  fixed: string,
): MapPose | null {
  // Walk parents from `target` upward, accumulating fixed_T_target by
  // applying parent_T_child at each step. Bail after 32 hops to avoid
  // pathological cycles in a malformed tree.
  let cx = 0
  let cy = 0
  let cyaw = 0
  let current = target
  let safety = 32
  while (current !== fixed) {
    if (--safety < 0) return null
    const step = tree.get(current)
    if (!step) return null
    const stepYaw = quatYaw(step.qx, step.qy, step.qz, step.qw)
    const c = Math.cos(stepYaw)
    const s = Math.sin(stepYaw)
    const nx = c * cx - s * cy + step.x
    const ny = s * cx + c * cy + step.y
    cx = nx
    cy = ny
    cyaw = wrapAngle(cyaw + stepYaw)
    current = step.parent
  }
  return { x: cx, y: cy, yaw: cyaw }
}

function wrapAngle(a: number): number {
  // Keep yaw in (-π, π] so accumulated drift across many compositions stays bounded.
  while (a > Math.PI) a -= 2 * Math.PI
  while (a <= -Math.PI) a += 2 * Math.PI
  return a
}

export function useMapPose(mapFrame: string, baseFrame: string): MapPose | null {
  const { mode, status, rosRef } = useRos()
  const [pose, setPose] = useState<MapPose | null>(null)
  const treeRef = useRef<Map<string, TfEntry>>(new Map())

  useEffect(() => {
    if (mode === 'mock') {
      const tick = () => setPose(mockMapPose())
      tick()
      const id = setInterval(tick, 100)
      return () => clearInterval(id)
    }
    if (status !== 'connected' || !rosRef.current) return

    const ros = rosRef.current
    const tree = treeRef.current
    tree.clear()

    const ingest = (msg: unknown) => {
      const m = msg as TfMessage
      if (!m?.transforms) return
      for (const t of m.transforms) {
        tree.set(t.child_frame_id, {
          parent: t.header.frame_id,
          x: t.transform.translation.x,
          y: t.transform.translation.y,
          qx: t.transform.rotation.x,
          qy: t.transform.rotation.y,
          qz: t.transform.rotation.z,
          qw: t.transform.rotation.w,
        })
      }
    }

    // /tf gets the dynamic transforms (map→odom from SLAM, odom→base_link
    // from the wheel base). /tf_static is also subscribed in case a future
    // refactor anchors mapFrame or baseFrame off a static link — skipping
    // it would silently break that case.
    const tfTopic = new ROSLIB.Topic({
      ros, name: '/tf', messageType: 'tf2_msgs/msg/TFMessage',
    })
    const tfStaticTopic = new ROSLIB.Topic({
      ros, name: '/tf_static', messageType: 'tf2_msgs/msg/TFMessage',
    })
    tfTopic.subscribe(ingest)
    tfStaticTopic.subscribe(ingest)

    const evalTimer = setInterval(() => {
      const next = composeChain(tree, baseFrame, mapFrame)
      if (!next) return
      setPose((prev) => {
        if (!prev) return next
        // Skip the React update if nothing meaningful changed — keeps the
        // map view from re-rendering a hundred times a second.
        if (
          Math.abs(prev.x - next.x) < 0.005 &&
          Math.abs(prev.y - next.y) < 0.005 &&
          Math.abs(wrapAngle(prev.yaw - next.yaw)) < 0.005
        ) {
          return prev
        }
        return next
      })
    }, 100)

    return () => {
      clearInterval(evalTimer)
      try { tfTopic.unsubscribe() } catch { /* swallow */ }
      try { tfStaticTopic.unsubscribe() } catch { /* swallow */ }
      tree.clear()
    }
  }, [mode, status, rosRef, mapFrame, baseFrame])

  return pose
}
