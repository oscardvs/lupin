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

import { useRos, usePublisher } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import type { Twist } from '@/types/ros'

const ZERO_TWIST: Twist = {
  linear: { x: 0, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0 },
}

export type EStopReason =
  | 'user'
  | 'visibility-hidden'
  | 'window-blur'
  | 'before-unload'
  | 'rosbridge-disconnect'
  | 'startup'

interface EStopValue {
  active: boolean
  reason: EStopReason | null
  trigger: (reason: EStopReason) => void
  reset: () => void
  publishCmdVel: (twist: Twist) => void
}

const EStopContext = createContext<EStopValue>({
  active: true,
  reason: 'startup',
  trigger: () => undefined,
  reset: () => undefined,
  publishCmdVel: () => undefined,
})

const ESTOP_HEARTBEAT_HZ = 10

export function EStopProvider({ children }: { children: ReactNode }) {
  const [{ cmdVelTopic, cmdVelType, estopAutoOnFocusLoss }] = useSettings()
  const { status } = useRos()
  const publishTwist = usePublisher<Twist>(cmdVelTopic, cmdVelType)

  const [active, setActive] = useState<boolean>(true)
  const [reason, setReason] = useState<EStopReason | null>('startup')
  const wasConnected = useRef(false)

  const trigger = useCallback((r: EStopReason) => {
    setActive(true)
    setReason(r)
  }, [])

  const reset = useCallback(() => {
    setActive(false)
    setReason(null)
  }, [])

  // beforeunload always fires e-stop — that's the page actually closing.
  useEffect(() => {
    const onBeforeUnload = () => trigger('before-unload')
    window.addEventListener('beforeunload', onBeforeUnload)
    return () => window.removeEventListener('beforeunload', onBeforeUnload)
  }, [trigger])

  // visibilitychange / window blur are gated by Settings → Safety so dev tab
  // switching doesn't fire e-stop constantly. Default-on for the real robot.
  useEffect(() => {
    if (!estopAutoOnFocusLoss) return
    const onVisibility = () => {
      if (document.visibilityState === 'hidden') trigger('visibility-hidden')
    }
    const onBlur = () => trigger('window-blur')
    document.addEventListener('visibilitychange', onVisibility)
    window.addEventListener('blur', onBlur)
    return () => {
      document.removeEventListener('visibilitychange', onVisibility)
      window.removeEventListener('blur', onBlur)
    }
  }, [trigger, estopAutoOnFocusLoss])

  // rosbridge connection drop → trigger. Only fire after we have a successful
  // connection at least once, so initial connecting state doesn't auto-trigger.
  useEffect(() => {
    if (status === 'connected') {
      wasConnected.current = true
    } else if (wasConnected.current && status !== 'connecting') {
      trigger('rosbridge-disconnect')
    }
  }, [status, trigger])

  // Heartbeat: while active, publish zero Twist at 10 Hz.
  useEffect(() => {
    if (!active) return
    publishTwist(ZERO_TWIST)
    const id = setInterval(() => publishTwist(ZERO_TWIST), Math.round(1000 / ESTOP_HEARTBEAT_HZ))
    return () => clearInterval(id)
  }, [active, publishTwist])

  const publishCmdVel = useCallback(
    (t: Twist) => {
      if (active) return // gate everything: nothing else publishes while e-stop is on
      publishTwist(t)
    },
    [active, publishTwist],
  )

  const value = useMemo<EStopValue>(
    () => ({ active, reason, trigger, reset, publishCmdVel }),
    [active, reason, trigger, reset, publishCmdVel],
  )

  return <EStopContext.Provider value={value}>{children}</EStopContext.Provider>
}

export function useEStop() {
  return useContext(EStopContext)
}

/** The only way teleop / autonomy / anything should send a Twist. */
export function useCmdVel() {
  const { publishCmdVel, active } = useEStop()
  return { publish: publishCmdVel, blocked: active }
}

export const ESTOP_REASON_LABELS: Record<EStopReason, string> = {
  user: 'User pressed E-STOP',
  'visibility-hidden': 'Tab hidden — auto-stop',
  'window-blur': 'Window lost focus — auto-stop',
  'before-unload': 'Page unloading — auto-stop',
  'rosbridge-disconnect': 'rosbridge disconnected — auto-stop',
  startup: 'Released on startup — press Reset to enable teleop',
}
