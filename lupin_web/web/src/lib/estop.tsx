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

import { invertTwist } from '@/lib/polarity'
import { useRos, usePublisher, useService } from '@/lib/ros'
import { useSettings } from '@/lib/settings'
import type { Twist } from '@/types/ros'

// action_msgs/srv/CancelGoal request — empty goal_info cancels all active goals.
// This is the canonical Nav2 cancel-everything pattern: hitting both
// NavigateToPose and NavigateThroughPoses servers covers either entry point
// from RViz or our HMI map widget.
const CANCEL_ALL_REQ = {
  goal_info: {
    goal_id: { uuid: Array(16).fill(0) },
    stamp: { sec: 0, nanosec: 0 },
  },
} as const

const ZERO_TWIST: Twist = {
  linear: { x: 0, y: 0, z: 0 },
  angular: { x: 0, y: 0, z: 0 },
}

export type EStopReason =
  | 'user'
  | 'voice-agent'
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

// 20 Hz matches Nav2's velocity_smoother output rate — gives our zero Twists
// parity on the cmd_vel bus during the ~100 ms while the cancel_goal service
// call is in flight to Nav2 over rosbridge.
const ESTOP_HEARTBEAT_HZ = 20

export function EStopProvider({ children }: { children: ReactNode }) {
  const [{ cmdVelTopic, cmdVelType, estopAutoOnFocusLoss, polarityInvertHmi }] = useSettings()
  const { status } = useRos()
  const publishTwist = usePublisher<Twist>(cmdVelTopic, cmdVelType)
  // Mirror the HMI e-stop onto a dedicated soft-stop topic; the robot-side
  // estop_bridge ORs it into /e_stop_state so the mission orchestrator + LED
  // safety override react to the HMI STOP, not just the local cmd_vel gate.
  const publishHmiEstop = usePublisher<{ data: boolean }>('/lupin/hmi/estop', 'std_msgs/Bool')
  // Nav2 cancel hooks — e-stop alone can't beat Nav2 on the cmd_vel bus
  // (BEST_EFFORT, no QoS priority, both publish at 10–20 Hz). Cancelling
  // the active goal is what actually stops Nav2's velocity_smoother from
  // emitting Twists; heartbeat zeros then own the topic uncontested.
  const cancelNavigateToPose = useService<typeof CANCEL_ALL_REQ>(
    '/navigate_to_pose/_action/cancel_goal',
    'action_msgs/srv/CancelGoal',
  )
  const cancelNavigateThroughPoses = useService<typeof CANCEL_ALL_REQ>(
    '/navigate_through_poses/_action/cancel_goal',
    'action_msgs/srv/CancelGoal',
  )

  const [active, setActive] = useState<boolean>(true)
  const [reason, setReason] = useState<EStopReason | null>('startup')
  const wasConnected = useRef(false)

  const trigger = useCallback((r: EStopReason) => {
    setActive(true)
    setReason(r)
    // Fire and forget — both cancels run in parallel, errors swallowed
    // (action server may not exist in mock mode or before Nav2 is up).
    cancelNavigateToPose(CANCEL_ALL_REQ).catch(() => undefined)
    cancelNavigateThroughPoses(CANCEL_ALL_REQ).catch(() => undefined)
  }, [cancelNavigateToPose, cancelNavigateThroughPoses])

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

  // Publish the HMI e-stop state to the soft-stop topic. Republish at 2 Hz so a
  // late-joining / VOLATILE-subscribed estop_bridge converges; the bridge dedups
  // edges and republishes /e_stop_state itself.
  useEffect(() => {
    publishHmiEstop({ data: active })
    const id = setInterval(() => publishHmiEstop({ data: active }), 500)
    return () => clearInterval(id)
  }, [active, publishHmiEstop])

  const publishCmdVel = useCallback(
    (t: Twist) => {
      if (active) return // gate everything: nothing else publishes while e-stop is on
      publishTwist(invertTwist(t, polarityInvertHmi))
    },
    [active, publishTwist, polarityInvertHmi],
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
  'voice-agent': 'Voice agent engaged E-STOP',
  'visibility-hidden': 'Tab hidden — auto-stop',
  'window-blur': 'Window lost focus — auto-stop',
  'before-unload': 'Page unloading — auto-stop',
  'rosbridge-disconnect': 'rosbridge disconnected — auto-stop',
  startup: 'Released on startup — press Reset to enable teleop',
}
