/**
 * useVoiceSession — owns the lifecycle of a voice conversation.
 *
 * Wires together:
 *   - GeminiLiveClient (WebSocket protocol)
 *   - MicCapture / AudioPlayer (browser audio)
 *   - ROS publishers / services / cached telemetry (tool dispatch)
 *   - EStop (gates motion)
 *
 * Mock mode (?mock=1 OR no API key set) replaces the live client with a small
 * scripted harness so the tab demos end-to-end without a network or key.
 */

import ROSLIB from 'roslib'
import { useCallback, useEffect, useMemo, useRef, useState } from 'react'

import { AudioPlayer, MicCapture } from './audio'
import { GeminiLiveClient } from './gemini-live'
import { runMockSession, type MockHandle } from './mock'
import { ROBOT_TOOL_DECLARATIONS, clampNumber, type ToolName } from './tools'
import type { ToolInvocation, TranscriptTurn, VoiceStatus } from './types'

import { useEStop } from '@/lib/estop'
import { isMockMode, useSettings, type Settings } from '@/lib/settings'
import { useRos, useTopic } from '@/lib/ros'
import { ROS_TYPE, quatToEuler, type BatteryState, type Odometry, type PoseStamped } from '@/types/ros'

const TURN_HISTORY_CAP = 80
const TOOL_HISTORY_CAP = 40
const MAX_DRIVE_SECONDS = 2

export interface VoiceSession {
  status: VoiceStatus
  errorDetail: string | null
  transcript: TranscriptTurn[]
  tools: ToolInvocation[]
  micActive: boolean
  speakerMuted: boolean
  /** Begin a session: open WS, request mic if push-to-talk is off, etc. */
  start: () => Promise<void>
  /** Tear everything down. */
  stop: () => Promise<void>
  /** Push-to-talk: while held, mic streams. */
  beginUtterance: () => Promise<void>
  endUtterance: () => Promise<void>
  toggleSpeakerMuted: () => void
  /** Send a typed prompt (used by the text fallback and during dev). */
  sendText: (text: string) => void
  /** Whether the chosen mode would actually hit the network (i.e. has a key, not mocked). */
  isLive: boolean
  /** Live mic input level in [0, 1] — read inside an animation loop. */
  getInputLevel: () => number
  /** Live model-voice output level in [0, 1] — read inside an animation loop. */
  getOutputLevel: () => number
}

export function useVoiceSession(): VoiceSession {
  const [{ ...settings }] = useSettings() as readonly [Settings, unknown, unknown]
  const ros = useRos()
  const estop = useEStop()

  const [status, setStatus] = useState<VoiceStatus>('idle')
  const [errorDetail, setErrorDetail] = useState<string | null>(null)
  const [transcript, setTranscript] = useState<TranscriptTurn[]>([])
  const [tools, setTools] = useState<ToolInvocation[]>([])
  const [micActive, setMicActive] = useState(false)
  const [speakerMuted, setSpeakerMuted] = useState(false)

  // Live telemetry refs — read by query_state without forcing a re-render.
  const odomRef = useTopic<Odometry>(settings.odomTopic, ROS_TYPE.Odometry)
  const batteryRef = useTopic<BatteryState>(settings.batteryTopic, ROS_TYPE.BatteryState)

  const liveRef = useRef<GeminiLiveClient | null>(null)
  const micRef = useRef<MicCapture | null>(null)
  const playerRef = useRef<AudioPlayer | null>(null)
  const mockRef = useRef<MockHandle | null>(null)
  const driveTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null)

  const isLive = useMemo(
    () => !isMockMode() && settings.geminiApiKey.trim().length > 0,
    [settings.geminiApiKey],
  )

  /** Append a transcript turn, capped. Coalesces consecutive non-final turns of the same role. */
  const pushTranscript = useCallback((turn: TranscriptTurn) => {
    setTranscript((prev) => {
      const last = prev[prev.length - 1]
      if (last && !last.final && last.role === turn.role) {
        const next = prev.slice(0, -1)
        next.push({ ...turn, id: last.id })
        return next
      }
      const next = [...prev, turn]
      return next.length > TURN_HISTORY_CAP ? next.slice(-TURN_HISTORY_CAP) : next
    })
  }, [])

  const pushTool = useCallback((inv: ToolInvocation) => {
    setTools((prev) => {
      const idx = prev.findIndex((t) => t.id === inv.id)
      const next = idx >= 0 ? [...prev.slice(0, idx), inv, ...prev.slice(idx + 1)] : [...prev, inv]
      return next.length > TOOL_HISTORY_CAP ? next.slice(-TOOL_HISTORY_CAP) : next
    })
  }, [])

  /** ───────────────────── Tool dispatch ───────────────────── */

  const dispatchTool = useCallback(
    async (
      name: string,
      id: string,
      args: Record<string, unknown>,
    ): Promise<Record<string, unknown>> => {
      const startedAt = Date.now()
      pushTool({ id, name, args, startedAt })

      const finish = (
        result: Record<string, unknown>,
        extra?: Partial<ToolInvocation>,
      ): Record<string, unknown> => {
        pushTool({ id, name, args, startedAt, result, ...extra })
        return result
      }

      try {
        switch (name as ToolName) {
          case 'stop': {
            ros.publish(settings.cmdVelTopic, settings.cmdVelType, {
              linear: { x: 0, y: 0, z: 0 },
              angular: { x: 0, y: 0, z: 0 },
            })
            if (driveTimerRef.current) {
              clearTimeout(driveTimerRef.current)
              driveTimerRef.current = null
            }
            return finish({ ok: true, action: 'stopped' })
          }

          case 'drive': {
            if (estop.active) {
              return finish(
                { ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` },
              )
            }
            const lx = clampNumber(args.linear_x, -settings.voiceMaxLinearMps, settings.voiceMaxLinearMps)
            const ly = clampNumber(args.linear_y, -settings.voiceMaxLinearMps, settings.voiceMaxLinearMps)
            const az = clampNumber(args.angular_z, -settings.voiceMaxAngularRps, settings.voiceMaxAngularRps)
            const dur = clampNumber(args.duration_s, 0.1, MAX_DRIVE_SECONDS, 0.5)
            const twist = {
              linear: { x: lx, y: ly, z: 0 },
              angular: { x: 0, y: 0, z: az },
            }
            ros.publish(settings.cmdVelTopic, settings.cmdVelType, twist)
            if (driveTimerRef.current) clearTimeout(driveTimerRef.current)
            driveTimerRef.current = setTimeout(() => {
              ros.publish(settings.cmdVelTopic, settings.cmdVelType, {
                linear: { x: 0, y: 0, z: 0 },
                angular: { x: 0, y: 0, z: 0 },
              })
              driveTimerRef.current = null
            }, Math.round(dur * 1000))
            return finish({ ok: true, action: 'drive_burst', linear_x: lx, linear_y: ly, angular_z: az, duration_s: dur })
          }

          case 'nav_goto': {
            if (estop.active) {
              return finish(
                { ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` },
              )
            }
            const x = clampNumber(args.x, -1e3, 1e3)
            const y = clampNumber(args.y, -1e3, 1e3)
            const yaw = clampNumber(args.yaw, -Math.PI, Math.PI)
            publishGoal(ros, settings, x, y, yaw)
            return finish({ ok: true, action: 'goal_sent', x, y, yaw })
          }

          case 'nav_cancel': {
            const rosLib = ros.rosRef.current
            if (!rosLib) {
              return finish({ ok: false, error: 'rosbridge not connected' })
            }
            try {
              // Empty CancelGoal request cancels all goals on the action server.
              const result = await new Promise<Record<string, unknown>>((resolve, reject) => {
                const svc = new ROSLIB.Service({
                  ros: rosLib,
                  name: '/navigate_to_pose/_action/cancel_goal',
                  serviceType: 'action_msgs/srv/CancelGoal',
                })
                svc.callService(
                  new ROSLIB.ServiceRequest({}),
                  (res: unknown) => resolve(res as Record<string, unknown>),
                  (err: unknown) => reject(err instanceof Error ? err : new Error(String(err))),
                )
              })
              return finish({ ok: true, action: 'nav_cancel', response: result })
            } catch (e) {
              return finish({
                ok: false,
                error: `nav cancel service unavailable: ${e instanceof Error ? e.message : String(e)}`,
              })
            }
          }

          case 'engage_estop': {
            estop.trigger('user')
            return finish({ ok: true, action: 'estop_engaged' })
          }

          case 'nav_goto_named': {
            const key = String(args.name ?? '')
            const loc = settings.voiceNamedLocations[key]
            if (!loc) {
              return finish({
                ok: false,
                error: `unknown location "${key}". Known: ${Object.keys(settings.voiceNamedLocations).join(', ') || '<none>'}`,
              })
            }
            if (estop.active) {
              return finish(
                { ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` },
              )
            }
            publishGoal(ros, settings, loc.x, loc.y, loc.yaw)
            return finish({ ok: true, action: 'goal_sent', name: key, ...loc })
          }

          case 'arm_preset': {
            if (estop.active) {
              return finish(
                { ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` },
              )
            }
            const preset = String(args.name ?? '')
            // Call the arm preset service directly via the underlying ROSLIB
            // handle. This avoids depending on a `callService` shim in ros.tsx
            // that lives on a different feature branch — keeps merge conflicts
            // with feat/web-arm-control to a minimum.
            const rosLib = ros.rosRef.current
            if (!rosLib) {
              return finish({ ok: false, error: 'rosbridge not connected' })
            }
            try {
              const result = await new Promise<Record<string, unknown>>((resolve, reject) => {
                const svc = new ROSLIB.Service({
                  ros: rosLib,
                  name: '/lupin/arm/preset',
                  serviceType: 'lupin_msgs/srv/SetArmPreset',
                })
                svc.callService(
                  new ROSLIB.ServiceRequest({ name: preset }),
                  (res: unknown) => resolve(res as Record<string, unknown>),
                  (err: unknown) => reject(err instanceof Error ? err : new Error(String(err))),
                )
              })
              return finish({ ok: true, action: 'arm_preset', name: preset, response: result })
            } catch (e) {
              return finish({
                ok: false,
                error: `arm service unavailable: ${e instanceof Error ? e.message : String(e)}`,
              })
            }
          }

          case 'query_state': {
            const fields = Array.isArray(args.fields) ? (args.fields as string[]) : ['pose', 'battery', 'estop']
            const out: Record<string, unknown> = {}
            if (fields.includes('pose')) out.pose = readPose(odomRef.current)
            if (fields.includes('battery')) out.battery = readBattery(batteryRef.current)
            if (fields.includes('estop')) out.estop = { active: estop.active, reason: estop.reason }
            if (fields.includes('nav_status')) out.nav_status = { note: 'not yet wired — subscribe /navigation_state' }
            return finish({ ok: true, ...out })
          }

          case 'speak': {
            const text = String(args.text ?? '')
            pushTranscript({
              id: `speak-${id}`,
              role: 'model',
              text,
              startedAt,
              final: true,
            })
            return finish({ ok: true })
          }

          default:
            return finish({ ok: false, error: `unknown tool: ${name}` })
        }
      } catch (e) {
        const error = e instanceof Error ? e.message : String(e)
        return finish({ ok: false, error }, { error })
      }
    },
    [ros, settings, estop, odomRef, batteryRef, pushTool, pushTranscript],
  )

  /** ───────────────────── Live mode wiring ───────────────────── */

  const buildSystemInstruction = useCallback((): string => {
    const names = Object.keys(settings.voiceNamedLocations)
    const locLine = names.length
      ? `Known named locations: ${names.join(', ')}.`
      : 'No named locations are configured yet.'
    return `${settings.voiceSystemPrompt}\n\n${locLine}`
  }, [settings.voiceSystemPrompt, settings.voiceNamedLocations])

  const start = useCallback(async () => {
    if (status !== 'idle' && status !== 'error') return
    setErrorDetail(null)
    setTranscript([])
    setTools([])

    if (!isLive) {
      // Mock harness — runs an internal scripted session.
      setStatus('connecting')
      mockRef.current = runMockSession({
        onStatus: (s, detail) => {
          setStatus(s)
          if (detail) setErrorDetail(detail)
        },
        onTranscript: pushTranscript,
        onToolCall: async (name, id, args) => dispatchTool(name, id, args),
      })
      return
    }

    // Real session.
    setStatus('connecting')
    playerRef.current = new AudioPlayer()
    playerRef.current.setMuted(speakerMuted)

    const client = new GeminiLiveClient({
      apiKey: settings.geminiApiKey.trim(),
      model: settings.geminiModel,
      systemInstruction: buildSystemInstruction(),
      languageCode: settings.voiceLanguage,
      tools: ROBOT_TOOL_DECLARATIONS,
      onAudio: (b64) => {
        playerRef.current?.enqueue(b64)
        setStatus((s) => (s === 'thinking' || s === 'ready' || s === 'listening' ? 'speaking' : s))
      },
      onModelText: (text, final) => {
        pushTranscript({
          id: `model-${Math.floor(Date.now() / 1000)}`,
          role: 'model',
          text,
          startedAt: Date.now(),
          final,
        })
      },
      onUserText: (text, final) => {
        pushTranscript({
          id: `user-${Math.floor(Date.now() / 1000)}`,
          role: 'user',
          text,
          startedAt: Date.now(),
          final,
        })
      },
      onToolCall: async (name, id, args) => dispatchTool(name, id, args),
      onTurnComplete: () => setStatus('ready'),
      onInterrupted: () => {
        playerRef.current?.flush()
        setStatus('listening')
      },
      onStatus: (s, detail) => {
        if (detail) setErrorDetail(detail)
        if (s === 'connecting') setStatus('connecting')
        if (s === 'open') setStatus('connecting')
        if (s === 'ready') setStatus('ready')
        if (s === 'closed') setStatus('idle')
        if (s === 'error') setStatus('error')
      },
    })
    liveRef.current = client
    client.open()

    if (!settings.voicePushToTalk) {
      // Hands-free: open mic immediately and keep it open.
      micRef.current = new MicCapture()
      try {
        await micRef.current.start((frame) => client.sendAudio(frame))
        setMicActive(true)
      } catch (e) {
        setStatus('error')
        setErrorDetail(`mic: ${e instanceof Error ? e.message : String(e)}`)
      }
    }
  }, [
    status,
    isLive,
    settings.geminiApiKey,
    settings.geminiModel,
    settings.voiceLanguage,
    settings.voicePushToTalk,
    buildSystemInstruction,
    dispatchTool,
    pushTranscript,
    speakerMuted,
  ])

  const stop = useCallback(async () => {
    if (mockRef.current) {
      mockRef.current.stop()
      mockRef.current = null
    }
    if (driveTimerRef.current) {
      clearTimeout(driveTimerRef.current)
      driveTimerRef.current = null
    }
    if (micRef.current) {
      await micRef.current.stop()
      micRef.current = null
    }
    if (playerRef.current) {
      await playerRef.current.close()
      playerRef.current = null
    }
    if (liveRef.current) {
      liveRef.current.close()
      liveRef.current = null
    }
    setMicActive(false)
    setStatus('idle')
  }, [])

  const beginUtterance = useCallback(async () => {
    if (!liveRef.current && !mockRef.current) return
    if (!settings.voicePushToTalk) return
    if (!micRef.current) micRef.current = new MicCapture()
    try {
      await micRef.current.start((frame) => liveRef.current?.sendAudio(frame))
      setMicActive(true)
      setStatus('listening')
    } catch (e) {
      setErrorDetail(`mic: ${e instanceof Error ? e.message : String(e)}`)
      setStatus('error')
    }
  }, [settings.voicePushToTalk])

  const endUtterance = useCallback(async () => {
    if (!micRef.current) return
    if (!settings.voicePushToTalk) return
    await micRef.current.stop()
    micRef.current = null
    setMicActive(false)
    setStatus((s) => (s === 'listening' ? 'thinking' : s))
  }, [settings.voicePushToTalk])

  const toggleSpeakerMuted = useCallback(() => {
    setSpeakerMuted((m) => {
      const next = !m
      playerRef.current?.setMuted(next)
      return next
    })
  }, [])

  const sendText = useCallback(
    (text: string) => {
      if (mockRef.current) {
        mockRef.current.sendText(text)
        return
      }
      liveRef.current?.sendUserText(text)
      pushTranscript({
        id: `user-typed-${Date.now()}`,
        role: 'user',
        text,
        startedAt: Date.now(),
        final: true,
      })
    },
    [pushTranscript],
  )

  // Tear down on unmount.
  useEffect(() => {
    return () => {
      void stop()
    }
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [])

  // E-stop trip while a session is running → flush playback (don't kill mic; user
  // can still ask the agent to query state).
  useEffect(() => {
    if (estop.active) playerRef.current?.flush()
  }, [estop.active])

  const getInputLevel = useCallback(() => micRef.current?.getLevel() ?? 0, [])
  const getOutputLevel = useCallback(() => playerRef.current?.getLevel() ?? 0, [])

  return {
    status,
    errorDetail,
    transcript,
    tools,
    micActive,
    speakerMuted,
    start,
    stop,
    beginUtterance,
    endUtterance,
    toggleSpeakerMuted,
    sendText,
    isLive,
    getInputLevel,
    getOutputLevel,
  }
}

function publishGoal(
  ros: ReturnType<typeof useRos>,
  settings: Settings,
  x: number,
  y: number,
  yaw: number,
): void {
  const half = yaw / 2
  const goal: PoseStamped = {
    header: {
      stamp: { sec: Math.floor(Date.now() / 1000), nanosec: (Date.now() % 1000) * 1e6 },
      frame_id: settings.mapFrame,
    },
    pose: {
      position: { x, y, z: 0 },
      orientation: { x: 0, y: 0, z: Math.sin(half), w: Math.cos(half) },
    },
  }
  ros.publish(settings.goalPoseTopic, ROS_TYPE.PoseStamped, goal)
}

function readPose(odom: Odometry | null): Record<string, number> | null {
  if (!odom) return null
  const p = odom.pose.pose.position
  const yaw = quatToEuler(odom.pose.pose.orientation).yaw
  return { x: p.x, y: p.y, yaw }
}

function readBattery(b: BatteryState | null): Record<string, number> | null {
  if (!b) return null
  return { voltage: b.voltage, percentage: b.percentage }
}
