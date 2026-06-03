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

import { useCallback, useEffect, useMemo, useRef, useState } from 'react'

import { AudioPlayer, MicCapture } from './audio'
import { GeminiLiveClient } from './gemini-live'
import { runMockSession, type MockHandle } from './mock'
import { ROBOT_TOOL_DECLARATIONS, clampNumber, type ToolName } from './tools'
import type { ToolInvocation, TranscriptTurn, VoiceStatus } from './types'
import { SpeechEndpointer } from './vad'

import { GRIPPER_CLOSE_DEG, GRIPPER_HMI_MAX_DEG, GRIPPER_OPEN_DEG } from '@/lib/arm'
import { useEStop } from '@/lib/estop'
import { invertTwist } from '@/lib/polarity'
import { isMockMode, useSettings, type Settings, type VoiceNamedLocation } from '@/lib/settings'
import { useMapPose, useRos, useTopic, type MapPose } from '@/lib/ros'
import {
  MIRTE_SRV,
  ROS_TYPE,
  quatToEuler,
  type BatteryState,
  type Odometry,
  type PoseStamped,
  type SetServoAngleWithSpeedRequest,
} from '@/types/ros'

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
  /** Open the mic for an utterance. The speech endpointer auto-ends it on silence. */
  beginUtterance: () => Promise<void>
  /** Close the mic. Called by the user (tap again) or by the endpointer. */
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
  const [settings, updateSettings] = useSettings()
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

  // Map-frame pose, used by save_named_location. Mirrored to a ref so the
  // dispatcher reads the latest value without re-creating itself per tick.
  const mapPose = useMapPose(settings.mapFrame, settings.baseFrame)
  const mapPoseRef = useRef<MapPose | null>(null)
  useEffect(() => {
    mapPoseRef.current = mapPose
  }, [mapPose])

  // Saved arm pose/sequence names, fetched once at session start so
  // buildSystemInstruction can list them for the model and arm_save_pose can
  // append new names mid-session.
  const armLibraryRef = useRef<{ poses: string[]; sequences: string[] }>({ poses: [], sequences: [] })

  const liveRef = useRef<GeminiLiveClient | null>(null)
  const micRef = useRef<MicCapture | null>(null)
  const playerRef = useRef<AudioPlayer | null>(null)
  const mockRef = useRef<MockHandle | null>(null)
  const vadRef = useRef<SpeechEndpointer | null>(null)
  // True between the moment beginUtterance starts opening the mic and the
  // moment that promise settles. Dedupes double-taps and tells endUtterance to
  // back off until the start completes.
  const startingRef = useRef<boolean>(false)
  // endUtterance is async + needs to read latest settings, so the VAD callback
  // dispatches through this ref rather than capturing a stale closure.
  const endUtteranceRef = useRef<(reason?: 'silence' | 'max-duration') => void>(() => {})
  // Mirrors settings.voicePushToTalk so the VAD's onSpeechEnd callback reads
  // the live value at fire time — toggling the mode mid-session shouldn't
  // strand a stale `mode` baked into the endpointer's constructor.
  const voicePushToTalkRef = useRef(settings.voicePushToTalk)
  useEffect(() => {
    voicePushToTalkRef.current = settings.voicePushToTalk
  }, [settings.voicePushToTalk])
  const driveTimerRef = useRef<ReturnType<typeof setTimeout> | null>(null)
  // Live multiplier on the voiceMax* speed caps. Adjusted by set_speed_cap, not
  // persisted — per-session by design so a "go slower" command doesn't bleed
  // into the next conversation.
  const speedCapRef = useRef<number>(1)

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
            const cap = speedCapRef.current
            const linMax = settings.voiceMaxLinearMps * cap
            const angMax = settings.voiceMaxAngularRps * cap
            const lx = clampNumber(args.linear_x, -linMax, linMax)
            const ly = clampNumber(args.linear_y, -linMax, linMax)
            const az = clampNumber(args.angular_z, -angMax, angMax)
            const dur = clampNumber(args.duration_s, 0.1, MAX_DRIVE_SECONDS, 0.5)
            const twist = invertTwist(
              {
                linear: { x: lx, y: ly, z: 0 },
                angular: { x: 0, y: 0, z: az },
              },
              settings.polarityInvertHmi,
            )
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

          case 'nav_forward': {
            if (estop.active) {
              return finish(
                { ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` },
              )
            }
            const pose = mapPoseRef.current
            if (!pose) {
              return finish({
                ok: false,
                error: 'no map→base transform — cannot compute relative goal',
              })
            }
            const fwdRaw = clampNumber(args.forward_m, -50, 50, 0)
            const latRaw = clampNumber(args.lateral_m, -50, 50, 0)
            const rotDegRaw = clampNumber(args.rotate_deg, -3600, 3600, 0)
            // User talks in physical-chassis frame; controller frame is rotated
            // 180° on this unit. Flip body-frame offsets when calibration is on.
            const sign = settings.polarityInvertHmi ? -1 : 1
            const fwd = fwdRaw * sign
            const lat = latRaw * sign
            const rotDeg = rotDegRaw * sign
            // Rotate body-frame offset (fwd, lat) by current yaw to get the
            // map-frame delta, then add to current pose.
            const c = Math.cos(pose.yaw)
            const s = Math.sin(pose.yaw)
            const dx = c * fwd - s * lat
            const dy = s * fwd + c * lat
            const goalX = pose.x + dx
            const goalY = pose.y + dy
            let goalYaw = pose.yaw + (rotDeg * Math.PI) / 180
            goalYaw = Math.atan2(Math.sin(goalYaw), Math.cos(goalYaw))
            publishGoal(ros, settings, goalX, goalY, goalYaw)
            return finish({
              ok: true,
              action: 'goal_sent',
              from: { x: pose.x, y: pose.y, yaw: pose.yaw },
              goal: { x: goalX, y: goalY, yaw: goalYaw },
              forward_m: fwd,
              lateral_m: lat,
              rotate_deg: rotDeg,
            })
          }

          case 'rotate': {
            if (estop.active) {
              return finish(
                { ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` },
              )
            }
            const pose = mapPoseRef.current
            if (!pose) {
              return finish({
                ok: false,
                error: 'no map→base transform — cannot compute relative rotation goal',
              })
            }
            const angleDegRaw = clampNumber(args.angle_deg, -3600, 3600)
            // User's "+90 deg = turn left" in physical chassis frame. Internal
            // frame is yaw-flipped on this unit, so flip the delta when the
            // calibration is on.
            const angleDeg = settings.polarityInvertHmi ? -angleDegRaw : angleDegRaw
            const deltaRad = (angleDeg * Math.PI) / 180
            // Wrap to [-π, π] so Nav2 takes the shortest direction.
            let yaw = pose.yaw + deltaRad
            yaw = Math.atan2(Math.sin(yaw), Math.cos(yaw))
            publishGoal(ros, settings, pose.x, pose.y, yaw)
            return finish({
              ok: true,
              action: 'rotate_goal_sent',
              from_yaw: pose.yaw,
              to_yaw: yaw,
              delta_deg: angleDegRaw,
            })
          }

          case 'set_speed_cap': {
            const v = clampNumber(args.value, 0, 1, 1)
            speedCapRef.current = v
            return finish({
              ok: true,
              action: 'speed_cap_set',
              value: v,
              effective_max_linear_mps: settings.voiceMaxLinearMps * v,
              effective_max_angular_rps: settings.voiceMaxAngularRps * v,
            })
          }

          case 'nav_cancel': {
            try {
              // Empty CancelGoal request (zero UUID + zero stamp) is the
              // "cancel all" sentinel on the Nav2 action server.
              const result = await ros.callService<Record<string, never>, Record<string, unknown>>(
                '/navigate_to_pose/_action/cancel_goal',
                'action_msgs/srv/CancelGoal',
                {},
              )
              return finish({ ok: true, action: 'nav_cancel', response: result })
            } catch (e) {
              return finish({
                ok: false,
                error: `nav cancel service unavailable: ${e instanceof Error ? e.message : String(e)}`,
              })
            }
          }

          case 'engage_estop': {
            estop.trigger('voice-agent')
            return finish({ ok: true, action: 'estop_engaged', reason: 'voice-agent' })
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

          case 'list_named_locations': {
            return finish({ ok: true, locations: settings.voiceNamedLocations })
          }

          case 'save_named_location': {
            const key = String(args.name ?? '').trim()
            if (!key) {
              return finish({ ok: false, error: 'name is required' })
            }
            const pose = mapPoseRef.current
            if (!pose) {
              return finish({
                ok: false,
                error:
                  'no map→base transform available — localization may not be running. Cannot save a map-frame pose.',
              })
            }
            const loc: VoiceNamedLocation = { x: pose.x, y: pose.y, yaw: pose.yaw }
            updateSettings({
              voiceNamedLocations: { ...settings.voiceNamedLocations, [key]: loc },
            })
            return finish({ ok: true, action: 'saved', name: key, ...loc })
          }

          case 'gripper': {
            if (estop.active) {
              return finish(
                { ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` },
              )
            }
            const action = String(args.action ?? '').toLowerCase()
            if (action !== 'open' && action !== 'close' && action !== 'set') {
              return finish({ ok: false, error: "action must be 'open', 'close', or 'set'" })
            }
            // Gripper window + direction come from the shared lib/arm source of
            // truth (GRIPPER_OPEN_DEG / GRIPPER_CLOSE_DEG), so the voice tool can
            // never drift from the Arm page or the backend bridge. The window is
            // still UNVERIFIED against the real mechanical stops. Service path is
            // the gripper_action_bridge, NOT the raw Hiwonder service.
            //
            // On Mirte-247264 the mechanically-open jaw is NEGATIVE HMI degrees
            // (URDF gripper_joint < 0): 0% closed → +30°, 100% open → -30°.
            const OPEN_DEG = GRIPPER_OPEN_DEG
            const CLOSE_DEG = GRIPPER_CLOSE_DEG
            let percent: number
            let angle: number
            if (action === 'set') {
              if (typeof args.percent !== 'number' || Number.isNaN(args.percent)) {
                return finish({
                  ok: false,
                  error: "action='set' requires a numeric 'percent' in [0, 100]",
                })
              }
              percent = clampNumber(args.percent, 0, 100, 0)
              angle = CLOSE_DEG + (percent / 100) * (OPEN_DEG - CLOSE_DEG)
            } else {
              percent = action === 'open' ? 100 : 0
              angle = action === 'open' ? OPEN_DEG : CLOSE_DEG
            }
            try {
              const res = await ros.callService<SetServoAngleWithSpeedRequest, { status: boolean }>(
                '/lupin/gripper/set_angle_with_speed',
                MIRTE_SRV.SetServoAngleWithSpeed,
                { angle, rate: settings.armRateDegPerSec, degrees: true },
              )
              return finish({
                ok: true,
                action: 'gripper',
                direction: action,
                percent,
                angle_deg: angle,
                note: `gripper range is unverified — angles capped to ±${GRIPPER_HMI_MAX_DEG}°`,
                response: res,
              })
            } catch (e) {
              return finish({
                ok: false,
                error: `gripper service unavailable: ${e instanceof Error ? e.message : String(e)}`,
              })
            }
          }

          case 'arm_preset': {
            if (estop.active) {
              return finish(
                { ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` },
              )
            }
            const preset = String(args.name ?? '').trim().toLowerCase()
            if (!preset) {
              return finish({ ok: false, error: 'arm_preset: name is required' })
            }
            try {
              const result = await ros.callService<
                { name: string },
                { success: boolean; message: string }
              >('/lupin/arm/preset', 'lupin_msgs/srv/SetArmPreset', { name: preset })
              const ok = !!result.success
              return finish({
                ok,
                action: 'arm_preset',
                name: preset,
                message: result.message,
                ...(ok ? {} : { error: result.message }),
              })
            } catch (e) {
              return finish({
                ok: false,
                error: `arm service unavailable: ${e instanceof Error ? e.message : String(e)}`,
              })
            }
          }

          case 'arm_goto_pose': {
            if (estop.active) {
              return finish({ ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` })
            }
            const poseName = String(args.name ?? '').trim().toLowerCase()
            if (!poseName) return finish({ ok: false, error: 'arm_goto_pose: name is required' })
            const builtin = ['home', 'zero', 'tuck', 'pick', 'place', 'inspect'].includes(poseName)
            const svc = builtin ? '/lupin/arm/preset' : '/lupin/arm/library/goto_pose'
            try {
              const r = await ros.callService<{ name: string }, { success: boolean; message: string }>(
                svc, 'lupin_msgs/srv/SetArmPreset', { name: poseName })
              return finish({ ok: !!r.success, action: 'arm_goto_pose', name: poseName, message: r.message,
                ...(r.success ? {} : { error: r.message }) })
            } catch (e) {
              return finish({ ok: false, error: `arm library unavailable: ${e instanceof Error ? e.message : String(e)}` })
            }
          }

          case 'arm_save_pose': {
            const poseName = String(args.name ?? '').trim()
            if (!poseName) return finish({ ok: false, error: 'arm_save_pose: name is required' })
            try {
              const r = await ros.callService<
                { name: string; from_current: boolean; arm_rad: number[]; gripper_rad: number; has_gripper: boolean; overwrite: boolean },
                { success: boolean; message: string }
              >('/lupin/arm/library/save_pose', 'lupin_msgs/srv/SaveArmPose',
                { name: poseName, from_current: true, arm_rad: [], gripper_rad: 0, has_gripper: false, overwrite: true })
              if (r.success) armLibraryRef.current.poses = Array.from(new Set([...armLibraryRef.current.poses, poseName.toLowerCase()]))
              return finish({ ok: !!r.success, action: 'arm_save_pose', name: poseName, message: r.message,
                ...(r.success ? {} : { error: r.message }) })
            } catch (e) {
              return finish({ ok: false, error: `arm library unavailable: ${e instanceof Error ? e.message : String(e)}` })
            }
          }

          case 'arm_run_sequence': {
            if (estop.active) {
              return finish({ ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` })
            }
            const seqName = String(args.name ?? '').trim().toLowerCase()
            if (!seqName) return finish({ ok: false, error: 'arm_run_sequence: name is required' })
            const speed = clampNumber(args.speed, 0.25, 2, 1)
            try {
              const r = await ros.callService<{ name: string; speed: number }, { success: boolean; message: string }>(
                '/lupin/arm/library/play', 'lupin_msgs/srv/PlayArmSequence', { name: seqName, speed })
              return finish({ ok: !!r.success, action: 'arm_run_sequence', name: seqName, speed, message: r.message,
                ...(r.success ? {} : { error: r.message }) })
            } catch (e) {
              return finish({ ok: false, error: `arm library unavailable: ${e instanceof Error ? e.message : String(e)}` })
            }
          }

          case 'arm_record': {
            const action = String(args.action ?? '').trim().toLowerCase()
            if (!['start', 'save', 'cancel'].includes(action))
              return finish({ ok: false, error: "arm_record: action must be start|save|cancel" })
            const mode = String(args.mode ?? 'teleop').trim().toLowerCase()
            if (action === 'start' && estop.active) {
              return finish({ ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` })
            }
            try {
              const r = await ros.callService<
                { action: string; name: string; mode: string; include_gripper: boolean; overwrite: boolean },
                { success: boolean; message: string; duration_s: number; n_waypoints: number }
              >('/lupin/arm/library/record', 'lupin_msgs/srv/ArmRecord',
                { action, name: String(args.name ?? ''), mode: action === 'start' ? mode : '',
                  include_gripper: true, overwrite: true })
              return finish({ ok: !!r.success, action: `arm_record_${action}`, message: r.message,
                ...(action === 'save' ? { duration_s: r.duration_s, n_waypoints: r.n_waypoints } : {}),
                ...(r.success ? {} : { error: r.message }) })
            } catch (e) {
              return finish({ ok: false, error: `arm library unavailable: ${e instanceof Error ? e.message : String(e)}` })
            }
          }

          case 'arm_list_library': {
            try {
              const r = await ros.callService<Record<string, never>, { success: boolean; json: string }>(
                '/lupin/arm/library/list', 'lupin_msgs/srv/GetArmLibrary', {})
              const lib = JSON.parse(r.json || '{}')
              return finish({ ok: true, poses: lib.poses ?? [], sequences: lib.sequences ?? [] })
            } catch (e) {
              return finish({ ok: false, error: `arm library unavailable: ${e instanceof Error ? e.message : String(e)}` })
            }
          }

          case 'arm_stop': {
            try {
              const r = await ros.callService<Record<string, never>, { success: boolean; message: string }>(
                '/lupin/arm/library/stop', 'std_srvs/srv/Trigger', {})
              return finish({ ok: !!r.success, action: 'arm_stop', message: r.message })
            } catch (e) {
              return finish({ ok: false, error: `arm library unavailable: ${e instanceof Error ? e.message : String(e)}` })
            }
          }

          case 'calibrate_arm': {
            const action = String(args.action ?? '')
            if (!['start', 'commit', 'cancel', 'status'].includes(action)) {
              return finish({
                ok: false,
                error: `calibrate_arm: action must be one of start|commit|cancel|status, got "${action}"`,
              })
            }
            // E-stop gates 'start' only — cancel/status must still work while
            // e-stopped so the operator can recover a stranded session.
            if (action === 'start' && estop.active) {
              return finish(
                { ok: false, error: `e-stop active: ${estop.reason}` },
                { blocked: true, error: `e-stop active: ${estop.reason}` },
              )
            }
            try {
              const result = await ros.callService<
                { action: string },
                {
                  success: boolean
                  state: string
                  message: string
                  joint_names: string[]
                  offsets_applied: number[]
                  diffs_observed: number[]
                }
              >('/lupin/arm/calibrate', 'lupin_msgs/srv/CalibrateArm', { action })
              return finish({
                ok: !!result.success,
                action: 'calibrate_arm',
                wizard_action: action,
                state: result.state,
                message: result.message,
                joint_names: result.joint_names,
                offsets_applied: result.offsets_applied,
                diffs_observed: result.diffs_observed,
              })
            } catch (e) {
              return finish({
                ok: false,
                error: `calibrate service unavailable: ${e instanceof Error ? e.message : String(e)}`,
              })
            }
          }

          case 'query_state': {
            const fields = Array.isArray(args.fields) ? (args.fields as string[]) : ['pose', 'battery', 'estop']
            const out: Record<string, unknown> = {}
            if (fields.includes('pose')) out.pose = readPose(odomRef.current)
            if (fields.includes('battery')) out.battery = readBattery(batteryRef.current)
            if (fields.includes('estop')) out.estop = { active: estop.active, reason: estop.reason }
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
    [ros, settings, estop, odomRef, batteryRef, mapPoseRef, updateSettings, pushTool, pushTranscript],
  )

  /** ───────────────────── Live mode wiring ───────────────────── */

  // True only during a VAD-detected speech burst. Gates frame forwarding so a
  // hands-free session doesn't pump room tone to the server, and so the orb
  // shows "listening" only while someone is actually talking.
  const streamingRef = useRef<boolean>(false)

  const sendFrame = useCallback((b64: string) => {
    if (!streamingRef.current) return
    liveRef.current?.sendAudio(b64)
  }, [])

  // Build a fresh endpointer for the current utterance. Mode is decided at
  // fire time from voicePushToTalkRef, not at construction, so toggling the
  // setting mid-session can't leave a stale closure misbehaving.
  const buildEndpointer = useCallback((): SpeechEndpointer => {
    return new SpeechEndpointer({
      startThreshold: settings.voiceVadStartThreshold,
      endThreshold: settings.voiceVadEndThreshold,
      endHoldMs: settings.voiceVadEndHoldMs,
      maxUtteranceMs: settings.voiceVadMaxUtteranceMs,
      onSpeechStart: () => {
        streamingRef.current = true
        // Hands-free: open the manual-VAD activity here. Push-to-talk already
        // sent activityStart in beginUtterance — don't double-emit.
        if (!voicePushToTalkRef.current) {
          liveRef.current?.sendActivityStart()
        }
        setStatus((s) => (s === 'ready' || s === 'speaking' ? 'listening' : s))
      },
      onSpeechEnd: () => {
        streamingRef.current = false
        if (voicePushToTalkRef.current) {
          // Tear the mic down — endUtterance sends activityEnd.
          void endUtteranceRef.current('silence')
        } else {
          // Hands-free: close the activity but keep the mic open for the
          // next utterance.
          liveRef.current?.sendActivityEnd()
          setStatus((s) => (s === 'listening' ? 'thinking' : s))
        }
      },
    })
  }, [
    settings.voiceVadStartThreshold,
    settings.voiceVadEndThreshold,
    settings.voiceVadEndHoldMs,
    settings.voiceVadMaxUtteranceMs,
  ])

  const buildSystemInstruction = useCallback((): string => {
    const names = Object.keys(settings.voiceNamedLocations)
    const locLine = names.length
      ? `Known named locations: ${names.join(', ')}.`
      : 'No named locations are configured yet.'
    const { poses, sequences } = armLibraryRef.current
    const poseLine = poses.length
      ? `Saved arm poses: ${poses.join(', ')}.`
      : 'No saved arm poses yet.'
    const seqLine = sequences.length
      ? `Saved arm sequences: ${sequences.join(', ')}.`
      : 'No saved arm sequences yet.'
    return `${settings.voiceSystemPrompt}\n\n${locLine}\n${poseLine}\n${seqLine}`
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
    // Eagerly create+resume the playback AudioContext while we're still inside
    // the user gesture (the click/tap that called start()). Otherwise the
    // model's first audio chunk lands on a suspended context and queues
    // silently until the next gesture happens to resume it — which is what
    // surfaces as "the answer is in but gated by the next button press".
    await playerRef.current.prepare()

    // Fetch the saved arm library so buildSystemInstruction can list the pose
    // and sequence names for the model. Best-effort: if the node is down we
    // start with an empty library rather than failing the session.
    try {
      const res = await ros.callService<Record<string, never>, { json: string }>(
        '/lupin/arm/library/list', 'lupin_msgs/srv/GetArmLibrary', {})
      const lib = JSON.parse(res.json || '{}')
      armLibraryRef.current = {
        poses: (lib.poses ?? []).map((p: { name: string }) => p.name),
        sequences: (lib.sequences ?? []).map((s: { name: string }) => s.name),
      }
    } catch { armLibraryRef.current = { poses: [], sequences: [] } }

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
      // Hands-free: open mic immediately and keep it open. VAD gates streaming
      // so we only forward audio while the user is actually speaking — the
      // server stays quiet during silence and the UI status flips back to
      // "thinking/ready" instead of perpetually "listening".
      const mic = new MicCapture()
      const vad = settings.voiceVadEnabled ? buildEndpointer() : null
      streamingRef.current = settings.voiceVadEnabled ? false : true
      try {
        await mic.start(
          (frame) => sendFrame(frame),
          (rms, frameMs) => vadRef.current?.feed(rms, frameMs),
        )
        // Commit only after start succeeds, so a failure-during-getUserMedia
        // can't leave a half-started mic stuck in micRef.
        micRef.current = mic
        vadRef.current = vad
        setMicActive(true)
      } catch (e) {
        await mic.stop().catch(() => {})
        streamingRef.current = false
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
    settings.voiceVadEnabled,
    buildEndpointer,
    buildSystemInstruction,
    dispatchTool,
    pushTranscript,
    sendFrame,
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
    vadRef.current = null
    streamingRef.current = false
    setMicActive(false)
    setStatus('idle')
  }, [])

  const beginUtterance = useCallback(async () => {
    if (!liveRef.current && !mockRef.current) return
    if (!settings.voicePushToTalk) return
    if (micRef.current || startingRef.current) return // already capturing or mid-open
    startingRef.current = true
    const mic = new MicCapture()
    const vad = settings.voiceVadEnabled ? buildEndpointer() : null
    // Stream from the moment the user opens the mic — VAD will close it on
    // sustained silence. We don't gate the leading audio on VAD's onSpeechStart
    // because the user already gave consent by tapping the orb, and they may
    // begin speaking before the start-threshold trips.
    streamingRef.current = true
    // Manual VAD: tell the server the activity is starting BEFORE the audio
    // frames flow, so the first frame isn't dropped as pre-activity noise.
    liveRef.current?.sendActivityStart()
    try {
      await mic.start(
        (frame) => sendFrame(frame),
        (rms, frameMs) => vadRef.current?.feed(rms, frameMs),
      )
      // Commit refs only after start resolves so a concurrent endUtterance
      // (or a thrown getUserMedia) can't see a half-initialized mic.
      micRef.current = mic
      vadRef.current = vad
      setMicActive(true)
      setStatus('listening')
    } catch (e) {
      // Tear down whatever stage of start managed to run before the throw.
      await mic.stop().catch(() => {})
      streamingRef.current = false
      // Match the activityStart we already emitted so the server doesn't see
      // an open activity that never closes.
      liveRef.current?.sendActivityEnd()
      setErrorDetail(`mic: ${e instanceof Error ? e.message : String(e)}`)
      setStatus('error')
    } finally {
      startingRef.current = false
    }
  }, [settings.voicePushToTalk, settings.voiceVadEnabled, buildEndpointer, sendFrame])

  const endUtterance = useCallback(async () => {
    if (!micRef.current) return
    if (!settings.voicePushToTalk) return
    // Claim the mic synchronously so a re-entrant tap (or a VAD onSpeechEnd
    // firing during teardown) sees a null micRef and bails out.
    const mic = micRef.current
    micRef.current = null
    vadRef.current = null
    streamingRef.current = false
    setMicActive(false)
    setStatus((s) => (s === 'listening' ? 'thinking' : s))
    // Manual VAD: signal end-of-activity so the server commits the turn now,
    // instead of timing out its own (disabled) silence detector. This is the
    // step that lets the response start streaming back without waiting for
    // the next button press.
    liveRef.current?.sendActivityEnd()
    await mic.stop()
  }, [settings.voicePushToTalk])

  // Keep the ref pointing at the latest endUtterance so VAD callbacks (built
  // before this hook returns) dispatch through the current closure.
  useEffect(() => {
    endUtteranceRef.current = () => {
      void endUtterance()
    }
  }, [endUtterance])

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
