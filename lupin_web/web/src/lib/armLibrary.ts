/**
 * Typed wrappers over the arm_library_server services (/lupin/arm/library/*).
 * The library is owned by the ROS node (laptop JSON), so both the Arm tab and
 * the voice agent call these — there is no browser-side source of truth.
 */
import {
  LUPIN_SRV,
  type ArmLibraryList,
  type GetArmLibraryResponse,
  type SaveArmPoseRequest,
  type ArmRecordRequest,
  type ArmRecordResponse,
  type PlayArmSequenceRequest,
  type ArmLibraryEditRequest,
  type SimpleAck,
} from '@/types/ros'

/** Built-in presets owned by the onboard arm_preset_server (not the library). */
export const BUILTIN_PRESETS = ['home', 'zero', 'tuck', 'pick', 'place', 'inspect'] as const
export function isBuiltinPreset(name: string): boolean {
  return (BUILTIN_PRESETS as readonly string[]).includes(name.trim().toLowerCase())
}

type Call = <Req, Res = unknown>(name: string, type: string, req: Req) => Promise<Res>

export async function fetchLibrary(call: Call): Promise<ArmLibraryList> {
  const res = await call<Record<string, never>, GetArmLibraryResponse>(
    '/lupin/arm/library/list', LUPIN_SRV.GetArmLibrary, {})
  try {
    const parsed = JSON.parse(res.json || '{}')
    return { poses: parsed.poses ?? [], sequences: parsed.sequences ?? [] }
  } catch {
    return { poses: [], sequences: [] }
  }
}

export function savePose(call: Call, name: string): Promise<SimpleAck> {
  const req: SaveArmPoseRequest = {
    name, from_current: true, arm_rad: [], gripper_rad: 0, has_gripper: false, overwrite: true,
  }
  return call<SaveArmPoseRequest, SimpleAck>('/lupin/arm/library/save_pose', LUPIN_SRV.SaveArmPose, req)
}

/** Move to a saved pose. Built-ins route to /lupin/arm/preset; user poses to the library. */
export function gotoPose(call: Call, name: string): Promise<SimpleAck> {
  if (isBuiltinPreset(name)) {
    return call<{ name: string }, SimpleAck>('/lupin/arm/preset', LUPIN_SRV.SetArmPreset, { name })
  }
  return call<{ name: string }, SimpleAck>('/lupin/arm/library/goto_pose', LUPIN_SRV.SetArmPreset, { name })
}

export function deleteEntry(call: Call, kind: 'pose' | 'sequence', name: string): Promise<SimpleAck> {
  const req: ArmLibraryEditRequest = { kind, name, new_name: '' }
  return call<ArmLibraryEditRequest, SimpleAck>('/lupin/arm/library/delete', LUPIN_SRV.ArmLibraryEdit, req)
}

export function renameEntry(call: Call, kind: 'pose' | 'sequence', name: string, newName: string): Promise<SimpleAck> {
  const req: ArmLibraryEditRequest = { kind, name, new_name: newName }
  return call<ArmLibraryEditRequest, SimpleAck>('/lupin/arm/library/delete', LUPIN_SRV.ArmLibraryEdit, req)
}

export function startRecording(call: Call, mode: 'kinesthetic' | 'teleop', includeGripper: boolean): Promise<ArmRecordResponse> {
  const req: ArmRecordRequest = { action: 'start', name: '', mode, include_gripper: includeGripper, overwrite: false }
  return call<ArmRecordRequest, ArmRecordResponse>('/lupin/arm/library/record', LUPIN_SRV.ArmRecord, req)
}

export function saveRecording(call: Call, name: string): Promise<ArmRecordResponse> {
  const req: ArmRecordRequest = { action: 'save', name, mode: '', include_gripper: false, overwrite: true }
  return call<ArmRecordRequest, ArmRecordResponse>('/lupin/arm/library/record', LUPIN_SRV.ArmRecord, req)
}

export function cancelRecording(call: Call): Promise<ArmRecordResponse> {
  const req: ArmRecordRequest = { action: 'cancel', name: '', mode: '', include_gripper: false, overwrite: false }
  return call<ArmRecordRequest, ArmRecordResponse>('/lupin/arm/library/record', LUPIN_SRV.ArmRecord, req)
}

export function runSequence(call: Call, name: string, speed = 1.0): Promise<SimpleAck> {
  const clamped = Math.max(0.25, Math.min(2, speed))
  const req: PlayArmSequenceRequest = { name, speed: clamped }
  return call<PlayArmSequenceRequest, SimpleAck>('/lupin/arm/library/play', LUPIN_SRV.PlayArmSequence, req)
}

export function stopSequence(call: Call): Promise<SimpleAck> {
  return call<Record<string, never>, SimpleAck>('/lupin/arm/library/stop', LUPIN_SRV.Trigger, {})
}
