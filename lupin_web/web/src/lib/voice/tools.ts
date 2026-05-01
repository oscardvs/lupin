/**
 * Tool surface the voice agent is allowed to call. Declarations are sent in
 * the BidiGenerateContentSetup; dispatch happens in `useVoiceSession` against
 * the live RosCore + EStop providers.
 *
 * Design rules (no speculative interfaces):
 *   - One tool per existing rosbridge-side capability.
 *   - Motion tools (`drive`, `nav_goto`, `nav_goto_named`, `arm_preset`) all
 *     go through e-stop and saturating speed caps from settings.
 *   - `query_state` is the read path — never produces motion.
 *   - `speak` is a no-op on the wire (audio comes from the model itself);
 *     it exists so the agent has a way to reply without taking action.
 */

import type { FunctionDeclaration } from './types'

export const ROBOT_TOOL_DECLARATIONS: FunctionDeclaration[] = [
  {
    name: 'drive',
    description:
      "Drive the base for a short burst. Use this for nudges and small repositions. Do not chain bursts longer than 2 seconds — for navigation use 'nav_goto' or 'nav_goto_named'. linear_x is forward (m/s), linear_y is left strafe (m/s, mecanum base), angular_z is yaw (rad/s, +ccw).",
    parameters: {
      type: 'object',
      properties: {
        linear_x: { type: 'number', description: 'Forward speed in m/s. Positive = forward.' },
        linear_y: { type: 'number', description: 'Lateral (strafe) speed in m/s. Positive = left.' },
        angular_z: { type: 'number', description: 'Yaw rate in rad/s. Positive = counter-clockwise.' },
        duration_s: {
          type: 'number',
          description: 'Burst duration in seconds. Capped to [0.1, 2.0].',
        },
      },
      required: ['duration_s'],
    },
  },
  {
    name: 'stop',
    description: 'Immediately publish a zero Twist. Always safe to call.',
    parameters: { type: 'object', properties: {} },
  },
  {
    name: 'nav_goto',
    description:
      "Send a Nav2 goal in the map frame. The robot's planner takes over and drives there autonomously.",
    parameters: {
      type: 'object',
      properties: {
        x: { type: 'number', description: 'X in map frame, metres.' },
        y: { type: 'number', description: 'Y in map frame, metres.' },
        yaw: { type: 'number', description: 'Goal heading, radians, 0 = +X.' },
      },
      required: ['x', 'y'],
    },
  },
  {
    name: 'nav_goto_named',
    description:
      'Send a Nav2 goal to a pre-configured named location (e.g. "kitchen", "home"). The list of valid names is given in the system prompt at session start.',
    parameters: {
      type: 'object',
      properties: {
        name: { type: 'string', description: 'Named location key.' },
      },
      required: ['name'],
    },
  },
  {
    name: 'arm_preset',
    description:
      "Move the 4-DOF Hiwonder arm (shoulder pan / lift, elbow, wrist) to a named preset pose ('home', 'tuck', 'pick', 'place'). The gripper jaw is a separate joint and is not driven by this tool. Presets are defined on the robot side.",
    parameters: {
      type: 'object',
      properties: {
        name: { type: 'string', description: 'Preset name.' },
      },
      required: ['name'],
    },
  },
  {
    name: 'query_state',
    description:
      "Read live telemetry from the robot. 'fields' is a list of items to fetch from: pose, battery, estop, nav_status. Returns whatever subset is currently known. Read-only — never moves the robot.",
    parameters: {
      type: 'object',
      properties: {
        fields: {
          type: 'array',
          items: { type: 'string' },
          description: 'Subset of: pose, battery, estop, nav_status.',
        },
      },
    },
  },
  {
    name: 'speak',
    description:
      'Speak a short response without taking any action. Use this when the user asks something purely conversational. Prefer this to silently doing nothing.',
    parameters: {
      type: 'object',
      properties: {
        text: { type: 'string', description: 'What to say.' },
      },
      required: ['text'],
    },
  },
]

export type ToolName =
  | 'drive'
  | 'stop'
  | 'nav_goto'
  | 'nav_goto_named'
  | 'arm_preset'
  | 'query_state'
  | 'speak'

export function clampNumber(n: unknown, min: number, max: number, fallback = 0): number {
  if (typeof n !== 'number' || Number.isNaN(n)) return fallback
  return Math.max(min, Math.min(max, n))
}
