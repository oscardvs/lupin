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
    name: 'rotate',
    description:
      "Rotate the base in place by a relative yaw angle, in degrees, using the Nav2 planner (same controller as nav_goto). Positive = counter-clockwise. Use for scanning the room or reorienting before a pick. Requires localization. The robot can refuse if the rotation would place its footprint into known obstacles.",
    parameters: {
      type: 'object',
      properties: {
        angle_deg: {
          type: 'number',
          description: 'Relative yaw, degrees. Positive = ccw, negative = cw. Wrapped into [-180, 180].',
        },
      },
      required: ['angle_deg'],
    },
  },
  {
    name: 'set_speed_cap',
    description:
      "Temporarily scale the voice agent's drive speed limits by a factor in [0, 1]. 1.0 means use the configured voiceMax* caps as-is; 0.5 halves them; 0.0 effectively disables drive bursts. The change is in-memory and resets when the session ends. Use when the user says 'go slower' / 'careful mode' or wants to ramp speed up after a cautious approach.",
    parameters: {
      type: 'object',
      properties: {
        value: {
          type: 'number',
          description: 'Speed cap factor in [0, 1]. Clamped.',
        },
      },
      required: ['value'],
    },
  },
  {
    name: 'nav_cancel',
    description:
      'Cancel any in-flight Nav2 goal. The robot stops planning toward its current target and holds position. Safe and idempotent — call this whenever the user wants to abort a navigation in progress (e.g. "never mind", "stop going there"). Does not stop teleop motion — for a hard stop use stop or engage_estop.',
    parameters: { type: 'object', properties: {} },
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
    name: 'list_named_locations',
    description:
      'Return the current map-frame named-locations dictionary the agent can navigate to. Use this to discover what locations exist before calling nav_goto_named, especially when the user asks "where can you go?" or refers to a location by a fuzzy name.',
    parameters: { type: 'object', properties: {} },
  },
  {
    name: 'save_named_location',
    description:
      "Snapshot the robot's current map-frame pose and store it under a name so future calls can `nav_goto_named` back here. Use when the user says \"remember this spot as <name>\" or \"call this <name>\". Requires that localization is up — fails if no map→base transform is available.",
    parameters: {
      type: 'object',
      properties: {
        name: {
          type: 'string',
          description: 'Short identifier (e.g. "kitchen", "charging_dock"). Overwrites if the name already exists.',
        },
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
    name: 'engage_estop',
    description:
      'Trigger the same software E-stop as the red bar in the UI. Halts all motion and gates further drive / nav / arm calls until the user manually presses Reset E-stop. Use this when the user says "stop everything", "emergency stop", "kill it", or signals real concern. Prefer stop or nav_cancel for routine halts.',
    parameters: { type: 'object', properties: {} },
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
  | 'rotate'
  | 'set_speed_cap'
  | 'nav_goto'
  | 'nav_cancel'
  | 'nav_goto_named'
  | 'list_named_locations'
  | 'save_named_location'
  | 'arm_preset'
  | 'engage_estop'
  | 'query_state'
  | 'speak'

export function clampNumber(n: unknown, min: number, max: number, fallback = 0): number {
  if (typeof n !== 'number' || Number.isNaN(n)) return fallback
  return Math.max(min, Math.min(max, n))
}
