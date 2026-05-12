/**
 * Tool surface the voice agent is allowed to call. Declarations are sent in
 * the BidiGenerateContentSetup; dispatch happens in `useVoiceSession` against
 * the live RosCore + EStop providers.
 *
 * Design rules (no speculative interfaces):
 *   - One tool per existing rosbridge-side capability. Tools without a real
 *     backend are NOT declared here — a tool that always errors trains the
 *     model to stop calling it; one that silently returns empty trains it to
 *     hallucinate "yes I did it".
 *   - Motion tools (`drive`, `nav_goto`, `nav_forward`, `nav_goto_named`,
 *     `rotate`, `gripper`, `arm_preset`) all go through e-stop and
 *     saturating speed caps from settings.
 *   - `query_state` is the read path — never produces motion.
 *   - `speak` exists for transcript-only replies (e.g. when the user has
 *     muted the speaker); normal spoken answers come from Gemini's audio
 *     stream directly and don't need a tool call.
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
      "Send a Nav2 goal in the MAP frame (absolute coordinates). The robot's planner drives there autonomously. Prefer 'nav_forward' for relative motion like \"go 1 m forward\" — that handles the trig server-side. Use this tool only when the user gives explicit map-frame coordinates or when targeting a known absolute pose.",
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
    name: 'nav_forward',
    description:
      "Drive to a pose offset from the robot's CURRENT position, expressed in the robot's body frame. The HMI reads the latest map→base TF, rotates (forward_m, lateral_m) by current yaw, and sends the resulting absolute Nav2 goal — the agent does NOT need to do the math. Fails if no map→base transform is available (localization down). Use this for any \"go N metres forward / back / left / right\" or \"reposition slightly\" request. forward_m is along the robot's current heading (positive = forward); lateral_m is the perpendicular strafe (positive = left); rotate_deg is an optional relative yaw change in degrees applied to the goal pose.",
    parameters: {
      type: 'object',
      properties: {
        forward_m: {
          type: 'number',
          description: 'Distance along robot heading, metres. Positive = forward, negative = backward.',
        },
        lateral_m: {
          type: 'number',
          description: 'Lateral offset, metres. Positive = left (mecanum strafe). Default 0.',
        },
        rotate_deg: {
          type: 'number',
          description: 'Relative yaw change at the goal, degrees. Positive = ccw. Default 0.',
        },
      },
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
    name: 'gripper',
    description:
      "Open or close the Hiwonder gripper jaw. The mechanical end-stop angles haven't been verified on the live robot yet, so we drive a conservative ±30° window. Use 'open' to release / clear the jaw, 'close' to grasp. The arm joints are unaffected — pair with arm_preset 'pick' or 'place' for full pick-and-place sequences.",
    parameters: {
      type: 'object',
      properties: {
        action: {
          type: 'string',
          enum: ['open', 'close'],
          description: "Direction. 'open' drives to +30°, 'close' to -30°.",
        },
      },
      required: ['action'],
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
    name: 'calibrate_arm',
    description:
      "CRITICAL: NEVER call this with action='commit' in the same turn as 'start'. You MUST wait for an explicit operator utterance like 'go ahead', 'commit', 'save it', or 'I've moved the arm' before calling 'commit'. Calling commit immediately leaves the arm calibrated to whatever pose it happened to be in when start fired — i.e. broken. Hiwonder zero-offset arm calibration is operator-in-the-loop: (1) action='start' disables the servos and the arm goes limp — tell the user to physically support the arm and move it into its mechanical home pose. (2) After the user confirms they have hand-posed the arm, action='commit' samples positions for ~2 s and writes new zero offsets. (3) action='cancel' aborts and re-enables without writing. (4) action='status' queries state without side effects. Hardware-only; in sim this returns synthetic results.",
    parameters: {
      type: 'object',
      properties: {
        action: {
          type: 'string',
          enum: ['start', 'commit', 'cancel', 'status'],
          description: 'Step in the calibration workflow.',
        },
      },
      required: ['action'],
    },
  },
  {
    name: 'query_state',
    description:
      "Read live telemetry from the robot. 'fields' is a list of items to fetch from: pose, battery, estop. Returns whatever subset is currently known. Read-only — never moves the robot.",
    parameters: {
      type: 'object',
      properties: {
        fields: {
          type: 'array',
          items: { type: 'string' },
          description: 'Subset of: pose, battery, estop.',
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
      "Push a written reply into the on-screen transcript without producing any motion or audio. Do NOT call this for ordinary chat — your normal voice output already reaches the user. Use it only when the user has muted the speaker, when you want to leave a written note in the transcript log, or in the offline mock harness where there is no audio channel.",
    parameters: {
      type: 'object',
      properties: {
        text: { type: 'string', description: 'What to write to the transcript.' },
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
  | 'nav_forward'
  | 'nav_cancel'
  | 'nav_goto_named'
  | 'list_named_locations'
  | 'save_named_location'
  | 'gripper'
  | 'arm_preset'
  | 'calibrate_arm'
  | 'engage_estop'
  | 'query_state'
  | 'speak'

export function clampNumber(n: unknown, min: number, max: number, fallback = 0): number {
  if (typeof n !== 'number' || Number.isNaN(n)) return fallback
  return Math.max(min, Math.min(max, n))
}
