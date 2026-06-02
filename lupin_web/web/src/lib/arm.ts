/**
 * Single source of truth for the MIRTE Master arm: per-joint command limits,
 * the gripper HMI↔URDF map, and joint id↔name helpers.
 *
 * WHY THIS FILE EXISTS
 * --------------------
 * The joint windows and the gripper rad↔deg map used to be copied verbatim
 * across ArmView.tsx, gripper_action_bridge.py, and voice/session.tsx, kept in
 * sync only by "KEEP IN SYNC" comments. That is exactly the kind of triplicated
 * magic-number drift that lets the HMI command angles the servo silently
 * rejects. ArmView + the voice tools now import from here; the Python backend
 * mirrors these values in `lupin_hmi/arm_limits.py` (the one cross-language
 * seam — guarded by the BACKEND-PARITY note below).
 *
 * GROUND TRUTH — where the limits come from
 * -----------------------------------------
 * The real per-servo software limits live in the vendor
 * `mirte_bringup/telemetrix_config/mirte_master_config.yaml` as raw Hiwonder
 * counts. The Hiwonder parser converts a raw count to degrees as
 *     deg = (angle_out - home_out) / 100
 * (then stores it internally as radians). Computed from the 2026-06 config:
 *
 *   joint           home_out  min_out  max_out   →  real servo limit (deg)
 *   shoulder_pan      12000     3400    21000        -86.0 … +90.0
 *   shoulder_lift     11450     2832    20000        -86.2 … +85.5
 *   elbow             11750      120    21000       -116.3 … +92.5
 *   wrist             12200     1128    21000       -110.7 … +88.0
 *   gripper           10524     6168    14224        -43.6 … +37.0
 *
 * The command path also passes through the URDF/JTC position envelope and the
 * bridge clamp (historically a blanket ±90° = ±π/2). The CANONICAL command
 * window below is the INTERSECTION of the servo limit and that ±90° envelope,
 * rounded INWARD to whole degrees so a slider can never request a pose the
 * servo rejects:
 *
 *   - pan/lift/wrist are TIGHTENED at the end where the servo stops before 90°
 *     (prevents the silent "send failed near the negative stop").
 *   - elbow/wrist keep the conservative ±90°/under-travel at the end where the
 *     servo reaches past 90° — recovering that reach has no value for the
 *     arm's two jobs (camera-tilt + top-down pick) and would mean widening the
 *     vendor URDF. Documented as deliberate, NOT the hardware limit.
 *
 * BACKEND PARITY: if you change ARM_JOINT_LIMITS or the GRIPPER_* constants
 * here, mirror the change in `lupin/lupin_hmi/lupin_hmi/arm_limits.py` (radian
 * values). The two files are asserted consistent at runtime by the bridge log.
 */

export const RAD2DEG = 180 / Math.PI
export const DEG2RAD = Math.PI / 180

/** Arm joint ids as they appear in `/lupin/arm/<id>/set_angle_with_speed`. */
export const ARM_JOINT_IDS = ['shoulder_pan', 'shoulder_lift', 'elbow', 'wrist'] as const
export type ArmJointId = (typeof ARM_JOINT_IDS)[number]

/** Map an arm joint id → its URDF/`/joint_states` joint name. */
export function armJointStateName(id: ArmJointId): string {
  return `${id}_joint`
}

/** The gripper's `/joint_states` joint name (it is NOT one of ARM_JOINT_IDS — it
 * goes through the GripperActionController, not the arm JTC). */
export const GRIPPER_JOINT_STATE_NAME = 'gripper_joint'

export interface JointLimitDeg {
  minDeg: number
  maxDeg: number
}

/**
 * Canonical command window per arm joint, in degrees. Intersection of the real
 * servo software limit and the ±90° URDF/bridge envelope, rounded inward. These
 * are the limits the HMI sliders and the backend per-joint clamp both use.
 */
export const ARM_JOINT_LIMITS: Record<ArmJointId, JointLimitDeg> = {
  // servo -86.0..+90.0 ; full +90 reach kept (within envelope)
  shoulder_pan: { minDeg: -86, maxDeg: 90 },
  // servo -86.2..+85.5 ; tightened both ends to avoid silent reject
  shoulder_lift: { minDeg: -86, maxDeg: 85 },
  // servo -116.3..+92.5 ; bridge/URDF-limited to ±90 (deliberate under-travel)
  elbow: { minDeg: -90, maxDeg: 90 },
  // servo -110.7..+88.0 ; -90 is the bridge/URDF floor, +88 the servo ceiling
  wrist: { minDeg: -90, maxDeg: 88 },
}

/**
 * Raw per-servo software limits (deg) straight from the vendor config — for
 * documentation, tooltips, and a future `GetServoRange`-backed live query. NOT
 * used for clamping (the canonical window above is what's enforced).
 */
export const ARM_SERVO_LIMITS_DEG: Record<ArmJointId, JointLimitDeg> = {
  shoulder_pan: { minDeg: -86.0, maxDeg: 90.0 },
  shoulder_lift: { minDeg: -86.2, maxDeg: 85.5 },
  elbow: { minDeg: -116.3, maxDeg: 92.5 },
  wrist: { minDeg: -110.7, maxDeg: 88.0 },
}

// ── Gripper map ──────────────────────────────────────────────────────────
// The gripper jaw is commanded through gripper_action_bridge, which maps the
// HMI ±30° window LINEARLY onto the URDF gripper_joint range [-0.20, +0.25] rad.
// This window is still UNVERIFIED against the real mechanical stops (the servo
// itself reaches -43.6°..+37.0°); the ±30° is a conservative guess pending a
// live tuning pass via GetServoRange. Direction on Mirte-247264 is INVERTED:
//   -30° (negative) = OPEN,  +30° (positive) = CLOSED.
// Any semantic open/close consumer must respect that sign (see voice gripper).
export const GRIPPER_HMI_MIN_DEG = -30
export const GRIPPER_HMI_MAX_DEG = 30
export const GRIPPER_URDF_MIN_RAD = -0.2
export const GRIPPER_URDF_MAX_RAD = 0.25
/** True until the live mechanical-stop tuning pass is done — drives the loud
 * "range unverified" badge. Flip to false only after recording real stops. */
export const GRIPPER_RANGE_UNVERIFIED = true
/** Semantic open/close endpoints in HMI degrees (inverted — see above). */
export const GRIPPER_OPEN_DEG = GRIPPER_HMI_MIN_DEG // -30 = OPEN
export const GRIPPER_CLOSE_DEG = GRIPPER_HMI_MAX_DEG // +30 = CLOSED

function clamp(v: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, v))
}

/** URDF gripper_joint radians → the HMI ±30° window (reverse of the bridge map). */
export function gripperRadToHmiDeg(rad: number): number {
  const ratio = (rad - GRIPPER_URDF_MIN_RAD) / (GRIPPER_URDF_MAX_RAD - GRIPPER_URDF_MIN_RAD)
  return GRIPPER_HMI_MIN_DEG + ratio * (GRIPPER_HMI_MAX_DEG - GRIPPER_HMI_MIN_DEG)
}

/** HMI ±30° → URDF gripper_joint radians (mirror of gripper_action_bridge). */
export function gripperHmiDegToRad(deg: number): number {
  const clamped = clamp(deg, GRIPPER_HMI_MIN_DEG, GRIPPER_HMI_MAX_DEG)
  const ratio = (clamped - GRIPPER_HMI_MIN_DEG) / (GRIPPER_HMI_MAX_DEG - GRIPPER_HMI_MIN_DEG)
  return GRIPPER_URDF_MIN_RAD + ratio * (GRIPPER_URDF_MAX_RAD - GRIPPER_URDF_MIN_RAD)
}

/**
 * A `/joint_states` angle (rad) → the HMI-degree scale a joint's slider uses.
 * The gripper goes through the bridge's linear map; every arm joint is a
 * straight rad→deg. Pass the arm joint id, or 'gripper'.
 */
export function jointStateRadToHmiDeg(id: ArmJointId | 'gripper', rad: number): number {
  return id === 'gripper' ? gripperRadToHmiDeg(rad) : rad * RAD2DEG
}

/** The HMI-degree target a joint's slider should send → radians the bridge
 * receives intent in (the bridge re-clamps). Gripper uses the linear map. */
export function hmiDegToTargetRad(id: ArmJointId | 'gripper', deg: number): number {
  return id === 'gripper' ? gripperHmiDegToRad(deg) : deg * DEG2RAD
}

/** Convergence tolerance (HMI degrees) used to decide a joint has "arrived". */
export const ARM_SETTLE_TOL_DEG = 3
/** Gripper convergence tolerance — the linear map compresses the jaw travel,
 * so a wider absolute-degree tolerance is appropriate. */
export const GRIPPER_SETTLE_TOL_DEG = 4
