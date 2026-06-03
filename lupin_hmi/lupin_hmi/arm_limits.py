"""arm_limits — single source of truth for the MIRTE Master arm command limits.

Mirrors the frontend `lupin_web/web/src/lib/arm.ts`. The bridge, the preset
server and the teleop node all import the canonical per-joint clamp and the
gripper map from here instead of each carrying their own magic numbers (which
used to drift — see the audit doc `docs/arm_control_audit_2026-06-02.md`).

GROUND TRUTH
------------
Real per-servo software limits live in the vendor
`mirte_bringup/telemetrix_config/mirte_master_config.yaml` as raw Hiwonder
counts. The parser converts a raw count to degrees as

    deg = (angle_out - home_out) / 100        # then stored as radians

Computed from the 2026-06 config:

    joint           home_out  min_out  max_out   →  servo limit (deg)
    shoulder_pan      12000     3400    21000        -86.0 … +90.0
    shoulder_lift     11450     2832    20000        -86.2 … +85.5
    elbow             11750      120    21000       -116.3 … +92.5
    wrist             12200     1128    21000       -110.7 … +88.0
    gripper           10524     6168    14224        -43.6 … +37.0

CANONICAL command window = intersection of the servo limit with the ±90°
(=±π/2) URDF/JTC envelope, rounded INWARD to whole degrees so we never command
a pose the servo silently rejects. This REPLACES the old blanket ±π/2 clamp:
pan/lift/wrist are tightened where the servo stops before 90°; elbow/wrist keep
the conservative ±90° where the servo reaches past it (deliberate under-travel,
not the hardware limit — widening it would mean editing the vendor URDF and has
no value for the arm's two jobs).

FRONTEND PARITY: keep ARM_JOINT_LIMITS_DEG and the GRIPPER_* constants identical
(in degrees) to `arm.ts`. `assert_frontend_parity()` documents the contract.
"""

from __future__ import annotations

import math
from typing import Dict, Tuple

# Arm joint ids (no `_joint` suffix) in canonical JTC order.
ARM_JOINTS: Tuple[str, ...] = ('shoulder_pan', 'shoulder_lift', 'elbow', 'wrist')
ARM_JOINT_FULL: Dict[str, str] = {j: f'{j}_joint' for j in ARM_JOINTS}

# Canonical command window per arm joint, in DEGREES (see module docstring).
ARM_JOINT_LIMITS_DEG: Dict[str, Tuple[float, float]] = {
    'shoulder_pan': (-86.0, 90.0),
    'shoulder_lift': (-86.0, 85.0),
    'elbow': (-90.0, 90.0),
    'wrist': (-90.0, 88.0),
}

# Same limits in radians — what the bridge actually clamps with.
ARM_JOINT_LIMITS_RAD: Dict[str, Tuple[float, float]] = {
    j: (math.radians(lo), math.radians(hi))
    for j, (lo, hi) in ARM_JOINT_LIMITS_DEG.items()
}

# Raw per-servo software limits (deg) for reference / logging only.
ARM_SERVO_LIMITS_DEG: Dict[str, Tuple[float, float]] = {
    'shoulder_pan': (-86.0, 90.0),
    'shoulder_lift': (-86.2, 85.5),
    'elbow': (-116.3, 92.5),
    'wrist': (-110.7, 88.0),
}

# ── Gripper map (HMI ±30° → URDF gripper_joint [-0.20, +0.25] rad, linear) ──
# Window UNVERIFIED against the real mechanical stops (servo reaches -43.6..+37);
# the ±30° is a conservative guess pending a live GetServoRange tuning pass.
# Direction on Mirte-247264 is INVERTED: -30° = OPEN, +30° = CLOSED.
GRIPPER_HMI_MIN_DEG = -30.0
GRIPPER_HMI_MAX_DEG = 30.0
GRIPPER_URDF_MIN_RAD = -0.20
GRIPPER_URDF_MAX_RAD = 0.25
GRIPPER_MAX_EFFORT = 2.0  # matches URDF effort cap on gripper_joint
GRIPPER_RANGE_UNVERIFIED = True


def clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def clamp_arm_joint(joint: str, value_rad: float) -> float:
    """Clamp a single arm joint command (rad) to its canonical window.

    Falls back to the ±π/2 envelope for an unknown joint name so a typo can
    never widen the command range.
    """
    lo, hi = ARM_JOINT_LIMITS_RAD.get(joint, (-math.pi / 2, math.pi / 2))
    return clamp(value_rad, lo, hi)


def gripper_deg_to_rad(angle_deg: float) -> float:
    deg = clamp(angle_deg, GRIPPER_HMI_MIN_DEG, GRIPPER_HMI_MAX_DEG)
    span_in = GRIPPER_HMI_MAX_DEG - GRIPPER_HMI_MIN_DEG
    span_out = GRIPPER_URDF_MAX_RAD - GRIPPER_URDF_MIN_RAD
    ratio = (deg - GRIPPER_HMI_MIN_DEG) / span_in
    return GRIPPER_URDF_MIN_RAD + ratio * span_out


def gripper_rad_to_deg(value_rad: float) -> float:
    """Inverse of gripper_deg_to_rad: URDF gripper_joint rad -> HMI deg.
    Mirrors arm.ts gripperRadToHmiDeg."""
    span_in = GRIPPER_URDF_MAX_RAD - GRIPPER_URDF_MIN_RAD
    span_out = GRIPPER_HMI_MAX_DEG - GRIPPER_HMI_MIN_DEG
    ratio = (clamp(value_rad, GRIPPER_URDF_MIN_RAD, GRIPPER_URDF_MAX_RAD) - GRIPPER_URDF_MIN_RAD) / span_in
    return GRIPPER_HMI_MIN_DEG + ratio * span_out


def limits_summary() -> str:
    """One-line human-readable summary for node startup logs."""
    parts = [
        f'{j}=[{lo:+.0f}°,{hi:+.0f}°]'
        for j, (lo, hi) in ARM_JOINT_LIMITS_DEG.items()
    ]
    return ', '.join(parts) + (
        f'; gripper window ±{GRIPPER_HMI_MAX_DEG:.0f}°'
        + (' (UNVERIFIED)' if GRIPPER_RANGE_UNVERIFIED else '')
    )
