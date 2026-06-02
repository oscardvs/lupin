"""arm_preset_server — named arm-pose service.

Exposes ``/lupin/arm/preset`` (``lupin_msgs/srv/SetArmPreset``). The voice
agent and any teammate code can call ``{name: 'home'}`` and have the arm
move to a known pose without needing to know joint angles.

Presets are a static dict here because v1 only needs four named poses and
introducing a YAML / parameter pipeline before there's a second consumer is
the kind of speculative interface we're avoiding (see
``feedback_no_speculative_interfaces``).

Output topic
------------
``/mirte_master_arm_controller/joint_trajectory`` — same topic
``arm_sim_shim`` and the vendor ``mirte_master_hw_check`` use. In sim it's
the ros2_control JointTrajectoryController; on the real Mirte the same
controller runs against the Hiwonder serial-bus servo hardware interface,
so this node works on both targets without branching.

Joint order
-----------
``shoulder_pan_joint``, ``shoulder_lift_joint``, ``elbow_joint``,
``wrist_joint``. Gripper is intentionally NOT driven here — the voice
``gripper`` tool calls ``/io/servo/hiwonder/gripper/set_angle_with_speed``
directly so pick-and-place sequences interleave preset moves with explicit
gripper open/close.

Default preset values (radians)
-------------------------------
``home``   — Mirte-247264's measured safe rest pose (project_arm_servo_thermal_trip).
             shoulder_lift near 0 = upper arm horizontal forward, wrist rotated
             -π/2 so the gripper jaw axis lies sideways. Chosen as the auto-home
             target on boot because at this pose the Hiwonder shoulder_lift
             servo carries the minimum gravity moment of any URDF-feasible pose
             we've measured. ``zero`` preserves the old (0,0,0,0) URDF zero
             pose for code that explicitly wants it.
``zero``   — All four joints at 0 (the URDF zero pose). Use only when you
             know you want the literal URDF zero — most operators want ``home``.
``tuck``   — Arm folded onto the chassis: shoulder_lift up, elbow back so
             the wrist sits over the base footprint. Conservative against
             the URDF ±π/2 limits.
``pick``   — Arm extended forward, wrist level with the table for a
             top-down approach.
``place``  — Arm extended forward, wrist raised so an item carried in the
             gripper clears table edges.
``inspect``— Arm reaches forward and pitches the wrist DOWN so the
             gripper/wrist camera looks into the pot from above. Used by the
             mission orchestrator's per-pot arm patrol (arm_patrol_enabled):
             the robot parks beside a pot, strikes this pose, and the flower
             detector classifies the bloom while the arm holds it. Low lift
             so the shoulder servo stays well inside its gravity-safe window
             (project_arm_servo_thermal_trip). Tunable — verify the exact
             camera framing in Gazebo and re-tune the angles there.

These are conservative starting points — re-tune on the real robot once
the arm is mounted in its final configuration. Values are clamped to the
canonical per-joint window from ``arm_limits`` before publishing, so a
mistuned preset can't command a pose the servo silently rejects.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory  # publisher message type

from lupin_msgs.srv import SetArmPreset

from lupin_hmi.arm_limits import ARM_JOINTS, clamp_arm_joint
from lupin_hmi.arm_traj import build_arm_trajectory


ARM_JOINT_NAMES: Tuple[str, ...] = tuple(f'{j}_joint' for j in ARM_JOINTS)

# All values in radians. Order matches ARM_JOINT_NAMES.
PRESETS: Dict[str, Tuple[float, float, float, float]] = {
    # Measured 2026-05-20 from /joint_states with arm in its rest pose
    # (project_arm_servo_thermal_trip). Hiwonder shoulder_lift carries minimum
    # gravity moment here, so this is the safe pose for auto-home on boot
    # and any long-duration "park" state.
    'home':  (0.04, -0.01,  0.01, -1.57),
    'zero':  (0.0,   0.0,   0.0,   0.0),
    'tuck':  (0.0,  -1.40,  1.40,  0.0),
    'pick':  (0.0,  -0.60,  0.80, -0.40),
    'place': (0.0,   0.20,  0.80, -0.40),
    # Per-pot patrol pose: reach forward, pitch the wrist down so the
    # gripper camera frames the bloom from above. Tune visually in Gazebo.
    'inspect': (0.0, -0.40,  0.90, -0.80),
}

# Time the controller is given to reach each preset. Conservative — slow
# enough that the joint_trajectory_controller's velocity ramp doesn't
# saturate even from the opposite end of travel.
PRESET_TRAVEL_SECONDS: float = 3.0


class ArmPresetServer(Node):
    def __init__(self) -> None:
        super().__init__('arm_preset_server')

        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/mirte_master_arm_controller/joint_trajectory',
            10,
        )

        self._srv = self.create_service(
            SetArmPreset, '/lupin/arm/preset', self._on_set_preset,
        )

        self.get_logger().info(
            f'arm_preset_server ready — presets: {sorted(PRESETS.keys())}'
        )

    def _on_set_preset(
        self,
        req: SetArmPreset.Request,
        resp: SetArmPreset.Response,
    ) -> SetArmPreset.Response:
        key = (req.name or '').strip().lower()
        if key not in PRESETS:
            resp.success = False
            resp.message = (
                f'unknown preset "{req.name}". Known: '
                f'{sorted(PRESETS.keys())}'
            )
            self.get_logger().warn(resp.message)
            return resp

        # Clamp defensively to the canonical per-joint window so a mistuned
        # preset can never command a pose the servo silently rejects.
        positions: List[float] = [
            clamp_arm_joint(j, p) for j, p in zip(ARM_JOINTS, PRESETS[key])
        ]
        self._traj_pub.publish(
            build_arm_trajectory(list(ARM_JOINT_NAMES), positions, PRESET_TRAVEL_SECONDS)
        )

        resp.success = True
        resp.message = (
            f'preset "{key}" sent: '
            + ', '.join(f'{n}={p:+.2f}' for n, p in zip(ARM_JOINT_NAMES, positions))
        )
        self.get_logger().info(resp.message)
        return resp


def main() -> None:
    rclpy.init()
    node = ArmPresetServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
