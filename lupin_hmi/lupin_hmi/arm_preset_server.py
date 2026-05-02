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
``home``   — all four joints at 0 (the URDF zero pose).
``tuck``   — arm folded onto the chassis: shoulder_lift up, elbow back so
             the wrist sits over the base footprint. Conservative against
             the URDF ±π/2 limits.
``pick``   — arm extended forward, wrist level with the table for a
             top-down approach.
``place``  — arm extended forward, wrist raised so an item carried in the
             gripper clears table edges.

These are conservative starting points — re-tune on the real robot once
the arm is mounted in its final configuration. Keep all values inside the
URDF ±π/2 limits and the ±90° HMI slider window.
"""

from __future__ import annotations

from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from lupin_msgs.srv import SetArmPreset


ARM_JOINT_NAMES: Tuple[str, ...] = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_joint',
)

# All values in radians. Order matches ARM_JOINT_NAMES.
PRESETS: Dict[str, Tuple[float, float, float, float]] = {
    'home':  (0.0,  0.0,   0.0,   0.0),
    'tuck':  (0.0, -1.40,  1.40,  0.0),
    'pick':  (0.0, -0.60,  0.80, -0.40),
    'place': (0.0,  0.20,  0.80, -0.40),
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

        positions: List[float] = list(PRESETS[key])

        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=int(PRESET_TRAVEL_SECONDS),
            nanosec=int(round((PRESET_TRAVEL_SECONDS % 1) * 1e9)),
        )
        traj.points = [point]
        self._traj_pub.publish(traj)

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
