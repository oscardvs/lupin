"""arm_sim_shim — expose Hiwonder-style arm services on top of ros2_control.

The Lupin Web HMI talks to the arm via the real-robot Hiwonder serial-bus
services (``/io/servo/hiwonder/<joint>/set_angle_with_speed``,
``/io/servo/hiwonder/enable_all_servos``) and reads ``ServoPosition``
feedback from ``/io/servo/hiwonder/<joint>/position``. In simulation those
services do not exist — the MIRTE Gazebo bringup spawns ros2_control
instead:

    * ``mirte_master_arm_controller`` (JointTrajectoryController) —
      command via ``/mirte_master_arm_controller/joint_trajectory``,
      joints ``shoulder_pan_joint`` / ``shoulder_lift_joint`` /
      ``elbow_joint`` / ``wrist_joint``.
    * ``mirte_master_gripper_controller`` (GripperActionController) —
      command via the ``gripper_cmd`` action, joint ``gripper_joint``.
    * ``joint_state_broadcaster`` publishes ``/joint_states`` for all five.

This node is a thin adapter so the same web UI works against the sim
without sim-aware branching in the frontend. On the real robot the node
is not launched — Hiwonder's ``mirte_telemetrix_cpp`` already serves
those names natively.

Gripper command path is owned by ``gripper_action_bridge`` (runs on both
sim and hardware) — the HMI gripper slider calls
``/lupin/gripper/set_angle_with_speed`` there so the controller learns
about the new commanded position. Routing the gripper through the raw
Hiwonder service caused the vendor HW interface to immediately re-assert
its stale 0 setpoint, snapping the gripper back. This shim still
republishes ``/io/servo/hiwonder/gripper/position`` for HMI feedback.

Mapping notes
-------------
* Arm joints: HMI degrees → radians 1:1, then clamped to the canonical
  per-joint window from ``arm_limits`` (parity with the hardware bridge).
* JointTrajectoryController in this YAML has
  ``allow_partial_joints_goal: false``, so every published trajectory
  carries all four arm joint targets. We remember the last commanded
  target per joint (seeded from the latest ``/joint_states`` reading)
  and rebuild the full vector on every call.
* The HMI's ``rate`` (deg/s) controls the trajectory ``time_from_start``:
  ``time = |target - current| / rate``, with a small floor so a
  zero-displacement target still gets a valid future timestamp.
"""

from __future__ import annotations

import math
import threading
from typing import Dict

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool
from trajectory_msgs.msg import JointTrajectory  # publisher message type

from mirte_msgs.msg import ServoPosition
from mirte_msgs.srv import SetServoAngleWithSpeed

from lupin_hmi.arm_limits import ARM_JOINTS, ARM_JOINT_FULL, clamp_arm_joint
from lupin_hmi.arm_traj import MIN_TRAJECTORY_TIME_S, build_arm_trajectory

GRIPPER_JOINT = 'gripper_joint'


class ArmSimShim(Node):
    def __init__(self) -> None:
        super().__init__('arm_sim_shim')

        # Mutable per-joint target cache. Seeded once from /joint_states so
        # the first slider command for one joint doesn't yank the others to
        # 0; after that we only update on explicit user commands. Keys are
        # the URDF joint names (with `_joint` suffix).
        self._targets: Dict[str, float] = {
            ARM_JOINT_FULL[j]: 0.0 for j in ARM_JOINTS
        }
        self._targets_seeded: bool = False
        self._latest_positions: Dict[str, float] = {}
        self._latest_velocities: Dict[str, float] = {}
        self._lock = threading.Lock()

        # /joint_states → cache + ServoPosition publishers.
        self._sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10,
        )
        self._position_pubs: Dict[str, 'rclpy.publisher.Publisher'] = {}
        for name in (*ARM_JOINTS, 'gripper'):
            topic = f'/io/servo/hiwonder/{name}/position'
            self._position_pubs[name] = self.create_publisher(
                ServoPosition, topic, 10,
            )

        # Trajectory publisher for the arm controller.
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/mirte_master_arm_controller/joint_trajectory',
            10,
        )

        # Hiwonder-compatible services for the four arm joints. The gripper
        # is handled by gripper_action_bridge under /lupin/gripper/* so the
        # HMI command goes through the GripperActionController instead of
        # racing against the vendor HW interface — see the module docstring.
        for name in ARM_JOINTS:
            srv = f'/io/servo/hiwonder/{name}/set_angle_with_speed'
            # Capture the joint name in the default arg — service callbacks
            # don't carry per-service context otherwise.
            self.create_service(
                SetServoAngleWithSpeed, srv,
                lambda req, resp, _name=name: self._handle_arm_set_angle(_name, req, resp),
            )

        self.create_service(
            SetBool,
            '/io/servo/hiwonder/enable_all_servos',
            self._handle_enable_all,
        )

        self.get_logger().info(
            'arm_sim_shim ready: forwarding /io/servo/hiwonder/<arm>/* to '
            'mirte_master_arm_controller (gripper handled by '
            'gripper_action_bridge)'
        )

    # ── /joint_states bridge ────────────────────────────────────────────
    def _on_joint_states(self, msg: JointState) -> None:
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._latest_positions[name] = float(msg.position[i])
                if i < len(msg.velocity):
                    self._latest_velocities[name] = float(msg.velocity[i])

            # Seed the target cache from the first /joint_states reading
            # that covers all four arm joints, so the first command for one
            # joint doesn't snap the un-touched joints back to 0 rad.
            # Skipped if Gazebo hasn't yet produced positions for everyone.
            if not self._targets_seeded and all(
                ARM_JOINT_FULL[j] in self._latest_positions for j in ARM_JOINTS
            ):
                for j in ARM_JOINTS:
                    name = ARM_JOINT_FULL[j]
                    self._targets[name] = self._latest_positions[name]
                self._targets_seeded = True

        # Republish each known servo position on the Hiwonder-style topic.
        for hmi_name, urdf_name in (
            *((j, ARM_JOINT_FULL[j]) for j in ARM_JOINTS),
            ('gripper', GRIPPER_JOINT),
        ):
            if urdf_name not in self._latest_positions:
                continue
            pub = self._position_pubs[hmi_name]
            sp = ServoPosition()
            sp.header = Header()
            sp.header.stamp = msg.header.stamp
            sp.header.frame_id = urdf_name
            sp.angle = self._latest_positions[urdf_name]
            # `raw` is a 12-bit servo count on hardware; we don't have one,
            # so leave it at 0. The HMI only reads `.angle`.
            sp.raw = 0
            pub.publish(sp)

    # ── /io/servo/hiwonder/<arm_joint>/set_angle_with_speed ─────────────
    def _handle_arm_set_angle(
        self,
        joint_name: str,
        req: SetServoAngleWithSpeed.Request,
        resp: SetServoAngleWithSpeed.Response,
    ) -> SetServoAngleWithSpeed.Response:
        # Convert request angle to radians and clamp to the canonical window
        # (parity with the hardware bridge — see arm_limits).
        angle_rad = float(req.angle) if not req.degrees else math.radians(float(req.angle))
        angle_rad = clamp_arm_joint(joint_name, angle_rad)
        rate_rad_s = float(req.rate)
        if req.degrees:
            rate_rad_s = math.radians(rate_rad_s)
        if rate_rad_s <= 0.0:
            rate_rad_s = math.radians(60.0)  # sensible default

        urdf_name = ARM_JOINT_FULL[joint_name]

        with self._lock:
            current = self._latest_positions.get(urdf_name, self._targets[urdf_name])
            self._targets[urdf_name] = angle_rad
            target_snapshot = dict(self._targets)

        displacement = abs(angle_rad - current)
        time_s = max(MIN_TRAJECTORY_TIME_S, displacement / rate_rad_s)
        joint_names = [ARM_JOINT_FULL[j] for j in ARM_JOINTS]
        self._traj_pub.publish(
            build_arm_trajectory(joint_names, [target_snapshot[n] for n in joint_names], time_s)
        )

        resp.status = True
        return resp

    # ── /io/servo/hiwonder/enable_all_servos ────────────────────────────
    def _handle_enable_all(
        self,
        req: SetBool.Request,
        resp: SetBool.Response,
    ) -> SetBool.Response:
        # ros2_control controllers in this sim are spawned active and
        # don't have a runtime enable/disable equivalent. Acknowledge so
        # the HMI's enable button shows green.
        resp.success = True
        resp.message = 'sim shim: ros2_control controllers always active'
        _ = req  # unused — sim has no per-servo torque toggle
        return resp


def main() -> None:
    rclpy.init()
    node = ArmSimShim()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
