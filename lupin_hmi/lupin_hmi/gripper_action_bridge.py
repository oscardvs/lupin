"""gripper_action_bridge — HMI gripper service → ros2_control gripper action.

Background
----------
The HMI's gripper slider used to call ``/io/servo/hiwonder/gripper/set_angle_with_speed``
directly. On the real Mirte that service does write the servo, but the
vendor ``mirte_master_arm_control`` ros2_control hardware interface (which
runs ``mirte_master_gripper_controller`` at 10 Hz) treats any external
servo motion > 0.05 rad as "moved by hand / by gravity" and immediately
re-asserts its own commanded position on the next tick. That commanded
position starts at 0 and stays at 0 until the GripperActionController
receives a goal — so the gripper would visibly move toward the slider
target and then snap back to the controller's stale 0 setpoint.

The fix is to send the HMI's command through the controller instead of
around it: this node accepts a ``SetServoAngleWithSpeed`` request (the
HMI's existing wire format) and forwards it as a ``GripperCommand``
action goal. The controller then knows about the new target, the HW
interface's "correction" writes that same target, and the gripper holds.

The bridge runs on both sim and hardware so the HMI gripper slider has
one service path everywhere. ``arm_sim_shim`` no longer needs to handle
the gripper case — its responsibility is now arm joints + position
republish + enable_all_servos.

Service contract
----------------
Name: ``/lupin/gripper/set_angle_with_speed``
Type: ``mirte_msgs/srv/SetServoAngleWithSpeed`` (matches the HMI's
existing wire format so the frontend only changes the service path).

The HMI gripper window is ±30° (see ``ArmView.tsx``); URDF
``gripper_joint`` is ``[-0.20, 0.25]`` rad. The slider degrees are
mapped linearly to the URDF rad window — so +30° fully opens and
-30° fully closes inside the joint limits — and forwarded as a
``GripperCommand`` goal. The bridge returns success synchronously
after sending the goal (fire-and-forget); the controller handles the
motion and preempts in-flight goals on its own.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import GripperCommand
from mirte_msgs.srv import SetServoAngleWithSpeed


GRIPPER_HMI_MIN_DEG = -30.0
GRIPPER_HMI_MAX_DEG = 30.0
GRIPPER_URDF_MIN_RAD = -0.20
GRIPPER_URDF_MAX_RAD = 0.25
GRIPPER_MAX_EFFORT = 2.0  # matches URDF effort cap on gripper_joint


def _clamp(value: float, lo: float, hi: float) -> float:
    if value < lo:
        return lo
    if value > hi:
        return hi
    return value


def _gripper_deg_to_rad(angle_deg: float) -> float:
    deg = _clamp(angle_deg, GRIPPER_HMI_MIN_DEG, GRIPPER_HMI_MAX_DEG)
    span_in = GRIPPER_HMI_MAX_DEG - GRIPPER_HMI_MIN_DEG
    span_out = GRIPPER_URDF_MAX_RAD - GRIPPER_URDF_MIN_RAD
    ratio = (deg - GRIPPER_HMI_MIN_DEG) / span_in
    return GRIPPER_URDF_MIN_RAD + ratio * span_out


class GripperActionBridge(Node):
    def __init__(self) -> None:
        super().__init__('gripper_action_bridge')

        self._gripper_client = ActionClient(
            self, GripperCommand,
            '/mirte_master_gripper_controller/gripper_cmd',
        )

        self.create_service(
            SetServoAngleWithSpeed,
            '/lupin/gripper/set_angle_with_speed',
            self._handle_set_angle,
        )

        self.get_logger().info(
            'gripper_action_bridge ready — /lupin/gripper/set_angle_with_speed '
            '→ /mirte_master_gripper_controller/gripper_cmd'
        )

    def _handle_set_angle(
        self,
        req: SetServoAngleWithSpeed.Request,
        resp: SetServoAngleWithSpeed.Response,
    ) -> SetServoAngleWithSpeed.Response:
        # The HMI always sends degrees=True, but accept rad inputs too —
        # the URDF mapping is defined against the degree window, so convert
        # rad to deg first and then run the linear map.
        angle_deg = (
            float(req.angle) if req.degrees else math.degrees(float(req.angle))
        )
        target_rad = _gripper_deg_to_rad(angle_deg)

        if not self._gripper_client.server_is_ready():
            # Wait briefly — on cold start the spawner may not have brought
            # mirte_master_gripper_controller up yet.
            if not self._gripper_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    'gripper_cmd action server not available; command dropped'
                )
                resp.status = False
                return resp

        goal = GripperCommand.Goal()
        goal.command.position = target_rad
        goal.command.max_effort = GRIPPER_MAX_EFFORT

        # Fire-and-forget — controller preempts in-flight goals.
        self._gripper_client.send_goal_async(goal)

        resp.status = True
        return resp


def main() -> None:
    rclpy.init()
    node = GripperActionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
