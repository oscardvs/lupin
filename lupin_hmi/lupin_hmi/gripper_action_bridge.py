"""arm_command_bridge — HMI arm + gripper services → ros2_control.

Background
----------
The HMI's per-joint sliders used to call ``/io/servo/hiwonder/<joint>/set_angle_with_speed``
directly. On the real Mirte that service does write the servo, but the
vendor ``mirte_master_arm_control`` ros2_control hardware interface (the
``ros2_arm_control_hw_interface`` + ``ros2_gripper_control_hw_interface``
plugins) ticks at 10 Hz with ``servo_moved_dead_band: 0.05 rad``. It
treats any external servo motion > 0.05 rad as "moved by hand / by
gravity" and immediately re-asserts its own commanded position on the
next tick. That commanded position is whatever the trajectory /
gripper-command controller last received — initially 0. So the joint
visibly moves toward the slider target and then snaps back.

The fix is to drive the controllers, not the raw Hiwonder services:

  - Arm joints:  /lupin/arm/<joint>/set_angle_with_speed  →  this node
                 → /mirte_master_arm_controller/joint_trajectory (the
                 JointTrajectoryController) with a single 4-joint point.
  - Gripper:     /lupin/gripper/set_angle_with_speed  →  this node
                 → /mirte_master_gripper_controller/gripper_cmd (a
                 GripperCommand action goal).

The bridge runs on both sim and hardware so the HMI has one service path
everywhere. In sim, ``arm_sim_shim`` still republishes
``/io/servo/hiwonder/<joint>/position`` for HMI feedback; the bridge no
longer overlaps with the shim because all command flows are namespaced
under ``/lupin/``.

Startup pin
-----------
On the first ``/joint_states`` reading that covers all four arm joints,
the bridge publishes ONE trajectory holding those exact positions. This
seeds the JointTrajectoryController's internal commanded position to
match the physical pose, so the HW interface stops fighting the next
slider command. Without this, the first slider call would build a
trajectory whose three untouched joints happen to match current state —
but the JTC's stale commanded setpoint (likely 0) would still cause a
yank in the gap.

Init service
------------
``/lupin/arm/init`` (std_srvs/Trigger) drives the arm to the URDF zero
pose (0, 0, 0, 0) over 5 s. Conservative enough that the JTC's velocity
ramp doesn't saturate from any starting pose inside ±π/2. Use this as
the operator-facing "go to a known position" button.

Joint order
-----------
``shoulder_pan_joint``, ``shoulder_lift_joint``, ``elbow_joint``,
``wrist_joint``. Matches ``arm_preset_server`` and ``arm_teleop`` so all
three publishers agree on the JTC's joint vector.

Mapping notes
-------------
* Arm joints: HMI degrees → radians 1:1. URDF declares ±π/2 for all
  four; the slider window is ±90°.
* Gripper: HMI ±30° → URDF gripper_joint [-0.20, +0.25] rad (linear).
  The HMI window is intentionally narrower than ±90° because the
  gripper_joint URDF limit is asymmetric and not in degree-scale.
* ``time_from_start`` per trajectory point is computed from the HMI's
  rate setting: ``time = max(MIN_TRAJECTORY_TIME_S, |Δ| / rate_rad_s)``.
* Every trajectory carries non-zero velocities — the Hiwonder Telemetrix
  hardware interface rejects points with velocity 0.0 (per arm_teleop
  comment). We set 1.0 rad/s as the per-joint velocity hint.
"""

from __future__ import annotations

import math
import threading
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mirte_msgs.srv import SetServoAngleWithSpeed


# ── Arm joint configuration ────────────────────────────────────────────
ARM_JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow', 'wrist')
ARM_JOINT_FULL = {j: f'{j}_joint' for j in ARM_JOINTS}
ARM_URDF_LIMIT_RAD = math.pi / 2  # ±π/2 per URDF / arm.xacro

# ── Gripper configuration ──────────────────────────────────────────────
GRIPPER_HMI_MIN_DEG = -30.0
GRIPPER_HMI_MAX_DEG = 30.0
GRIPPER_URDF_MIN_RAD = -0.20
GRIPPER_URDF_MAX_RAD = 0.25
GRIPPER_MAX_EFFORT = 2.0  # matches URDF effort cap on gripper_joint

# ── Trajectory tunables ────────────────────────────────────────────────
# Floor on time_from_start: zero is invalid for JTC; very small values
# race with the controller's 10 Hz update period.
MIN_TRAJECTORY_TIME_S = 0.1
# Per-joint velocity hint. The Hiwonder Telemetrix HW interface rejects
# points with velocity 0.0 on hardware (see arm_teleop comment); pick a
# non-zero default. In sim the JTC uses it as a trajectory hint.
TRAJECTORY_VELOCITY_RAD_S = 1.0
# Time over which /lupin/arm/init drives the arm to (0,0,0,0). 5 s is
# slow enough that any starting pose inside ±π/2 lands without saturating
# the JTC's velocity ramp.
INIT_TRAVEL_S = 5.0


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


def _duration_from_seconds(seconds: float) -> Duration:
    sec = int(seconds)
    nsec = int(round((seconds - sec) * 1e9))
    d = Duration()
    d.sec = sec
    d.nanosec = nsec
    return d


class LupinArmCommandBridge(Node):
    def __init__(self) -> None:
        # Node name kept as 'gripper_action_bridge' so existing systemd
        # units and onboard.launch.py logs still resolve. The class name
        # reflects the expanded scope.
        super().__init__('gripper_action_bridge')

        self._lock = threading.Lock()
        self._latest_positions: Dict[str, float] = {}
        # Set once we've published the startup pin trajectory.
        self._jtc_pinned: bool = False

        # ── Subscriptions ────────────────────────────────────────────
        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10,
        )

        # ── Publishers / action clients ──────────────────────────────
        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/mirte_master_arm_controller/joint_trajectory',
            10,
        )
        self._gripper_client = ActionClient(
            self, GripperCommand,
            '/mirte_master_gripper_controller/gripper_cmd',
        )

        # ── Per-joint arm services ───────────────────────────────────
        for name in ARM_JOINTS:
            srv = f'/lupin/arm/{name}/set_angle_with_speed'
            # Capture the joint name in a default arg — service callbacks
            # don't carry per-service context otherwise.
            self.create_service(
                SetServoAngleWithSpeed, srv,
                lambda req, resp, _name=name: self._handle_arm_set_angle(_name, req, resp),
            )

        # ── Gripper service (unchanged wire format) ──────────────────
        self.create_service(
            SetServoAngleWithSpeed,
            '/lupin/gripper/set_angle_with_speed',
            self._handle_gripper_set_angle,
        )

        # ── Init service: drive arm to (0,0,0,0) over INIT_TRAVEL_S ──
        self.create_service(
            Trigger, '/lupin/arm/init', self._handle_init,
        )

        self.get_logger().info(
            'lupin arm+gripper bridge ready: '
            '/lupin/arm/<joint>/set_angle_with_speed → JTC, '
            '/lupin/gripper/set_angle_with_speed → gripper_cmd, '
            '/lupin/arm/init → home over %.1fs' % INIT_TRAVEL_S
        )

    # ── /joint_states ────────────────────────────────────────────────
    def _on_joint_states(self, msg: JointState) -> None:
        all_arm_seen = False
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._latest_positions[name] = float(msg.position[i])
            all_arm_seen = all(
                ARM_JOINT_FULL[j] in self._latest_positions for j in ARM_JOINTS
            )
            should_pin = all_arm_seen and not self._jtc_pinned
            if should_pin:
                positions = [self._latest_positions[ARM_JOINT_FULL[j]] for j in ARM_JOINTS]
                self._jtc_pinned = True

        if should_pin:
            # Publish one trajectory holding the current pose, so the JTC's
            # internal commanded setpoint matches physical reality. Without
            # this the HW interface keeps reasserting whatever stale value
            # it last had — typically 0.
            self._publish_arm_trajectory(positions, MIN_TRAJECTORY_TIME_S)
            self.get_logger().info(
                'JTC pinned to current pose: ' + ', '.join(
                    f'{ARM_JOINT_FULL[j]}={p:+.3f}'
                    for j, p in zip(ARM_JOINTS, positions)
                )
            )

    # ── Arm: per-joint slider call ───────────────────────────────────
    def _handle_arm_set_angle(
        self,
        joint_name: str,
        req: SetServoAngleWithSpeed.Request,
        resp: SetServoAngleWithSpeed.Response,
    ) -> SetServoAngleWithSpeed.Response:
        angle_rad = float(req.angle) if not req.degrees else math.radians(float(req.angle))
        rate_rad_s = float(req.rate)
        if req.degrees:
            rate_rad_s = math.radians(rate_rad_s)
        if rate_rad_s <= 0.0:
            rate_rad_s = math.radians(60.0)  # sensible default
        angle_rad = _clamp(angle_rad, -ARM_URDF_LIMIT_RAD, ARM_URDF_LIMIT_RAD)

        target_urdf = ARM_JOINT_FULL[joint_name]
        positions, current_target = self._snapshot_arm_with_override(target_urdf, angle_rad)
        if positions is None:
            resp.status = False
            self.get_logger().warn(
                f'/lupin/arm/{joint_name}: /joint_states not yet received; '
                'command dropped to avoid yanking unseeded joints'
            )
            return resp

        displacement = abs(angle_rad - current_target)
        time_s = max(MIN_TRAJECTORY_TIME_S, displacement / rate_rad_s)
        self._publish_arm_trajectory(positions, time_s)

        resp.status = True
        return resp

    # ── Gripper: slider call ─────────────────────────────────────────
    def _handle_gripper_set_angle(
        self,
        req: SetServoAngleWithSpeed.Request,
        resp: SetServoAngleWithSpeed.Response,
    ) -> SetServoAngleWithSpeed.Response:
        # HMI always sends degrees=True, but accept rad too — the URDF
        # mapping is defined against the degree window, so convert
        # rad → deg first and then run the linear map.
        angle_deg = (
            float(req.angle) if req.degrees else math.degrees(float(req.angle))
        )
        target_rad = _gripper_deg_to_rad(angle_deg)

        if not self._gripper_client.server_is_ready():
            if not self._gripper_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn(
                    'gripper_cmd action server not available; command dropped'
                )
                resp.status = False
                return resp

        goal = GripperCommand.Goal()
        goal.command.position = target_rad
        goal.command.max_effort = GRIPPER_MAX_EFFORT
        # Fire-and-forget by design (the HMI does not await execution), but
        # attach callbacks so a rejected/aborted goal — e.g. a Hiwonder
        # effort/thermal stall — is logged instead of silently masked
        # behind status:true. The synchronous resp.status only reports that
        # the command was dispatched, not that the jaw reached the target.
        future = self._gripper_client.send_goal_async(goal)
        future.add_done_callback(self._on_gripper_goal_response)

        resp.status = True
        return resp

    def _on_gripper_goal_response(self, future) -> None:
        try:
            handle = future.result()
        except Exception as exc:  # noqa: BLE001 - log and move on
            self.get_logger().warn(f'gripper goal dispatch failed: {exc}')
            return
        if not handle.accepted:
            self.get_logger().warn('gripper goal REJECTED by controller')
            return
        handle.get_result_async().add_done_callback(self._on_gripper_result)

    def _on_gripper_result(self, future) -> None:
        try:
            status = future.result().status
        except Exception as exc:  # noqa: BLE001 - log and move on
            self.get_logger().warn(f'gripper result unavailable: {exc}')
            return
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'gripper goal did not succeed (GoalStatus={status}) — '
                'jaw may not have reached the commanded position'
            )

    # ── /lupin/arm/init ──────────────────────────────────────────────
    def _handle_init(
        self,
        req: Trigger.Request,
        resp: Trigger.Response,
    ) -> Trigger.Response:
        _ = req  # Trigger has no fields
        zero_positions = [0.0] * len(ARM_JOINTS)
        self._publish_arm_trajectory(zero_positions, INIT_TRAVEL_S)
        resp.success = True
        resp.message = f'arm/init: driving to (0,0,0,0) over {INIT_TRAVEL_S:.1f}s'
        self.get_logger().info(resp.message)
        return resp

    # ── Helpers ──────────────────────────────────────────────────────
    def _snapshot_arm_with_override(
        self, target_urdf: str, override_rad: float,
    ) -> tuple[Optional[list[float]], float]:
        """Return (positions, current_target_value).

        positions is a length-4 list ordered by ARM_JOINTS, with
        target_urdf replaced by override_rad and the other three joints
        seeded from the latest /joint_states. Returns (None, 0.0) if
        /joint_states hasn't covered all four arm joints yet — caller
        must drop the command rather than build a trajectory with
        unknown joint positions (which would yank to 0).
        """
        with self._lock:
            if not all(ARM_JOINT_FULL[j] in self._latest_positions for j in ARM_JOINTS):
                return None, 0.0
            positions = []
            current_target_value = 0.0
            for j in ARM_JOINTS:
                urdf_name = ARM_JOINT_FULL[j]
                if urdf_name == target_urdf:
                    positions.append(override_rad)
                    current_target_value = self._latest_positions[urdf_name]
                else:
                    positions.append(self._latest_positions[urdf_name])
        return positions, current_target_value

    def _publish_arm_trajectory(self, positions: list[float], time_s: float) -> None:
        # NOTE: /mirte_master_arm_controller/joint_trajectory is shared with
        # arm_teleop (xbox), arm_preset_server, and (in sim) arm_sim_shim.
        # There is no arbiter/mux on the arm topic (unlike the chassis'
        # twist_mux): the JTC treats each trajectory as a new goal that
        # preempts the in-flight one. Single-input is expected — drive the
        # arm from ONE surface at a time (HMI sliders / xbox / voice presets).
        traj = JointTrajectory()
        traj.joint_names = [ARM_JOINT_FULL[j] for j in ARM_JOINTS]
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.velocities = [TRAJECTORY_VELOCITY_RAD_S] * len(ARM_JOINTS)
        point.time_from_start = _duration_from_seconds(time_s)
        traj.points = [point]
        self._traj_pub.publish(traj)


def main() -> None:
    rclpy.init()
    node = LupinArmCommandBridge()
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
