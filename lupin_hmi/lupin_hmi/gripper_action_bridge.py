"""arm_command_bridge — HMI arm + gripper services → ros2_control.

Background
----------
The HMI's per-joint sliders used to call ``/io/servo/hiwonder/<joint>/set_angle_with_speed``
directly. On the real Mirte that service does write the servo, but the vendor
``mirte_master_arm_control`` ros2_control hardware interface re-asserts its own
commanded position. The exact mechanism (confirmed against
``mirte_master_arm_control.cpp::write``): on every control cycle the interface
re-sends ``set_angle`` for a joint when either the new command differs from the
last by more than ``SERVO_COMMAND_DIFF`` (0.05 rad) **or** the servo's measured
position has drifted more than ``SERVO_MOVED_DIFF`` (0.05 rad) since it last
moved (``servo.moved``). So an external/raw servo command is treated as "moved
by hand / by gravity" and snapped back to the controller's last commanded value
(initially 0). It is NOT a fixed 100 ms re-poll — it is a dead-band re-assert.

The fix is to drive the controllers, not the raw Hiwonder services:

  - Arm joints:  /lupin/arm/<joint>/set_angle_with_speed  →  this node
                 → /mirte_master_arm_controller/joint_trajectory (the JTC) with
                 a single 4-joint point.
  - Gripper:     /lupin/gripper/set_angle_with_speed  →  this node
                 → /mirte_master_gripper_controller/gripper_cmd (a GripperCommand
                 action goal).

The bridge runs on both sim and hardware so the HMI has one service path
everywhere.

Single commanded-pose owner
---------------------------
A per-joint slider command names ONE joint; the JTC needs all four. The bridge
keeps the authoritative HMI-commanded pose in ``self._commanded`` (seeded from
the startup pin) and rebuilds the trajectory from it, overriding only the one
joint that changed. Untouched joints are held at their last *commanded* value —
NOT at raw ``/joint_states`` feedback, which lags during motion and would yank a
still-settling joint. To stay correct when another surface (xbox teleop, a
preset, calibration) has moved the arm, ``self._commanded`` re-syncs an
untouched joint from *fresh* feedback only when it has diverged by more than
``RESYNC_THRESHOLD_RAD`` — a deliberate external move, not settling lag (slider
commands are discrete, so the arm is at rest between them).

Startup pin
-----------
On the first ``/joint_states`` reading that covers all four arm joints, the
bridge publishes ONE trajectory holding those exact positions. This seeds the
JTC's internal commanded position to match physical reality so the HW interface
stops fighting the next slider command.

Torque / enable
---------------
``/lupin/arm/set_torque`` (std_srvs/SetBool) is the operator-facing arm power
switch. It orchestrates the TWO independent gates the HMI used to conflate:

  - ``enable_all_servos`` (Hiwonder bus, SetBool): cuts/restores servo TORQUE.
  - ``enable_arm_control`` (the HW interface's own SetBool gate): stops/resumes
    the interface re-sending ``set_angle``. ``set_angle`` re-energizes a Hiwonder
    servo, so without closing this gate a "disabled" arm would be re-powered on
    the next control tick.

On DISABLE: close ``enable_arm_control`` first (stop commands), then cut torque.
On ENABLE: re-pin the JTC to the current (possibly gravity-sagged) pose so the
commanded setpoint matches reality, restore torque, then re-open the command
gate — so the first thing the interface sends equals where the arm already is,
killing the re-enable lurch (which used to come from re-asserting a stale
setpoint). ``enable_arm_control`` is skipped gracefully when absent (sim).

Init service
------------
``/lupin/arm/init`` (std_srvs/Trigger) drives the arm to the URDF zero pose
(0,0,0,0) over 5 s.

Joint order
-----------
``shoulder_pan_joint``, ``shoulder_lift_joint``, ``elbow_joint``,
``wrist_joint``. Matches ``arm_preset_server`` and ``arm_teleop``.

Limits / mapping
----------------
Per-joint clamp + gripper map come from ``arm_limits`` (the single source of
truth, mirrored by the frontend ``lib/arm.ts``). The clamp is now per-joint and
asymmetric (the real servo software limits intersected with the ±π/2 envelope),
replacing the old blanket ±π/2.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Dict, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from mirte_msgs.srv import SetServoAngleWithSpeed

from lupin_hmi.arm_limits import (
    ARM_JOINTS,
    ARM_JOINT_FULL,
    GRIPPER_MAX_EFFORT,
    clamp_arm_joint,
    gripper_deg_to_rad,
    limits_summary,
)
from lupin_hmi.arm_traj import MIN_TRAJECTORY_TIME_S, build_arm_trajectory

# Time over which /lupin/arm/init drives the arm to (0,0,0,0).
INIT_TRAVEL_S = 5.0
# A /joint_states sample older than this (s) is treated as no fresh reading.
JOINT_STATE_STALE_S = 0.5
# An untouched joint whose fresh feedback differs from the last commanded value
# by more than this (rad ≈ 4.6°) is assumed to have been moved by another
# surface (teleop / preset / calibration) and is re-synced from feedback.
RESYNC_THRESHOLD_RAD = 0.08
# Bounded wait (s) for an orchestrated SetBool sub-service to answer.
SETBOOL_TIMEOUT_S = 2.0


class LupinArmCommandBridge(Node):
    def __init__(self) -> None:
        # Node name kept as 'gripper_action_bridge' so existing systemd units
        # and onboard.launch.py logs still resolve. The class name reflects the
        # expanded scope.
        super().__init__('gripper_action_bridge')

        # Service names are parameters so a deploy can retarget them without a
        # code change (e.g. if the HW-interface node namespaces its gate).
        self.declare_parameter('torque_service', '/io/servo/hiwonder/enable_all_servos')
        self.declare_parameter('enable_arm_control_service', '/enable_arm_control')
        self._torque_srv_name = self.get_parameter('torque_service').value
        self._enable_ctrl_srv_name = self.get_parameter('enable_arm_control_service').value

        self._lock = threading.Lock()
        # Serializes the orchestrated set_torque so two near-simultaneous toggles
        # (e.g. kinesthetic-start disable racing a cancel enable) can't interleave
        # on the shared sub-clients and leave the arm in a nondeterministic state.
        self._torque_lock = threading.Lock()
        self._latest_positions: Dict[str, float] = {}
        self._latest_stamp_ns: Dict[str, int] = {}
        # Authoritative HMI-commanded pose (URDF-name keyed). NaN until the pin.
        self._commanded: Dict[str, float] = {
            ARM_JOINT_FULL[j]: float('nan') for j in ARM_JOINTS
        }
        self._jtc_pinned: bool = False

        # Reentrant group so the orchestrated set_torque callback can wait on
        # its sub-service futures (serviced by other executor threads) without
        # deadlocking.
        self._cb_group = ReentrantCallbackGroup()

        # ── Subscriptions ────────────────────────────────────────────
        self._joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_states, 10,
        )

        # ── Publishers / action clients ──────────────────────────────
        self._traj_pub = self.create_publisher(
            JointTrajectory, '/mirte_master_arm_controller/joint_trajectory', 10,
        )
        self._gripper_client = ActionClient(
            self, GripperCommand, '/mirte_master_gripper_controller/gripper_cmd',
        )

        # ── Torque sub-service clients ───────────────────────────────
        self._torque_client = self.create_client(
            SetBool, self._torque_srv_name, callback_group=self._cb_group,
        )
        self._enable_ctrl_client = self.create_client(
            SetBool, self._enable_ctrl_srv_name, callback_group=self._cb_group,
        )

        # ── Per-joint arm services ───────────────────────────────────
        for name in ARM_JOINTS:
            srv = f'/lupin/arm/{name}/set_angle_with_speed'
            self.create_service(
                SetServoAngleWithSpeed, srv,
                lambda req, resp, _name=name: self._handle_arm_set_angle(_name, req, resp),
            )

        # ── Gripper service ──────────────────────────────────────────
        self.create_service(
            SetServoAngleWithSpeed,
            '/lupin/gripper/set_angle_with_speed',
            self._handle_gripper_set_angle,
        )

        # ── Init + torque services ───────────────────────────────────
        self.create_service(Trigger, '/lupin/arm/init', self._handle_init)
        self.create_service(
            SetBool, '/lupin/arm/set_torque', self._handle_set_torque,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            'lupin arm+gripper bridge ready: '
            '/lupin/arm/<joint>/set_angle_with_speed → JTC, '
            '/lupin/gripper/set_angle_with_speed → gripper_cmd, '
            '/lupin/arm/set_torque → torque+enable, '
            '/lupin/arm/init → home over %.1fs' % INIT_TRAVEL_S
        )
        self.get_logger().info('arm limits: ' + limits_summary())

    # ── /joint_states ────────────────────────────────────────────────
    def _on_joint_states(self, msg: JointState) -> None:
        now_ns = self.get_clock().now().nanoseconds
        positions = None
        with self._lock:
            for i, name in enumerate(msg.name):
                if i < len(msg.position):
                    self._latest_positions[name] = float(msg.position[i])
                    self._latest_stamp_ns[name] = now_ns
            all_arm_seen = all(
                ARM_JOINT_FULL[j] in self._latest_positions for j in ARM_JOINTS
            )
            if all_arm_seen and not self._jtc_pinned:
                positions = [self._latest_positions[ARM_JOINT_FULL[j]] for j in ARM_JOINTS]
                for j in ARM_JOINTS:
                    self._commanded[ARM_JOINT_FULL[j]] = self._latest_positions[ARM_JOINT_FULL[j]]
                self._jtc_pinned = True

        if positions is not None:
            # Publish one trajectory holding the current pose so the JTC's
            # internal commanded setpoint matches physical reality.
            self._traj_pub.publish(
                build_arm_trajectory(
                    [ARM_JOINT_FULL[j] for j in ARM_JOINTS], positions, MIN_TRAJECTORY_TIME_S,
                )
            )
            self.get_logger().info(
                'JTC pinned to current pose: ' + ', '.join(
                    f'{ARM_JOINT_FULL[j]}={p:+.3f}' for j, p in zip(ARM_JOINTS, positions)
                )
            )

    def _fresh(self, urdf_name: str, now_ns: int) -> bool:
        stamp = self._latest_stamp_ns.get(urdf_name)
        return stamp is not None and (now_ns - stamp) <= JOINT_STATE_STALE_S * 1e9

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
        angle_rad = clamp_arm_joint(joint_name, angle_rad)

        target_urdf = ARM_JOINT_FULL[joint_name]
        positions, prev_target = self._commanded_with_override(target_urdf, angle_rad)
        if positions is None:
            resp.status = False
            self.get_logger().warn(
                f'/lupin/arm/{joint_name}: no seeded pose yet '
                '(/joint_states not received) — command dropped to avoid '
                'yanking unseeded joints'
            )
            return resp

        displacement = abs(angle_rad - prev_target)
        time_s = max(MIN_TRAJECTORY_TIME_S, displacement / rate_rad_s)
        self._traj_pub.publish(
            build_arm_trajectory([ARM_JOINT_FULL[j] for j in ARM_JOINTS], positions, time_s)
        )
        resp.status = True
        return resp

    def _commanded_with_override(
        self, target_urdf: str, override_rad: float,
    ) -> tuple[Optional[list[float]], float]:
        """Build the next 4-joint command from the authoritative commanded pose.

        Returns (positions, prev_target_for_the_overridden_joint). Untouched
        joints hold their last commanded value, except where *fresh* feedback
        shows a deliberate external move (> RESYNC_THRESHOLD_RAD), in which case
        the commanded value is re-synced from feedback first. Returns (None, 0)
        until the startup pin has seeded the pose.
        """
        now_ns = self.get_clock().now().nanoseconds
        with self._lock:
            if not self._jtc_pinned:
                return None, 0.0
            prev_target = self._commanded.get(target_urdf, 0.0)
            # Re-sync untouched joints that another surface clearly moved.
            for j in ARM_JOINTS:
                urdf = ARM_JOINT_FULL[j]
                if urdf == target_urdf:
                    continue
                cmd = self._commanded.get(urdf)
                live = self._latest_positions.get(urdf)
                if (
                    cmd is not None and live is not None and self._fresh(urdf, now_ns)
                    and not math.isnan(cmd) and abs(live - cmd) > RESYNC_THRESHOLD_RAD
                ):
                    self._commanded[urdf] = live
            self._commanded[target_urdf] = override_rad
            positions = [self._commanded[ARM_JOINT_FULL[j]] for j in ARM_JOINTS]
        if any(math.isnan(p) for p in positions):
            return None, 0.0
        return positions, prev_target

    # ── Gripper: slider call ─────────────────────────────────────────
    def _handle_gripper_set_angle(
        self,
        req: SetServoAngleWithSpeed.Request,
        resp: SetServoAngleWithSpeed.Response,
    ) -> SetServoAngleWithSpeed.Response:
        angle_deg = (
            float(req.angle) if req.degrees else math.degrees(float(req.angle))
        )
        target_rad = gripper_deg_to_rad(angle_deg)

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

    # ── /lupin/arm/set_torque ────────────────────────────────────────
    def _handle_set_torque(
        self, req: SetBool.Request, resp: SetBool.Response,
    ) -> SetBool.Response:
        # Serialize: overlapping enable/disable orchestrations would interleave
        # on the shared sub-clients and leave torque in a nondeterministic state.
        with self._torque_lock:
            if req.data:
                # ENABLE: re-pin to current pose BEFORE re-energizing so the first
                # command equals where the arm physically is (no lurch).
                self._repin_to_current()
                ok_torque, msg_torque = self._call_setbool(self._torque_client, True)
                ok_ctrl, msg_ctrl = self._call_setbool(self._enable_ctrl_client, True, optional=True)
                resp.success = ok_torque
                resp.message = f'arm torque ON ({msg_torque}); command gate ({msg_ctrl})'
            else:
                # DISABLE: close the command gate first (stop re-sends that would
                # re-energize the servo), then cut torque.
                ok_ctrl, msg_ctrl = self._call_setbool(self._enable_ctrl_client, False, optional=True)
                ok_torque, msg_torque = self._call_setbool(self._torque_client, False)
                resp.success = ok_torque
                resp.message = f'arm torque OFF ({msg_torque}); command gate ({msg_ctrl})'
        self.get_logger().info(f'set_torque({req.data}): {resp.message}')
        return resp

    def _repin_to_current(self) -> None:
        with self._lock:
            if not all(ARM_JOINT_FULL[j] in self._latest_positions for j in ARM_JOINTS):
                return
            positions = [self._latest_positions[ARM_JOINT_FULL[j]] for j in ARM_JOINTS]
            for j in ARM_JOINTS:
                self._commanded[ARM_JOINT_FULL[j]] = self._latest_positions[ARM_JOINT_FULL[j]]
        self._traj_pub.publish(
            build_arm_trajectory(
                [ARM_JOINT_FULL[j] for j in ARM_JOINTS], positions, MIN_TRAJECTORY_TIME_S,
            )
        )

    def _call_setbool(self, client, value: bool, optional: bool = False) -> tuple[bool, str]:
        """Call a SetBool sub-service and wait (bounded) for the result. When
        ``optional`` (e.g. enable_arm_control, absent in sim), a missing service
        is reported as a soft skip rather than a failure."""
        if not client.service_is_ready():
            if not client.wait_for_service(timeout_sec=0.5):
                msg = f'{client.srv_name} unavailable'
                if optional:
                    return True, f'{msg} — skipped'
                self.get_logger().warn(msg)
                return False, msg
        req = SetBool.Request()
        req.data = value
        future = client.call_async(req)
        deadline = time.monotonic() + SETBOOL_TIMEOUT_S
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            return False, f'{client.srv_name} timed out'
        try:
            res = future.result()
        except Exception as exc:  # noqa: BLE001
            return False, f'{client.srv_name} error: {exc}'
        return bool(res.success), res.message or 'ok'

    # ── /lupin/arm/init ──────────────────────────────────────────────
    def _handle_init(self, req: Trigger.Request, resp: Trigger.Response) -> Trigger.Response:
        _ = req
        zero = [0.0] * len(ARM_JOINTS)
        with self._lock:
            for j in ARM_JOINTS:
                self._commanded[ARM_JOINT_FULL[j]] = 0.0
        self._traj_pub.publish(
            build_arm_trajectory([ARM_JOINT_FULL[j] for j in ARM_JOINTS], zero, INIT_TRAVEL_S)
        )
        resp.success = True
        resp.message = f'arm/init: driving to (0,0,0,0) over {INIT_TRAVEL_S:.1f}s'
        self.get_logger().info(resp.message)
        return resp


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
