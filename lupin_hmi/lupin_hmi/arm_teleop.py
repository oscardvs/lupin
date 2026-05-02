"""arm_teleop — drive the 4-DOF arm from /joy.

Axis / button indices are parameterised so the same node works for
DualShock and Xbox layouts:

    Xbox (default):                   DualShock 4 (legacy):
      shoulder_pan  = axes[3]           axes[2]
      shoulder_lift = axes[4]           axes[3]
      elbow ±       = axes[6]           buttons[11/12]   (D-pad)
      wrist ±       = axes[7]           buttons[13/14]

For the DualShock layout, override these via launch parameters:

    parameters=[{
        'shoulder_pan_axis': 2,
        'shoulder_lift_axis': 3,
        'elbow_axis': -1, 'elbow_plus_button': 11, 'elbow_minus_button': 12,
        'wrist_axis': -1, 'wrist_plus_button': 13, 'wrist_minus_button': 14,
    }]

Setting an `*_axis` to -1 falls back to the *_plus/*_minus button pair.

Shoulder enable button: when ``shoulder_enable_button`` is ≥ 0, the
shoulder pan/lift axes are only read while that button is held. This
matters when the same stick (Xbox right stick X) is also bound to
chassis yaw on twist_mux — without an enable button, every yaw command
would also try to swing the arm. The wrist/elbow channels intentionally
have no gating: D-pad presses are deliberate and don't conflict with
anything else.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')

        # Defaults match the Xbox Wireless Controller mapping under xpad.
        self.declare_parameter('shoulder_pan_axis', 3)
        self.declare_parameter('shoulder_lift_axis', 4)
        self.declare_parameter('elbow_axis', 6)
        self.declare_parameter('wrist_axis', 7)
        self.declare_parameter('elbow_plus_button', -1)
        self.declare_parameter('elbow_minus_button', -1)
        self.declare_parameter('wrist_plus_button', -1)
        self.declare_parameter('wrist_minus_button', -1)
        # Gates the shoulder pan/lift axes only — see module docstring.
        # -1 disables gating (axes always live). Default 4 = Xbox LB.
        self.declare_parameter('shoulder_enable_button', 4)
        # Hardware fix: Hiwonder steppers stall on tiny moves; 0.15 rad/tick is
        # the smallest reliable step we measured on the real arm.
        self.declare_parameter('step_rad', 0.15)
        # Deadzone large enough to ignore stick noise but small enough to
        # respond to a deliberate nudge.
        self.declare_parameter('deadzone', 0.2)

        self._pan_ax = self._iparam('shoulder_pan_axis')
        self._lift_ax = self._iparam('shoulder_lift_axis')
        self._elbow_ax = self._iparam('elbow_axis')
        self._wrist_ax = self._iparam('wrist_axis')
        self._elbow_plus_btn = self._iparam('elbow_plus_button')
        self._elbow_minus_btn = self._iparam('elbow_minus_button')
        self._wrist_plus_btn = self._iparam('wrist_plus_button')
        self._wrist_minus_btn = self._iparam('wrist_minus_button')
        self._shoulder_enable_btn = self._iparam('shoulder_enable_button')
        self._step = self.get_parameter('step_rad').value
        self._deadzone = self.get_parameter('deadzone').value

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/mirte_master_arm_controller/joint_trajectory',
            10)
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        # Hardware Fix: Slower 10Hz loop so the stepper motors don't choke.
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_joint',
        ]

        # Start at neutral positions matching the URDF's stowed pose.
        self.current_positions = [0.0, -1.56, -1.56, 1.56]
        self.joy_cmds = [0.0, 0.0, 0.0, 0.0]

    def _iparam(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    @staticmethod
    def _read_axis(msg: Joy, idx: int) -> float:
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        return float(msg.axes[idx])

    @staticmethod
    def _read_button_pair(msg: Joy, plus_idx: int, minus_idx: int) -> float:
        if 0 <= plus_idx < len(msg.buttons) and msg.buttons[plus_idx] == 1:
            return 1.0
        if 0 <= minus_idx < len(msg.buttons) and msg.buttons[minus_idx] == 1:
            return -1.0
        return 0.0

    def joy_callback(self, msg: Joy) -> None:
        shoulder_live = (
            self._shoulder_enable_btn < 0
            or (self._shoulder_enable_btn < len(msg.buttons)
                and msg.buttons[self._shoulder_enable_btn] == 1)
        )
        if shoulder_live:
            self.joy_cmds[0] = self._read_axis(msg, self._pan_ax)
            self.joy_cmds[1] = self._read_axis(msg, self._lift_ax)
        else:
            self.joy_cmds[0] = 0.0
            self.joy_cmds[1] = 0.0

        elbow_axis_val = self._read_axis(msg, self._elbow_ax)
        self.joy_cmds[2] = (
            elbow_axis_val if abs(elbow_axis_val) > 1e-3
            else self._read_button_pair(msg, self._elbow_plus_btn, self._elbow_minus_btn)
        )
        wrist_axis_val = self._read_axis(msg, self._wrist_ax)
        self.joy_cmds[3] = (
            wrist_axis_val if abs(wrist_axis_val) > 1e-3
            else self._read_button_pair(msg, self._wrist_plus_btn, self._wrist_minus_btn)
        )

    def timer_callback(self) -> None:
        moved = False
        for i in range(4):
            if abs(self.joy_cmds[i]) > self._deadzone:
                self.current_positions[i] += self.joy_cmds[i] * self._step
                moved = True
        if moved:
            self.send_trajectory()

    def send_trajectory(self) -> None:
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = self.current_positions
        # HARDWARE FIX: Force a positive velocity to bypass Telemetrix 0.0 error.
        point.velocities = [1.0, 1.0, 1.0, 1.0]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100_000_000  # 0.1 s

        traj.points.append(point)
        self.publisher_.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = ArmTeleop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
