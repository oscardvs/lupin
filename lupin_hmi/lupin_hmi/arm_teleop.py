"""arm_teleop — drive the 4-DOF arm + 1-DOF gripper from /joy.

The vendor `mirte_master_arm_control` config exposes two controllers:
  - mirte_master_arm_controller    (JointTrajectoryController) → 4 joints
  - mirte_master_gripper_controller (GripperActionController)  → gripper_joint
Both run inside a controller_manager ticking at 10 Hz, so this node
publishes trajectories at the same rate (faster just thrashes goals).

Xbox (default — verified live for Xbox Wireless Controller via SDL2):
    shoulder_pan  = axes[2]   (right stick X)
    shoulder_lift = axes[3]   (right stick Y)
    elbow ±       = axes[7]   (D-pad UP/DOWN)
    wrist ±       = axes[6]   (D-pad LEFT/RIGHT)
    gripper close = axes[5]   (LT trigger; rest +1, full pull -1)
    gripper open  = axes[4]   (RT trigger)

DualShock 4 (legacy — override at launch):
    shoulder_pan  = axes[2]
    shoulder_lift = axes[3]
    elbow ±       = buttons[11/12]   (D-pad)
    wrist ±       = buttons[13/14]

Setting an `*_axis` to -1 falls back to the `*_plus`/`*_minus` button pair.

Shoulder gating — both directions supported, mutually exclusive:
  - ``shoulder_enable_button`` (≥ 0): shoulder lives ONLY while held.
  - ``shoulder_disable_button`` (≥ 0): shoulder lives ONLY while NOT held.
Defaults: ``enable=99`` (out of range → shoulder OFF by default),
          ``disable=-1`` (unused when enable gates).
Rationale: an earlier version gated shoulder behind "LB not held" so the
right stick reverted to shoulder duty whenever the operator wasn't
driving. That left the shoulder commandable by stick drift the moment LB
released — Xbox controllers routinely drift past a 0.2 deadzone — so the
arm could swing while the operator pressed an unrelated D-pad button or
sat idle. New default: shoulder demands a positive button hold. Probe a
free button on your controller (Back/Select, stick-click, etc.) and
override ``shoulder_enable_button`` at launch when you want shoulder
control. Wrist/elbow (D-pad axes) and gripper (triggers) stay
ungated — those input devices don't drift.
"""

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState, Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ArmTeleop(Node):
    def __init__(self):
        super().__init__('arm_teleop')

        # Defaults match the Xbox Wireless Controller mapping under SDL2.
        self.declare_parameter('shoulder_pan_axis', 2)
        self.declare_parameter('shoulder_lift_axis', 3)
        self.declare_parameter('elbow_axis', 7)         # D-pad UP/DOWN
        self.declare_parameter('wrist_axis', 6)         # D-pad LEFT/RIGHT
        self.declare_parameter('elbow_plus_button', -1)
        self.declare_parameter('elbow_minus_button', -1)
        self.declare_parameter('wrist_plus_button', -1)
        self.declare_parameter('wrist_minus_button', -1)
        # Shoulder gating (mutually exclusive — see docstring). Default
        # enable=99 (no controller has that many buttons) → shoulder OFF;
        # override at launch with a probed safe button to opt in.
        self.declare_parameter('shoulder_enable_button', 99)
        self.declare_parameter('shoulder_disable_button', -1)
        # Per-joint sign flips so we can flip a stick or D-pad axis without
        # hard-coding it. +1 = upstream sign, -1 = invert.
        self.declare_parameter('shoulder_pan_sign', 1)
        self.declare_parameter('shoulder_lift_sign', 1)
        self.declare_parameter('elbow_sign', 1)
        self.declare_parameter('wrist_sign', 1)
        # Loop tunables — defaults safe for the real Hiwonder steppers (10 Hz,
        # 0.15 rad/tick, 100 ms time_from_start, forced 1 rad/s velocity to
        # bypass the Telemetrix 0.0 error). Sim overrides these in the launch
        # file. The 10 Hz default also matches the vendor controller_manager
        # update_rate — publishing faster just thrashes the JTC.
        self.declare_parameter('update_rate_hz', 10.0)
        self.declare_parameter('step_rad', 0.15)
        self.declare_parameter('time_from_start_s', 0.1)
        self.declare_parameter('joint_velocity', 1.0)
        # Deadzone large enough to ignore stick noise but small enough to
        # respond to a deliberate nudge.
        self.declare_parameter('deadzone', 0.2)
        # Gripper (GripperActionController action goal — URDF limits the
        # gripper_joint to [-0.20, 0.25]). Triggers rest at +1 and pull to
        # -1 on this controller, so pull magnitude is (1 - axis_value)/2.
        self.declare_parameter('gripper_action', '/mirte_master_gripper_controller/gripper_cmd')
        self.declare_parameter('gripper_open_axis', 4)    # RT
        self.declare_parameter('gripper_close_axis', 5)   # LT
        self.declare_parameter('gripper_pos_min', -0.20)
        self.declare_parameter('gripper_pos_max', 0.25)
        self.declare_parameter('gripper_step_rad', 0.03)
        self.declare_parameter('gripper_max_effort', 2.0)
        self.declare_parameter('gripper_initial_pos', 0.0)
        self.declare_parameter('gripper_trigger_threshold', 0.1)

        self._pan_ax = self._iparam('shoulder_pan_axis')
        self._lift_ax = self._iparam('shoulder_lift_axis')
        self._elbow_ax = self._iparam('elbow_axis')
        self._wrist_ax = self._iparam('wrist_axis')
        self._elbow_plus_btn = self._iparam('elbow_plus_button')
        self._elbow_minus_btn = self._iparam('elbow_minus_button')
        self._wrist_plus_btn = self._iparam('wrist_plus_button')
        self._wrist_minus_btn = self._iparam('wrist_minus_button')
        self._shoulder_enable_btn = self._iparam('shoulder_enable_button')
        self._shoulder_disable_btn = self._iparam('shoulder_disable_button')
        self._signs = [
            float(self.get_parameter('shoulder_pan_sign').value),
            float(self.get_parameter('shoulder_lift_sign').value),
            float(self.get_parameter('elbow_sign').value),
            float(self.get_parameter('wrist_sign').value),
        ]
        self._update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self._step = float(self.get_parameter('step_rad').value)
        self._tfs_s = float(self.get_parameter('time_from_start_s').value)
        self._joint_vel = float(self.get_parameter('joint_velocity').value)
        self._deadzone = float(self.get_parameter('deadzone').value)
        self._last_active = [False, False, False, False]
        # Gripper state.
        self._gripper_open_ax = self._iparam('gripper_open_axis')
        self._gripper_close_ax = self._iparam('gripper_close_axis')
        self._gripper_pos_min = float(self.get_parameter('gripper_pos_min').value)
        self._gripper_pos_max = float(self.get_parameter('gripper_pos_max').value)
        self._gripper_step = float(self.get_parameter('gripper_step_rad').value)
        self._gripper_effort = float(self.get_parameter('gripper_max_effort').value)
        self._gripper_thresh = float(self.get_parameter('gripper_trigger_threshold').value)
        self._gripper_pos = float(self.get_parameter('gripper_initial_pos').value)
        self._last_gripper_goal = self._gripper_pos
        # Trigger axes rest at +1 by convention; before the user pulls a
        # trigger they may publish 0 (uninitialised). Treat values close to
        # +1 OR exactly 0 as "rest" until proven otherwise.
        self._trigger_init = {self._gripper_open_ax: False,
                              self._gripper_close_ax: False}
        self._gripper_action_name = self.get_parameter('gripper_action').value
        self._gripper_client = ActionClient(
            self, GripperCommand, self._gripper_action_name)
        self._gripper_pull_open = 0.0
        self._gripper_pull_close = 0.0

        self.publisher_ = self.create_publisher(
            JointTrajectory,
            '/mirte_master_arm_controller/joint_trajectory',
            10)
        self.subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)
        # Diagnostic — confirm whether the JTC actually executed our
        # trajectory by sampling the joint feedback. Logs at most once per
        # second per joint so it's not spammy in steady state.
        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        self._js_logged_ns = {n: 0 for n in [
            'shoulder_pan_joint', 'shoulder_lift_joint',
            'elbow_joint', 'wrist_joint', 'gripper_joint',
        ]}
        self._js_last = {}

        period = 1.0 / max(self._update_rate_hz, 1.0)
        self.timer = self.create_timer(period, self.timer_callback)
        self.get_logger().info(
            f'arm_teleop ready @ {self._update_rate_hz:.0f} Hz, '
            f'step={self._step:.3f} rad, tfs={self._tfs_s*1000:.0f} ms, '
            f'joint_vel={self._joint_vel:.2f}, deadzone={self._deadzone:.2f}'
        )
        self.get_logger().info(
            f'gripper: open_axis={self._gripper_open_ax} (RT), '
            f'close_axis={self._gripper_close_ax} (LT), '
            f'range=[{self._gripper_pos_min:.2f}, {self._gripper_pos_max:.2f}], '
            f'step={self._gripper_step:.3f} rad, '
            f'action={self._gripper_action_name}'
        )
        # Loudly surface the shoulder-gating state — it's the difference
        # between "right stick safely inert" and "right stick drift becomes
        # arm motion." Operators reading the journal should never have to
        # guess.
        shoulder_status = (
            f'DISABLED (shoulder_enable_button={self._shoulder_enable_btn} '
            f'is out of range)'
            if self._shoulder_enable_btn >= 16 or (
                self._shoulder_enable_btn < 0
                and self._shoulder_disable_btn < 0
            )
            else f'enable_btn={self._shoulder_enable_btn} '
                 f'disable_btn={self._shoulder_disable_btn}'
        )
        self.get_logger().info(f'shoulder pan/lift: {shoulder_status}')

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
        enable_held = (
            self._shoulder_enable_btn < 0
            or (self._shoulder_enable_btn < len(msg.buttons)
                and msg.buttons[self._shoulder_enable_btn] == 1)
        )
        disable_held = (
            0 <= self._shoulder_disable_btn < len(msg.buttons)
            and msg.buttons[self._shoulder_disable_btn] == 1
        )
        shoulder_live = enable_held and not disable_held
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

        # Gripper triggers — pull strength in [0, 1].
        self._gripper_pull_open = self._trigger_pull(msg, self._gripper_open_ax)
        self._gripper_pull_close = self._trigger_pull(msg, self._gripper_close_ax)

    def _on_joint_state(self, msg: JointState) -> None:
        """Log when a tracked joint's actual position changes by ≥ 0.02 rad
        since the last logged value. Lets us see whether the JTC is actually
        executing what arm_teleop commanded — separates 'didn't command'
        from 'commanded but joint didn't move'."""
        now_ns = self.get_clock().now().nanoseconds
        for name, pos in zip(msg.name, msg.position):
            if name not in self._js_logged_ns:
                continue
            last = self._js_last.get(name)
            if last is not None and abs(pos - last) < 0.02:
                continue
            # Rate-limit per joint so steady continuous motion still logs but
            # at most once / 0.5 s.
            if (now_ns - self._js_logged_ns[name]) < 500_000_000:
                continue
            short = name.replace('_joint', '')
            cmd = self.current_positions[
                ['shoulder_pan_joint', 'shoulder_lift_joint',
                 'elbow_joint', 'wrist_joint'].index(name)
            ] if name in (
                'shoulder_pan_joint', 'shoulder_lift_joint',
                'elbow_joint', 'wrist_joint',
            ) else self._gripper_pos
            self.get_logger().info(
                f'js {short}: actual={pos:+.3f}  commanded={cmd:+.3f}  '
                f'Δ={pos - cmd:+.3f}'
            )
            self._js_last[name] = pos
            self._js_logged_ns[name] = now_ns

    def _trigger_pull(self, msg: Joy, idx: int) -> float:
        """Convert an Xbox-style trigger axis (rest +1, full pull -1) to a
        pull magnitude in [0, 1]. Returns 0 until the axis has produced a
        clearly-pulled value at least once — ignores the 0.0 reading some
        drivers send before the first physical event."""
        if idx < 0 or idx >= len(msg.axes):
            return 0.0
        v = float(msg.axes[idx])
        if not self._trigger_init.get(idx, False):
            # Initialised once we see a value clearly different from the 0
            # reading uninitialised joy_node sometimes emits.
            if v > 0.5 or v < -0.1:
                self._trigger_init[idx] = True
            else:
                return 0.0
        pull = (1.0 - v) / 2.0
        return max(0.0, min(1.0, pull))

    def timer_callback(self) -> None:
        moved = False
        active = [False, False, False, False]
        for i in range(4):
            if abs(self.joy_cmds[i]) > self._deadzone:
                self.current_positions[i] += self.joy_cmds[i] * self._signs[i] * self._step
                # URDF clamp — keep the joint inside ±π/2 so JTC doesn't
                # silently refuse the trajectory point.
                self.current_positions[i] = max(-1.5707, min(1.5707, self.current_positions[i]))
                moved = True
                active[i] = True
        # Edge-triggered debug: log when the set of moving joints changes.
        # Quiet during steady-state holds, loud when something starts/stops.
        if active != self._last_active:
            labels = ['pan', 'lift', 'elbow', 'wrist']
            now = [labels[i] for i in range(4) if active[i]] or ['(none)']
            self.get_logger().info(f'arm joints active: {",".join(now)}')
            self._last_active = active
        if moved:
            self.send_trajectory()
        self._tick_gripper()

    def _tick_gripper(self) -> None:
        net = self._gripper_pull_open - self._gripper_pull_close
        if abs(net) < self._gripper_thresh:
            return
        new_pos = self._gripper_pos + net * self._gripper_step
        new_pos = max(self._gripper_pos_min, min(self._gripper_pos_max, new_pos))
        if abs(new_pos - self._last_gripper_goal) < 1e-3:
            self._gripper_pos = new_pos
            return
        if not self._gripper_client.server_is_ready():
            # Don't block — controller may not be up yet on cold start.
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(new_pos)
        goal.command.max_effort = self._gripper_effort
        # Fire-and-forget — the action server preempts in-flight goals,
        # so we don't track futures or await results.
        self._gripper_client.send_goal_async(goal)
        self._gripper_pos = new_pos
        self._last_gripper_goal = new_pos

    def send_trajectory(self) -> None:
        traj = JointTrajectory()
        traj.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = list(self.current_positions)
        # joint_velocity > 0 bypasses the Telemetrix 0.0 error on real hardware.
        # In sim it's the JTC's velocity hint along the trajectory.
        point.velocities = [self._joint_vel] * 4
        tfs_ns = int(self._tfs_s * 1e9)
        point.time_from_start.sec = tfs_ns // 1_000_000_000
        point.time_from_start.nanosec = tfs_ns % 1_000_000_000

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
