"""cmd_vel_mux — multiplex manual + autonomous velocity commands.

Subscribes to /cmd_vel_manual (operator joystick / web teleop) and
/cmd_vel_auto (Nav2 velocity_smoother output) and publishes to a
configurable downstream topic. Manual takes priority for a short window
after each manual message so a human can take over mid-mission without
fighting Nav2 on the same command stream.

Topology (sim):

    Nav2 controller_server → /cmd_vel_nav
    velocity_smoother      → /cmd_vel_auto  ┐
                                            ├─► cmd_vel_mux ──► /mirte_base_controller/cmd_vel_unstamped
    web/joystick           → /cmd_vel_manual┘                       │
                                                                    ▼ (vendor twist_mux at priority 200)
                                                        /cmd_vel  → gazebo_planar_move (P3D) → robot
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist


class CmdVelMux(Node):
    def __init__(self):
        super().__init__('cmd_vel_mux')

        # Real Mirte firmware listens on /mirte_base_controller/cmd_vel; sim's
        # vendor twist_mux listens on /mirte_base_controller/cmd_vel_unstamped.
        # Override via the `cmd_vel_topic` parameter from the launch.
        self.declare_parameter('cmd_vel_topic', '/mirte_base_controller/cmd_vel')
        # How long after the last manual message Nav2 commands are blocked.
        # Short enough that pausing the joystick releases control quickly,
        # long enough to span the gap between a 10 Hz joystick's frames.
        self.declare_parameter('manual_takeover_seconds', 0.5)

        out_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        self._takeover_ns = int(
            self.get_parameter('manual_takeover_seconds').get_parameter_value().double_value
            * 1e9
        )
        self.get_logger().info(
            f'cmd_vel_mux publishing on {out_topic} '
            f'(manual takeover {self._takeover_ns / 1e9:.2f} s)'
        )

        # Both the sim's twist_mux and the real mecanum controller subscribe
        # BEST_EFFORT. A RELIABLE publisher here silently drops messages on
        # the QoS mismatch — symptom is wheels idle even though we publish.
        cmd_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._pub = self.create_publisher(Twist, out_topic, cmd_qos)

        self._last_manual_ns = 0
        self.create_subscription(Twist, '/cmd_vel_manual', self._on_manual, 10)
        self.create_subscription(Twist, '/cmd_vel_auto', self._on_auto, 10)

    def _on_manual(self, msg: Twist) -> None:
        self._last_manual_ns = self.get_clock().now().nanoseconds
        self._pub.publish(msg)

    def _on_auto(self, msg: Twist) -> None:
        elapsed = self.get_clock().now().nanoseconds - self._last_manual_ns
        if elapsed < self._takeover_ns:
            return
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelMux()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
