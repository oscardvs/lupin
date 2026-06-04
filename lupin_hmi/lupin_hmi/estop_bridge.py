"""estop_bridge — republish the MIRTE emergency button as the orchestrator e-stop.

The mission orchestrator (and, through MissionState.estop_engaged, the LED
safety override) consume ``std_msgs/Bool`` on ``/e_stop_state``. The real MIRTE
hardware publishes the physical emergency button on
``/io/intensity/emergency_button/digital`` (``mirte_msgs/IntensityDigital``,
a bool ``value``). Nothing connected the two, so pressing the button never froze
the autonomous mission or cancelled the in-flight Nav2 goal. This bridge closes
that gap so the physical e-stop actually reaches the FSM.

The HMI software-STOP publishes the same ``/e_stop_state`` Bool directly, so
both the physical button and the HMI button now drive the orchestrator e-stop.

Polarity (``engaged_value``) is a parameter and is logged loudly at startup
because it MUST be verified on the physical button: press it and confirm
``/e_stop_state`` goes ``true``. We republish the current state on a steady
tick so a late-joining (VOLATILE) orchestrator always converges — EStopMonitor
dedups on edges, so the repeats are harmless.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Bool

from mirte_msgs.msg import IntensityDigital


def button_is_engaged(value: bool, engaged_value: bool) -> bool:
    """Map a raw button `value` to e-stop-engaged under the configured polarity.

    Pure + ROS-free so the polarity contract is unit-testable. ``engaged_value``
    is which raw value means "pressed"; it MUST be verified on the real button.
    """
    return bool(value) == bool(engaged_value)


class EStopBridge(Node):
    def __init__(self) -> None:
        super().__init__('estop_bridge')

        self.declare_parameter(
            'button_topic', '/io/intensity/emergency_button/digital'
        )
        self.declare_parameter('estop_topic', '/e_stop_state')
        # Which raw button `value` means "pressed / e-stop engaged". MUST be
        # verified on the physical button — flip this (launch param) if pressing
        # the button does not drive /e_stop_state true.
        self.declare_parameter('engaged_value', True)
        self.declare_parameter('publish_rate_hz', 5.0)

        self._engaged_value = bool(self.get_parameter('engaged_value').value)
        button_topic = str(self.get_parameter('button_topic').value)
        estop_topic = str(self.get_parameter('estop_topic').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        # Match EStopMonitor's subscriber QoS (RELIABLE, VOLATILE, depth 10).
        # We converge late subscribers via the steady republish below rather
        # than a latched publisher.
        pub_qos = QoSProfile(
            depth=10,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._pub = self.create_publisher(Bool, estop_topic, pub_qos)

        # Latest known engaged state. False until the button says otherwise so
        # a silent (idle) button means "not engaged".
        self._engaged = False
        self._sub = self.create_subscription(
            IntensityDigital, button_topic, self._on_button, 10
        )
        self._timer = self.create_timer(1.0 / max(rate, 0.5), self._tick)

        self.get_logger().warn(
            f'estop_bridge: {button_topic} -> {estop_topic}; '
            f'engaged when value=={self._engaged_value}. '
            'VERIFY on hardware: press the button and confirm /e_stop_state=true.'
        )

    def _on_button(self, msg: IntensityDigital) -> None:
        engaged = button_is_engaged(msg.value, self._engaged_value)
        if engaged != self._engaged:
            self._engaged = engaged
            self.get_logger().info(
                f'e-stop {"ENGAGED" if engaged else "released"} (button)'
            )
            self._publish()

    def _tick(self) -> None:
        # Steady republish so a late/again-subscribing orchestrator converges.
        self._publish()

    def _publish(self) -> None:
        msg = Bool()
        msg.data = self._engaged
        self._pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = EStopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
