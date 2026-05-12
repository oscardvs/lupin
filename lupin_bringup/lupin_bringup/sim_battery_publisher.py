"""Simulated battery publisher for the sim chain.

The greenhouse world has no real battery hardware, so nothing publishes
to /io/power/power_watcher in simulation. The v2 mission orchestrator's
BatteryMonitor (and Nav2's IsBatteryLow BT condition) subscribe to this
topic and need a valid sensor_msgs/BatteryState stream to function.

This node publishes a linearly draining BatteryState on
/io/power/power_watcher and a Float32 estimated remaining time on
/battery/time_remaining at a configurable rate.

Hardware doesn't need it — the real MIRTE power watcher publishes on
the same topic natively.

Run via ``ros2 run lupin_bringup sim_battery_publisher``.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32


class SimBatteryPublisher(Node):
    def __init__(self):
        super().__init__('sim_battery_publisher')
        self.declare_parameter('initial_charge', 1.0)   # 0.0–1.0
        self.declare_parameter('drain_rate_per_sec', 0.001)  # tune for testing
        self.declare_parameter('publish_rate_hz', 1.0)

        self._charge = float(self.get_parameter('initial_charge').value)
        self._drain = float(self.get_parameter('drain_rate_per_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)

        self._bat_pub = self.create_publisher(
            BatteryState, '/io/power/power_watcher', 10)
        self._time_pub = self.create_publisher(
            Float32, '/battery/time_remaining', 10)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'SimBatteryPublisher started: initial={self._charge:.2f}, '
            f'drain={self._drain}/s')

    def _tick(self):
        self._charge = max(0.0, self._charge - self._drain)

        bat = BatteryState()
        bat.header.stamp = self.get_clock().now().to_msg()
        bat.percentage = self._charge
        bat.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        bat.present = True
        self._bat_pub.publish(bat)

        time_remaining = Float32()
        time_remaining.data = (
            self._charge / self._drain if self._drain > 0.0 else float('inf')
        )
        self._time_pub.publish(time_remaining)


def main(args=None):
    rclpy.init(args=args)
    node = SimBatteryPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()