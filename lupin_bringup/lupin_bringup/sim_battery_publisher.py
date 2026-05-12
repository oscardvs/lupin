"""Simulated battery publisher for the sim chain.

The greenhouse world has no real battery hardware, so nothing publishes
to /io/power/power_watcher in simulation. Nav2's IsBatteryLow BT
condition and the mission orchestrator's BatteryMonitor subscribe there
and need a valid sensor_msgs/BatteryState stream to function.

Publishes a linearly draining BatteryState on /io/power/power_watcher.
Hardware doesn't need it — the real MIRTE power watcher publishes on
the same topic natively.

Note on Nav2 IsBatteryLow: that BT condition defaults to
``is_voltage: true`` and reads ``BatteryState.voltage``. The condition
will not behave correctly if it inspects the percentage and voltage is
NaN, so this node also populates a plausible mock voltage tracking the
charge percentage. Set ``is_voltage: false`` on the BT condition if
you'd rather gate purely on percentage.

Run via ``ros2 run lupin_bringup sim_battery_publisher``.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState


# Maps charge percentage [0..1] linearly to a Li-ion-ish voltage band so
# Nav2's IsBatteryLow (with the default is_voltage=true) gets a usable
# signal. 11.0 V at empty, 12.0 V at full — covers the typical 11.4 V
# low-battery threshold at ~40% charge.
VOLTAGE_EMPTY_V = 11.0
VOLTAGE_FULL_V = 12.0


class SimBatteryPublisher(Node):
    def __init__(self):
        super().__init__('sim_battery_publisher')
        self.declare_parameter('initial_charge', 1.0)   # 0.0–1.0
        self.declare_parameter('drain_rate_per_sec', 0.001)
        self.declare_parameter('publish_rate_hz', 1.0)

        self._charge = float(self.get_parameter('initial_charge').value)
        drain_per_sec = float(self.get_parameter('drain_rate_per_sec').value)
        rate = float(self.get_parameter('publish_rate_hz').value)
        # Scale the per-second drain to per-tick so bumping publish_rate_hz
        # doesn't silently 10× the actual drain (and 10× over-report the
        # remaining-time signals downstream).
        self._drain_per_tick = drain_per_sec / rate if rate > 0.0 else drain_per_sec

        self._bat_pub = self.create_publisher(
            BatteryState, '/io/power/power_watcher', 10)

        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'SimBatteryPublisher started: initial={self._charge:.2f}, '
            f'drain={drain_per_sec}/s @ {rate} Hz '
            f'(= {self._drain_per_tick}/tick)')

    def _tick(self):
        self._charge = max(0.0, self._charge - self._drain_per_tick)

        bat = BatteryState()
        bat.header.stamp = self.get_clock().now().to_msg()
        bat.header.frame_id = 'base_link'
        bat.percentage = self._charge
        bat.voltage = (
            VOLTAGE_EMPTY_V
            + (VOLTAGE_FULL_V - VOLTAGE_EMPTY_V) * self._charge
        )
        bat.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        bat.present = True
        self._bat_pub.publish(bat)


def main(args=None):
    rclpy.init(args=args)
    node = SimBatteryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
