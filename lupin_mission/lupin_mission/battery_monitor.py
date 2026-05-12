"""Cross-cutting battery subscriber.

Mirrors /io/power/power_watcher (sensor_msgs/BatteryState) onto flags
the orchestrator can poll, and fires callbacks on the low/recovered
edges — exactly the same pattern as EStopMonitor.

Trigger condition:
  - percentage < low_threshold  (default 0.20)

Convention: once low is triggered, it stays latched until the battery
recovers above the threshold. The orchestrator stays paused after
recovery — operator must call /mission/resume, same as E-stop.
"""

from __future__ import annotations

from typing import Callable, Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from sensor_msgs.msg import BatteryState


class BatteryMonitor:
    """Subscribes to /io/power/power_watcher and reports low/recovered edges."""

    def __init__(
        self,
        node: Node,
        battery_topic: str,
        *,
        low_threshold: float = 0.20,
        on_low: Optional[Callable[[], None]] = None,
        on_recovered: Optional[Callable[[], None]] = None,
        callback_group: Optional[CallbackGroup] = None,
    ):
        self._node = node
        self._low_threshold = low_threshold
        self._on_low = on_low
        self._on_recovered = on_recovered

        self._low: bool = False
        self._percentage: float = 1.0

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._bat_sub = node.create_subscription(
            BatteryState, battery_topic, self._on_battery_msg, qos,
            callback_group=callback_group,
        )

    @property
    def low(self) -> bool:
        return self._low

    @property
    def percentage(self) -> float:
        return self._percentage

    def _evaluate(self) -> None:
        """Re-evaluate low condition and fire edge callbacks."""
        is_low = self._percentage < self._low_threshold
        if is_low == self._low:
            return
        self._low = is_low
        if is_low:
            self._node.get_logger().warn(
                f'Battery LOW: {self._percentage:.1%} remaining. '
                f'Triggering dock.'
            )
            if self._on_low is not None:
                self._on_low()
        else:
            self._node.get_logger().info(
                'Battery recovered above threshold; awaiting /mission/resume.'
            )
            if self._on_recovered is not None:
                self._on_recovered()

    def _on_battery_msg(self, msg: BatteryState) -> None:
        self._percentage = float(msg.percentage)
        self._evaluate()