"""Cross-cutting E-stop subscriber.

Mirrors `/e_stop_state` (std_msgs/Bool, true = engaged) onto a flag the
orchestrator can poll, and fires callbacks on the rising and falling edges
so the node can cancel goals / log transitions exactly once per edge.

On hardware the physical emergency button reaches this topic via
`lupin_hmi/estop_bridge` (mirte_msgs/IntensityDigital → std_msgs/Bool); the HMI
software-STOP (`lupin_web/web/src/lib/estop.tsx`) publishes the same Bool. Both
paths drive this one flag. Tests fake it by publishing on the same topic.
"""

from __future__ import annotations

from typing import Callable, Optional

from rclpy.callback_groups import CallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool


class EStopMonitor:
    """Subscribes to /e_stop_state and reports edges to the orchestrator."""

    def __init__(
        self,
        node: Node,
        topic: str,
        *,
        on_engaged: Optional[Callable[[], None]] = None,
        on_released: Optional[Callable[[], None]] = None,
        callback_group: Optional[CallbackGroup] = None,
    ):
        self._node = node
        self._topic = topic
        self._on_engaged = on_engaged
        self._on_released = on_released

        self._engaged: bool = False

        # The e-stop publisher in lupin_web uses default rclpy QoS (RELIABLE,
        # VOLATILE, depth 10). Mirror that. If we ever switch to a latched
        # publisher, flip durability to TRANSIENT_LOCAL here too — otherwise
        # late subscribers will miss the initial "released" message.
        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._sub = node.create_subscription(
            Bool, topic, self._on_msg, qos, callback_group=callback_group
        )

    @property
    def engaged(self) -> bool:
        return self._engaged

    def _on_msg(self, msg: Bool) -> None:
        new = bool(msg.data)
        if new == self._engaged:
            return
        self._engaged = new
        if new:
            self._node.get_logger().warn(
                f"E-stop ENGAGED via {self._topic}; preempting active state."
            )
            if self._on_engaged is not None:
                self._on_engaged()
        else:
            self._node.get_logger().info(
                f"E-stop released via {self._topic}; awaiting /mission/resume."
            )
            if self._on_released is not None:
                self._on_released()
