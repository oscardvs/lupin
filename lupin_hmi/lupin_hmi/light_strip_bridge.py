from __future__ import annotations

from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from lupin_msgs.msg import MissionState
from mirte_msgs.msg import NeopixelColor
from mirte_msgs.srv import SetNeopixel


RGB = Tuple[int, int, int]


class LightStripBridge(Node):
    """
    Bridge MissionState -> MIRTE neopixel strip.

    Subscribes to the latched mission-state snapshot and maps the current
    lifecycle/phase/safety flags to one whole-strip color via the MIRTE
    SetNeopixel service.
    """

    def __init__(self) -> None:
        super().__init__('light_strip_bridge')

        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('led_service', '/io/leds/leds/set_color')
        self.declare_parameter('set_on_startup', True)
        self.declare_parameter('unknown_state_off', True)

        self._mission_state_topic = str(
            self.get_parameter('mission_state_topic').value
        )
        self._led_service = str(
            self.get_parameter('led_service').value
        )
        self._set_on_startup = bool(
            self.get_parameter('set_on_startup').value
        )
        self._unknown_state_off = bool(
            self.get_parameter('unknown_state_off').value
        )

        self._last_rgb: Optional[RGB] = None
        self._pending_rgb: Optional[RGB] = None
        self._service_ready_logged = False

        self._client = self.create_client(SetNeopixel, self._led_service)

        state_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._sub = self.create_subscription(
            MissionState,
            self._mission_state_topic,
            self._on_state,
            state_qos,
        )

        self._wait_timer = self.create_timer(1.0, self._poll_service_ready)

        self.get_logger().info(
            f'light_strip_bridge listening on {self._mission_state_topic}, '
            f'calling {self._led_service}'
        )

        if self._set_on_startup:
            self._send_color((0, 0, 0), reason='startup')

    def _poll_service_ready(self) -> None:
        if self._client.service_is_ready():
            if not self._service_ready_logged:
                self.get_logger().info(
                    f'LED service ready: {self._led_service}'
                )
                self._service_ready_logged = True
            return

        self.get_logger().warn(
            f'Waiting for LED service {self._led_service}...',
            throttle_duration_sec=5.0,
        )

    def _on_state(self, msg: MissionState) -> None:
        rgb = self._state_to_rgb(msg)
        if rgb == self._last_rgb:
            return
        self._send_color(rgb, reason=self._describe_state(msg))

    def _state_to_rgb(self, msg: MissionState) -> RGB:
        
        # BRG order is what the current strip expects based on the telemetrix config
        # Tried changing it in mirte_user_config.yaml on the robot but it didn't seem to have any effect, so hardcoding it here for now.

        lifecycle = (msg.lifecycle_state or '').upper()
        phase = (msg.mission_phase or '').upper()

        if msg.estop_engaged:
            # RED
            return (0, 255, 0)

        # TODO: add battery_low condition
        if msg.battery_low :
            # YELLOW
            return (0, 255, 150)

        if msg.paused:
            # YELLOW
            return (0, 255, 150)

        if lifecycle == 'FAULT':
            # RED
            return (0, 255, 0)

        if lifecycle == 'BOOT':
            # WHITE
            return (255, 255, 255)

        if lifecycle == 'READY':
            # GREEN
            return (0, 0, 255)

        if lifecycle == 'PREPARE':
            if phase == 'LOCALIZING':
                return (255, 255, 0)
            return (255, 255, 0)

        if lifecycle == 'INSPECTING':
            if phase == 'NAVIGATING':
                return (0, 255, 255)
            if phase == 'SCANNING':
                return (180, 0, 255)
            if phase == 'CONFIRMING':
                return (255, 255, 255)
            if phase == 'PUBLISHING':
                return (255, 255, 255)
            return (0, 255, 255)

        if lifecycle == 'RETURNING':
            return (255, 128, 0)

        if lifecycle == 'DONE':
            # GREEN
            return (0, 0, 255)

        if self._unknown_state_off:
            return (0, 0, 0)

        return (255, 255, 255)

    def _describe_state(self, msg: MissionState) -> str:
        lifecycle = msg.lifecycle_state or ''
        phase = msg.mission_phase or ''
        paused = ' paused' if msg.paused else ''
        estop = ' estop' if msg.estop_engaged else ''
        return f'{lifecycle}/{phase}{paused}{estop}'.strip()

    def _send_color(self, rgb: RGB, reason: str = '') -> None:
        if not self._client.service_is_ready():
            self.get_logger().warn(
                f'LED service not ready, skipping color {rgb}',
                throttle_duration_sec=5.0,
            )
            return

        req = SetNeopixel.Request()
        req.color = NeopixelColor(
            r=int(rgb[0]),
            g=int(rgb[1]),
            b=int(rgb[2]),
        )

        self._pending_rgb = rgb
        future = self._client.call_async(req)
        future.add_done_callback(
            lambda fut, target_rgb=rgb, why=reason: self._on_set_color_done(
                fut, target_rgb, why
            )
        )

    def _on_set_color_done(self, future, rgb: RGB, reason: str) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f'LED service call failed for {rgb}: {exc}'
            )
            return

        if not response.status:
            self.get_logger().warn(
                f'LED service returned status=False for {rgb}'
            )
            return

        self._last_rgb = rgb
        self._pending_rgb = None
        self.get_logger().info(
            f'LED color set to {rgb} ({reason})',
            throttle_duration_sec=1.0,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LightStripBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()