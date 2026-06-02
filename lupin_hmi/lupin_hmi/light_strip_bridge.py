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

from std_srvs.srv import Trigger

from lupin_msgs.msg import MissionState
from mirte_msgs.msg import NeopixelColor
from mirte_msgs.srv import SetNeopixel


RGB = Tuple[int, int, int]

# ── Status palette ──────────────────────────────────────────────────────
# One whole-strip colour per mission state, chosen so an operator can read
# the robot's high-level activity from across the room. Keep these in sync
# with the manual presets in the HMI (lupin_web LightControl widget).
OFF = (0, 0, 0)
RED = (255, 0, 0)        # e-stop / FAULT — danger; overrides everything
AMBER = (255, 191, 0)    # paused — mission held
WHITE = (255, 255, 255)  # boot / data-publish flash
BLUE = (0, 0, 255)       # ready — idle, awaiting a mission
YELLOW = (255, 255, 0)   # prepare — localizing before a run
SPRING = (0, 255, 128)   # exploring — frontier search for tags
CYAN = (0, 255, 255)     # inspecting — driving the known tag sequence
TEAL = (0, 180, 140)     # monitoring — continuous re-scan loop
PURPLE = (180, 0, 255)   # scanning / confirming a tag (the read moment)
ORANGE = (255, 128, 0)   # returning to dock
GREEN = (0, 255, 0)      # mission done


class LightStripBridge(Node):
    """
    Bridge MissionState -> MIRTE neopixel strip, with a manual override.

    In ``auto`` mode (default) it subscribes to the latched mission-state
    snapshot and maps the current lifecycle/phase/safety flags to one
    whole-strip colour via the MIRTE SetNeopixel service.

    An operator can take over from the HMI: ``/lupin/leds/set`` (SetNeopixel)
    holds a manual colour and stops the strip following mission state;
    ``/lupin/leds/auto`` (Trigger) hands control back to the state machine.
    Safety states (e-stop, FAULT) always show red — they win even over a
    manual hold — so the indicator can never mask a stopped robot.
    """

    def __init__(self) -> None:
        super().__init__('light_strip_bridge')

        self.declare_parameter('mission_state_topic', '/mission/state')
        self.declare_parameter('led_service', '/io/leds/leds/set_color')
        self.declare_parameter('manual_service', '/lupin/leds/set')
        self.declare_parameter('auto_service', '/lupin/leds/auto')
        self.declare_parameter('set_on_startup', True)
        self.declare_parameter('unknown_state_off', True)

        self._mission_state_topic = str(
            self.get_parameter('mission_state_topic').value
        )
        self._led_service = str(
            self.get_parameter('led_service').value
        )
        self._manual_service = str(
            self.get_parameter('manual_service').value
        )
        self._auto_service = str(
            self.get_parameter('auto_service').value
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

        # Manual override: 'auto' follows MissionState, 'manual' holds a colour
        # the operator chose. _last_msg caches the latest state so we can
        # re-derive the auto colour the instant control is handed back.
        self._mode = 'auto'
        self._manual_rgb: Optional[RGB] = None
        self._last_msg: Optional[MissionState] = None

        self._client = self.create_client(SetNeopixel, self._led_service)

        # Operator surface for the HMI light button.
        self._manual_srv = self.create_service(
            SetNeopixel, self._manual_service, self._on_set_manual
        )
        self._auto_srv = self.create_service(
            Trigger, self._auto_service, self._on_set_auto
        )

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
            f'calling {self._led_service}; manual override via '
            f'{self._manual_service} / {self._auto_service}'
        )

        if self._set_on_startup:
            self._send_color(OFF, reason='startup')

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
        self._last_msg = msg
        rgb = self._desired_rgb(msg)
        # Skip if already on the strip (_last_rgb) or already in flight
        # (_pending_rgb) — the latter stops a 5 Hz state tick re-dispatching a
        # colour an async send is still resolving.
        if rgb == self._last_rgb or rgb == self._pending_rgb:
            return
        self._send_color(rgb, reason=self._describe_state(msg))

    def _desired_rgb(self, msg: MissionState) -> RGB:
        """Colour to show right now, accounting for a manual hold.

        Safety states win unconditionally: a stopped or faulted robot is red
        even while the operator is holding a manual colour, so the strip can
        never imply the robot is fine when it isn't.
        """
        safety = self._safety_rgb(msg)
        if safety is not None:
            return safety
        if self._mode == 'manual' and self._manual_rgb is not None:
            return self._manual_rgb
        return self._state_to_rgb(msg)

    @staticmethod
    def _safety_rgb(msg: MissionState) -> Optional[RGB]:
        """Red for the two states that must never be masked, else None."""
        if msg.estop_engaged:
            return RED
        if (msg.lifecycle_state or '').upper() == 'FAULT':
            return RED
        return None

    def _state_to_rgb(self, msg: MissionState) -> RGB:
<<<<<<< HEAD
        
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

=======
        """Pure mission-state -> colour map for auto mode.

        e-stop/FAULT are handled upstream in _desired_rgb; this stays the
        state-machine view so every lifecycle in MissionState.msg maps to a
        distinct, legible colour.
        """
        lifecycle = (msg.lifecycle_state or '').upper()
        phase = (msg.mission_phase or '').upper()

        if msg.paused:
            return AMBER

        if lifecycle == 'BOOT':
            return WHITE
        if lifecycle == 'READY':
            return BLUE
>>>>>>> 170769fcb63f18e2508ee4d3ae6f1ea10aef135e
        if lifecycle == 'PREPARE':
            return YELLOW
        if lifecycle == 'EXPLORING':
            return SPRING
        if lifecycle == 'INSPECTING':
            return self._scan_phase_rgb(phase, base=CYAN)
        if lifecycle == 'MONITORING':
            return self._scan_phase_rgb(phase, base=TEAL)
        if lifecycle == 'RETURNING':
            return ORANGE
        if lifecycle == 'DONE':
<<<<<<< HEAD
            # GREEN
            return (0, 0, 255)
=======
            return GREEN
        if lifecycle == 'FAULT':
            return RED
>>>>>>> 170769fcb63f18e2508ee4d3ae6f1ea10aef135e

        return OFF if self._unknown_state_off else WHITE

    @staticmethod
    def _scan_phase_rgb(phase: str, base: RGB) -> RGB:
        """Shared INSPECTING/MONITORING sub-phase colouring.

        Both lifecycles share the NAVIGATING/SCANNING/PUBLISHING sub-machine
        (and the CONFIRMING hardware-confirm hook). The tag read is the
        operationally interesting instant, so highlight SCANNING/CONFIRMING in
        purple and the brief PUBLISHING in white; everything else keeps the
        per-lifecycle base colour so INSPECTING (cyan) and MONITORING (teal)
        stay distinguishable.
        """
        if phase in ('SCANNING', 'CONFIRMING'):
            return PURPLE
        if phase == 'PUBLISHING':
            return WHITE
        return base

    def _describe_state(self, msg: MissionState) -> str:
        lifecycle = msg.lifecycle_state or ''
        phase = msg.mission_phase or ''
        paused = ' paused' if msg.paused else ''
        estop = ' estop' if msg.estop_engaged else ''
        mode = '' if self._mode == 'auto' else ' [manual]'
        return f'{lifecycle}/{phase}{paused}{estop}{mode}'.strip()

    def _on_set_manual(
        self, request: SetNeopixel.Request, response: SetNeopixel.Response
    ) -> SetNeopixel.Response:
        """HMI -> hold a manual colour; stop following mission state.

        Honoured immediately, but an active e-stop / FAULT still forces red so
        the operator can't accidentally paint over a stopped robot.
        """
        rgb = (
            int(request.color.r),
            int(request.color.g),
            int(request.color.b),
        )

        # Don't claim success the operator can't see: if the MIRTE LED service
        # is down the colour goes nowhere, so reject rather than enter a manual
        # hold the HMI would render as applied.
        if not self._client.service_is_ready():
            self.get_logger().warn(
                f'manual colour {rgb} rejected: LED service '
                f'{self._led_service} not ready'
            )
            response.status = False
            return response

        self._mode = 'manual'
        self._manual_rgb = rgb

        safety = self._safety_rgb(self._last_msg) if self._last_msg else None
        target = safety if safety is not None else rgb
        self._send_color(target, reason=f'manual {rgb}')

        if safety is not None:
            self.get_logger().warn(
                f'manual colour {rgb} held but overridden by safety state '
                f'{self._describe_state(self._last_msg)}'
            )
        response.status = True
        return response

    def _on_set_auto(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """HMI -> hand colouring back to the mission state machine."""
        self._mode = 'auto'
        self._manual_rgb = None
        if self._last_msg is not None:
            self._send_color(
                self._desired_rgb(self._last_msg),
                reason=self._describe_state(self._last_msg),
            )
            response.message = (
                f'auto: {self._describe_state(self._last_msg)}'
            )
        else:
            response.message = 'auto: awaiting /mission/state'
        response.success = True
        return response

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
        # call_async completions can resolve out of order, so only the most
        # recently dispatched colour (_pending_rgb) is allowed to update the
        # dedup cache. Clear the pending marker the moment it resolves — pass
        # or fail — so a failed send is retried on the next state tick rather
        # than being suppressed by a stale _pending_rgb.
        is_latest = rgb == self._pending_rgb
        if is_latest:
            self._pending_rgb = None

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

        if not is_latest:
            # A newer colour was dispatched after this one; ignore the stale
            # completion so _last_rgb keeps tracking what's actually shown.
            return

        self._last_rgb = rgb
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
