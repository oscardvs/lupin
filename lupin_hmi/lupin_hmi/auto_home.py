"""auto_home — call /lupin/arm/preset {name: home} once on boot.

Why this exists
---------------
The Hiwonder shoulder_lift servo on Mirte-247264 thermally trips when held
against gravity for extended periods. Booting into a low-load pose every
time prevents the trip from accumulating wear and avoids the "robot is up
but lift servo is dead" failure mode at the start of a demo.

The 'home' preset values are measured from the operator's chosen rest
pose (see PRESETS in arm_preset_server.py); they put shoulder_lift near
the URDF zero, which on this kinematic chain is the position of minimum
gravity moment that we've validated.

Lifecycle
---------
Runs as a one-shot from ``lupin-auto-home.service``, ordered after
``lupin-onboard.service``. Procedure:

  1. Wait for ``/joint_states`` to publish all 4 arm joints (= arm
     controller is loaded and reading hardware).
  2. Wait for ``/lupin/arm/preset`` service to exist (= arm_preset_server
     is up inside lupin-onboard).
  3. Brief 2s settle so the JTC has its open-loop reference written.
  4. Call the preset service with name='home'.
  5. Exit. The systemd unit is RemainAfterExit=yes so it doesn't loop.

Opt-out
-------
Set ``LUPIN_AUTO_HOME=false`` in ``~/.mirte_settings.sh`` (or any
environment file the lupin-auto-home service inherits). On boot the
node logs that auto-home is disabled and exits 0 without touching
the arm.

Failure modes
-------------
- Timeout waiting for ``/joint_states`` → arm controller never came up.
  Exits non-zero; doesn't block boot. Run ``ros2 control list_controllers``
  to diagnose.
- Timeout waiting for ``/lupin/arm/preset`` → arm_preset_server didn't
  start. lupin-onboard probably failed; check ``journalctl -u lupin-onboard``.
- Service returns ``success=false`` → preset name is wrong (we hardcode
  'home' so this shouldn't happen unless someone edits the script).
- The arm fails to physically reach the pose (e.g. shoulder_lift is in
  a thermal-trip lock from a previous session) → JTC may report success
  even though the joint is stuck. auto_home doesn't verify the final
  pose; that's the operator's job on the first sanity-check after boot.
"""

from __future__ import annotations

import os
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from lupin_msgs.srv import SetArmPreset


REQUIRED_ARM_JOINTS = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_joint',
)
# Generous timeouts — the arm controllers can take a while to load on a
# cold boot (telemetrix + spawners + hardware init). 30s covers the 95th
# percentile we've measured on Mirte-247264.
JOINT_STATE_TIMEOUT_S = 30.0
SERVICE_TIMEOUT_S = 30.0
# Time we wait between the preset becoming reachable and firing the call.
# The JTC needs its open-loop "current commanded position" populated by
# at least one /joint_states cycle before it can interpolate cleanly.
SETTLE_S = 2.0
# Cap on the preset call itself. The default PRESET_TRAVEL_SECONDS in
# arm_preset_server is 3s; we wait a bit longer to cover the network +
# the JTC's action_monitor cycle.
CALL_TIMEOUT_S = 10.0


class AutoHome(Node):
    def __init__(self) -> None:
        super().__init__('lupin_auto_home')
        self.sub = self.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10,
        )
        self.cli = self.create_client(SetArmPreset, '/lupin/arm/preset')
        self.arm_joints_seen = False

    def _on_joint_state(self, msg: JointState) -> None:
        if all(j in msg.name for j in REQUIRED_ARM_JOINTS):
            self.arm_joints_seen = True


def main() -> int:
    # Opt-out path. Empty / unset → enabled by default.
    opt = os.environ.get('LUPIN_AUTO_HOME', 'true').strip().lower()
    if opt in ('false', '0', 'no', 'off'):
        print('[lupin_auto_home] LUPIN_AUTO_HOME=%s — skipping' % opt)
        return 0

    rclpy.init()
    node = AutoHome()

    try:
        # 1) /joint_states with all 4 arm joints
        print('[lupin_auto_home] waiting up to %.0fs for /joint_states with arm joints'
              % JOINT_STATE_TIMEOUT_S)
        deadline = time.monotonic() + JOINT_STATE_TIMEOUT_S
        while time.monotonic() < deadline and not node.arm_joints_seen:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.arm_joints_seen:
            print('[lupin_auto_home] timeout — /joint_states never published '
                  'all 4 arm joints. Likely cause: spawner failed to load '
                  'controllers. ros2 control list_controllers to diagnose.')
            return 1
        print('[lupin_auto_home] /joint_states ready')

        # 2) /lupin/arm/preset service alive
        print('[lupin_auto_home] waiting up to %.0fs for /lupin/arm/preset'
              % SERVICE_TIMEOUT_S)
        if not node.cli.wait_for_service(timeout_sec=SERVICE_TIMEOUT_S):
            print('[lupin_auto_home] timeout — /lupin/arm/preset never appeared. '
                  'arm_preset_server (lupin-onboard) likely failed to start.')
            return 1
        print('[lupin_auto_home] /lupin/arm/preset ready')

        # 3) Settle
        time.sleep(SETTLE_S)

        # 4) Call
        req = SetArmPreset.Request()
        req.name = 'home'
        print('[lupin_auto_home] calling /lupin/arm/preset name=%r' % req.name)
        fut = node.cli.call_async(req)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=CALL_TIMEOUT_S)
        if not fut.done():
            print('[lupin_auto_home] preset call timed out after %.0fs' % CALL_TIMEOUT_S)
            return 1
        resp = fut.result()
        print('[lupin_auto_home] response: success=%s message=%r'
              % (resp.success, resp.message))
        return 0 if resp.success else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
