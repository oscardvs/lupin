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

  0. Self-heal the ros2_control stack. The vendor mirte-ros first-boot
     races between ros2_control_node's YAML auto-load and the spawners,
     leaving joint_state_broadcaster + pid_wheels_controller stuck
     ``unconfigured`` (see project_2026_05_21_session_state). We list
     controllers, call configure_controller on any unconfigured target,
     then activate the lot via switch_controller. Idempotent: a clean
     boot finds everything active and returns immediately.
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

from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    SwitchController,
)
from lupin_msgs.srv import SetArmPreset


REQUIRED_ARM_JOINTS = (
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_joint',
)
# Controllers that must be active for /joint_states to publish and for
# wheel + arm commands to take effect. mirte_master_arm_controller is
# usually fine on first boot (separate launch path); pid_wheels +
# joint_state_broadcaster lose the vendor's "already loaded" race
# (see project_2026_05_21_session_state) and stay stuck `unconfigured`.
HEAL_TARGETS = (
    'joint_state_broadcaster',
    'pid_wheels_controller',
    'mirte_master_arm_controller',
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


def heal_controllers(node: Node, timeout_s: float = 30.0) -> bool:
    """Detect controllers stuck `unconfigured` and bring them to `active`.

    Workaround for the vendor stack's first-boot race where ros2_control_node's
    YAML auto-load and the spawner both try to load the same controller — the
    spawner sees "already loaded", FATALs out, and the controller stays
    `unconfigured` until something configures it manually. This function does
    exactly that, idempotently. Returns True if all HEAL_TARGETS ended up
    active (or were already), False if controller_manager itself is unreachable
    (a deeper DDS problem that we can't fix from here).
    """
    list_cli = node.create_client(
        ListControllers, '/controller_manager/list_controllers',
    )
    print('[lupin_auto_home] heal: waiting up to %.0fs for controller_manager'
          % timeout_s)
    if not list_cli.wait_for_service(timeout_sec=timeout_s):
        print('[lupin_auto_home] heal: controller_manager unreachable — '
              'self-heal cannot run (DDS / wedge problem upstream)')
        return False

    fut = list_cli.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    if not fut.done() or fut.result() is None:
        print('[lupin_auto_home] heal: list_controllers call timed out')
        return False

    states = {c.name: c.state for c in fut.result().controller}
    print('[lupin_auto_home] heal: current states: %s'
          % ', '.join('%s=%s' % (n, s) for n, s in sorted(states.items())))

    to_configure = [n for n in HEAL_TARGETS if states.get(n) == 'unconfigured']
    needs_activate = [n for n in HEAL_TARGETS
                      if states.get(n) in ('unconfigured', 'inactive')]
    if not needs_activate:
        print('[lupin_auto_home] heal: all targets already active — nothing to do')
        return True

    # Step 1: unconfigured → inactive
    if to_configure:
        conf_cli = node.create_client(
            ConfigureController, '/controller_manager/configure_controller',
        )
        if not conf_cli.wait_for_service(timeout_sec=5.0):
            print('[lupin_auto_home] heal: configure_controller service missing')
            return False
        for name in to_configure:
            req = ConfigureController.Request()
            req.name = name
            print('[lupin_auto_home] heal: configure %s' % name)
            fut = conf_cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
            if not fut.done() or fut.result() is None or not fut.result().ok:
                print('[lupin_auto_home] heal: configure FAILED for %s' % name)
                # Keep trying the others; switch will silently skip the ones
                # still in `unconfigured`.

    # Step 2: inactive → active via switch (BEST_EFFORT so a single bad one
    # doesn't block the rest — partial recovery beats none).
    switch_cli = node.create_client(
        SwitchController, '/controller_manager/switch_controller',
    )
    if not switch_cli.wait_for_service(timeout_sec=5.0):
        print('[lupin_auto_home] heal: switch_controller service missing')
        return False
    req = SwitchController.Request()
    req.activate_controllers = list(needs_activate)
    req.strictness = SwitchController.Request.BEST_EFFORT
    req.activate_asap = True
    print('[lupin_auto_home] heal: activate %s' % needs_activate)
    fut = switch_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    if not fut.done() or fut.result() is None or not fut.result().ok:
        print('[lupin_auto_home] heal: switch_controller (activate) FAILED')
        return False

    # Verify
    fut = list_cli.call_async(ListControllers.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    if fut.done() and fut.result() is not None:
        states = {c.name: c.state for c in fut.result().controller}
        ok = all(states.get(n) == 'active' for n in HEAL_TARGETS)
        print('[lupin_auto_home] heal: post-heal states: %s'
              % ', '.join('%s=%s' % (n, s) for n, s in sorted(states.items())))
        return ok
    return True


def main() -> int:
    # Opt-out path. Empty / unset → enabled by default.
    opt = os.environ.get('LUPIN_AUTO_HOME', 'true').strip().lower()
    if opt in ('false', '0', 'no', 'off'):
        print('[lupin_auto_home] LUPIN_AUTO_HOME=%s — skipping' % opt)
        return 0

    rclpy.init()
    node = AutoHome()

    try:
        # 0) Self-heal controllers if they got stuck `unconfigured` on this
        #    boot. This is what makes auto_home close the wedge instead of
        #    just being a victim of it. Idempotent — if everything's already
        #    active the function returns quickly.
        heal_controllers(node)

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
