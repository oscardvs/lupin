"""arm_calibrate_server — Hiwonder zero-offset calibration as a service.

Ports the upstream ``mirte_test.mirte_master_calibrate`` script into a
stateful node so an HMI button (or the voice agent) can drive the
operator-in-the-loop calibration flow remotely.

Flow (matches the upstream script, split into two service calls so the
operator can hand-pose the arm between them):

    1. start  →  /enable_arm_control(False), zero all _set_offset values,
                 command angle 0 on each joint, disable all servos.
                 State IDLE → AWAITING_POSE.
    2.        (operator physically moves arm to mechanical home)
    3. commit →  Sample ServoPosition.raw for ~2 s, compute
                 totalDiff = (raw - home) + curr_offset (centidegrees),
                 write to /io/servo/hiwonder/<j>/_set_offset,
                 re-command angle 0, /enable_arm_control(True).
                 State AWAITING_POSE → IDLE.

``cancel`` re-enables servos + arm control without writing offsets.
``status`` returns current state with no side effects.

Hardware-only. Sim has no Hiwonder ``_set_offset`` services
(``arm_sim_shim`` only forwards angle commands), so the server's
``wait_for_service`` on those endpoints will time out and ``start``
returns an error.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_srvs.srv import SetBool

from mirte_msgs.msg import ServoPosition
from mirte_msgs.srv import GetServoOffset, SetServoAngle, SetServoOffset

from lupin_msgs.srv import CalibrateArm


# Per-servo calibration data. Lifted from
# mirte-ros-packages/mirte_test/mirte_test/mirte_master_calibrate.py (the
# values the teaching team ship). shoulder_pan (id 2) is intentionally
# OMITTED — see the note above the SERVOS list below. All rows present
# here match upstream.
#
# `home` is the raw encoder value the servo should report when the arm is
# in its mechanical home pose. `min`/`max` are unused at runtime; kept here
# because they're useful when re-deriving `home` on a new robot.
@dataclass(frozen=True)
class ServoCal:
    name: str
    min: int
    max: int
    home: int


# shoulder_pan is intentionally omitted to match upstream mirte_test, which
# commented it out (`mirte_master_calibrate.py:5-13`). Its home raw value
# wasn't trusted by the teaching team and a bad calibration there could
# swing the arm into the chassis on re-enable. Add it back here once
# `home` is verified on Mirte-247264.
SERVOS: List[ServoCal] = [
    ServoCal('shoulder_lift', 2832,  20000, 11450),
    ServoCal('elbow',          120,  21000, 11750),
    ServoCal('wrist',         1128,  21672, 12200),
    ServoCal('gripper',       6168,  14224, 10524),
]

# Raw-tick → centidegree conversion factor. Hiwonder HX servos report 24000
# raw ticks across 360°, so 24 raw ticks ≈ 1 centidegree. Used only for the
# safety clamp below — the actual offset write copies the raw-tick diff
# directly into SetServoOffset.centidegrees, matching upstream
# (the field name in mirte_msgs is misleading; the bus consumes the same
# unit it returns from GetServoOffset, which is the raw-tick offset).
TICKS_PER_CENTIDEG = 24

# Minimum |totalDiff| (raw ticks) before we bother writing a new offset.
# Upstream uses 200 — a noise floor gate.
WRITE_THRESHOLD_TICKS = 200

# Safety clamp from upstream: if |totalDiff / 24| > 125 the servo is
# mechanically misassembled by more than ~1.25°, beyond what an offset
# write can sensibly correct. Skip that joint and surface an error.
MAX_CENTIDEG_CLAMP = 125

# How long to sample ServoPosition.raw on commit. Upstream uses 2 s.
COMMIT_SAMPLE_SECONDS = 2.0

# Bus service timeouts. The Hiwonder bus is slow; the upstream script
# spin_until_future_completes with no timeout. We pick generous bounds.
SERVICE_WAIT_S = 2.0
SERVICE_CALL_S = 3.0

STATE_IDLE = 'IDLE'
STATE_AWAITING_POSE = 'AWAITING_POSE'

HIWONDER_NS = '/io/servo/hiwonder'
ENABLE_ARM_CONTROL_SRV = '/enable_arm_control'
ENABLE_ALL_SERVOS_SRV = f'{HIWONDER_NS}/enable_all_servos'


class ArmCalibrateServer(Node):
    def __init__(self) -> None:
        super().__init__('arm_calibrate_server')

        # ReentrantCallbackGroup so the service handler can call other
        # services and have their futures resolve on the executor's other
        # threads (we use MultiThreadedExecutor in main()).
        self._cb_group = ReentrantCallbackGroup()

        self._state: str = STATE_IDLE
        self._state_lock = threading.Lock()

        # Latest raw position per joint name. Updated by the
        # /io/servo/hiwonder/<name>/position subscriptions while awaiting
        # pose; read once at commit time.
        self._latest_raw: Dict[str, Optional[int]] = {s.name: None for s in SERVOS}
        self._raw_lock = threading.Lock()
        self._sampling = False  # only update _latest_raw while True
        self._subs: list = []

        # Service clients (created lazily; bus services only exist on the
        # real robot).
        self._enable_arm_client = self.create_client(
            SetBool, ENABLE_ARM_CONTROL_SRV, callback_group=self._cb_group,
        )
        self._enable_all_servos_client = self.create_client(
            SetBool, ENABLE_ALL_SERVOS_SRV, callback_group=self._cb_group,
        )
        self._set_offset_clients: Dict[str, rclpy.client.Client] = {}
        self._get_offset_clients: Dict[str, rclpy.client.Client] = {}
        self._set_angle_clients: Dict[str, rclpy.client.Client] = {}
        for servo in SERVOS:
            self._set_offset_clients[servo.name] = self.create_client(
                SetServoOffset,
                f'{HIWONDER_NS}/{servo.name}/_set_offset',
                callback_group=self._cb_group,
            )
            self._get_offset_clients[servo.name] = self.create_client(
                GetServoOffset,
                f'{HIWONDER_NS}/{servo.name}/_offset',
                callback_group=self._cb_group,
            )
            self._set_angle_clients[servo.name] = self.create_client(
                SetServoAngle,
                f'{HIWONDER_NS}/{servo.name}/set_angle',
                callback_group=self._cb_group,
            )

        self._srv = self.create_service(
            CalibrateArm,
            '/lupin/arm/calibrate',
            self._on_request,
            callback_group=self._cb_group,
        )

        self.get_logger().info(
            'arm_calibrate_server ready — /lupin/arm/calibrate '
            f'{{start|commit|cancel|status}} · joints={[s.name for s in SERVOS]}'
        )

    # ── service dispatch ───────────────────────────────────────────────

    def _on_request(
        self,
        req: CalibrateArm.Request,
        resp: CalibrateArm.Response,
    ) -> CalibrateArm.Response:
        action = (req.action or '').strip().lower()
        if action == 'status':
            return self._fill(resp, success=True, message='ok')
        if action == 'start':
            return self._do_start(resp)
        if action == 'commit':
            return self._do_commit(resp)
        if action == 'cancel':
            return self._do_cancel(resp)
        return self._fill(
            resp, success=False,
            message=f'unknown action "{req.action}". '
                    'Use start | commit | cancel | status.',
        )

    # ── start ──────────────────────────────────────────────────────────

    def _do_start(self, resp: CalibrateArm.Response) -> CalibrateArm.Response:
        # Claim the transition under the lock so a concurrent start/commit
        # request can't race past the IDLE check. We use a transient BUSY
        # sentinel until the bus calls complete so cancel/status see a
        # consistent intermediate state.
        with self._state_lock:
            if self._state != STATE_IDLE:
                return self._fill(
                    resp, success=False,
                    message=f'cannot start from state {self._state}; '
                            'call cancel or commit first.',
                )
            self._state = STATE_AWAITING_POSE

        self.get_logger().info('calibrate.start: disabling arm controller')
        if not self._call_set_bool(self._enable_arm_client, False):
            with self._state_lock:
                self._state = STATE_IDLE
            return self._fill(
                resp, success=False,
                message=f'failed to call {ENABLE_ARM_CONTROL_SRV}. '
                        'Is mirte-ros running? Calibration is hardware-only.',
            )

        # Wait briefly to let the arm controller release before we
        # commandeer the servos. Upstream sleeps 2 s here — match it.
        time.sleep(2.0)

        # Zero offsets and command angle 0 on every joint, in lockstep
        # with what the upstream calibrate() does. Any failure here aborts
        # and re-enables arm control so we don't leave the robot in a
        # half-configured state.
        for servo in SERVOS:
            if not self._set_offset(servo.name, 0):
                self._abort_to_idle()
                return self._fill(
                    resp, success=False,
                    message=f'failed to zero offset for {servo.name}',
                )
            if not self._set_angle_zero(servo.name):
                self._abort_to_idle()
                return self._fill(
                    resp, success=False,
                    message=f'failed to send angle=0 to {servo.name}',
                )

        # Disable all servos so the operator can hand-pose. After this the
        # arm goes limp and gravity wins — warn the operator in the HMI
        # before they press Start.
        if not self._call_set_bool(self._enable_all_servos_client, False):
            self._abort_to_idle()
            return self._fill(
                resp, success=False,
                message=f'failed to call {ENABLE_ALL_SERVOS_SRV}',
            )

        # Subscribe to position topics so commit can read the operator's
        # hand-posed values. We subscribe (rather than poll) so the user
        # can take as long as they need.
        self._start_sampling()

        self.get_logger().info(
            'calibrate.start: servos disabled, awaiting operator pose'
        )
        return self._fill(
            resp, success=True,
            message='servos disabled — hand-pose the arm to its home '
                    'position, then call commit.',
        )

    # ── commit ─────────────────────────────────────────────────────────

    def _do_commit(self, resp: CalibrateArm.Response) -> CalibrateArm.Response:
        # Flip to IDLE eagerly under the lock so a concurrent commit can't
        # also pass the AWAITING_POSE check and double-write offsets.
        with self._state_lock:
            if self._state != STATE_AWAITING_POSE:
                return self._fill(
                    resp, success=False,
                    message=f'cannot commit from state {self._state}; '
                            'call start first.',
                )
            self._state = STATE_IDLE

        self.get_logger().info(
            f'calibrate.commit: sampling positions for {COMMIT_SAMPLE_SECONDS:.1f}s'
        )
        # Wait while the position subscriptions fill _latest_raw on other
        # executor threads. We are on a worker thread of the
        # MultiThreadedExecutor — sleeping here does not block the subs.
        time.sleep(COMMIT_SAMPLE_SECONDS)
        self._stop_sampling()

        # Read what we got. Any joint with no sample → bail out, because
        # we'd otherwise compute a nonsense offset.
        with self._raw_lock:
            samples = dict(self._latest_raw)
        missing = [name for name, raw in samples.items() if raw is None]
        if missing:
            # Re-enable so the robot is not left limp.
            self._reenable_after_commit()
            return self._fill(
                resp, success=False,
                message='no ServoPosition received for: '
                        + ', '.join(missing)
                        + '. Are the Hiwonder servos powered?',
            )

        # Compute and write offsets. Errors per-joint are reported but
        # don't abort the whole commit — we want as much of the arm
        # calibrated as possible.
        joint_names: List[str] = []
        offsets_applied: List[int] = []
        diffs_observed: List[int] = []
        per_joint_msgs: List[str] = []

        for servo in SERVOS:
            raw = samples[servo.name]
            assert raw is not None  # guarded above
            curr_offset = self._get_offset(servo.name)
            if curr_offset is None:
                per_joint_msgs.append(
                    f'{servo.name}: get_offset failed; skipped'
                )
                joint_names.append(servo.name)
                offsets_applied.append(0)
                diffs_observed.append(raw - servo.home)
                continue

            diff = raw - servo.home  # raw ticks, signed
            total_diff = diff + curr_offset  # centidegree-scale-ish in the bus

            joint_names.append(servo.name)
            diffs_observed.append(diff)

            if abs(total_diff) <= WRITE_THRESHOLD_TICKS:
                # Below noise floor — leave existing offset alone.
                offsets_applied.append(curr_offset)
                per_joint_msgs.append(
                    f'{servo.name}: diff={diff} within ±{WRITE_THRESHOLD_TICKS}; no write'
                )
                continue

            # Safety clamp in centidegrees: |total_diff/24| > 125
            # ≈ >1.25° mechanical misassembly, beyond an offset's reach.
            if abs(total_diff) / TICKS_PER_CENTIDEG > MAX_CENTIDEG_CLAMP:
                offsets_applied.append(curr_offset)
                per_joint_msgs.append(
                    f'{servo.name}: |{total_diff}|ticks exceeds clamp '
                    f'(±{MAX_CENTIDEG_CLAMP * TICKS_PER_CENTIDEG} ticks); '
                    'servo may be misassembled — skipped'
                )
                self.get_logger().error(
                    f'{servo.name}: total_diff={total_diff} ticks > clamp; '
                    'not writing offset'
                )
                continue

            # Write the raw-tick diff. The field is misleadingly named
            # `centidegrees` in mirte_msgs/srv/SetServoOffset, but upstream
            # mirte_test writes raw ticks here and GetServoOffset returns
            # the same unit on read-back — preserve that contract.
            if self._set_offset(servo.name, total_diff):
                offsets_applied.append(total_diff)
                per_joint_msgs.append(
                    f'{servo.name}: wrote offset={total_diff}ticks (diff={diff})'
                )
            else:
                offsets_applied.append(curr_offset)
                per_joint_msgs.append(
                    f'{servo.name}: set_offset({total_diff}) FAILED'
                )

        # Re-command angle 0 with the new offsets active.
        for servo in SERVOS:
            self._set_angle_zero(servo.name)

        # Re-enable the arm controller. After this the JTC's stale
        # commanded position (likely 0) becomes the new setpoint — which
        # is exactly what we want, since we just zeroed every joint.
        self._call_set_bool(self._enable_arm_client, True)

        msg = 'calibration complete — ' + ' | '.join(per_joint_msgs)
        self.get_logger().info(msg)

        resp.joint_names = joint_names
        resp.offsets_applied = offsets_applied
        resp.diffs_observed = diffs_observed
        return self._fill(resp, success=True, message=msg)

    # ── cancel ─────────────────────────────────────────────────────────

    def _do_cancel(self, resp: CalibrateArm.Response) -> CalibrateArm.Response:
        # Eagerly flip to IDLE so a concurrent commit can't fire on the
        # same AWAITING_POSE state.
        with self._state_lock:
            if self._state == STATE_IDLE:
                return self._fill(
                    resp, success=True,
                    message='already idle; nothing to cancel',
                )
            self._state = STATE_IDLE

        self._stop_sampling()
        # Best effort — re-enable both layers so the robot isn't limp.
        self._call_set_bool(self._enable_all_servos_client, True)
        self._call_set_bool(self._enable_arm_client, True)

        self.get_logger().info('calibrate.cancel: re-enabled, offsets unchanged')
        return self._fill(
            resp, success=True,
            message='cancelled — servos re-enabled, offsets unchanged',
        )

    # ── helpers ────────────────────────────────────────────────────────

    def _abort_to_idle(self) -> None:
        """Best-effort cleanup when start() partially completes. Re-enable
        the arm control loop so the robot doesn't sit half-configured,
        and drop back to IDLE so the next start can run."""
        self._stop_sampling()
        self._call_set_bool(self._enable_all_servos_client, True)
        self._call_set_bool(self._enable_arm_client, True)
        with self._state_lock:
            self._state = STATE_IDLE

    def _reenable_after_commit(self) -> None:
        """Best-effort cleanup when commit() fails after the eager
        state→IDLE transition. State is already IDLE; just power the
        robot back up so it isn't limp."""
        self._stop_sampling()
        self._call_set_bool(self._enable_all_servos_client, True)
        self._call_set_bool(self._enable_arm_client, True)

    def _start_sampling(self) -> None:
        with self._raw_lock:
            self._latest_raw = {s.name: None for s in SERVOS}
            self._sampling = True
        # Subscribe lazily here so we don't keep five subscriptions alive
        # while idle.
        for servo in SERVOS:
            topic = f'{HIWONDER_NS}/{servo.name}/position'
            sub = self.create_subscription(
                ServoPosition,
                topic,
                lambda msg, name=servo.name: self._on_position(name, msg),
                10,
                callback_group=self._cb_group,
            )
            self._subs.append(sub)

    def _stop_sampling(self) -> None:
        with self._raw_lock:
            self._sampling = False
        for sub in self._subs:
            try:
                self.destroy_subscription(sub)
            except Exception:  # noqa: BLE001
                pass
        self._subs = []

    def _on_position(self, name: str, msg: ServoPosition) -> None:
        with self._raw_lock:
            if not self._sampling:
                return
            self._latest_raw[name] = int(msg.raw)

    def _call_set_bool(self, client, value: bool) -> bool:
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            self.get_logger().error(
                f'{client.srv_name} not available within {SERVICE_WAIT_S}s'
            )
            return False
        req = SetBool.Request()
        req.data = value
        fut = client.call_async(req)
        result = self._wait_future(fut, SERVICE_CALL_S, client=client)
        return result is not None  # SetBool has a success field but the
                                   # vendor implementations are inconsistent
                                   # about populating it; "response came
                                   # back" is the strongest signal we have.

    def _set_offset(self, joint: str, raw_offset: int) -> bool:
        client = self._set_offset_clients[joint]
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            return False
        req = SetServoOffset.Request()
        # `centidegrees` is the field name in mirte_msgs; the bus actually
        # consumes raw ticks. See SERVOS comment block at top of file.
        req.centidegrees = raw_offset
        fut = client.call_async(req)
        return self._wait_future(fut, SERVICE_CALL_S, client=client) is not None

    def _get_offset(self, joint: str) -> Optional[int]:
        client = self._get_offset_clients[joint]
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            return None
        req = GetServoOffset.Request()
        fut = client.call_async(req)
        result = self._wait_future(fut, SERVICE_CALL_S, client=client)
        if result is None:
            return None
        return int(result.centidegrees)

    def _set_angle_zero(self, joint: str) -> bool:
        client = self._set_angle_clients[joint]
        if not client.wait_for_service(timeout_sec=SERVICE_WAIT_S):
            return False
        req = SetServoAngle.Request()
        req.angle = 0.0
        req.degrees = False
        fut = client.call_async(req)
        return self._wait_future(fut, SERVICE_CALL_S, client=client) is not None

    def _wait_future(self, fut, timeout_s: float, client=None):
        """Block this thread until `fut` is done. Other executor threads
        keep spinning, so service responses still resolve. On timeout,
        cancel the pending request on the owning client so a late reply
        doesn't end up assigned to the next service call's future."""
        deadline = time.monotonic() + timeout_s
        while not fut.done():
            if time.monotonic() > deadline:
                self.get_logger().warn('service call timed out')
                if client is not None:
                    try:
                        client.remove_pending_request(fut)
                    except Exception:  # noqa: BLE001
                        pass
                return None
            time.sleep(0.02)
        return fut.result()

    def _fill(
        self,
        resp: CalibrateArm.Response,
        *,
        success: bool,
        message: str,
    ) -> CalibrateArm.Response:
        resp.success = success
        resp.message = message
        with self._state_lock:
            resp.state = self._state
        # joint_names / offsets_applied / diffs_observed default to empty;
        # commit() populates them before calling _fill.
        return resp


def main() -> None:
    rclpy.init()
    node = ArmCalibrateServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
