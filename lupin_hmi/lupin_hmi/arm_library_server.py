"""arm_library_server — laptop-side owner of the arm pose/sequence library.

Owns ${XDG_CONFIG_HOME:-~/.config}/lupin/arm_library.json and exposes CRUD +
record + replay under /lupin/arm/library/*. Thin wiring over lupin_hmi.arm_library.
Records by subscribing /joint_states; replays by publishing a multi-point
JointTrajectory to /mirte_master_arm_controller/joint_trajectory and firing
/lupin/gripper/set_angle_with_speed; toggles torque via /lupin/arm/set_torque.

Built-in presets stay in arm_preset_server (onboard) — this node only owns
USER poses/sequences. Consumers (HMI, voice) route built-in names to /lupin/arm/preset.
"""
from __future__ import annotations

import json
import os
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectory

from mirte_msgs.srv import SetServoAngleWithSpeed
from lupin_msgs.srv import (
    GetArmLibrary, SaveArmPose, ArmRecord, PlayArmSequence, ArmLibraryEdit, SetArmPreset,
)

from lupin_hmi.arm_library import (
    ArmLibraryError, ArmLibraryStore, RecordingBuffer, Sequence,
    extract_gripper_events,
)
from lupin_hmi.arm_limits import (
    ARM_JOINTS, ARM_JOINT_FULL, clamp_arm_joint, gripper_rad_to_deg, limits_summary,
)
from lupin_hmi.arm_traj import build_arm_trajectory, build_arm_trajectory_multi

ARM_JOINT_NAMES = [ARM_JOINT_FULL[j] for j in ARM_JOINTS]
GRIPPER_JOINT_NAME = "gripper_joint"
TRAJ_TOPIC = "/mirte_master_arm_controller/joint_trajectory"
GRIPPER_SRV = "/lupin/gripper/set_angle_with_speed"
TORQUE_SRV = "/lupin/arm/set_torque"
GOTO_TRAVEL_S = 3.0


def _default_library_path() -> Path:
    base = os.environ.get("XDG_CONFIG_HOME") or os.path.join(os.path.expanduser("~"), ".config")
    return Path(base) / "lupin" / "arm_library.json"


def _now_iso() -> str:
    return datetime.now(timezone.utc).replace(microsecond=0).isoformat()


class ArmLibraryServer(Node):
    def __init__(self) -> None:
        super().__init__("arm_library_server")
        path = Path(self.declare_parameter("library_path", str(_default_library_path()))
                    .get_parameter_value().string_value)
        self.store = ArmLibraryStore(path)
        self.store.load()

        self._cb = ReentrantCallbackGroup()
        self._state_lock = threading.Lock()
        self._latest: Optional[JointState] = None
        self._recording: Optional[RecordingBuffer] = None
        self._rec_mode = ""
        self._rec_start_clock = None       # rclpy Time when recording started
        self._playing_name = ""
        self._play_start_clock = None      # rclpy Time when replay started
        self._play_total_s = 0.0           # expected replay duration (scaled)
        self._torque_on = True
        self._idx_names: tuple = ()        # cache: last /joint_states name order
        self._idx_map: dict = {}           # cache: name -> index for that order

        self._traj_pub = self.create_publisher(JointTrajectory, TRAJ_TOPIC, 10)
        self._state_pub = self.create_publisher(
            String, "/lupin/arm/library/state",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(JointState, "/joint_states", self._on_joint_states, 50)

        self._gripper_cli = self.create_client(SetServoAngleWithSpeed, GRIPPER_SRV, callback_group=self._cb)
        self._torque_cli = self.create_client(SetBool, TORQUE_SRV, callback_group=self._cb)

        srv = lambda t, n, h: self.create_service(t, n, h, callback_group=self._cb)
        srv(GetArmLibrary, "/lupin/arm/library/list", self._on_list)
        srv(SaveArmPose, "/lupin/arm/library/save_pose", self._on_save_pose)
        srv(SetArmPreset, "/lupin/arm/library/goto_pose", self._on_goto_pose)
        srv(ArmRecord, "/lupin/arm/library/record", self._on_record)
        srv(PlayArmSequence, "/lupin/arm/library/play", self._on_play)
        srv(Trigger, "/lupin/arm/library/stop", self._on_stop)
        srv(ArmLibraryEdit, "/lupin/arm/library/delete", self._on_edit)

        self._play_done_timer = None
        self._gripper_timers: List = []
        self.create_timer(0.2, self._publish_state)
        self.get_logger().info(
            f"arm_library_server ready — {path} — limits: {limits_summary()}")

    # ── feedback + state ────────────────────────────────────────────────
    def _on_joint_states(self, msg: JointState) -> None:
        self._latest = msg
        # Extract outside the lock — this only reads the message.
        arm = self._extract_arm(msg)
        grip = self._extract_gripper(msg)
        t = self._stamp(msg)
        if arm is not None:
            with self._state_lock:
                if self._recording is not None:
                    self._recording.add(t, arm, grip)

    def _stamp(self, msg: JointState) -> float:
        return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _index_for(self, msg: JointState) -> dict:
        # /joint_states names are static; rebuild the map only when they change.
        names = tuple(msg.name)
        if names != self._idx_names:
            self._idx_names = names
            self._idx_map = {n: i for i, n in enumerate(names)}
        return self._idx_map

    def _extract_arm(self, msg: JointState) -> Optional[List[float]]:
        idx = self._index_for(msg)
        try:
            return [float(msg.position[idx[n]]) for n in ARM_JOINT_NAMES]
        except (KeyError, IndexError):
            return None

    def _extract_gripper(self, msg: JointState) -> Optional[float]:
        idx = self._index_for(msg)
        i = idx.get(GRIPPER_JOINT_NAME)
        return float(msg.position[i]) if i is not None and i < len(msg.position) else None

    def _publish_state(self) -> None:
        now = self.get_clock().now()
        with self._state_lock:
            recording = self._recording is not None
            n = len(self._recording.waypoints) if recording else 0
            rec_start = self._rec_start_clock
            playing_name = self._playing_name
            play_start = self._play_start_clock
            play_total_s = self._play_total_s
            rec_mode = self._rec_mode
            torque_on = self._torque_on
        elapsed = 0.0
        if recording and rec_start is not None:
            elapsed = (now - rec_start).nanoseconds * 1e-9
        progress = 0.0
        if playing_name and play_start is not None and play_total_s > 0:
            progress = min(1.0, (now - play_start).nanoseconds * 1e-9 / play_total_s)
        payload = {
            "recording": recording, "playing": bool(playing_name),
            "name": playing_name, "mode": rec_mode,
            "progress": progress, "elapsed_s": elapsed, "n_waypoints": n,
            "torque": torque_on,
        }
        self._state_pub.publish(String(data=json.dumps(payload)))

    # ── list / save_pose / goto_pose / delete ───────────────────────────
    def _on_list(self, req, resp):
        resp.success = True
        resp.json = json.dumps(self.store.list_metadata())
        return resp

    def _on_save_pose(self, req, resp):
        try:
            if req.from_current:
                if self._latest is None:
                    raise ArmLibraryError("no /joint_states yet — cannot snapshot current pose")
                arm = self._extract_arm(self._latest)
                grip = self._extract_gripper(self._latest)
                if arm is None:
                    raise ArmLibraryError("/joint_states missing arm joints")
            else:
                arm = list(req.arm_rad)
                grip = req.gripper_rad if req.has_gripper else None
            arm = [clamp_arm_joint(j, v) for j, v in zip(ARM_JOINTS, arm)]
            self.store.save_pose(req.name, arm, grip, overwrite=req.overwrite, created=_now_iso())
            resp.success, resp.message = True, f'pose "{req.name}" saved'
        except ArmLibraryError as e:
            resp.success, resp.message = False, str(e)
        return resp

    def _on_goto_pose(self, req, resp):
        try:
            pose = self.store.get_pose(req.name)
        except ArmLibraryError as e:
            resp.success, resp.message = False, str(e)
            return resp
        positions = [clamp_arm_joint(j, v) for j, v in zip(ARM_JOINTS, pose.arm)]
        self._traj_pub.publish(build_arm_trajectory(ARM_JOINT_NAMES, positions, GOTO_TRAVEL_S))
        if pose.gripper is not None:
            self._send_gripper(pose.gripper)
        resp.success, resp.message = True, f'moving to pose "{req.name}"'
        return resp

    def _on_edit(self, req, resp):
        try:
            if req.new_name:
                self.store.rename(req.kind, req.name, req.new_name)
                resp.message = f'{req.kind} "{req.name}" renamed to "{req.new_name}"'
            else:
                self.store.delete(req.kind, req.name)
                resp.message = f'{req.kind} "{req.name}" deleted'
            resp.success = True
        except ArmLibraryError as e:
            resp.success, resp.message = False, str(e)
        return resp

    # ── record / play / stop are filled in by Tasks 7 & 8 ──────────────
    def _on_record(self, req, resp):
        action = (req.action or "").strip().lower()
        if action == "start":
            mode = (req.mode or "teleop").strip().lower()
            if mode not in ("kinesthetic", "teleop"):
                resp.success, resp.message = False, f'unknown mode "{req.mode}"'
                return resp
            with self._state_lock:
                if self._playing_name:
                    resp.success, resp.message = False, "cannot record while a sequence is playing"
                    return resp
                if self._recording is not None:
                    resp.success, resp.message = False, "already recording"
                    return resp
                self._recording = RecordingBuffer(mode=mode, include_gripper=req.include_gripper)
                self._rec_mode = mode
                self._rec_start_clock = self.get_clock().now()
            if mode == "kinesthetic":
                self._set_torque(False)  # consumer shows the "support the arm" countdown first
            resp.success = True
            resp.message = f"recording started ({mode})"
            return resp

        if action in ("save", "cancel"):
            with self._state_lock:
                if self._recording is None:
                    resp.success, resp.message = False, "not recording"
                    return resp
                buf = self._recording
                mode = self._rec_mode
                self._recording = None
                self._rec_mode = ""
                self._rec_start_clock = None
            if mode == "kinesthetic":
                self._set_torque(True)  # restore + re-pin
            if action == "cancel":
                resp.success, resp.message = True, "recording cancelled"
                return resp
            end_t = self._stamp(self._latest) if self._latest is not None else 0.0
            seq = buf.finalize(end_t=end_t, created=_now_iso())
            if not seq.waypoints:
                resp.success, resp.message = False, "nothing recorded"
                return resp
            try:
                self.store.save_sequence(req.name, seq, overwrite=req.overwrite)
            except ArmLibraryError as e:
                resp.success, resp.message = False, str(e)
                return resp
            if buf.truncated:
                self.get_logger().warn(
                    f'sequence "{req.name}" hit the waypoint cap and was truncated')
            resp.success = True
            resp.message = (f'sequence "{req.name}" saved'
                            + (" (TRUNCATED at cap)" if buf.truncated else ""))
            resp.duration_s = seq.duration_s
            resp.n_waypoints = len(seq.waypoints)
            return resp

        resp.success, resp.message = False, f'unknown action "{req.action}"'
        return resp

    def _on_play(self, req, resp):
        # Validate (read-only) outside the lock.
        try:
            seq = self.store.get_sequence(req.name)
        except ArmLibraryError as e:
            resp.success, resp.message = False, str(e)
            return resp
        if not seq.waypoints:
            resp.success, resp.message = False, "sequence has no waypoints"
            return resp
        speed = max(0.25, min(2.0, req.speed if req.speed else 1.0))
        total = max(0.1, seq.duration_s / speed)

        # Atomically check the guards and commit the playing state + timers so two
        # near-simultaneous calls can't both pass the guard (TOCTOU).
        with self._state_lock:
            if self._recording is not None:
                resp.success, resp.message = False, "cannot play while recording"
                return resp
            if self._playing_name:
                resp.success, resp.message = False, f'already playing "{self._playing_name}"'
                return resp
            self._playing_name = req.name
            self._play_start_clock = self.get_clock().now()
            self._play_total_s = total
            # Gripper events on a timeline (cancel any previous timers first).
            self._clear_gripper_timers()
            if seq.include_gripper:
                for t, grip in extract_gripper_events(seq.waypoints):
                    delay = max(0.0, t / speed)
                    self._gripper_timers.append(
                        self.create_timer(delay, self._make_gripper_cb(grip), callback_group=self._cb))
            self._play_done_timer = self.create_timer(total, self._on_play_done, callback_group=self._cb)

        # Motion commands operate on the local seq — safe to run after release.
        self._set_torque(True)  # force torque on + re-pin before motion
        names = ARM_JOINT_NAMES
        wps = [(w.t, [clamp_arm_joint(j, v) for j, v in zip(ARM_JOINTS, w.arm)])
               for w in seq.waypoints]
        self._traj_pub.publish(build_arm_trajectory_multi(names, wps, speed=speed))
        resp.success, resp.message = True, f'playing "{req.name}" at {speed:.2f}x'
        return resp

    def _make_gripper_cb(self, grip: float):
        fired = {"done": False}
        def _cb():
            if not fired["done"]:
                fired["done"] = True
                self._send_gripper(grip)
        return _cb

    def _clear_gripper_timers(self) -> None:
        for tmr in self._gripper_timers:
            tmr.cancel()
        self._gripper_timers = []

    def _on_play_done(self) -> None:
        with self._state_lock:
            if getattr(self, "_play_done_timer", None) is not None:
                self._play_done_timer.cancel()
                self._play_done_timer = None
            self._clear_gripper_timers()
            self._playing_name = ""
            self._play_start_clock = None

    def _on_stop(self, req, resp):
        # Abort any in-flight replay and hold at the current pose.
        with self._state_lock:
            was_playing = bool(self._playing_name)
            if getattr(self, "_play_done_timer", None) is not None:
                self._play_done_timer.cancel()
                self._play_done_timer = None
            self._clear_gripper_timers()
            self._playing_name = ""
            self._play_start_clock = None
            # If a kinesthetic record was somehow active, clear it (restore torque below).
            restore_torque = self._recording is not None and self._rec_mode == "kinesthetic"
            if restore_torque:
                self._recording = None
                self._rec_mode = ""
        if was_playing and self._latest is not None:
            arm = self._extract_arm(self._latest)
            if arm is not None:
                hold = [clamp_arm_joint(j, v) for j, v in zip(ARM_JOINTS, arm)]
                self._traj_pub.publish(build_arm_trajectory(ARM_JOINT_NAMES, hold, 0.3))
        if restore_torque:
            self._set_torque(True)
        resp.success, resp.message = True, "stopped"
        return resp

    # ── helpers used by later tasks ─────────────────────────────────────
    def _send_gripper(self, gripper_rad: float) -> None:
        if not self._gripper_cli.service_is_ready():
            self.get_logger().warn("gripper service not ready — skipping gripper command")
            return
        req = SetServoAngleWithSpeed.Request()
        req.angle = float(gripper_rad_to_deg(gripper_rad))
        req.rate = 45.0
        req.degrees = True
        self._gripper_cli.call_async(req)

    def _set_torque(self, enable: bool) -> None:
        self._torque_on = enable
        if not self._torque_cli.service_is_ready():
            self.get_logger().warn("set_torque service not ready")
            return
        r = SetBool.Request()
        r.data = enable
        self._torque_cli.call_async(r)


def main() -> None:
    rclpy.init()
    node = ArmLibraryServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
