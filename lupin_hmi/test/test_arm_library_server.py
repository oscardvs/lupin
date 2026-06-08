"""Node-level tests for :mod:`lupin_hmi.arm_library_server`.

These cover the torque-orchestration logic that the pure-model tests in
test_arm_library.py cannot reach: the record start/cancel flow and the
``set_torque`` round-trip. They instantiate the real node (rclpy required) and
stub only the *unavoidable* external torque RPC (``_call_torque``) — the
state-flip and abort decisions under test are the node's own logic.

XDG_CONFIG_HOME is redirected to a tmp dir so the node never touches the real
~/.config/lupin/arm_library.json.
"""
from __future__ import annotations

import pytest

rclpy = pytest.importorskip("rclpy")

from lupin_msgs.srv import ArmRecord  # noqa: E402
from lupin_hmi.arm_library_server import ArmLibraryServer  # noqa: E402


@pytest.fixture
def node(tmp_path, monkeypatch):
    monkeypatch.setenv("XDG_CONFIG_HOME", str(tmp_path))
    rclpy.init()
    n = ArmLibraryServer()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _rec(action, *, mode="", include_gripper=False, name="", overwrite=False):
    r = ArmRecord.Request()
    r.action = action
    r.mode = mode
    r.include_gripper = include_gripper
    r.name = name
    r.overwrite = overwrite
    return r


# ── F1: the published torque state must not lie ──────────────────────────────
def test_set_torque_does_not_flip_state_when_call_fails(node, monkeypatch):
    """If the underlying torque RPC fails, _torque_on must stay as it was —
    the state topic must not claim the arm went limp when it didn't."""
    node._torque_on = True
    monkeypatch.setattr(node, "_call_torque", lambda enable: (False, "unavailable"))
    node._set_torque(False)
    assert node._torque_on is True


def test_set_torque_flips_state_only_on_success(node, monkeypatch):
    node._torque_on = True
    monkeypatch.setattr(node, "_call_torque", lambda enable: (True, "ok"))
    ok, _msg = node._set_torque(False)
    assert ok is True
    assert node._torque_on is False


# ── F2: kinesthetic start must abort if torque cannot be disabled ────────────
def test_kinesthetic_start_aborts_when_torque_cannot_disable(node, monkeypatch):
    monkeypatch.setattr(node, "_call_torque", lambda enable: (False, "boom"))
    resp = ArmRecord.Response()
    node._on_record(_rec("start", mode="kinesthetic"), resp)
    assert resp.success is False
    assert "torque" in resp.message.lower()
    # recording state must be rolled back so the UI doesn't show a phantom
    # "recording (limp)" with a still-rigid arm.
    assert node._recording is None
    assert node._rec_mode == ""


def test_kinesthetic_start_succeeds_when_torque_confirmed_off(node, monkeypatch):
    seen = []
    monkeypatch.setattr(node, "_call_torque", lambda enable: (seen.append(enable) or (True, "ok")))
    resp = ArmRecord.Response()
    node._on_record(_rec("start", mode="kinesthetic"), resp)
    assert resp.success is True
    assert node._recording is not None
    assert seen == [False]  # asked the servos to power DOWN


def test_teleop_start_does_not_touch_torque(node, monkeypatch):
    seen = []
    monkeypatch.setattr(node, "_call_torque", lambda enable: (seen.append(enable) or (True, "ok")))
    resp = ArmRecord.Response()
    node._on_record(_rec("start", mode="teleop"), resp)
    assert resp.success is True
    assert seen == []  # teleop keeps torque ON — never calls set_torque


# ── cancel must restore torque after a kinesthetic recording ─────────────────
def test_cancel_kinesthetic_restores_torque(node, monkeypatch):
    seen = []
    monkeypatch.setattr(node, "_call_torque", lambda enable: (seen.append(enable) or (True, "ok")))
    node._on_record(_rec("start", mode="kinesthetic"), ArmRecord.Response())
    resp = ArmRecord.Response()
    node._on_record(_rec("cancel"), resp)
    assert resp.success is True
    assert node._recording is None
    assert seen == [False, True]  # disabled on start, re-enabled on cancel
