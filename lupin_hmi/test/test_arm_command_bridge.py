"""Concurrency test for ``LupinArmCommandBridge._handle_set_torque`` (F4).

A rapid kinesthetic-start (disable) followed by a cancel (enable) sends two
``/lupin/arm/set_torque`` calls that, on a ReentrantCallbackGroup +
MultiThreadedExecutor, run on separate threads. If the handler isn't
serialized, the disable and enable orchestrations interleave on the shared
sub-clients and the arm's final torque state is nondeterministic.

This stubs the unavoidable external bits (the SetBool sub-RPC and the re-pin
publish) and asserts that two orchestrations never overlap.
"""
from __future__ import annotations

import threading
import time

import pytest

rclpy = pytest.importorskip("rclpy")

from std_srvs.srv import SetBool  # noqa: E402
from lupin_hmi.gripper_action_bridge import LupinArmCommandBridge  # noqa: E402


@pytest.fixture
def node():
    rclpy.init()
    n = LupinArmCommandBridge()
    try:
        yield n
    finally:
        n.destroy_node()
        rclpy.shutdown()


def _req(value: bool) -> SetBool.Request:
    r = SetBool.Request()
    r.data = value
    return r


def test_set_torque_orchestrations_do_not_interleave(node, monkeypatch):
    inflight = {"now": 0, "max": 0}
    guard = threading.Lock()

    def fake_call(client, value, optional=False):
        with guard:
            inflight["now"] += 1
            inflight["max"] = max(inflight["max"], inflight["now"])
        time.sleep(0.05)  # hold the "orchestration" open long enough to overlap
        with guard:
            inflight["now"] -= 1
        return True, "ok"

    monkeypatch.setattr(node, "_call_setbool", fake_call)
    monkeypatch.setattr(node, "_repin_to_current", lambda: None)

    threads = [
        threading.Thread(target=lambda v=v: node._handle_set_torque(_req(v), SetBool.Response()))
        for v in (True, False, True, False)
    ]
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    assert inflight["max"] == 1, (
        f"set_torque orchestrations overlapped (max concurrent={inflight['max']}) — "
        "handler is not serialized"
    )
