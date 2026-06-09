"""Integration test: the erase-map service clears costmaps with Nav2's type.

``slam_reset_node`` best-effort clears the Nav2 costmaps when the operator wipes
the SLAM map. Nav2 serves ``*/clear_entirely_*_costmap`` as
``nav2_msgs/srv/ClearEntireCostmap`` — NOT ``std_srvs/srv/Empty``. If the node
uses the wrong client type, the client/server types mismatch and the clear
silently no-ops.

This test stands up fake costmap servers of the CORRECT Nav2 type, fires
``/lupin/nav/clear_map``, and asserts both servers were actually invoked. The
slam_toolbox SIGTERM path is neutered so the test never touches real processes.
"""

import threading
import time

import pytest
import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

from lupin_navigation.slam_reset_node import (
    COSTMAP_CLEAR_SERVICES,
    SlamResetNode,
)


@pytest.fixture
def rclpy_context():
    """Init/shutdown rclpy around each test."""
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def test_clear_map_invokes_costmaps_with_nav2_type(rclpy_context) -> None:
    """clear_map must reach ClearEntireCostmap-typed servers (not Empty)."""
    hits: list[str] = []

    server = rclpy.create_node('fake_costmap_servers')

    def make_cb(name):
        def _cb(_req, resp):
            hits.append(name)
            return resp
        return _cb

    for srv_name in COSTMAP_CLEAR_SERVICES:
        server.create_service(ClearEntireCostmap, srv_name, make_cb(srv_name))

    reset = SlamResetNode()
    # Never SIGTERM real slam_toolbox processes from a unit test.
    reset._sigterm_slam_toolbox = lambda: 0  # type: ignore[method-assign]

    caller = reset.create_client(Trigger, '/lupin/nav/clear_map')

    executor = SingleThreadedExecutor()
    for node in (server, reset):
        executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        # Let intra-process discovery settle so the handler's internal
        # wait_for_service(0.5) reliably sees the fake servers.
        assert caller.wait_for_service(timeout_sec=5.0), \
            '/lupin/nav/clear_map never came up'
        time.sleep(1.5)

        future = caller.call_async(Trigger.Request())
        deadline = time.time() + 5.0
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        assert future.done(), 'clear_map did not respond'

        # Costmap clears are fire-and-forget inside the handler; give the
        # executor a beat to deliver them to the fake servers.
        deadline = time.time() + 3.0
        while set(hits) != set(COSTMAP_CLEAR_SERVICES) and time.time() < deadline:
            time.sleep(0.05)

        assert set(hits) == set(COSTMAP_CLEAR_SERVICES), (
            'expected both costmaps cleared via ClearEntireCostmap, '
            f'got {sorted(hits)}'
        )
    finally:
        executor.shutdown()
        reset.destroy_node()
        server.destroy_node()
