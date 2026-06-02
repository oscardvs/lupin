"""slam_reset_node — operator-facing /lupin/nav/clear_map service.

slam_toolbox 2.6 (humble) does not expose a runtime "wipe map" service —
``/slam_toolbox/clear_changes`` only drops the localization edit buffer,
not the scan-built occupancy grid. The pragmatic v1 here SIGTERMs the
slam_toolbox process; the launch file pairs this with ``respawn=True``
on the slam_toolbox Node so it comes back up with an empty pose graph
and starts mapping from scratch.

Costmaps are cleared too so Nav2 doesn't keep stale lethal cells
inflated under the robot for the few seconds slam_toolbox is down.
"""

from __future__ import annotations

import subprocess
from typing import Iterable

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty, Trigger


# Substring to match in ``ps`` output. The Node action launches
# ``async_slam_toolbox_node`` regardless of whether sync or async mapper
# is selected, so this catches both online_async and online_sync.
SLAM_PROCESS_PATTERN = 'async_slam_toolbox_node'

# Best-effort costmap clears. Failure is logged, not propagated — the
# headline action ("erase map") is the slam_toolbox kill.
COSTMAP_CLEAR_SERVICES = (
    '/local_costmap/clear_entirely_local_costmap',
    '/global_costmap/clear_entirely_global_costmap',
)


class SlamResetNode(Node):
    def __init__(self) -> None:
        super().__init__('slam_reset_node')
        self._srv = self.create_service(
            Trigger, '/lupin/nav/clear_map', self._handle_clear_map,
        )
        self.get_logger().info(
            "slam_reset_node ready — call /lupin/nav/clear_map "
            "(std_srvs/srv/Trigger) to wipe the SLAM map.",
        )

    def _handle_clear_map(
        self,
        _request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self._clear_costmaps(COSTMAP_CLEAR_SERVICES)

        killed = self._sigterm_slam_toolbox()
        if killed:
            response.success = True
            response.message = (
                f'sent SIGTERM to {killed} slam_toolbox process(es); '
                'expect /map to repopulate after respawn (~5–10 s).'
            )
        else:
            response.success = False
            response.message = (
                f'no process matching "{SLAM_PROCESS_PATTERN}" found; '
                'slam_toolbox may not be running.'
            )
        return response

    def _sigterm_slam_toolbox(self) -> int:
        """SIGTERM every matching process. Returns the count killed."""
        try:
            res = subprocess.run(
                ['pgrep', '-f', SLAM_PROCESS_PATTERN],
                capture_output=True, text=True, check=False, timeout=2.0,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f'pgrep failed: {exc}')
            return 0
        pids = [p for p in res.stdout.split() if p.isdigit()]
        if not pids:
            return 0
        try:
            subprocess.run(
                ['kill', '-TERM', *pids],
                check=False, timeout=2.0,
            )
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            self.get_logger().error(f'kill failed: {exc}')
            return 0
        self.get_logger().warn(
            f'erase-map: SIGTERM sent to slam_toolbox pids={pids}',
        )
        return len(pids)

    def _clear_costmaps(self, services: Iterable[str]) -> None:
        for name in services:
            client = self.create_client(Empty, name)
            if not client.wait_for_service(timeout_sec=0.5):
                self.get_logger().warn(
                    f'costmap clear service {name} unavailable, skipping',
                )
                continue
            future = client.call_async(Empty.Request())
            # Fire-and-forget; we don't block the Trigger response on
            # costmap clears. Log on completion for diagnostics.
            future.add_done_callback(
                lambda f, n=name: self.get_logger().info(
                    f'cleared costmap via {n}'
                    if not f.exception() else
                    f'costmap clear {n} failed: {f.exception()}',
                ),
            )


def main() -> None:
    rclpy.init()
    node = SlamResetNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
