"""Block until a TF transform resolves on the laptop's tf2 buffer.

Used as a launch sentinel between ``wait_for_map`` and the Nav2 lifecycle
start on hardware. Cold DDS-over-WiFi subscription to ``/tf`` takes ~5–10 s
on a fresh laptop launch to populate the buffer with the robot's
``odom → base_link`` and ``base_link → laser`` transforms. Nav2's costmap
activation does a single ``canTransform`` call with a short retry budget
and fails activation if TF isn't hot yet — so we gate Nav2's start on a
working external transform lookup instead.

Run via ``ros2 run lupin_bringup wait_for_tf [target] [source] [timeout]``.
Defaults: target=``odom``, source=``base_link``, timeout=``30.0`` seconds.
Exits 0 on success, 1 on timeout.
"""

import sys
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener


def main(argv=None):
    argv = list(sys.argv if argv is None else argv)

    target = argv[1] if len(argv) > 1 else 'odom'
    source = argv[2] if len(argv) > 2 else 'base_link'
    timeout = float(argv[3]) if len(argv) > 3 else 30.0

    rclpy.init()
    node = Node('lupin_wait_for_tf')
    buf = Buffer(cache_time=Duration(seconds=30.0))
    TransformListener(buf, node)

    node.get_logger().info(
        f'waiting up to {timeout:.1f}s for tf {target} -> {source}'
    )

    deadline = time.monotonic() + timeout
    success = False
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if buf.can_transform(target, source, Time()):
            success = True
            break

    if success:
        node.get_logger().info(
            f'tf {target} -> {source} is hot — Nav2 can safely activate'
        )
    else:
        node.get_logger().error(
            f'timed out waiting {timeout:.1f}s for tf {target} -> {source}'
        )

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
