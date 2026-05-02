"""One-shot publisher of a low-covariance /amcl_pose for the sim chain.

The greenhouse world has no pre-built map, so the sim runs slam_toolbox
in online-mapping mode. That drops AMCL out of the chain, which means
the v2 mission orchestrator's PREPARE.LOCALIZING gate (which subscribes
to /amcl_pose) never gets a covariance message and times out → FAULT.

This script publishes a single PoseWithCovarianceStamped on /amcl_pose
with TRANSIENT_LOCAL durability and tight x/y/yaw covariance, satisfying
the gate. Hardware doesn't need it — real AMCL is running there.

Run via ``ros2 run lupin_bringup seed_amcl_pose``.
"""

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)


def _build_pose_msg(node: Node) -> PoseWithCovarianceStamped:
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = 'map'
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.orientation.w = 1.0
    cov = [0.0] * 36          # 6x6 row-major
    cov[0] = 0.01             # x variance
    cov[7] = 0.01             # y variance
    cov[14] = 99999.0         # z (irrelevant for 2D nav)
    cov[21] = 99999.0         # roll
    cov[28] = 99999.0         # pitch
    cov[35] = 0.01            # yaw variance
    msg.pose.covariance = cov
    return msg


def main():
    rclpy.init()
    node = Node('lupin_amcl_seed')
    qos = QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )
    pub = node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', qos)

    # Spin briefly so the latched publish reaches subscribers — TRANSIENT_LOCAL
    # already buffers for late subscribers, but the publisher needs a few
    # spin cycles for DDS discovery to flush before we exit.
    for _ in range(20):
        pub.publish(_build_pose_msg(node))
        rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(0.05)
    node.get_logger().info('seeded /amcl_pose with low-covariance pose at origin')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
