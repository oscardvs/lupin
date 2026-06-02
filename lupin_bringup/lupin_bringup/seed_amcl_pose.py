"""Continuous /amcl_pose republisher for the SLAM-based sim chain.

The greenhouse world has no pre-built map, so the sim runs slam_toolbox in
online-mapping mode — which drops AMCL out of the chain. But the v2 mission
orchestrator gates on /amcl_pose exactly as it does against real AMCL on
hardware: PREPARE.LOCALIZING waits for a low-covariance pose, and each approach
leg re-checks AMCL drift.

This node is AMCL's sim stand-in. At `rate_hz` it looks up the map->base_link
transform (slam_toolbox map->odom ∘ planar_move odom->base_link) and republishes
it as a low-covariance PoseWithCovarianceStamped on /amcl_pose. Two reasons it is
a *continuous* node, not the old one-shot seed:

  1. The orchestrator only subscribes when a mission starts (often minutes after
     bringup). TRANSIENT_LOCAL latching only persists while the PUBLISHER is
     alive, so a publish-then-exit seed leaves a late subscriber with nothing —
     localization never converges → FAULT. (That was the bug.)
  2. A static origin seed would make the per-leg drift gate see the moving robot
     as wildly drifted. Tracking the real map->base_link pose keeps observations
     and the drift gate correct.

Hardware doesn't run this — real AMCL is there. Keep `use_sim_time:=true` so the
stamps are on the sim clock (the orchestrator may reject stale wall-clock poses).

Run via ``ros2 run lupin_bringup seed_amcl_pose``.
"""

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy,
)
import tf2_ros

# 6x6 row-major covariance: tight x/y/yaw (well under the orchestrator's 0.25
# threshold), huge on the unobservable 2D-nav axes.
_COV = [0.0] * 36
_COV[0] = 0.01       # x
_COV[7] = 0.01       # y
_COV[14] = 99999.0   # z
_COV[21] = 99999.0   # roll
_COV[28] = 99999.0   # pitch
_COV[35] = 0.01      # yaw


class AmclSeed(Node):
    def __init__(self):
        super().__init__('lupin_amcl_seed')
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        rate = float(self.declare_parameter('rate_hz', 10.0).value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', qos)
        self._published = False
        self._warned = False
        self.create_timer(1.0 / rate, self._tick)

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except tf2_ros.TransformException:
            if not self._warned:
                self.get_logger().info(
                    f'waiting for {self.map_frame}->{self.base_frame} TF…')
                self._warned = True
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = tf.transform.rotation
        msg.pose.covariance = list(_COV)
        self.pub.publish(msg)
        if not self._published:
            self.get_logger().info(
                f'publishing /amcl_pose from {self.map_frame}->{self.base_frame} '
                f'TF (AMCL stand-in for the SLAM sim)')
            self._published = True


def main():
    rclpy.init()
    node = AmclSeed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
