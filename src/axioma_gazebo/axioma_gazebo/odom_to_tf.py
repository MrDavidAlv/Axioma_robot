#!/usr/bin/env python3
"""Subscribe to /odom and publish TF odom -> base_footprint."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Accept both RELIABLE and BEST_EFFORT publishers
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, '/odom', self._odom_cb, qos)
        self.get_logger().info('odom_to_tf node started (BEST_EFFORT QoS)')

    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        # Force canonical frame names regardless of what Gz sends.
        #
        # The child is base_footprint, not base_link. base_link is already the
        # child of base_footprint through the fixed joint that comes from the
        # URDF, and a frame can only have one parent. Publishing odom ->
        # base_link here would fight robot_state_publisher for it.
        #
        # It is also the more honest frame: the DiffDrive plugin publishes
        # planar odometry with z = 0, which is the ground, and base_link sits
        # 2.58 mm below the ground on this robot.
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
