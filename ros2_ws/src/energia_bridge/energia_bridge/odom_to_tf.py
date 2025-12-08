#!/usr/bin/env python3
"""
Odometry to TF Publisher

Converts odometry messages to TF transforms.
Publishes the transform from odom -> base_footprint.

This is useful when bridging from Gazebo where direct TF bridging
can cause timestamp issues.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    """Convert odometry to TF transforms."""

    def __init__(self):
        super().__init__('odom_to_tf')

        # Declare parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.get_logger().info(
            f'Publishing TF: {self.odom_frame} -> {self.base_frame}')

    def odom_callback(self, msg):
        """Convert odometry to TF and broadcast."""
        t = TransformStamped()

        # Use current time instead of message timestamp to avoid timing issues
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame

        # Copy translation
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        # Copy rotation
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTf()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
