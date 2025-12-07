#!/usr/bin/env python3
"""
Odometry to TF Broadcaster

Subscribes to /odom and broadcasts the odom -> base_link transform.
This is needed because bridging TF directly from Gazebo Harmonic causes
timestamp synchronization issues with robot_state_publisher.

The odometry message from Gazebo contains the pose of base_link in the
odom frame, which we broadcast as a TF transform.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTF(Node):
    """Converts odometry messages to TF broadcasts"""

    def __init__(self):
        super().__init__('odom_to_tf')

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Use BEST_EFFORT QoS to match ros_gz_bridge output
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribe to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            odom_qos
        )

        self.get_logger().info('Odom to TF broadcaster started')

    def odom_callback(self, msg: Odometry):
        """Broadcast TF from odometry message"""
        t = TransformStamped()

        # Use current ROS time (wall clock when use_sim_time=false)
        # This avoids timestamp conflicts with robot_state_publisher
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = msg.header.frame_id  # Should be 'odom'
        # Use base_footprint to match Gazebo diff-drive convention
        t.child_frame_id = 'base_footprint'

        # Copy pose from odometry
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomToTF()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
