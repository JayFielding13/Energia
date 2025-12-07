#!/usr/bin/env python3
"""
Timestamp Republisher Node

Subscribes to sensor topics from Gazebo (which have sim time timestamps)
and republishes them with wall clock timestamps. This allows proper
TF lookup when using wall clock for transforms.

Topics republished:
- /scan -> /scan (with wall clock timestamp)
- /imu -> /imu (with wall clock timestamp)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan, Imu
import copy


class TimestampRepublisher(Node):
    """Republishes sensor messages with wall clock timestamps"""

    def __init__(self):
        super().__init__('timestamp_republisher')

        # QoS for sensor data - match ros_gz_bridge output
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscribers (from Gazebo bridge with sim time)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            sensor_qos
        )

        # Publishers (with wall clock timestamps) - republish to same topic
        self.scan_pub = self.create_publisher(LaserScan, '/scan_wall', sensor_qos)
        self.imu_pub = self.create_publisher(Imu, '/imu_wall', sensor_qos)

        self.get_logger().info('Timestamp republisher started')
        self.get_logger().info('  /scan -> /scan_wall (wall clock)')
        self.get_logger().info('  /imu -> /imu_wall (wall clock)')

    def scan_callback(self, msg: LaserScan):
        """Republish laser scan with wall clock timestamp"""
        new_msg = copy.deepcopy(msg)
        new_msg.header.stamp = self.get_clock().now().to_msg()
        self.scan_pub.publish(new_msg)

    def imu_callback(self, msg: Imu):
        """Republish IMU with wall clock timestamp"""
        new_msg = copy.deepcopy(msg)
        new_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TimestampRepublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
