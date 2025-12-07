#!/usr/bin/env python3
"""
LiDAR Obstacle Detector
Processes 360° laser scans for obstacle detection
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
import numpy as np

class LidarObstacleDetector(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_detector')

        # Parameters
        self.declare_parameter('danger_distance', 0.8)   # meters
        self.declare_parameter('warning_distance', 2.0)  # meters
        self.declare_parameter('field_of_view', 120.0)   # degrees (front arc)
        self.declare_parameter('min_valid_range', 0.1)   # meters
        self.declare_parameter('max_valid_range', 12.0)  # meters

        self.danger_dist = self.get_parameter('danger_distance').value
        self.warning_dist = self.get_parameter('warning_distance').value
        self.fov = np.radians(self.get_parameter('field_of_view').value)
        self.min_range = self.get_parameter('min_valid_range').value
        self.max_range = self.get_parameter('max_valid_range').value

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Publishers
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacles/lidar/detected', 10)
        self.danger_zone_pub = self.create_publisher(Bool, '/obstacles/lidar/danger', 10)
        self.closest_obstacle_pub = self.create_publisher(Vector3, '/obstacles/lidar/closest', 10)

        self.get_logger().info('LiDAR Obstacle Detector started')
        self.get_logger().info(f'  Danger distance: {self.danger_dist}m')
        self.get_logger().info(f'  Warning distance: {self.warning_dist}m')
        self.get_logger().info(f'  Field of view: {np.degrees(self.fov):.0f}°')

    def scan_callback(self, msg):
        """Process LiDAR scan for obstacles"""
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # Filter for front arc only (adjustable FOV)
        half_fov = self.fov / 2
        front_indices = np.where(np.abs(angles) <= half_fov)[0]

        if len(front_indices) == 0:
            return

        front_ranges = ranges[front_indices]
        front_angles = angles[front_indices]

        # Remove invalid readings
        valid_mask = (front_ranges >= self.min_range) & (front_ranges <= self.max_range)
        valid_ranges = front_ranges[valid_mask]
        valid_angles = front_angles[valid_mask]

        if len(valid_ranges) == 0:
            # No obstacles detected
            self.obstacle_detected_pub.publish(Bool(data=False))
            self.danger_zone_pub.publish(Bool(data=False))
            return

        # Find closest obstacle
        min_idx = np.argmin(valid_ranges)
        min_distance = valid_ranges[min_idx]
        min_angle = valid_angles[min_idx]

        # Convert to cartesian (rover frame: x=forward, y=left)
        obstacle_x = min_distance * np.cos(min_angle)
        obstacle_y = min_distance * np.sin(min_angle)

        # Publish closest obstacle position
        obstacle_msg = Vector3()
        obstacle_msg.x = obstacle_x
        obstacle_msg.y = obstacle_y
        obstacle_msg.z = 0.0
        self.closest_obstacle_pub.publish(obstacle_msg)

        # Determine zones
        obstacle_detected = bool(min_distance < self.warning_dist)
        danger_zone = bool(min_distance < self.danger_dist)

        # Publish status
        self.obstacle_detected_pub.publish(Bool(data=obstacle_detected))
        self.danger_zone_pub.publish(Bool(data=danger_zone))

        # Log warnings
        if danger_zone:
            self.get_logger().warn(
                f'LiDAR DANGER: Obstacle at {min_distance:.2f}m, {np.degrees(min_angle):.1f}°',
                throttle_duration_sec=0.5
            )
        elif obstacle_detected:
            self.get_logger().info(
                f'LiDAR WARNING: Obstacle at {min_distance:.2f}m, {np.degrees(min_angle):.1f}°',
                throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
