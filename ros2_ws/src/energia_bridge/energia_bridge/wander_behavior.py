#!/usr/bin/env python3
"""
Simple Wander Behavior for Rover with Safety Bubble

This node implements a wander behavior that maintains a safety bubble around
the rover using LiDAR data. The rover:
1. Drives forward when clear
2. Steers away from obstacles to maintain 1m clearance
3. Backs up only when very close to obstacles

Uses actual sensor data to determine which direction is safest.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import random
import time


class WanderBehavior(Node):
    def __init__(self):
        super().__init__('wander_behavior')

        # Parameters
        self.declare_parameter('forward_speed', 1.5)  # m/s (real robot top speed ~4.5 m/s)
        self.declare_parameter('turn_speed', 1.5)  # rad/s (real robot can rotate quickly)
        self.declare_parameter('safety_distance', 1.5)  # meters - safety bubble radius
        self.declare_parameter('danger_distance', 0.6)  # meters - trigger backup
        self.declare_parameter('enabled', True)

        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.enabled = self.get_parameter('enabled').value

        # State - LiDAR scan data
        self.latest_scan = None
        self.min_front_distance = float('inf')
        self.clearance_left = float('inf')
        self.clearance_right = float('inf')

        # Subscribe to LiDAR scan
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop timer (20 Hz)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('Wander behavior with safety bubble started')
        self.get_logger().info(f'  Forward speed: {self.forward_speed} m/s')
        self.get_logger().info(f'  Turn speed: {self.turn_speed} rad/s')
        self.get_logger().info(f'  Safety bubble: {self.safety_distance}m')
        self.get_logger().info(f'  Danger distance: {self.danger_distance}m')

    def lidar_callback(self, msg):
        """Process LiDAR scan to extract clearance information"""
        self.latest_scan = msg

        # Convert ranges to numpy array, replacing inf with max_range
        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max

        # Calculate angle for each beam
        num_beams = len(ranges)
        angles = msg.angle_min + np.arange(num_beams) * msg.angle_increment

        # Front sector (±30 degrees)
        front_mask = np.abs(angles) < np.radians(30)
        self.min_front_distance = np.min(ranges[front_mask]) if np.any(front_mask) else msg.range_max

        # Left sector (30 to 150 degrees)
        left_mask = (angles > np.radians(30)) & (angles < np.radians(150))
        self.clearance_left = np.min(ranges[left_mask]) if np.any(left_mask) else msg.range_max

        # Right sector (-150 to -30 degrees)
        right_mask = (angles < np.radians(-30)) & (angles > np.radians(-150))
        self.clearance_right = np.min(ranges[right_mask]) if np.any(right_mask) else msg.range_max

    def control_loop(self):
        """
        Safety bubble control: maintains clearance around rover
        Uses actual LiDAR data to steer intelligently
        Now includes in-place rotation when trapped
        """
        if not self.enabled or self.latest_scan is None:
            return

        cmd = Twist()

        # Check if we're too close (danger zone)
        if self.min_front_distance < self.danger_distance:
            # Very close obstacle detected
            # Check if we're trapped (obstacles on both sides too)
            if self.clearance_left < self.safety_distance and self.clearance_right < self.safety_distance:
                # Trapped! Rotate in place toward clearest side (no forward motion)
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed * 1.2 if self.clearance_left > self.clearance_right else -self.turn_speed * 1.2
                self.get_logger().info(f'Trapped - rotating in place! L:{self.clearance_left:.2f}m R:{self.clearance_right:.2f}m', throttle_duration_sec=1.0)
            else:
                # Not fully trapped - back up while turning
                cmd.linear.x = -0.2
                cmd.angular.z = self.turn_speed if self.clearance_left > self.clearance_right else -self.turn_speed

        # Check if we're within safety bubble
        elif self.min_front_distance < self.safety_distance:
            # Inside safety bubble - decide whether to turn in place or slow down
            proximity_ratio = self.min_front_distance / self.safety_distance

            # If very close to safety boundary, consider in-place rotation
            if proximity_ratio < 0.5:
                # Very close - rotate in place if needed
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed if self.clearance_left > self.clearance_right else -self.turn_speed
            else:
                # Moderate distance - slow down and turn
                cmd.linear.x = self.forward_speed * proximity_ratio * 0.5
                # Turn aggressively toward the side with more clearance
                if self.clearance_left > self.clearance_right:
                    cmd.angular.z = self.turn_speed  # Turn left
                else:
                    cmd.angular.z = -self.turn_speed  # Turn right

        # Path is clear - drive forward
        else:
            cmd.linear.x = self.forward_speed

            # Add small bias to steer slightly toward more open side
            clearance_diff = self.clearance_left - self.clearance_right
            if abs(clearance_diff) > 0.5:  # Significant difference
                cmd.angular.z = 0.2 * self.turn_speed * np.sign(clearance_diff)

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WanderBehavior()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the rover
        stop_cmd = Twist()
        node.cmd_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
