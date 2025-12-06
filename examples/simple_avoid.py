#!/usr/bin/env python3
"""
Simple obstacle avoidance using ultrasonic sensors

The rover will:
- Move forward when path is clear
- Turn away from obstacles when detected
- Stop if surrounded by obstacles

Usage:
    python3 simple_avoid.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Subscribe to front sensors
        self.create_subscription(Range, '/ultrasonic/front',
                                self.front_callback, 10)
        self.create_subscription(Range, '/ultrasonic/corner_left',
                                self.corner_left_callback, 10)
        self.create_subscription(Range, '/ultrasonic/corner_right',
                                self.corner_right_callback, 10)

        # Publisher for motor control
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Sensor distances
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')

        # Control parameters
        self.OBSTACLE_THRESHOLD = 1.0  # meters
        self.CRITICAL_THRESHOLD = 0.5  # meters
        self.FORWARD_SPEED = 0.3  # m/s
        self.TURN_SPEED = 0.5  # rad/s

        # Control loop at 10 Hz
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Obstacle avoider started!')
        self.get_logger().info(f'Obstacle threshold: {self.OBSTACLE_THRESHOLD}m')
        self.get_logger().info(f'Critical threshold: {self.CRITICAL_THRESHOLD}m')

    def front_callback(self, msg):
        self.front = msg.range

    def corner_left_callback(self, msg):
        self.left = msg.range

    def corner_right_callback(self, msg):
        self.right = msg.range

    def control_loop(self):
        """Main control logic - runs at 10 Hz"""
        twist = Twist()

        # Check if critical obstacle ahead
        if self.front < self.CRITICAL_THRESHOLD:
            # Emergency stop and turn
            self.get_logger().warn(f'CRITICAL: Obstacle at {self.front:.2f}m!')
            twist.linear.x = 0.0
            twist.angular.z = self.TURN_SPEED if self.left > self.right else -self.TURN_SPEED

        elif self.front < self.OBSTACLE_THRESHOLD:
            # Obstacle ahead - turn towards clearer side
            if self.left > self.right:
                self.get_logger().info(f'Obstacle ahead ({self.front:.2f}m) - Turning left')
                twist.linear.x = self.FORWARD_SPEED * 0.5  # Slow down while turning
                twist.angular.z = self.TURN_SPEED
            else:
                self.get_logger().info(f'Obstacle ahead ({self.front:.2f}m) - Turning right')
                twist.linear.x = self.FORWARD_SPEED * 0.5
                twist.angular.z = -self.TURN_SPEED

        else:
            # Path clear - move forward
            twist.linear.x = self.FORWARD_SPEED
            twist.angular.z = 0.0

        # Publish command
        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop robot on exit
        twist = Twist()
        node.cmd_pub.publish(twist)
        node.get_logger().info('Stopping rover...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
