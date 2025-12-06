#!/usr/bin/env python3
"""
Ultrasonic Obstacle Detector
Monitors 6 ultrasonic sensors and publishes obstacle warnings
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import Bool
from geometry_msgs.msg import Vector3
import numpy as np

class UltrasonicObstacleDetector(Node):
    def __init__(self):
        super().__init__('ultrasonic_obstacle_detector')

        # Configurable thresholds
        self.declare_parameter('danger_distance', 0.5)  # meters
        self.declare_parameter('warning_distance', 1.0)  # meters
        self.declare_parameter('min_valid_range', 0.2)   # meters
        self.declare_parameter('max_valid_range', 6.0)   # meters

        self.danger_dist = self.get_parameter('danger_distance').value
        self.warning_dist = self.get_parameter('warning_distance').value
        self.min_range = self.get_parameter('min_valid_range').value
        self.max_range = self.get_parameter('max_valid_range').value

        # Sensor positions (angles in radians from front)
        self.sensors = {
            'front': 0.0,           # 0°
            'corner_left': 0.785,   # 45°
            'side_left': 1.571,     # 90°
            'rear': 3.142,          # 180°
            'side_right': -1.571,   # -90°
            'corner_right': -0.785  # -45°
        }

        # Store latest readings
        self.readings = {name: float('inf') for name in self.sensors.keys()}
        self.last_update_time = {name: None for name in self.sensors.keys()}

        # Subscribers for each ultrasonic sensor
        for sensor_name in self.sensors.keys():
            self.create_subscription(
                Range,
                f'/ultrasonic/{sensor_name}',
                lambda msg, name=sensor_name: self.sensor_callback(msg, name),
                10
            )

        # Publishers
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacles/ultrasonic/detected', 10)
        self.danger_zone_pub = self.create_publisher(Bool, '/obstacles/ultrasonic/danger', 10)
        self.closest_obstacle_pub = self.create_publisher(Vector3, '/obstacles/ultrasonic/closest', 10)

        # Timer for processing
        self.create_timer(0.1, self.process_obstacles)  # 10 Hz

        self.get_logger().info('Ultrasonic Obstacle Detector started')
        self.get_logger().info(f'  Danger distance: {self.danger_dist}m')
        self.get_logger().info(f'  Warning distance: {self.warning_dist}m')

    def sensor_callback(self, msg, sensor_name):
        """Store sensor reading if valid"""
        if self.min_range <= msg.range <= self.max_range:
            self.readings[sensor_name] = msg.range
            self.last_update_time[sensor_name] = self.get_clock().now()
        else:
            # Out of range - treat as no obstacle
            self.readings[sensor_name] = float('inf')

    def process_obstacles(self):
        """Analyze sensor readings and publish obstacle status"""
        # Check for stale data
        now = self.get_clock().now()
        for sensor_name, last_time in self.last_update_time.items():
            if last_time is not None:
                age = (now - last_time).nanoseconds / 1e9
                if age > 1.0:  # 1 second timeout
                    self.readings[sensor_name] = float('inf')

        # Find closest obstacle and its location
        min_distance = min(self.readings.values())
        closest_sensor = min(self.readings, key=self.readings.get)

        # Determine which zone
        obstacle_detected = min_distance < self.warning_dist
        danger_zone = min_distance < self.danger_dist

        # Publish status
        self.obstacle_detected_pub.publish(Bool(data=obstacle_detected))
        self.danger_zone_pub.publish(Bool(data=danger_zone))

        # Publish closest obstacle position (in rover frame)
        if min_distance < self.warning_dist:
            angle = self.sensors[closest_sensor]
            obstacle_x = min_distance * np.cos(angle)
            obstacle_y = min_distance * np.sin(angle)

            closest_msg = Vector3()
            closest_msg.x = obstacle_x
            closest_msg.y = obstacle_y
            closest_msg.z = 0.0
            self.closest_obstacle_pub.publish(closest_msg)

        # Log warnings
        if danger_zone:
            self.get_logger().warn(
                f'DANGER: Obstacle at {min_distance:.2f}m on {closest_sensor}',
                throttle_duration_sec=1.0
            )
        elif obstacle_detected:
            self.get_logger().info(
                f'WARNING: Obstacle at {min_distance:.2f}m on {closest_sensor}',
                throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicObstacleDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
