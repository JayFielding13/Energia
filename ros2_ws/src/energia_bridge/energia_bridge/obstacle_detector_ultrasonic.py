#!/usr/bin/env python3
"""
Ultrasonic Obstacle Detector
Monitors 6 ultrasonic sensors and publishes obstacle warnings
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
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

        # QoS profile for sensor data (match bridge's BEST_EFFORT)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers for each ultrasonic sensor
        # Create individual subscriptions using explicit callback methods
        # We create separate methods to avoid lambda/closure issues with rclpy message serialization
        self.create_subscription(Range, '/ultrasonic/front', self._callback_front, qos_profile)
        self.create_subscription(Range, '/ultrasonic/corner_left', self._callback_corner_left, qos_profile)
        self.create_subscription(Range, '/ultrasonic/side_left', self._callback_side_left, qos_profile)
        self.create_subscription(Range, '/ultrasonic/rear', self._callback_rear, qos_profile)
        self.create_subscription(Range, '/ultrasonic/side_right', self._callback_side_right, qos_profile)
        self.create_subscription(Range, '/ultrasonic/corner_right', self._callback_corner_right, qos_profile)

        # Publishers
        self.obstacle_detected_pub = self.create_publisher(Bool, '/obstacles/ultrasonic/detected', 10)
        self.danger_zone_pub = self.create_publisher(Bool, '/obstacles/ultrasonic/danger', 10)
        self.closest_obstacle_pub = self.create_publisher(Vector3, '/obstacles/ultrasonic/closest', 10)

        # Timer for processing
        self.create_timer(0.1, self.process_obstacles)  # 10 Hz

        self.get_logger().info('Ultrasonic Obstacle Detector started')
        self.get_logger().info(f'  Danger distance: {self.danger_dist}m')
        self.get_logger().info(f'  Warning distance: {self.warning_dist}m')

    # Individual callbacks for each sensor (avoids lambda/closure issues)
    # Each callback stores data directly to avoid any closure/argument passing issues
    def _callback_front(self, msg):
        if self.min_range <= msg.range <= self.max_range:
            self.readings['front'] = msg.range
            self.last_update_time['front'] = self.get_clock().now()
        else:
            self.readings['front'] = float('inf')

    def _callback_corner_left(self, msg):
        if self.min_range <= msg.range <= self.max_range:
            self.readings['corner_left'] = msg.range
            self.last_update_time['corner_left'] = self.get_clock().now()
        else:
            self.readings['corner_left'] = float('inf')

    def _callback_side_left(self, msg):
        if self.min_range <= msg.range <= self.max_range:
            self.readings['side_left'] = msg.range
            self.last_update_time['side_left'] = self.get_clock().now()
        else:
            self.readings['side_left'] = float('inf')

    def _callback_rear(self, msg):
        if self.min_range <= msg.range <= self.max_range:
            self.readings['rear'] = msg.range
            self.last_update_time['rear'] = self.get_clock().now()
        else:
            self.readings['rear'] = float('inf')

    def _callback_side_right(self, msg):
        if self.min_range <= msg.range <= self.max_range:
            self.readings['side_right'] = msg.range
            self.last_update_time['side_right'] = self.get_clock().now()
        else:
            self.readings['side_right'] = float('inf')

    def _callback_corner_right(self, msg):
        if self.min_range <= msg.range <= self.max_range:
            self.readings['corner_right'] = msg.range
            self.last_update_time['corner_right'] = self.get_clock().now()
        else:
            self.readings['corner_right'] = float('inf')

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
        obstacle_detected = bool(min_distance < self.warning_dist)
        danger_zone = bool(min_distance < self.danger_dist)

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
