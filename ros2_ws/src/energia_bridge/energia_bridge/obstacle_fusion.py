#!/usr/bin/env python3
"""
Obstacle Sensor Fusion
Combines LiDAR and Ultrasonic obstacle detection
Uses OR logic: if EITHER sensor detects danger, trigger emergency stop
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist, Vector3

class ObstacleFusion(Node):
    def __init__(self):
        super().__init__('obstacle_fusion')

        # State tracking
        self.lidar_danger = False
        self.ultrasonic_danger = False
        self.lidar_warning = False
        self.ultrasonic_warning = False
        self.lidar_obstacle_pos = None
        self.ultrasonic_obstacle_pos = None

        # Subscribers
        self.create_subscription(Bool, '/obstacles/lidar/danger', self.lidar_danger_cb, 10)
        self.create_subscription(Bool, '/obstacles/lidar/detected', self.lidar_warning_cb, 10)
        self.create_subscription(Vector3, '/obstacles/lidar/closest', self.lidar_pos_cb, 10)

        self.create_subscription(Bool, '/obstacles/ultrasonic/danger', self.ultrasonic_danger_cb, 10)
        self.create_subscription(Bool, '/obstacles/ultrasonic/detected', self.ultrasonic_warning_cb, 10)
        self.create_subscription(Vector3, '/obstacles/ultrasonic/closest', self.ultrasonic_pos_cb, 10)

        # Publishers
        self.danger_pub = self.create_publisher(Bool, '/obstacles/danger', 10)
        self.warning_pub = self.create_publisher(Bool, '/obstacles/warning', 10)
        self.closest_obstacle_pub = self.create_publisher(Vector3, '/obstacles/closest', 10)

        # Timer for fusion logic
        self.create_timer(0.1, self.fusion_callback)  # 10 Hz

        self.get_logger().info('Obstacle Fusion started (LiDAR + Ultrasonic)')

    def lidar_danger_cb(self, msg):
        self.lidar_danger = msg.data

    def lidar_warning_cb(self, msg):
        self.lidar_warning = msg.data

    def lidar_pos_cb(self, msg):
        self.lidar_obstacle_pos = msg

    def ultrasonic_danger_cb(self, msg):
        self.ultrasonic_danger = msg.data

    def ultrasonic_warning_cb(self, msg):
        self.ultrasonic_warning = msg.data

    def ultrasonic_pos_cb(self, msg):
        self.ultrasonic_obstacle_pos = msg

    def fusion_callback(self):
        """Combine sensor inputs using OR logic"""
        # If EITHER sensor sees danger, it's danger
        danger = self.lidar_danger or self.ultrasonic_danger
        warning = self.lidar_warning or self.ultrasonic_warning

        # Publish fused status
        self.danger_pub.publish(Bool(data=danger))
        self.warning_pub.publish(Bool(data=warning))

        # Determine closest obstacle from both sensors
        if self.lidar_obstacle_pos is not None and self.ultrasonic_obstacle_pos is not None:
            # Both sensors see obstacles - pick closer one
            lidar_dist = (self.lidar_obstacle_pos.x**2 + self.lidar_obstacle_pos.y**2)**0.5
            ultra_dist = (self.ultrasonic_obstacle_pos.x**2 + self.ultrasonic_obstacle_pos.y**2)**0.5

            if ultra_dist < lidar_dist:
                self.closest_obstacle_pub.publish(self.ultrasonic_obstacle_pos)
            else:
                self.closest_obstacle_pub.publish(self.lidar_obstacle_pos)
        elif self.lidar_obstacle_pos is not None:
            self.closest_obstacle_pub.publish(self.lidar_obstacle_pos)
        elif self.ultrasonic_obstacle_pos is not None:
            self.closest_obstacle_pub.publish(self.ultrasonic_obstacle_pos)

        # Log danger events
        if danger:
            source = []
            if self.lidar_danger:
                source.append('LiDAR')
            if self.ultrasonic_danger:
                source.append('Ultrasonic')

            self.get_logger().warn(
                f'DANGER ZONE: {", ".join(source)} detected obstacle',
                throttle_duration_sec=1.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFusion()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
