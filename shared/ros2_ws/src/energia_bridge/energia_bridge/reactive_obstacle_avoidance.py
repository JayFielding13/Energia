#!/usr/bin/env python3
"""
Reactive Obstacle Avoidance using Vector Field Histogram (VFH) approach
Instead of stopping, the robot actively steers away from obstacles
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Bool
import numpy as np

class ReactiveObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('reactive_obstacle_avoidance')

        # Parameters
        self.declare_parameter('critical_distance', 0.4)    # meters - absolute minimum
        self.declare_parameter('influence_distance', 2.5)   # meters - obstacles affect steering up to this distance
        self.declare_parameter('obstacle_weight', 2.0)      # How strongly to avoid obstacles
        self.declare_parameter('sector_angle', 5.0)         # degrees - histogram resolution
        self.declare_parameter('num_sectors', 72)           # 360/5 = 72 sectors
        self.declare_parameter('speed_reduction_factor', 0.3)  # Slow down when obstacles near

        self.critical_dist = self.get_parameter('critical_distance').value
        self.influence_dist = self.get_parameter('influence_distance').value
        self.obstacle_weight = self.get_parameter('obstacle_weight').value
        self.sector_angle = np.radians(self.get_parameter('sector_angle').value)
        self.num_sectors = self.get_parameter('num_sectors').value
        self.speed_factor = self.get_parameter('speed_reduction_factor').value

        # Histogram for obstacle density
        self.histogram = np.zeros(self.num_sectors)

        # Sensor data storage
        self.lidar_data = None
        self.ultrasonic_readings = {}
        self.ultrasonic_positions = {
            'front': 0.0,           # 0°
            'corner_left': 0.785,   # 45°
            'side_left': 1.571,     # 90°
            'rear': 3.142,          # 180°
            'side_right': -1.571,   # -90°
            'corner_right': -0.785  # -45°
        }

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        for sensor_name in self.ultrasonic_positions.keys():
            self.create_subscription(
                Range,
                f'/ultrasonic/{sensor_name}',
                lambda msg, name=sensor_name: self.ultrasonic_callback(msg, name),
                10
            )

        # Publishers
        self.avoidance_vector_pub = self.create_publisher(Vector3, '/avoidance/steering_vector', 10)
        self.safe_directions_pub = self.create_publisher(Vector3, '/avoidance/safe_direction', 10)
        self.danger_pub = self.create_publisher(Bool, '/avoidance/critical_danger', 10)
        self.speed_scale_pub = self.create_publisher(Vector3, '/avoidance/speed_scale', 10)

        # Timer for processing
        self.create_timer(0.1, self.process_avoidance)  # 10 Hz

        self.get_logger().info('Reactive Obstacle Avoidance started')
        self.get_logger().info(f'  Critical distance: {self.critical_dist}m')
        self.get_logger().info(f'  Influence distance: {self.influence_dist}m')
        self.get_logger().info('  Strategy: Steer away, keep moving!')

    def lidar_callback(self, msg):
        """Store LiDAR scan"""
        self.lidar_data = msg

    def ultrasonic_callback(self, msg, sensor_name):
        """Store ultrasonic reading"""
        if 0.1 <= msg.range <= 6.0:  # Valid range
            self.ultrasonic_readings[sensor_name] = msg.range
        else:
            self.ultrasonic_readings[sensor_name] = float('inf')

    def build_histogram(self):
        """Build polar histogram of obstacle density"""
        self.histogram = np.zeros(self.num_sectors)

        # Add LiDAR data to histogram
        if self.lidar_data is not None:
            ranges = np.array(self.lidar_data.ranges)
            angles = np.linspace(
                self.lidar_data.angle_min,
                self.lidar_data.angle_max,
                len(ranges)
            )

            for angle, distance in zip(angles, ranges):
                if distance < self.lidar_data.range_min or distance > self.lidar_data.range_max:
                    continue

                if distance < self.influence_dist:
                    # Calculate influence based on distance
                    # Closer obstacles have more influence
                    influence = self.obstacle_weight * (1.0 - distance / self.influence_dist) ** 2

                    # Add to appropriate sector
                    sector = self.angle_to_sector(angle)
                    if 0 <= sector < self.num_sectors:
                        self.histogram[sector] += influence

        # Add ultrasonic data to histogram
        for sensor_name, distance in self.ultrasonic_readings.items():
            if distance < self.influence_dist:
                angle = self.ultrasonic_positions[sensor_name]
                influence = self.obstacle_weight * (1.0 - distance / self.influence_dist) ** 2

                sector = self.angle_to_sector(angle)
                if 0 <= sector < self.num_sectors:
                    # Ultrasonics have wider beam - spread to adjacent sectors
                    self.histogram[sector] += influence
                    self.histogram[(sector + 1) % self.num_sectors] += influence * 0.5
                    self.histogram[(sector - 1) % self.num_sectors] += influence * 0.5

    def angle_to_sector(self, angle):
        """Convert angle (radians) to histogram sector"""
        # Normalize angle to [0, 2π)
        normalized = (angle + 2 * np.pi) % (2 * np.pi)
        sector = int(normalized / self.sector_angle)
        return sector % self.num_sectors

    def sector_to_angle(self, sector):
        """Convert sector to angle (radians)"""
        return sector * self.sector_angle

    def find_safe_direction(self, desired_angle=0.0):
        """
        Find safest direction to move, considering desired direction

        Args:
            desired_angle: Where we want to go (radians, 0 = forward)

        Returns:
            best_angle: Safest direction to steer (radians)
            speed_scale: How much to slow down (0-1)
        """
        # Find sectors with low obstacle density
        sector_scores = 1.0 / (self.histogram + 0.1)  # Invert: low density = high score

        # Convert desired angle to sector
        desired_sector = self.angle_to_sector(desired_angle)

        # Weight sectors by how close they are to desired direction
        for i in range(self.num_sectors):
            sector_angle = self.sector_to_angle(i)
            angle_diff = abs(self.normalize_angle(sector_angle - desired_angle))

            # Prefer directions close to desired
            direction_weight = np.exp(-angle_diff**2 / (np.pi/4)**2)
            sector_scores[i] *= direction_weight

        # Find best sector
        best_sector = np.argmax(sector_scores)
        best_angle = self.sector_to_angle(best_sector)

        # Determine speed scaling based on closest obstacle
        min_distance = self.get_min_obstacle_distance()

        if min_distance < self.critical_dist:
            # Critical - slow way down but don't stop
            speed_scale = 0.1
        elif min_distance < self.influence_dist:
            # Moderate - scale speed with distance
            speed_scale = 0.2 + 0.8 * (min_distance - self.critical_dist) / (self.influence_dist - self.critical_dist)
        else:
            # Clear - full speed
            speed_scale = 1.0

        return best_angle, speed_scale

    def get_min_obstacle_distance(self):
        """Get distance to closest obstacle from all sensors"""
        min_dist = float('inf')

        # Check LiDAR
        if self.lidar_data is not None:
            valid_ranges = [r for r in self.lidar_data.ranges
                          if self.lidar_data.range_min <= r <= self.lidar_data.range_max]
            if valid_ranges:
                min_dist = min(min_dist, min(valid_ranges))

        # Check ultrasonics
        for distance in self.ultrasonic_readings.values():
            if distance < 6.0:  # Valid reading
                min_dist = min(min_dist, distance)

        return min_dist

    def normalize_angle(self, angle):
        """Normalize angle to [-π, π]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle

    def process_avoidance(self):
        """Main processing loop"""
        # Build obstacle density histogram
        self.build_histogram()

        # Find safe direction (forward by default)
        best_angle, speed_scale = self.find_safe_direction(desired_angle=0.0)

        # Convert to steering vector
        steering_x = np.cos(best_angle)
        steering_y = np.sin(best_angle)

        # Publish steering vector
        steering_msg = Vector3()
        steering_msg.x = steering_x
        steering_msg.y = steering_y
        steering_msg.z = 0.0
        self.avoidance_vector_pub.publish(steering_msg)

        # Publish safe direction (angle)
        safe_dir_msg = Vector3()
        safe_dir_msg.x = best_angle  # radians
        safe_dir_msg.y = speed_scale
        safe_dir_msg.z = 0.0
        self.safe_directions_pub.publish(safe_dir_msg)

        # Publish speed scale
        speed_msg = Vector3()
        speed_msg.x = speed_scale
        speed_msg.y = self.get_min_obstacle_distance()
        speed_msg.z = 0.0
        self.speed_scale_pub.publish(speed_msg)

        # Only publish critical danger if VERY close
        min_dist = self.get_min_obstacle_distance()
        critical = min_dist < self.critical_dist
        self.danger_pub.publish(Bool(data=critical))

        # Log status
        if critical:
            self.get_logger().warn(
                f'CRITICAL: {min_dist:.2f}m | Steer: {np.degrees(best_angle):.0f}° | Speed: {speed_scale:.0%}',
                throttle_duration_sec=0.5
            )
        elif min_dist < self.influence_dist:
            self.get_logger().info(
                f'Avoiding: {min_dist:.2f}m | Steer: {np.degrees(best_angle):.0f}° | Speed: {speed_scale:.0%}',
                throttle_duration_sec=2.0
            )

def main(args=None):
    rclpy.init(args=args)
    node = ReactiveObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
