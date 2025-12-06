#!/usr/bin/env python3
"""
Predictive Obstacle Avoidance with Goal-Aware Path Planning

Instead of reactive "panic stops", this node:
1. Looks ahead toward the goal direction (up to 10m+)
2. Detects obstacles early in the path corridor
3. Calculates a smooth steering adjustment to curve around obstacles
4. Provides a blended heading that the navigator can follow directly

The output is a steering adjustment that smoothly increases as obstacles
get closer, allowing the rover to gracefully curve around obstacles while
maintaining progress toward the waypoint.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Bool, Float32
from nav_msgs.msg import Odometry
import numpy as np
import math


class PredictiveObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('reactive_obstacle_avoidance')

        # Parameters - Planning distances
        self.declare_parameter('planning_distance', 8.0)      # Start planning at this distance
        self.declare_parameter('critical_distance', 0.5)      # Emergency slowdown distance
        self.declare_parameter('robot_width', 0.8)            # Rover width for clearance
        self.declare_parameter('safety_margin', 0.3)          # Extra clearance on each side

        # Parameters - Steering behavior
        self.declare_parameter('max_steering_angle', 45.0)    # Max degrees to steer around obstacle
        self.declare_parameter('steering_smoothness', 2.0)    # Higher = more gradual steering
        self.declare_parameter('goal_weight', 0.7)            # How much to favor goal direction (0-1)

        # Parameters - Speed control
        self.declare_parameter('min_speed_scale', 0.15)       # Minimum speed when very close
        self.declare_parameter('full_speed_distance', 5.0)    # Distance for full speed

        # Get parameters
        self.planning_dist = self.get_parameter('planning_distance').value
        self.critical_dist = self.get_parameter('critical_distance').value
        self.robot_width = self.get_parameter('robot_width').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.max_steer = math.radians(self.get_parameter('max_steering_angle').value)
        self.steer_smooth = self.get_parameter('steering_smoothness').value
        self.goal_weight = self.get_parameter('goal_weight').value
        self.min_speed = self.get_parameter('min_speed_scale').value
        self.full_speed_dist = self.get_parameter('full_speed_distance').value

        # Clearance corridor half-width
        self.corridor_half_width = (self.robot_width / 2) + self.safety_margin

        # Sensor data
        self.latest_scan = None
        self.ultrasonic_readings = {}
        self.ultrasonic_positions = {
            'front': 0.0,
            'corner_left': math.radians(45),
            'side_left': math.radians(90),
            'rear': math.radians(180),
            'side_right': math.radians(-90),
            'corner_right': math.radians(-45)
        }

        # Current state
        self.current_heading = 0.0
        self.goal_direction = 0.0  # Direction to waypoint (robot frame)

        # Obstacle tracking
        self.obstacles_left = []   # (distance, angle) pairs
        self.obstacles_right = []
        self.obstacles_front = []
        self.closest_obstacle_dist = float('inf')

        # Subscribers - match publisher's RELIABLE QoS
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            durability=DurabilityPolicy.VOLATILE
        )
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.lidar_callback, scan_qos)

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Subscribe to goal direction from waypoint navigator
        self.goal_sub = self.create_subscription(
            Float32, '/navigation/goal_direction', self.goal_direction_callback, 10)

        # Ultrasonic sensors - disabled for simulation (topics don't exist)
        # TODO: Re-enable for real hardware when ultrasonic sensors are connected
        # for sensor_name in self.ultrasonic_positions.keys():
        #     self.create_subscription(
        #         Range,
        #         f'/ultrasonic/{sensor_name}',
        #         lambda msg, name=sensor_name: self.ultrasonic_callback(msg, name),
        #         10
        #     )

        # Publishers
        # Steering adjustment (radians) - positive = steer left, negative = steer right
        self.steering_pub = self.create_publisher(Float32, '/avoidance/steering_adjustment', 10)

        # Speed scale (0-1)
        self.speed_pub = self.create_publisher(Float32, '/avoidance/speed_scale', 10)

        # Critical danger flag (for emergency stops only)
        self.danger_pub = self.create_publisher(Bool, '/avoidance/critical_danger', 10)

        # Debug: obstacle info
        self.obstacle_info_pub = self.create_publisher(Vector3, '/avoidance/obstacle_info', 10)

        # Legacy publishers for compatibility
        self.safe_dir_pub = self.create_publisher(Vector3, '/avoidance/safe_direction', 10)

        # Timer for processing
        self.create_timer(0.1, self.process_avoidance)  # 10 Hz

        self.get_logger().info('Predictive Obstacle Avoidance started')
        self.get_logger().info(f'  Planning distance: {self.planning_dist}m')
        self.get_logger().info(f'  Critical distance: {self.critical_dist}m')
        self.get_logger().info(f'  Corridor width: {self.corridor_half_width * 2:.2f}m')
        self.get_logger().info('  Strategy: Early detection, smooth curves!')

    def lidar_callback(self, msg):
        """Store LiDAR scan"""
        self.latest_scan = msg

    def odom_callback(self, msg):
        """Update current heading from odometry"""
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)

    def goal_direction_callback(self, msg):
        """Update goal direction (angle in robot frame)"""
        self.goal_direction = msg.data

    def ultrasonic_callback(self, msg, sensor_name):
        """Store ultrasonic reading"""
        if 0.1 <= msg.range <= 6.0:
            self.ultrasonic_readings[sensor_name] = msg.range
        else:
            self.ultrasonic_readings[sensor_name] = float('inf')

    def analyze_obstacles(self):
        """
        Analyze sensor data to find obstacles and classify by position.
        Returns obstacles grouped by left/right/front relative to goal direction.
        """
        self.obstacles_left = []
        self.obstacles_right = []
        self.obstacles_front = []
        self.closest_obstacle_dist = float('inf')

        # Define corridor around goal direction
        goal_dir = self.goal_direction
        corridor_angle = math.atan2(self.corridor_half_width, self.planning_dist)

        # Process LiDAR data
        if self.latest_scan is not None and len(self.latest_scan.ranges) > 0:
            scan = self.latest_scan
            angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))

            for i, distance in enumerate(scan.ranges):
                # Skip invalid readings
                if not np.isfinite(distance):
                    continue
                if distance < scan.range_min or distance > scan.range_max:
                    continue
                if distance > self.planning_dist:
                    continue

                angle = angles[i]  # Angle in robot frame

                # Track closest obstacle
                if distance < self.closest_obstacle_dist:
                    self.closest_obstacle_dist = distance

                # Calculate angle relative to goal direction
                angle_from_goal = self.normalize_angle(angle - goal_dir)

                # Check if obstacle is in the path corridor
                # Use perpendicular distance to path line
                perp_distance = abs(distance * math.sin(angle_from_goal))
                forward_distance = distance * math.cos(angle_from_goal)

                # Only consider obstacles ahead of us (in goal direction)
                if forward_distance < 0:
                    continue

                # Is this obstacle blocking our path?
                if perp_distance < self.corridor_half_width:
                    # Obstacle in corridor - classify by which side
                    if angle_from_goal > 0:
                        self.obstacles_left.append((distance, angle))
                    elif angle_from_goal < 0:
                        self.obstacles_right.append((distance, angle))
                    else:
                        self.obstacles_front.append((distance, angle))

        # Process ultrasonic data
        for sensor_name, distance in self.ultrasonic_readings.items():
            if distance > self.planning_dist:
                continue

            angle = self.ultrasonic_positions[sensor_name]

            if distance < self.closest_obstacle_dist:
                self.closest_obstacle_dist = distance

            angle_from_goal = self.normalize_angle(angle - goal_dir)
            perp_distance = abs(distance * math.sin(angle_from_goal))
            forward_distance = distance * math.cos(angle_from_goal)

            if forward_distance < 0:
                continue

            if perp_distance < self.corridor_half_width:
                if angle_from_goal > 0:
                    self.obstacles_left.append((distance, angle))
                elif angle_from_goal < 0:
                    self.obstacles_right.append((distance, angle))
                else:
                    self.obstacles_front.append((distance, angle))

    def calculate_steering_adjustment(self) -> float:
        """
        Calculate how much to steer away from obstacles.

        Returns:
            Steering adjustment in radians (positive = left, negative = right)
        """
        # No obstacles in corridor - no adjustment needed
        all_obstacles = self.obstacles_left + self.obstacles_right + self.obstacles_front
        if not all_obstacles:
            return 0.0

        # Find the closest obstacle in the corridor
        min_dist = min(obs[0] for obs in all_obstacles)

        # Calculate steering magnitude based on distance
        # Closer obstacle = stronger steering
        # At planning_dist: 0 steering, at critical_dist: max steering
        if min_dist >= self.planning_dist:
            steer_magnitude = 0.0
        elif min_dist <= self.critical_dist:
            steer_magnitude = self.max_steer
        else:
            # Smooth interpolation using inverse square for more aggressive early response
            normalized_dist = (min_dist - self.critical_dist) / (self.planning_dist - self.critical_dist)
            # Use power function for non-linear response (steers more as obstacle gets closer)
            steer_magnitude = self.max_steer * (1.0 - normalized_dist ** self.steer_smooth)

        # Decide which direction to steer
        # Count obstacle "weight" on each side (closer = heavier)
        left_weight = sum(1.0 / (obs[0] + 0.1) for obs in self.obstacles_left)
        right_weight = sum(1.0 / (obs[0] + 0.1) for obs in self.obstacles_right)
        front_weight = sum(1.0 / (obs[0] + 0.1) for obs in self.obstacles_front)

        # Also consider which side has more clearance by checking for clear space
        left_clear = self.check_clearance(math.radians(30))   # 30° to the left
        right_clear = self.check_clearance(math.radians(-30)) # 30° to the right

        # Prefer the side with more clearance
        if left_weight > right_weight:
            # More obstacles on left, steer right
            steer_direction = -1.0
        elif right_weight > left_weight:
            # More obstacles on right, steer left
            steer_direction = 1.0
        elif left_clear > right_clear:
            # Equal obstacles, but more clear on left
            steer_direction = 1.0
        elif right_clear > left_clear:
            steer_direction = -1.0
        else:
            # Default: steer in direction of goal if offset, else right
            if self.goal_direction > 0.1:
                steer_direction = 1.0
            elif self.goal_direction < -0.1:
                steer_direction = -1.0
            else:
                steer_direction = -1.0  # Default right

        # For front obstacles, ensure we steer enough
        if front_weight > 0 and steer_magnitude < self.max_steer * 0.5:
            steer_magnitude = max(steer_magnitude, self.max_steer * 0.3)

        return steer_direction * steer_magnitude

    def check_clearance(self, direction: float) -> float:
        """
        Check how much clearance exists in a given direction.

        Args:
            direction: Angle in robot frame (radians)

        Returns:
            Clearance distance (larger = more clear)
        """
        if self.latest_scan is None:
            return self.planning_dist

        scan = self.latest_scan
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))

        # Find readings in a cone around the direction
        cone_half_angle = math.radians(20)  # 40° cone
        min_dist = self.planning_dist

        for i, distance in enumerate(scan.ranges):
            if not np.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle_diff = abs(self.normalize_angle(angles[i] - direction))
            if angle_diff < cone_half_angle:
                if distance < min_dist:
                    min_dist = distance

        return min_dist

    def calculate_speed_scale(self) -> float:
        """
        Calculate speed scaling based on nearest obstacle.

        Returns:
            Speed scale (0 to 1)
        """
        if self.closest_obstacle_dist >= self.full_speed_dist:
            return 1.0
        elif self.closest_obstacle_dist <= self.critical_dist:
            return self.min_speed
        else:
            # Linear interpolation
            normalized = (self.closest_obstacle_dist - self.critical_dist) / \
                        (self.full_speed_dist - self.critical_dist)
            return self.min_speed + (1.0 - self.min_speed) * normalized

    def process_avoidance(self):
        """Main processing loop"""
        # Analyze current obstacles
        self.analyze_obstacles()

        # Calculate steering adjustment
        steering_adj = self.calculate_steering_adjustment()

        # Calculate speed scale
        speed_scale = self.calculate_speed_scale()

        # Check for critical danger
        critical = self.closest_obstacle_dist < self.critical_dist

        # Publish steering adjustment
        steer_msg = Float32()
        steer_msg.data = steering_adj
        self.steering_pub.publish(steer_msg)

        # Publish speed scale
        speed_msg = Float32()
        speed_msg.data = speed_scale
        self.speed_pub.publish(speed_msg)

        # Publish critical danger
        self.danger_pub.publish(Bool(data=critical))

        # Publish obstacle info for debugging
        info_msg = Vector3()
        info_msg.x = self.closest_obstacle_dist
        info_msg.y = float(len(self.obstacles_left + self.obstacles_right + self.obstacles_front))
        info_msg.z = steering_adj
        self.obstacle_info_pub.publish(info_msg)

        # Legacy: publish safe_direction for compatibility
        safe_msg = Vector3()
        safe_msg.x = steering_adj  # Now contains steering adjustment
        safe_msg.y = speed_scale
        safe_msg.z = 0.0
        self.safe_dir_pub.publish(safe_msg)

        # Logging
        if critical:
            self.get_logger().warn(
                f'CRITICAL: {self.closest_obstacle_dist:.2f}m | '
                f'Steer: {math.degrees(steering_adj):.0f}° | Speed: {speed_scale:.0%}',
                throttle_duration_sec=0.5
            )
        elif abs(steering_adj) > math.radians(5):
            self.get_logger().info(
                f'Adjusting: {self.closest_obstacle_dist:.2f}m | '
                f'Steer: {math.degrees(steering_adj):.0f}° | Speed: {speed_scale:.0%}',
                throttle_duration_sec=1.0
            )

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-π, π]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PredictiveObstacleAvoidance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
