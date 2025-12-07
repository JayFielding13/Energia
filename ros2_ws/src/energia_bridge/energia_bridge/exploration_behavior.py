#!/usr/bin/env python3
"""
Exploration Behavior with AprilTag Goal Seeking

This node implements an exploration strategy that:
1. Uses the SLAM map to identify frontiers (boundaries between known and unknown space)
2. Navigates toward unexplored areas while avoiding obstacles
3. Searches for AprilTags and considers them as high-priority goals
4. Uses LiDAR for reactive obstacle avoidance

The rover will actively explore the environment and seek out AprilTags.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math
from collections import deque

# AprilTag support is optional
try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    APRILTAG_AVAILABLE = True
except ImportError:
    APRILTAG_AVAILABLE = False


class ExplorationBehavior(Node):
    def __init__(self):
        super().__init__('exploration_behavior')

        # Parameters
        self.declare_parameter('forward_speed', 1.5)  # m/s (real robot top speed ~4.5 m/s)
        self.declare_parameter('turn_speed', 1.5)  # rad/s (real robot can rotate quickly)
        self.declare_parameter('safety_distance', 1.5)
        self.declare_parameter('danger_distance', 0.6)
        self.declare_parameter('frontier_search_radius', 3.0)  # meters
        self.declare_parameter('goal_tolerance', 0.5)  # meters

        self.forward_speed = self.get_parameter('forward_speed').value
        self.turn_speed = self.get_parameter('turn_speed').value
        self.safety_distance = self.get_parameter('safety_distance').value
        self.danger_distance = self.get_parameter('danger_distance').value
        self.frontier_search_radius = self.get_parameter('frontier_search_radius').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # State
        self.current_map = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.latest_scan = None
        self.min_front_distance = float('inf')
        self.clearance_left = float('inf')
        self.clearance_right = float('inf')

        # Exploration state
        self.current_goal = None  # (x, y) in map frame
        self.goal_type = None  # 'frontier' or 'apriltag'
        self.apriltag_detected = False
        self.apriltag_position = None
        self.stuck_counter = 0
        self.last_position = (0.0, 0.0)
        self.position_history = deque(maxlen=50)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Only subscribe to AprilTags if available
        if APRILTAG_AVAILABLE:
            self.create_subscription(
                AprilTagDetectionArray,
                '/apriltag/detections',
                self.apriltag_callback,
                10
            )
            self.get_logger().info('  AprilTag detection enabled')

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Control loop timer (10 Hz)
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Exploration behavior started')
        self.get_logger().info(f'  Forward speed: {self.forward_speed} m/s')
        self.get_logger().info(f'  Turn speed: {self.turn_speed} rad/s')
        self.get_logger().info(f'  Safety distance: {self.safety_distance}m')
        self.get_logger().info(f'  Frontier search radius: {self.frontier_search_radius}m')

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract yaw from quaternion
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

        # Track position history for stuck detection
        self.position_history.append((self.robot_x, self.robot_y))

    def apriltag_callback(self, msg):
        """Handle AprilTag detections - set as high priority goal"""
        if len(msg.detections) > 0:
            detection = msg.detections[0]

            # Convert AprilTag position from camera frame to map frame
            # Assuming tag is at some distance in front of robot
            tag_distance = 2.0  # Estimate, could be refined with actual detection data
            tag_x = self.robot_x + tag_distance * math.cos(self.robot_yaw)
            tag_y = self.robot_y + tag_distance * math.sin(self.robot_yaw)

            self.apriltag_detected = True
            self.apriltag_position = (tag_x, tag_y)

            # Set as goal if we don't have one or if closer than current goal
            if self.current_goal is None or self.goal_type != 'apriltag':
                self.current_goal = self.apriltag_position
                self.goal_type = 'apriltag'
                self.get_logger().info(f'AprilTag detected! Setting as goal at ({tag_x:.2f}, {tag_y:.2f})')

    def map_callback(self, msg):
        """Store the latest map for frontier detection"""
        self.current_map = msg

    def lidar_callback(self, msg):
        """Process LiDAR scan for reactive obstacle avoidance"""
        self.latest_scan = msg

        ranges = np.array(msg.ranges)
        ranges[np.isinf(ranges)] = msg.range_max

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

    def find_frontier_goal(self):
        """Find nearest frontier (boundary between known and unknown space)"""
        if self.current_map is None:
            return None

        map_data = np.array(self.current_map.data).reshape(
            (self.current_map.info.height, self.current_map.info.width)
        )

        # Find frontiers: cells that are free (0) adjacent to unknown (-1)
        frontiers = []

        for y in range(1, self.current_map.info.height - 1):
            for x in range(1, self.current_map.info.width - 1):
                # Check if cell is free
                if map_data[y, x] == 0:
                    # Check if any neighbor is unknown
                    neighbors = [
                        map_data[y-1, x], map_data[y+1, x],
                        map_data[y, x-1], map_data[y, x+1]
                    ]
                    if -1 in neighbors:
                        # Convert grid coordinates to world coordinates
                        world_x = self.current_map.info.origin.position.x + x * self.current_map.info.resolution
                        world_y = self.current_map.info.origin.position.y + y * self.current_map.info.resolution

                        # Calculate distance from robot
                        dist = math.sqrt((world_x - self.robot_x)**2 + (world_y - self.robot_y)**2)

                        # Only consider frontiers within search radius
                        if dist < self.frontier_search_radius and dist > 0.5:
                            frontiers.append((world_x, world_y, dist))

        if not frontiers:
            return None

        # Find closest frontier
        frontiers.sort(key=lambda f: f[2])
        return (frontiers[0][0], frontiers[0][1])

    def is_stuck(self):
        """Detect if robot is stuck (not making progress)"""
        if len(self.position_history) < 30:
            return False

        # Check if robot has moved less than 0.3m in last 30 updates
        recent_positions = list(self.position_history)[-30:]
        x_coords = [p[0] for p in recent_positions]
        y_coords = [p[1] for p in recent_positions]

        x_range = max(x_coords) - min(x_coords)
        y_range = max(y_coords) - min(y_coords)
        total_movement = math.sqrt(x_range**2 + y_range**2)

        return total_movement < 0.3

    def control_loop(self):
        """Main control loop: exploration + reactive obstacle avoidance"""
        if self.latest_scan is None:
            return

        cmd = Twist()

        # Check if we've reached current goal
        if self.current_goal is not None:
            dist_to_goal = math.sqrt(
                (self.current_goal[0] - self.robot_x)**2 +
                (self.current_goal[1] - self.robot_y)**2
            )

            if dist_to_goal < self.goal_tolerance:
                if self.goal_type == 'apriltag':
                    self.get_logger().info('Reached AprilTag goal!')
                else:
                    self.get_logger().info('Reached frontier goal, finding new frontier...')
                self.current_goal = None
                self.goal_type = None

        # Select goal: AprilTag takes priority over frontiers
        if self.apriltag_detected and self.apriltag_position is not None:
            if self.goal_type != 'apriltag':
                self.current_goal = self.apriltag_position
                self.goal_type = 'apriltag'
        elif self.current_goal is None or self.is_stuck():
            # Find new frontier goal
            frontier = self.find_frontier_goal()
            if frontier is not None:
                self.current_goal = frontier
                self.goal_type = 'frontier'
                self.get_logger().info(f'New frontier goal: ({frontier[0]:.2f}, {frontier[1]:.2f})')

        # Reactive obstacle avoidance (takes priority)
        obstacle_detected = False

        # Danger zone - very close obstacle
        if self.min_front_distance < self.danger_distance:
            obstacle_detected = True
            if self.clearance_left < self.safety_distance and self.clearance_right < self.safety_distance:
                # Trapped - rotate in place
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed * 1.2 if self.clearance_left > self.clearance_right else -self.turn_speed * 1.2
                self.get_logger().info(f'Trapped - rotating in place!', throttle_duration_sec=2.0)
            else:
                # Back up while turning
                cmd.linear.x = -0.2
                cmd.angular.z = self.turn_speed if self.clearance_left > self.clearance_right else -self.turn_speed

        # Safety bubble
        elif self.min_front_distance < self.safety_distance:
            obstacle_detected = True
            proximity_ratio = self.min_front_distance / self.safety_distance

            if proximity_ratio < 0.5:
                # Rotate in place
                cmd.linear.x = 0.0
                cmd.angular.z = self.turn_speed if self.clearance_left > self.clearance_right else -self.turn_speed
            else:
                # Slow down and turn
                cmd.linear.x = self.forward_speed * proximity_ratio * 0.5
                cmd.angular.z = self.turn_speed if self.clearance_left > self.clearance_right else -self.turn_speed

        # No immediate obstacle - navigate toward goal
        if not obstacle_detected and self.current_goal is not None:
            # Calculate angle to goal
            dx = self.current_goal[0] - self.robot_x
            dy = self.current_goal[1] - self.robot_y
            angle_to_goal = math.atan2(dy, dx)

            # Calculate angle difference
            angle_diff = angle_to_goal - self.robot_yaw
            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi

            # If facing goal, drive forward
            if abs(angle_diff) < 0.3:  # ~17 degrees
                cmd.linear.x = self.forward_speed
                cmd.angular.z = 0.5 * angle_diff  # Small correction
            else:
                # Rotate toward goal
                cmd.linear.x = self.forward_speed * 0.3
                cmd.angular.z = self.turn_speed * np.sign(angle_diff)

        # If no obstacle and no goal, do wander behavior
        if not obstacle_detected and self.current_goal is None:
            cmd.linear.x = self.forward_speed * 0.5
            # Add slight random turning to explore
            clearance_diff = self.clearance_left - self.clearance_right
            if abs(clearance_diff) > 0.5:
                cmd.angular.z = 0.2 * self.turn_speed * np.sign(clearance_diff)

        self.cmd_vel_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationBehavior()

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
