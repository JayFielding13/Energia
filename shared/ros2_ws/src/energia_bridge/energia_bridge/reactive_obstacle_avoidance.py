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

        # =====================================================================
        # ROBOT FOOTPRINT - Energia Rover physical dimensions
        # From URDF: chassis is 27" x 23.75" (0.6858m x 0.60325m)
        # LiDAR is mounted on electronics box at center of robot
        # =====================================================================
        self.declare_parameter('robot_length', 0.6858)        # 27 inches - front to back
        self.declare_parameter('robot_width', 0.60325)        # 23.75 inches - side to side
        self.declare_parameter('lidar_offset_x', 0.0)         # LiDAR is centered on robot
        self.declare_parameter('safety_margin', 0.25)         # Extra clearance around robot

        # Parameters - Planning distances
        self.declare_parameter('planning_distance', 8.0)      # Start planning at this distance
        self.declare_parameter('critical_distance', 0.5)      # Emergency slowdown distance

        # Parameters - Steering behavior
        # SKID STEER: Rover can rotate 360° in place! Use this to our advantage.
        self.declare_parameter('max_steering_angle', 90.0)    # Can turn hard - skid steer!
        self.declare_parameter('steering_smoothness', 2.0)    # Higher = more gradual steering
        self.declare_parameter('goal_weight', 0.7)            # How much to favor goal direction (0-1)

        # Rotate-in-place threshold - if blocked, just spin to face clear direction
        self.declare_parameter('rotate_in_place_threshold', 1.0)  # Distance to trigger rotation

        # Get parameters
        self.robot_length = self.get_parameter('robot_length').value
        self.robot_width = self.get_parameter('robot_width').value
        self.lidar_offset_x = self.get_parameter('lidar_offset_x').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.planning_dist = self.get_parameter('planning_distance').value
        self.critical_dist = self.get_parameter('critical_distance').value
        self.max_steer = math.radians(self.get_parameter('max_steering_angle').value)
        self.steer_smooth = self.get_parameter('steering_smoothness').value
        self.goal_weight = self.get_parameter('goal_weight').value
        self.rotate_threshold = self.get_parameter('rotate_in_place_threshold').value

        # =====================================================================
        # FOOTPRINT-AWARE CLEARANCE CALCULATIONS
        # The LiDAR measures distance from its position, but the robot body
        # extends beyond the LiDAR in all directions.
        # =====================================================================

        # Half-dimensions of the robot from LiDAR center
        self.robot_half_length = self.robot_length / 2  # ~0.34m front/back from center
        self.robot_half_width = self.robot_width / 2    # ~0.30m left/right from center

        # Distance from LiDAR to front edge of robot (where we'll hit first)
        self.front_edge_offset = self.robot_half_length - self.lidar_offset_x

        # Effective clearance needed on each side when passing an obstacle
        # This is the half-width plus safety margin
        self.side_clearance_needed = self.robot_half_width + self.safety_margin

        # For legacy compatibility
        self.corridor_half_width = self.side_clearance_needed

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

        # Subscribers - use BEST_EFFORT for sensor data from ros_gz_bridge
        # ros_gz_bridge publishes sensor data as BEST_EFFORT, not RELIABLE
        # See: https://gazebosim.org/docs/harmonic/ros2_integration/
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
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

        # Ultrasonic sensors - 6 sensors for close-range obstacle detection
        # These provide complementary coverage to LiDAR, especially for low obstacles
        # NOTE: Ultrasonic subscriptions temporarily disabled due to a ROS2 Humble bug
        # that causes "Unable to convert call argument to Python object" errors when
        # receiving sensor_msgs/Range messages from Gazebo. The footprint-aware collision
        # detection logic is ready in the process_avoidance() method.
        # TODO: Re-enable when ROS2/Gazebo compatibility issue is resolved
        # from rclpy.qos import qos_profile_sensor_data
        # self._us_front_sub = self.create_subscription(
        #     Range, '/ultrasonic/front', self._us_front_cb, qos_profile_sensor_data)
        # self._us_corner_left_sub = self.create_subscription(
        #     Range, '/ultrasonic/corner_left', self._us_corner_left_cb, qos_profile_sensor_data)
        # self._us_corner_right_sub = self.create_subscription(
        #     Range, '/ultrasonic/corner_right', self._us_corner_right_cb, qos_profile_sensor_data)
        # self._us_side_left_sub = self.create_subscription(
        #     Range, '/ultrasonic/side_left', self._us_side_left_cb, qos_profile_sensor_data)
        # self._us_side_right_sub = self.create_subscription(
        #     Range, '/ultrasonic/side_right', self._us_side_right_cb, qos_profile_sensor_data)
        # self._us_rear_sub = self.create_subscription(
        #     Range, '/ultrasonic/rear', self._us_rear_cb, qos_profile_sensor_data)
        self.get_logger().info('Ultrasonic sensors: processing logic ready (subscriptions disabled due to ROS2 bug)')

        # Publishers
        # Steering adjustment (radians) - positive = steer left, negative = steer right
        self.steering_pub = self.create_publisher(Float32, '/avoidance/steering_adjustment', 10)

        # Speed scale (0-1) - always 1.0 now, kept for compatibility
        self.speed_pub = self.create_publisher(Float32, '/avoidance/speed_scale', 10)

        # Rotate in place flag - when True, rover should stop forward motion and just rotate
        self.rotate_pub = self.create_publisher(Bool, '/avoidance/rotate_in_place', 10)

        # Critical danger flag (for emergency stops only)
        self.danger_pub = self.create_publisher(Bool, '/avoidance/critical_danger', 10)

        # Debug: obstacle info
        self.obstacle_info_pub = self.create_publisher(Vector3, '/avoidance/obstacle_info', 10)

        # Legacy publishers for compatibility
        self.safe_dir_pub = self.create_publisher(Vector3, '/avoidance/safe_direction', 10)

        # Timer for processing
        self.create_timer(0.1, self.process_avoidance)  # 10 Hz

        self.get_logger().info('Predictive Obstacle Avoidance started')
        self.get_logger().info(f'  Robot footprint: {self.robot_length:.3f}m x {self.robot_width:.3f}m')
        self.get_logger().info(f'  Front edge offset: {self.front_edge_offset:.3f}m from LiDAR')
        self.get_logger().info(f'  Side clearance needed: {self.side_clearance_needed:.3f}m')
        self.get_logger().info(f'  Planning distance: {self.planning_dist}m')
        self.get_logger().info(f'  Critical distance: {self.critical_dist}m')
        self.get_logger().info('  Strategy: Footprint-aware obstacle avoidance!')

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

    # Individual ultrasonic callbacks (ROS2 Python doesn't handle closures well)
    def _us_front_cb(self, msg):
        self._process_ultrasonic(msg, 'front')

    def _us_corner_left_cb(self, msg):
        self._process_ultrasonic(msg, 'corner_left')

    def _us_corner_right_cb(self, msg):
        self._process_ultrasonic(msg, 'corner_right')

    def _us_side_left_cb(self, msg):
        self._process_ultrasonic(msg, 'side_left')

    def _us_side_right_cb(self, msg):
        self._process_ultrasonic(msg, 'side_right')

    def _us_rear_cb(self, msg):
        self._process_ultrasonic(msg, 'rear')

    def _process_ultrasonic(self, msg, sensor_name):
        """Store ultrasonic reading with validation"""
        # Ultrasonic sensors have min_range=0.2m, max_range=6.0m per URDF
        # Also check the message's own range limits
        min_range = max(0.1, msg.min_range) if msg.min_range > 0 else 0.1
        max_range = min(6.0, msg.max_range) if msg.max_range > 0 else 6.0

        if min_range <= msg.range <= max_range:
            self.ultrasonic_readings[sensor_name] = msg.range
            # Log when close obstacles detected for debugging
            if msg.range < 1.0:
                self.get_logger().debug(
                    f'Ultrasonic {sensor_name}: {msg.range:.2f}m',
                    throttle_duration_sec=0.5
                )
        else:
            self.ultrasonic_readings[sensor_name] = float('inf')

    def would_collide_with_footprint(self, distance: float, angle: float) -> tuple:
        """
        Check if an obstacle at (distance, angle) would collide with the robot footprint.

        The robot is NOT a point - it has dimensions. An obstacle might be at 45° from
        the LiDAR, but if the robot is wide enough, its corner could still hit that obstacle.

        Returns:
            (would_collide, effective_distance) - whether collision would occur and
            the effective distance to the robot's edge (not LiDAR center)
        """
        # Convert polar to cartesian (obstacle position relative to LiDAR)
        obs_x = distance * math.cos(angle)  # Forward distance
        obs_y = distance * math.sin(angle)  # Lateral distance (+ = left)

        # Check if obstacle is within the robot's forward sweep path
        # The robot sweeps a corridor as it moves forward:
        # - Width: robot_width + 2 * safety_margin
        # - The corridor starts from the front edge of the robot

        # Calculate effective clearance needed at this angle
        # For forward obstacles: need full front_edge_offset clearance
        # For side obstacles: need side_clearance clearance

        if abs(obs_y) <= self.side_clearance_needed:
            # Obstacle is within the lateral sweep path
            # Effective distance is how far until we hit it with our FRONT edge
            effective_dist = obs_x - self.front_edge_offset

            if effective_dist < 0:
                # Already past our front edge - we've hit it!
                return (True, 0.0)
            elif effective_dist < self.planning_dist:
                return (True, effective_dist)

        # For obstacles outside the direct path, check if we'd clip them
        # when turning or if our corners would hit
        abs_y = abs(obs_y)
        if abs_y < self.robot_half_width + self.safety_margin + 0.3:
            # Close enough laterally that we should consider it
            # Calculate the closest approach distance
            if obs_x > 0:  # Only care about obstacles ahead
                effective_dist = obs_x - self.front_edge_offset
                if effective_dist > 0:
                    return (True, effective_dist)

        return (False, distance)

    def analyze_obstacles(self):
        """
        Analyze sensor data to find obstacles and classify by position.

        FOOTPRINT-AWARE: We now consider the robot's actual dimensions when
        determining if an obstacle is a threat. An obstacle at 45° might still
        be in our path if we're wide enough!

        Obstacles are classified as:
        - front: directly ahead OR would collide with robot body
        - left: to the left (30° to 90°)
        - right: to the right (-30° to -90°)
        """
        self.obstacles_left = []
        self.obstacles_right = []
        self.obstacles_front = []
        self.closest_obstacle_dist = float('inf')
        self.closest_front_obstacle_dist = float('inf')  # Track front obstacles separately
        self.closest_effective_dist = float('inf')  # Distance to robot edge, not LiDAR

        # Define front detection zones (in robot frame, not goal frame!)
        front_half_angle = math.radians(30)   # ±30° = front zone
        side_max_angle = math.radians(90)     # ±90° = consider obstacles to the sides

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

                angle = angles[i]  # Angle in robot frame (0 = forward)

                # Track closest obstacle overall (raw LiDAR distance)
                if distance < self.closest_obstacle_dist:
                    self.closest_obstacle_dist = distance

                # Only consider obstacles in the forward hemisphere (±90°)
                if abs(angle) > side_max_angle:
                    continue

                # Check if this obstacle would collide with our footprint
                would_collide, effective_dist = self.would_collide_with_footprint(distance, angle)

                if would_collide and effective_dist < self.closest_effective_dist:
                    self.closest_effective_dist = effective_dist

                # Classify obstacles based on their angle IN ROBOT FRAME
                # BUT also consider footprint collisions
                if abs(angle) < front_half_angle:
                    # Directly ahead - MUST avoid!
                    self.obstacles_front.append((distance, angle))
                    if distance < self.closest_front_obstacle_dist:
                        self.closest_front_obstacle_dist = distance
                elif would_collide and effective_dist < 3.0:
                    # Not directly ahead, but would collide with our body!
                    # Treat as a front obstacle for avoidance purposes
                    self.obstacles_front.append((distance, angle))
                    if effective_dist < self.closest_front_obstacle_dist:
                        self.closest_front_obstacle_dist = effective_dist
                elif angle > 0:
                    # Left side
                    self.obstacles_left.append((distance, angle))
                else:
                    # Right side
                    self.obstacles_right.append((distance, angle))

        # Process ultrasonic data (if available)
        # Apply the SAME footprint-aware logic as LiDAR for consistent behavior
        for sensor_name, distance in self.ultrasonic_readings.items():
            if distance > self.planning_dist:
                continue

            angle = self.ultrasonic_positions[sensor_name]

            # Track closest obstacle overall
            if distance < self.closest_obstacle_dist:
                self.closest_obstacle_dist = distance

            # Only consider obstacles in forward hemisphere
            if abs(angle) > side_max_angle:
                continue

            # Check if this obstacle would collide with robot footprint
            # (Same logic as LiDAR for consistent behavior)
            would_collide, effective_dist = self.would_collide_with_footprint(distance, angle)

            if would_collide and effective_dist < self.closest_effective_dist:
                self.closest_effective_dist = effective_dist

            # Classify obstacle
            if abs(angle) < front_half_angle:
                # Directly ahead
                self.obstacles_front.append((distance, angle))
                if distance < self.closest_front_obstacle_dist:
                    self.closest_front_obstacle_dist = distance
            elif would_collide and effective_dist < 3.0:
                # Not directly ahead, but would collide with robot body
                self.obstacles_front.append((distance, angle))
                if effective_dist < self.closest_front_obstacle_dist:
                    self.closest_front_obstacle_dist = effective_dist
            elif angle > 0:
                self.obstacles_left.append((distance, angle))
            else:
                self.obstacles_right.append((distance, angle))

    def calculate_steering_adjustment(self) -> float:
        """
        Calculate how much to steer away from obstacles.

        CRITICAL: When obstacles are detected directly in front, we MUST steer away
        aggressively, regardless of where the waypoint is. The waypoint navigator
        will blend this steering with the goal direction, but we provide the
        avoidance steering it needs to avoid collision.

        Returns:
            Steering adjustment in radians (positive = left, negative = right)
        """
        # No obstacles detected - no adjustment needed
        all_obstacles = self.obstacles_left + self.obstacles_right + self.obstacles_front
        if not all_obstacles:
            return 0.0

        # Use FRONT obstacle distance as primary trigger for steering
        # This ensures we react to walls ahead even if there are closer side obstacles
        front_min_dist = self.closest_front_obstacle_dist
        overall_min_dist = self.closest_obstacle_dist

        # Check clearance at multiple angles for direction decision
        left_clear_30 = self.check_clearance(math.radians(30))
        right_clear_30 = self.check_clearance(math.radians(-30))
        left_clear_60 = self.check_clearance(math.radians(60))
        right_clear_60 = self.check_clearance(math.radians(-60))
        left_clear_90 = self.check_clearance(math.radians(90))
        right_clear_90 = self.check_clearance(math.radians(-90))

        # Weighted clearance score
        left_clearance_score = left_clear_30 * 0.5 + left_clear_60 * 0.3 + left_clear_90 * 0.2
        right_clearance_score = right_clear_30 * 0.5 + right_clear_60 * 0.3 + right_clear_90 * 0.2

        # Calculate obstacle weights (closer = heavier)
        left_weight = sum(1.0 / (obs[0] + 0.1) for obs in self.obstacles_left)
        right_weight = sum(1.0 / (obs[0] + 0.1) for obs in self.obstacles_right)
        front_weight = sum(1.0 / (obs[0] + 0.1) for obs in self.obstacles_front)

        # Detect wall ahead - triggers aggressive avoidance
        has_front_obstacles = len(self.obstacles_front) > 0
        has_wall_ahead = front_weight > 0.3 or len(self.obstacles_front) >= 2

        # =====================================================================
        # STEERING DIRECTION - which way to turn
        # =====================================================================
        steer_direction = 0.0

        # Priority 1: Steer away from the side with more obstacles
        weight_diff = right_weight - left_weight
        if abs(weight_diff) > 0.3:
            steer_direction = 1.0 if weight_diff > 0 else -1.0
        else:
            # Priority 2: Steer toward the side with more clearance
            clearance_diff = left_clearance_score - right_clearance_score

            if abs(clearance_diff) > 0.3:
                steer_direction = 1.0 if clearance_diff > 0 else -1.0
            elif has_front_obstacles:
                # Front obstacles with similar clearance - pick a side and commit
                if left_clearance_score > right_clearance_score:
                    steer_direction = 1.0
                elif right_clearance_score > left_clearance_score:
                    steer_direction = -1.0
                elif hasattr(self, 'committed_direction'):
                    steer_direction = self.committed_direction
                else:
                    # Default: steer right (arbitrary but consistent)
                    steer_direction = -1.0
            else:
                # No front obstacles - minor adjustment based on goal
                if self.goal_direction > 0.1:
                    steer_direction = 1.0
                elif self.goal_direction < -0.1:
                    steer_direction = -1.0
                else:
                    steer_direction = 0.0  # No adjustment needed

        # Commit to direction when facing obstacles (prevents oscillation)
        if has_front_obstacles and front_min_dist < self.planning_dist * 0.6:
            self.committed_direction = steer_direction

        # =====================================================================
        # STEERING MAGNITUDE - how hard to turn
        # =====================================================================

        # Base magnitude from front obstacle distance (FRONT obstacles trigger steering!)
        if not has_front_obstacles:
            # No front obstacles - use minimal steering based on side obstacles
            if overall_min_dist >= self.planning_dist:
                steer_magnitude = 0.0
            else:
                normalized_dist = (overall_min_dist - self.critical_dist) / (self.planning_dist - self.critical_dist)
                normalized_dist = max(0.0, min(1.0, normalized_dist))
                steer_magnitude = self.max_steer * 0.3 * (1.0 - normalized_dist)
        else:
            # FRONT OBSTACLES DETECTED - steer aggressively!
            if front_min_dist <= self.critical_dist:
                # Emergency - maximum steering
                steer_magnitude = self.max_steer
            elif front_min_dist <= 1.5:
                # Very close - strong steering (70-100% of max)
                normalized = (front_min_dist - self.critical_dist) / (1.5 - self.critical_dist)
                normalized = max(0.0, min(1.0, normalized))
                steer_magnitude = self.max_steer * (0.7 + 0.3 * (1.0 - normalized))
            elif front_min_dist <= 3.0:
                # Close - moderate to strong steering (40-70% of max)
                normalized = (front_min_dist - 1.5) / (3.0 - 1.5)
                steer_magnitude = self.max_steer * (0.4 + 0.3 * (1.0 - normalized))
            elif front_min_dist <= self.planning_dist:
                # Planning range - gentle steering that increases (10-40% of max)
                normalized = (front_min_dist - 3.0) / (self.planning_dist - 3.0)
                steer_magnitude = self.max_steer * (0.1 + 0.3 * (1.0 - normalized))
            else:
                steer_magnitude = 0.0

            # MINIMUM steering when any front obstacle detected within planning range
            # This ensures we ALWAYS steer when there's something ahead
            if front_min_dist < self.planning_dist:
                min_steer = self.max_steer * 0.25  # At least 25% steering
                steer_magnitude = max(steer_magnitude, min_steer)

        # Extra boost for wall situations (multiple front readings)
        if has_wall_ahead and front_min_dist < 3.0:
            steer_magnitude = max(steer_magnitude, self.max_steer * 0.6)

        return steer_direction * steer_magnitude

    def check_clearance(self, direction: float) -> float:
        """
        Check how much clearance exists in a given direction, accounting for robot width.

        FOOTPRINT-AWARE: When checking clearance to the left or right, we need
        to account for the fact that the robot body extends ~0.30m to each side.
        So an obstacle that appears "clear" at 45° might still hit our corner.

        Args:
            direction: Angle in robot frame (radians)

        Returns:
            Clearance distance (larger = more clear), adjusted for robot footprint
        """
        if self.latest_scan is None:
            return self.planning_dist

        scan = self.latest_scan
        angles = np.linspace(scan.angle_min, scan.angle_max, len(scan.ranges))

        # Find readings in a cone around the direction
        cone_half_angle = math.radians(25)  # Slightly wider cone for safety
        min_effective_dist = self.planning_dist

        for i, distance in enumerate(scan.ranges):
            if not np.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            angle_diff = abs(self.normalize_angle(angles[i] - direction))
            if angle_diff < cone_half_angle:
                # Convert to cartesian to check footprint collision
                obs_x = distance * math.cos(angles[i])
                obs_y = distance * math.sin(angles[i])

                # Check if this obstacle is within our path considering robot width
                if abs(obs_y) < self.side_clearance_needed:
                    # It's in our width corridor - calculate effective distance
                    effective_dist = obs_x - self.front_edge_offset
                    if effective_dist > 0 and effective_dist < min_effective_dist:
                        min_effective_dist = effective_dist
                elif distance < min_effective_dist:
                    # Outside our width corridor - use raw distance as a reference
                    # but discount it since we might still clip it when turning
                    lateral_margin = abs(obs_y) - self.side_clearance_needed
                    if lateral_margin < 0.5:  # Close enough to worry about
                        effective_dist = distance * 0.8  # Discount factor
                        if effective_dist < min_effective_dist:
                            min_effective_dist = effective_dist
                    elif distance < min_effective_dist:
                        min_effective_dist = distance

        return min_effective_dist

    def calculate_speed_scale(self) -> float:
        """
        Speed scaling - DISABLED for confident movement.

        The rover should move at full speed everywhere. Obstacle avoidance
        is handled by steering, not by slowing down. The rover has skid-steer
        and can rotate in place if needed.

        Returns:
            Always 1.0 - full speed
        """
        # FULL SPEED ALWAYS - the rover should move with confidence
        # Obstacle avoidance is handled by steering, not slowing down
        return 1.0

    def process_avoidance(self):
        """Main processing loop"""
        # Analyze current obstacles
        self.analyze_obstacles()

        # Calculate steering adjustment
        steering_adj = self.calculate_steering_adjustment()

        # Speed is always 1.0 now - move with confidence!
        speed_scale = 1.0

        # Check if we should rotate in place (blocked and need to turn)
        # Rotate in place when: obstacle very close AND steering is significant
        effective_dist = min(self.closest_obstacle_dist, self.closest_effective_dist)
        should_rotate = (effective_dist < self.rotate_threshold and
                        abs(steering_adj) > math.radians(30) and
                        len(self.obstacles_front) > 0)

        # Check for critical danger (only if we somehow can't steer out)
        critical = effective_dist < self.critical_dist

        # Publish steering adjustment
        steer_msg = Float32()
        steer_msg.data = steering_adj
        self.steering_pub.publish(steer_msg)

        # Publish speed scale (always 1.0)
        speed_msg = Float32()
        speed_msg.data = speed_scale
        self.speed_pub.publish(speed_msg)

        # Publish rotate in place flag
        self.rotate_pub.publish(Bool(data=should_rotate))

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
        safe_msg.z = 1.0 if should_rotate else 0.0  # Z indicates rotate-in-place
        self.safe_dir_pub.publish(safe_msg)

        # Logging - more verbose to help debug
        if self.latest_scan is None:
            self.get_logger().warn(
                'NO LIDAR DATA - check /scan topic and QoS settings!',
                throttle_duration_sec=2.0
            )
        elif should_rotate:
            self.get_logger().warn(
                f'ROTATE IN PLACE: {effective_dist:.2f}m | '
                f'Turn: {math.degrees(steering_adj):.0f}° | '
                f'Front obstacles: {len(self.obstacles_front)}',
                throttle_duration_sec=0.5
            )
        elif abs(steering_adj) > math.radians(15):
            self.get_logger().info(
                f'STEERING: dist={effective_dist:.2f}m | '
                f'turn={math.degrees(steering_adj):.0f}° | '
                f'front={len(self.obstacles_front)} L={len(self.obstacles_left)} R={len(self.obstacles_right)}',
                throttle_duration_sec=0.5
            )
        elif len(self.obstacles_front) > 0:
            self.get_logger().debug(
                f'Front obstacle detected: {self.closest_front_obstacle_dist:.2f}m | '
                f'steering={math.degrees(steering_adj):.0f}°',
                throttle_duration_sec=0.5
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
