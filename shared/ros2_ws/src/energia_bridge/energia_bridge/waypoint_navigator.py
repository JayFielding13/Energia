#!/usr/bin/env python3
"""
Waypoint Navigator Node with Predictive Obstacle Avoidance Integration

Navigates the rover to GPS waypoints while smoothly avoiding obstacles.
Instead of reactive "stop and turn" behavior, this navigator:
1. Publishes goal direction to obstacle avoidance node
2. Receives steering adjustments from avoidance
3. Blends waypoint heading with avoidance adjustment for smooth curves

Subscribes to:
    /waypoint/target (NavSatFix) - GPS waypoint goal from HTTP bridge
    /gps/fix (NavSatFix) - Current GPS position
    /odom (Odometry) - Heading and velocity from odometry
    /avoidance/steering_adjustment (Float32) - Steering correction from avoidance
    /avoidance/speed_scale (Float32) - Speed scaling from avoidance
    /avoidance/critical_danger (Bool) - Emergency stop flag
    /rover/armed (Bool) - Arm state

Publishes to:
    /cmd_vel (Twist) - Velocity commands
    /navigation/status (String) - Navigation status feedback
    /navigation/goal_direction (Float32) - Goal direction for avoidance node
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, PoseStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String, Float32
from visualization_msgs.msg import Marker
import math
import json
from enum import Enum


class NavigationState(Enum):
    """Navigation state machine states"""
    IDLE = "idle"
    NAVIGATING = "navigating"
    ARRIVED = "arrived"
    EMERGENCY_STOP = "emergency_stop"


class WaypointNavigator(Node):
    """Waypoint navigation with predictive obstacle avoidance"""

    def __init__(self):
        super().__init__('waypoint_navigator')

        # Declare parameters
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('arrival_distance', 0.5)       # meters
        self.declare_parameter('slow_approach_distance', 2.0) # meters
        self.declare_parameter('max_linear_speed', 0.5)       # m/s
        self.declare_parameter('max_angular_speed', 0.8)      # rad/s
        self.declare_parameter('min_linear_speed', 0.1)       # m/s
        self.declare_parameter('linear_kp', 0.5)              # P gain for distance
        self.declare_parameter('angular_kp', 1.2)             # P gain for heading
        self.declare_parameter('origin_lat', 37.7749)         # GPS origin latitude
        self.declare_parameter('origin_lon', -122.4194)       # GPS origin longitude
        self.declare_parameter('avoidance_blend', 1.0)        # How much to blend avoidance (0-1)

        # Get parameters
        self.simulation_mode = self.get_parameter('simulation_mode').value
        self.arrival_distance = self.get_parameter('arrival_distance').value
        self.slow_approach_distance = self.get_parameter('slow_approach_distance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.min_linear = self.get_parameter('min_linear_speed').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.origin_lat = self.get_parameter('origin_lat').value
        self.origin_lon = self.get_parameter('origin_lon').value
        self.avoidance_blend = self.get_parameter('avoidance_blend').value

        # GPS conversion constants (must match gps_bridge.py)
        self.meters_per_degree_lat = 111320.0
        self.meters_per_degree_lon = 111320.0 * math.cos(math.radians(self.origin_lat))

        # State machine
        self.state = NavigationState.IDLE
        self.prev_state = NavigationState.IDLE

        # Position and target tracking
        self.current_lat = None
        self.current_lon = None
        self.current_heading = 0.0  # radians, 0 = East, CCW positive
        self.current_x = 0.0  # Local X (meters)
        self.current_y = 0.0  # Local Y (meters)

        self.target_lat = None
        self.target_lon = None
        self.target_x = None
        self.target_y = None

        # Status tracking
        self.armed = False
        self.distance_to_target = float('inf')
        self.heading_to_target = 0.0  # Heading to waypoint (world frame)
        self.heading_error = 0.0      # Error from current heading

        # Obstacle avoidance inputs
        self.steering_adjustment = 0.0  # From avoidance node (radians)
        self.steering_adjustment_raw = 0.0  # Unfiltered steering adjustment
        self.speed_scale = 1.0          # From avoidance node (0-1)
        self.critical_danger = False    # Emergency stop flag
        self.rotate_in_place = False    # Skid-steer: rotate without moving forward

        # Smoothing parameters for reducing oscillation
        self.steering_filter_alpha = 0.3  # Low-pass filter: 0.1=very smooth, 0.5=responsive
        self.prev_angular_vel = 0.0       # For rate limiting
        self.max_angular_accel = 0.5      # Max rad/s^2 change per control loop (10Hz)

        # Timing
        self.arrived_time = None
        self.arrived_delay = 2.0  # seconds to wait at waypoint

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            NavSatFix, '/waypoint/target', self.waypoint_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.armed_sub = self.create_subscription(
            Bool, '/rover/armed', self.armed_callback, 10)

        # Avoidance inputs - new interface
        self.steer_adj_sub = self.create_subscription(
            Float32, '/avoidance/steering_adjustment', self.steering_adjustment_callback, 10)
        self.speed_scale_sub = self.create_subscription(
            Float32, '/avoidance/speed_scale', self.speed_scale_callback, 10)
        self.danger_sub = self.create_subscription(
            Bool, '/avoidance/critical_danger', self.danger_callback, 10)
        self.rotate_sub = self.create_subscription(
            Bool, '/avoidance/rotate_in_place', self.rotate_callback, 10)

        # Legacy avoidance interface (for compatibility)
        self.safe_dir_sub = self.create_subscription(
            Vector3, '/avoidance/safe_direction', self.safe_direction_callback, 10)

        # RViz 2D Goal Pose
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/navigation/status', 10)
        self.marker_pub = self.create_publisher(Marker, '/waypoint/marker', 10)

        # Goal direction publisher for avoidance node
        self.goal_dir_pub = self.create_publisher(Float32, '/navigation/goal_direction', 10)

        # Control loop timer (10 Hz)
        self.create_timer(0.1, self.control_loop)

        # Status publishing timer (2 Hz)
        self.create_timer(0.5, self.publish_status)

        # Marker publishing timer (1 Hz)
        self.create_timer(1.0, self.publish_waypoint_marker)

        self.get_logger().info('Waypoint Navigator started (Predictive Avoidance)')
        self.get_logger().info(f'  Mode: {"Simulation" if self.simulation_mode else "Hardware"}')
        self.get_logger().info(f'  GPS Origin: {self.origin_lat:.6f}, {self.origin_lon:.6f}')
        self.get_logger().info(f'  Arrival distance: {self.arrival_distance}m')
        self.get_logger().info(f'  Avoidance blend: {self.avoidance_blend:.0%}')

    # =========================================================================
    # GPS Coordinate Conversion
    # =========================================================================

    def gps_to_local(self, lat: float, lon: float) -> tuple:
        """Convert GPS coordinates to local XY coordinates."""
        lat_offset = lat - self.origin_lat
        lon_offset = lon - self.origin_lon
        y = lat_offset * self.meters_per_degree_lat
        x = lon_offset * self.meters_per_degree_lon
        return x, y

    def local_to_gps(self, x: float, y: float) -> tuple:
        """Convert local XY coordinates to GPS coordinates."""
        lat = self.origin_lat + (y / self.meters_per_degree_lat)
        lon = self.origin_lon + (x / self.meters_per_degree_lon)
        return lat, lon

    # =========================================================================
    # Callbacks
    # =========================================================================

    def waypoint_callback(self, msg: NavSatFix):
        """Receive new waypoint target"""
        self.target_lat = msg.latitude
        self.target_lon = msg.longitude
        self.target_x, self.target_y = self.gps_to_local(self.target_lat, self.target_lon)

        self.get_logger().info(
            f'New waypoint: ({self.target_lat:.6f}, {self.target_lon:.6f}) '
            f'-> local ({self.target_x:.2f}, {self.target_y:.2f})'
        )

        self.publish_waypoint_marker()

        if self.armed and self.state == NavigationState.IDLE:
            self.transition_to(NavigationState.NAVIGATING)

    def goal_pose_callback(self, msg: PoseStamped):
        """Handle 2D Goal Pose from RViz (SIMULATION ONLY)"""
        # Only allow RViz goal pose in simulation mode for safety
        if not self.simulation_mode:
            self.get_logger().warn(
                'RViz 2D Goal Pose ignored - only available in simulation mode. '
                'Use HTTP API for real hardware.'
            )
            return

        self.target_x = msg.pose.position.x
        self.target_y = msg.pose.position.y
        self.target_lat, self.target_lon = self.local_to_gps(self.target_x, self.target_y)

        self.get_logger().info(
            f'[SIMULATION] RViz goal: local ({self.target_x:.2f}, {self.target_y:.2f}) '
            f'-> GPS ({self.target_lat:.6f}, {self.target_lon:.6f})'
        )

        self.publish_waypoint_marker()

        # Auto-arm for RViz goals (simulation only)
        if not self.armed:
            self.armed = True
            self.get_logger().info('[SIMULATION] Auto-armed for RViz goal')

        if self.state == NavigationState.IDLE:
            self.transition_to(NavigationState.NAVIGATING)

    def gps_callback(self, msg: NavSatFix):
        """Update current GPS position"""
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude

    def odom_callback(self, msg: Odometry):
        """Update current position and heading from odometry"""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_heading = math.atan2(siny_cosp, cosy_cosp)

    def armed_callback(self, msg: Bool):
        """Update armed state"""
        prev_armed = self.armed
        self.armed = msg.data

        if self.armed and not prev_armed:
            self.get_logger().info('Rover ARMED')
            if self.target_lat is not None and self.state == NavigationState.IDLE:
                self.transition_to(NavigationState.NAVIGATING)
        elif not self.armed and prev_armed:
            self.get_logger().info('Rover DISARMED')
            self.transition_to(NavigationState.IDLE)
            self.stop()

    def steering_adjustment_callback(self, msg: Float32):
        """Receive steering adjustment from avoidance node with low-pass filtering"""
        self.steering_adjustment_raw = msg.data
        # Apply low-pass filter: new = alpha * raw + (1-alpha) * old
        self.steering_adjustment = (
            self.steering_filter_alpha * self.steering_adjustment_raw +
            (1.0 - self.steering_filter_alpha) * self.steering_adjustment
        )

    def speed_scale_callback(self, msg: Float32):
        """Receive speed scale from avoidance node"""
        self.speed_scale = msg.data

    def danger_callback(self, msg: Bool):
        """Update critical danger flag"""
        prev_danger = self.critical_danger
        self.critical_danger = msg.data

        if self.critical_danger and not prev_danger:
            self.get_logger().warn('CRITICAL DANGER - Emergency slowdown!')

    def rotate_callback(self, msg: Bool):
        """Update rotate-in-place flag from avoidance"""
        self.rotate_in_place = msg.data

    def safe_direction_callback(self, msg: Vector3):
        """Legacy interface - safe_direction now contains steering adjustment"""
        # x = steering adjustment (radians)
        # y = speed scale
        # z = rotate_in_place flag (1.0 = true)
        self.steering_adjustment_raw = msg.x
        # Apply low-pass filter: new = alpha * raw + (1-alpha) * old
        self.steering_adjustment = (
            self.steering_filter_alpha * self.steering_adjustment_raw +
            (1.0 - self.steering_filter_alpha) * self.steering_adjustment
        )
        self.speed_scale = msg.y
        self.rotate_in_place = (msg.z > 0.5)

    # =========================================================================
    # State Machine
    # =========================================================================

    def transition_to(self, new_state: NavigationState):
        """Transition to a new state"""
        if new_state != self.state:
            self.prev_state = self.state
            self.state = new_state
            self.get_logger().info(f'State: {self.prev_state.value} -> {new_state.value}')

            if new_state == NavigationState.ARRIVED:
                self.arrived_time = self.get_clock().now()
            elif new_state == NavigationState.IDLE:
                self.arrived_time = None

    def update_navigation_state(self):
        """Update target tracking and state machine"""
        # Check disarm
        if not self.armed:
            if self.state != NavigationState.IDLE:
                self.transition_to(NavigationState.IDLE)
            return

        # No target
        if self.target_x is None or self.target_y is None:
            if self.state != NavigationState.IDLE:
                self.transition_to(NavigationState.IDLE)
            return

        # Calculate distance and heading to target
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        self.distance_to_target = math.sqrt(dx * dx + dy * dy)
        self.heading_to_target = math.atan2(dy, dx)
        self.heading_error = self.normalize_angle(self.heading_to_target - self.current_heading)

        # State transitions
        if self.state == NavigationState.IDLE:
            pass  # Wait for waypoint

        elif self.state == NavigationState.NAVIGATING:
            if self.distance_to_target < self.arrival_distance:
                self.transition_to(NavigationState.ARRIVED)

        elif self.state == NavigationState.ARRIVED:
            if self.arrived_time is not None:
                elapsed = (self.get_clock().now() - self.arrived_time).nanoseconds / 1e9
                if elapsed > self.arrived_delay:
                    self.get_logger().info('Waypoint reached!')
                    self.delete_waypoint_marker()
                    self.target_lat = None
                    self.target_lon = None
                    self.target_x = None
                    self.target_y = None
                    self.transition_to(NavigationState.IDLE)

        elif self.state == NavigationState.EMERGENCY_STOP:
            # Wait for danger to clear
            if not self.critical_danger:
                self.transition_to(NavigationState.NAVIGATING)

    # =========================================================================
    # Control
    # =========================================================================

    def compute_velocity(self) -> tuple:
        """
        Compute velocity commands with integrated obstacle avoidance.

        The heading is computed as:
        final_heading = waypoint_heading + avoidance_steering_adjustment

        This allows the rover to smoothly curve around obstacles while
        maintaining overall progress toward the waypoint.

        Returns:
            (linear_vel, angular_vel): Velocity commands
        """
        if self.target_x is None or self.target_y is None:
            return 0.0, 0.0

        # Base heading error to waypoint
        base_heading_error = self.heading_error

        # Apply steering adjustment from obstacle avoidance
        # The adjustment is added to the heading, so positive adjustment
        # steers left (counterclockwise), negative steers right
        adjusted_heading_error = base_heading_error + (self.steering_adjustment * self.avoidance_blend)

        # P-controller for angular velocity
        angular_vel = self.angular_kp * adjusted_heading_error
        angular_vel = self.clamp(angular_vel, -self.max_angular, self.max_angular)

        # CRITICAL: When avoidance steering is significant, ensure we're turning hard
        # The steering_adjustment directly indicates how much we need to turn
        avoidance_urgency = abs(self.steering_adjustment)
        if avoidance_urgency > 0.3:  # More than ~17 degrees of steering requested
            # Ensure angular velocity matches the steering direction and magnitude
            min_turn_speed = min(self.max_angular * 0.5, avoidance_urgency * 0.8)
            if abs(angular_vel) < min_turn_speed:
                # Force minimum turn rate in the steering direction
                angular_vel = min_turn_speed if self.steering_adjustment > 0 else -min_turn_speed

        # Base linear velocity from distance
        linear_vel = self.linear_kp * self.distance_to_target
        linear_vel = self.clamp(linear_vel, 0.0, self.max_linear)

        # Slow down near target
        if self.distance_to_target < self.slow_approach_distance:
            slow_factor = self.distance_to_target / self.slow_approach_distance
            linear_vel *= slow_factor

        # Reduce forward speed based on heading error (smooth curve driving)
        # Large heading error = slow down to turn
        heading_factor = max(0.0, math.cos(adjusted_heading_error))
        heading_factor = max(heading_factor, 0.2) if abs(adjusted_heading_error) < math.pi / 2 else heading_factor
        linear_vel *= heading_factor

        # CRITICAL: When avoidance urgency is high, reduce forward speed more aggressively
        if avoidance_urgency > 0.5:  # More than ~30 degrees steering
            # Scale down linear speed based on how much steering is needed
            avoidance_speed_factor = max(0.1, 1.0 - (avoidance_urgency / math.pi))
            linear_vel *= avoidance_speed_factor

        # Apply speed scale from obstacle avoidance
        linear_vel *= self.speed_scale

        # Enforce minimum speed if moving (but allow 0 when steering hard)
        if linear_vel > 0 and linear_vel < self.min_linear and avoidance_urgency < 0.5:
            linear_vel = self.min_linear

        # Rate limit angular velocity to prevent sudden direction changes
        # This smooths out oscillations when switching between avoidance and waypoint tracking
        angular_delta = angular_vel - self.prev_angular_vel
        max_delta = self.max_angular_accel  # Per control loop (10Hz = 0.1s)
        if abs(angular_delta) > max_delta:
            angular_vel = self.prev_angular_vel + (max_delta if angular_delta > 0 else -max_delta)
        self.prev_angular_vel = angular_vel

        return linear_vel, angular_vel

    def control_loop(self):
        """Main control loop"""
        # Update state
        self.update_navigation_state()

        # Publish goal direction for avoidance node (in robot frame)
        if self.target_x is not None:
            goal_msg = Float32()
            goal_msg.data = self.heading_error  # Direction to goal in robot frame
            self.goal_dir_pub.publish(goal_msg)

        # Compute velocity based on state
        cmd = Twist()

        if self.state == NavigationState.IDLE:
            pass  # Stopped

        elif self.state == NavigationState.NAVIGATING:
            # Check for critical danger - slow to minimum but don't stop
            if self.critical_danger:
                # Very slow forward movement while danger
                linear_vel, angular_vel = self.compute_velocity()
                cmd.linear.x = self.min_linear * 0.5  # Crawl speed
                cmd.angular.z = angular_vel
            elif self.rotate_in_place:
                # SKID-STEER: Rotate in place to face clear direction
                # Stop forward motion, just rotate based on steering adjustment
                cmd.linear.x = 0.0
                cmd.angular.z = self.angular_kp * self.steering_adjustment
                cmd.angular.z = self.clamp(cmd.angular.z, -self.max_angular, self.max_angular)
                # Ensure minimum rotation speed
                if abs(cmd.angular.z) < 0.3 and abs(self.steering_adjustment) > 0.1:
                    cmd.angular.z = 0.3 if self.steering_adjustment > 0 else -0.3
            else:
                linear_vel, angular_vel = self.compute_velocity()
                cmd.linear.x = linear_vel
                cmd.angular.z = angular_vel

        elif self.state == NavigationState.ARRIVED:
            pass  # Stopped at waypoint

        elif self.state == NavigationState.EMERGENCY_STOP:
            pass  # Full stop

        self.cmd_vel_pub.publish(cmd)

    def stop(self):
        """Stop the rover immediately"""
        self.cmd_vel_pub.publish(Twist())

    def publish_status(self):
        """Publish navigation status"""
        status = {
            'state': self.state.value,
            'armed': self.armed,
            'target': {
                'lat': self.target_lat,
                'lon': self.target_lon,
                'x': self.target_x,
                'y': self.target_y
            } if self.target_lat is not None else None,
            'current': {
                'lat': self.current_lat,
                'lon': self.current_lon,
                'x': self.current_x,
                'y': self.current_y,
                'heading': math.degrees(self.current_heading)
            },
            'distance_to_target': self.distance_to_target if self.target_lat else None,
            'heading_error': math.degrees(self.heading_error) if self.target_lat else None,
            'obstacle_avoidance': {
                'steering_adjustment': math.degrees(self.steering_adjustment),
                'speed_scale': self.speed_scale,
                'critical_danger': self.critical_danger,
                'rotate_in_place': self.rotate_in_place
            }
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    # =========================================================================
    # Visualization
    # =========================================================================

    def publish_waypoint_marker(self):
        """Publish waypoint marker for RViz"""
        if self.target_x is None or self.target_y is None:
            return

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoint"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = self.target_x
        marker.pose.position.y = self.target_y
        marker.pose.position.z = 0.5

        # Rotate to diamond shape
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.3826834
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 0.9238795

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # Blue color
        marker.color.r = 0.0
        marker.color.g = 0.4
        marker.color.b = 1.0
        marker.color.a = 0.9

        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        self.marker_pub.publish(marker)

    def delete_waypoint_marker(self):
        """Delete waypoint marker from RViz"""
        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "waypoint"
        marker.id = 0
        marker.action = Marker.DELETE
        self.marker_pub.publish(marker)

    # =========================================================================
    # Utilities
    # =========================================================================

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    @staticmethod
    def clamp(value: float, min_val: float, max_val: float) -> float:
        """Clamp value to range"""
        return max(min_val, min(max_val, value))


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = WaypointNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
