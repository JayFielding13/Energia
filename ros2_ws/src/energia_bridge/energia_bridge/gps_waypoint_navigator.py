#!/usr/bin/env python3
"""
GPS Waypoint Navigator for Energia Rover

Receives GPS waypoints (in local coordinates) from the MQTT-ROS2 bridge
and commands the rover to navigate to them using the Nav2 stack.

Subscribes:
  - /energia/gps/waypoint (geometry_msgs/PoseStamped): Target waypoint in local coords
  - /energia/gps/position (geometry_msgs/PoseStamped): Current GPS position
  - /odom (nav_msgs/Odometry): Robot odometry for position updates

Publishes:
  - /cmd_vel (geometry_msgs/Twist): Velocity commands when using simple navigation
  - /goal_pose (geometry_msgs/PoseStamped): Goal for Nav2 navigation

Actions:
  - /navigate_to_pose (nav2_msgs/NavigateToPose): Nav2 action for path planning
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math


class GpsWaypointNavigator(Node):
    """Navigate to GPS waypoints using Nav2 or simple proportional control."""

    def __init__(self):
        super().__init__('gps_waypoint_navigator')

        # Declare parameters
        self.declare_parameter('use_nav2', False)  # Use Nav2 action or simple control
        self.declare_parameter('goal_tolerance', 0.5)  # Distance to consider goal reached (m)
        self.declare_parameter('max_linear_speed', 0.5)  # Maximum forward speed (m/s)
        self.declare_parameter('max_angular_speed', 1.0)  # Maximum rotation speed (rad/s)
        self.declare_parameter('linear_gain', 0.5)  # Proportional gain for linear velocity
        self.declare_parameter('angular_gain', 2.0)  # Proportional gain for angular velocity

        # Get parameters
        self.use_nav2 = self.get_parameter('use_nav2').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.linear_gain = self.get_parameter('linear_gain').value
        self.angular_gain = self.get_parameter('angular_gain').value

        # State variables
        self.current_pose = None
        self.current_waypoint = None
        self.navigating = False

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseStamped, '/energia/gps/waypoint', self.waypoint_callback, 10)
        self.gps_position_sub = self.create_subscription(
            PoseStamped, '/energia/gps/position', self.gps_position_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        # Subscribe to RViz 2D Goal Pose for click-to-navigate
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.waypoint_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.goal_reached_pub = self.create_publisher(Bool, '/energia/goal_reached', 10)

        # Nav2 action client (if enabled)
        if self.use_nav2:
            try:
                from nav2_msgs.action import NavigateToPose
                self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
                self.get_logger().info('Nav2 action client initialized')
            except ImportError:
                self.get_logger().warning('nav2_msgs not available, falling back to simple control')
                self.use_nav2 = False

        # Control loop timer (10 Hz)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('GPS Waypoint Navigator initialized')
        self.get_logger().info(f'  Use Nav2: {self.use_nav2}')
        self.get_logger().info(f'  Goal tolerance: {self.goal_tolerance} m')

    def waypoint_callback(self, msg):
        """Handle new waypoint from MQTT bridge."""
        self.current_waypoint = msg
        self.navigating = True
        self.get_logger().info(
            f'New waypoint received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

        if self.use_nav2:
            self.send_nav2_goal(msg)
        else:
            # For simple control, just store the waypoint
            pass

    def gps_position_callback(self, msg):
        """Handle GPS position update from MQTT bridge."""
        # Could be used to correct odometry drift
        # For now, just log occasionally
        pass

    def odom_callback(self, msg):
        """Update current robot pose from odometry."""
        self.current_pose = PoseStamped()
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def send_nav2_goal(self, waypoint):
        """Send waypoint to Nav2 as a navigation goal."""
        if not self.use_nav2:
            return

        try:
            from nav2_msgs.action import NavigateToPose

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = waypoint
            goal_msg.pose.header.frame_id = 'map'

            self.get_logger().info('Sending goal to Nav2...')

            if not self.nav2_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warning('Nav2 action server not available')
                return

            self.nav2_future = self.nav2_client.send_goal_async(
                goal_msg, feedback_callback=self.nav2_feedback_callback)
            self.nav2_future.add_done_callback(self.nav2_goal_response_callback)

        except Exception as e:
            self.get_logger().error(f'Failed to send Nav2 goal: {e}')

    def nav2_feedback_callback(self, feedback_msg):
        """Handle Nav2 navigation feedback."""
        feedback = feedback_msg.feedback
        # Could publish progress updates here

    def nav2_goal_response_callback(self, future):
        """Handle Nav2 goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Nav2 goal rejected')
            self.navigating = False
            return

        self.get_logger().info('Nav2 goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.nav2_result_callback)

    def nav2_result_callback(self, future):
        """Handle Nav2 navigation result."""
        result = future.result().result
        self.navigating = False
        self.get_logger().info('Nav2 navigation completed')

        # Publish goal reached
        msg = Bool()
        msg.data = True
        self.goal_reached_pub.publish(msg)

    def control_loop(self):
        """Simple proportional controller for waypoint navigation."""
        if self.use_nav2 or not self.navigating:
            return

        if self.current_pose is None or self.current_waypoint is None:
            return

        # Calculate distance and angle to waypoint
        dx = self.current_waypoint.pose.position.x - self.current_pose.pose.position.x
        dy = self.current_waypoint.pose.position.y - self.current_pose.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        # Check if goal reached
        if distance < self.goal_tolerance:
            self.stop_robot()
            self.navigating = False
            self.get_logger().info(f'Goal reached! Distance: {distance:.2f} m')

            # Publish goal reached
            msg = Bool()
            msg.data = True
            self.goal_reached_pub.publish(msg)
            return

        # Calculate angle to waypoint
        target_angle = math.atan2(dy, dx)

        # Get current yaw from quaternion
        q = self.current_pose.pose.orientation
        current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Calculate angular error (normalized to [-pi, pi])
        angle_error = target_angle - current_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Proportional control
        cmd = Twist()

        # Angular velocity
        angular_vel = self.angular_gain * angle_error
        angular_vel = max(-self.max_angular_speed, min(self.max_angular_speed, angular_vel))
        cmd.angular.z = angular_vel

        # Linear velocity (reduce when turning sharply)
        turn_factor = 1.0 - min(abs(angle_error) / math.pi, 1.0)
        linear_vel = self.linear_gain * distance * turn_factor
        linear_vel = max(0.0, min(self.max_linear_speed, linear_vel))
        cmd.linear.x = linear_vel

        self.cmd_vel_pub.publish(cmd)

        # Log progress occasionally
        self.get_logger().debug(
            f'Distance: {distance:.2f} m, Angle error: {math.degrees(angle_error):.1f} deg')

    def stop_robot(self):
        """Stop the robot by publishing zero velocities."""
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)

    def destroy_node(self):
        """Clean up on shutdown."""
        self.stop_robot()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GpsWaypointNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
