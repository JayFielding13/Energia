#!/usr/bin/env python3
"""
Waypoint Visualizer for Gazebo

Visualizes GPS waypoints and navigation path in Gazebo Harmonic using
ROS 2 marker messages that are rendered via ros_gz_bridge.

Subscribes:
  - /energia/gps/waypoint (geometry_msgs/PoseStamped): Target waypoint
  - /energia/gps/position (geometry_msgs/PoseStamped): Current GPS position
  - /odom (nav_msgs/Odometry): Robot odometry for tracking path
  - /energia/goal_reached (std_msgs/Bool): Goal reached notification

Publishes:
  - /waypoint_markers (visualization_msgs/MarkerArray): Waypoint markers for RViz/Gazebo
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import math


class WaypointVisualizer(Node):
    """Visualize waypoints and navigation path in Gazebo."""

    def __init__(self):
        super().__init__('waypoint_visualizer')

        # Declare parameters
        self.declare_parameter('waypoint_marker_scale', 0.5)  # Size of waypoint markers
        self.declare_parameter('path_line_width', 0.05)  # Width of path line
        self.declare_parameter('max_path_points', 500)  # Maximum points in path trail
        self.declare_parameter('path_sample_distance', 0.1)  # Min distance between path points

        # Get parameters
        self.waypoint_scale = self.get_parameter('waypoint_marker_scale').value
        self.path_line_width = self.get_parameter('path_line_width').value
        self.max_path_points = self.get_parameter('max_path_points').value
        self.path_sample_distance = self.get_parameter('path_sample_distance').value

        # State
        self.waypoints = []  # List of (x, y, reached) tuples
        self.current_waypoint_idx = -1
        self.path_points = []  # Trail of robot positions
        self.last_path_point = None
        self.robot_position = None

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseStamped, '/energia/gps/waypoint', self.waypoint_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.goal_reached_sub = self.create_subscription(
            Bool, '/energia/goal_reached', self.goal_reached_callback, 10)
        # Subscribe to RViz 2D Goal Pose for click-to-navigate visualization
        self.goal_pose_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.waypoint_callback, 10)

        # Publisher
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        # Timer for publishing markers (5 Hz)
        self.marker_timer = self.create_timer(0.2, self.publish_markers)

        self.get_logger().info('Waypoint Visualizer initialized')
        self.get_logger().info('  Markers published to /waypoint_markers')
        self.get_logger().info('  Use ros_gz_bridge to visualize in Gazebo')

    def waypoint_callback(self, msg):
        """Handle new waypoint."""
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Add new waypoint
        self.waypoints.append({'x': x, 'y': y, 'reached': False})
        self.current_waypoint_idx = len(self.waypoints) - 1

        self.get_logger().info(f'Waypoint {self.current_waypoint_idx} added: ({x:.2f}, {y:.2f})')

    def odom_callback(self, msg):
        """Track robot position for path visualization."""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.robot_position = (x, y)

        # Add to path if moved enough
        if self.last_path_point is None:
            self.path_points.append(Point(x=x, y=y, z=0.05))
            self.last_path_point = (x, y)
        else:
            dx = x - self.last_path_point[0]
            dy = y - self.last_path_point[1]
            dist = math.sqrt(dx * dx + dy * dy)

            if dist >= self.path_sample_distance:
                self.path_points.append(Point(x=x, y=y, z=0.05))
                self.last_path_point = (x, y)

                # Limit path length
                if len(self.path_points) > self.max_path_points:
                    self.path_points.pop(0)

    def goal_reached_callback(self, msg):
        """Mark current waypoint as reached."""
        if msg.data and 0 <= self.current_waypoint_idx < len(self.waypoints):
            self.waypoints[self.current_waypoint_idx]['reached'] = True
            self.get_logger().info(f'Waypoint {self.current_waypoint_idx} reached!')

    def publish_markers(self):
        """Publish all visualization markers."""
        marker_array = MarkerArray()
        marker_id = 0

        # Clear old markers first
        clear_marker = Marker()
        clear_marker.header.frame_id = 'odom'
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = 'waypoints'
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        # Waypoint markers
        for i, wp in enumerate(self.waypoints):
            # Cylinder for waypoint
            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'waypoints'
            marker.id = marker_id
            marker_id += 1
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = wp['x']
            marker.pose.position.y = wp['y']
            marker.pose.position.z = self.waypoint_scale / 2.0

            marker.scale.x = self.waypoint_scale
            marker.scale.y = self.waypoint_scale
            marker.scale.z = self.waypoint_scale

            # Color: green if reached, red if current, yellow if pending
            if wp['reached']:
                marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=0.8)
            elif i == self.current_waypoint_idx:
                marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
            else:
                marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.8)

            marker_array.markers.append(marker)

            # Text label for waypoint number
            text_marker = Marker()
            text_marker.header.frame_id = 'odom'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'waypoint_labels'
            text_marker.id = marker_id
            marker_id += 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = wp['x']
            text_marker.pose.position.y = wp['y']
            text_marker.pose.position.z = self.waypoint_scale + 0.3

            text_marker.scale.z = 0.3  # Text height
            text_marker.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text_marker.text = f'WP{i}'

            marker_array.markers.append(text_marker)

        # Line to current waypoint
        if (self.robot_position is not None and
            0 <= self.current_waypoint_idx < len(self.waypoints) and
            not self.waypoints[self.current_waypoint_idx]['reached']):

            wp = self.waypoints[self.current_waypoint_idx]
            line_marker = Marker()
            line_marker.header.frame_id = 'odom'
            line_marker.header.stamp = self.get_clock().now().to_msg()
            line_marker.ns = 'target_line'
            line_marker.id = marker_id
            marker_id += 1
            line_marker.type = Marker.LINE_STRIP
            line_marker.action = Marker.ADD

            line_marker.scale.x = 0.03  # Line width

            # Dashed effect using color gradient
            line_marker.color = ColorRGBA(r=1.0, g=0.5, b=0.0, a=0.6)

            line_marker.points.append(
                Point(x=self.robot_position[0], y=self.robot_position[1], z=0.1))
            line_marker.points.append(
                Point(x=wp['x'], y=wp['y'], z=0.1))

            marker_array.markers.append(line_marker)

        # Path trail
        if len(self.path_points) >= 2:
            path_marker = Marker()
            path_marker.header.frame_id = 'odom'
            path_marker.header.stamp = self.get_clock().now().to_msg()
            path_marker.ns = 'path_trail'
            path_marker.id = marker_id
            marker_id += 1
            path_marker.type = Marker.LINE_STRIP
            path_marker.action = Marker.ADD

            path_marker.scale.x = self.path_line_width
            path_marker.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=0.7)

            path_marker.points = self.path_points

            marker_array.markers.append(path_marker)

        # Publish all markers
        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
