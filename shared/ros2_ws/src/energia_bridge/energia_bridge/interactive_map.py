#!/usr/bin/env python3
"""
Interactive Map - Click to send GPS waypoints
Shows top-down view with rover position, click to send waypoints
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
import tkinter as tk
from tkinter import ttk
import math
import requests


class InteractiveMapNode(Node):
    """Interactive map for waypoint selection"""

    def __init__(self, map_widget):
        super().__init__('interactive_map')
        self.map_widget = map_widget

        # Current robot position
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0

        # GPS origin (same as gps_bridge)
        self.origin_lat = 37.7749
        self.origin_lon = -122.4194
        self.meters_per_degree_lat = 111320.0
        self.meters_per_degree_lon = 111320.0 * math.cos(math.radians(self.origin_lat))

        # Subscribe to odometry for robot position
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('Interactive Map Node started')

    def odom_callback(self, msg):
        """Update robot position from odometry"""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y

        # Extract heading from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.robot_heading = math.atan2(siny_cosp, cosy_cosp)

        # Update map display
        self.map_widget.update_robot_pos(self.robot_x, self.robot_y, self.robot_heading)

    def xy_to_gps(self, x, y):
        """Convert local XY to GPS lat/lon"""
        lat_offset = y / self.meters_per_degree_lat
        lon_offset = x / self.meters_per_degree_lon

        latitude = self.origin_lat + lat_offset
        longitude = self.origin_lon + lon_offset

        return latitude, longitude


class MapWidget(tk.Canvas):
    """Canvas widget for interactive map"""

    def __init__(self, parent, ros_node, **kwargs):
        super().__init__(parent, **kwargs)
        self.ros_node = ros_node

        self.width = 800
        self.height = 600
        self.config(width=self.width, height=self.height, bg='white')

        # Map parameters
        self.scale = 20  # pixels per meter
        self.center_x = self.width / 2
        self.center_y = self.height / 2

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_heading = 0.0
        self.robot_marker = None

        # Waypoint markers
        self.waypoints = []

        # HTTP API endpoint
        self.api_url = "http://127.0.0.1:5001/api/target"

        self.draw_grid()
        self.draw_robot()

        # Bind click event
        self.bind('<Button-1>', self.on_click)

        self.update_display()

    def draw_grid(self):
        """Draw grid lines"""
        # Draw grid every 5 meters
        grid_spacing = 5 * self.scale  # pixels

        # Vertical lines
        for x in range(0, self.width, int(grid_spacing)):
            self.create_line(x, 0, x, self.height, fill='lightgray', width=1)

        # Horizontal lines
        for y in range(0, self.height, int(grid_spacing)):
            self.create_line(0, y, self.width, y, fill='lightgray', width=1)

        # Draw axes
        self.create_line(self.center_x, 0, self.center_x, self.height, fill='gray', width=2)
        self.create_line(0, self.center_y, self.width, self.center_y, fill='gray', width=2)

        # Labels
        self.create_text(self.center_x + 10, 20, text="North (+Y)", fill='blue', anchor='w')
        self.create_text(self.width - 20, self.center_y - 10, text="East (+X)", fill='blue', anchor='e')

    def draw_robot(self):
        """Draw robot marker"""
        if self.robot_marker:
            self.delete(self.robot_marker)

        # Convert robot position to screen coordinates
        screen_x = self.center_x + (self.robot_x * self.scale)
        screen_y = self.center_y - (self.robot_y * self.scale)  # Y is inverted

        # Draw robot as triangle pointing in heading direction
        size = 15
        angle = self.robot_heading

        # Triangle points (nose points in heading direction)
        points = [
            screen_x + size * math.cos(angle),
            screen_y - size * math.sin(angle),
            screen_x + size/2 * math.cos(angle + 2.5),
            screen_y - size/2 * math.sin(angle + 2.5),
            screen_x + size/2 * math.cos(angle - 2.5),
            screen_y - size/2 * math.sin(angle - 2.5)
        ]

        self.robot_marker = self.create_polygon(points, fill='orange', outline='black', width=2)

        # Draw position text
        self.create_text(screen_x, screen_y - 25,
                        text=f"({self.robot_x:.1f}, {self.robot_y:.1f})",
                        fill='black', font=('Arial', 9))

    def update_robot_pos(self, x, y, heading):
        """Update robot position"""
        self.robot_x = x
        self.robot_y = y
        self.robot_heading = heading

    def update_display(self):
        """Periodic update"""
        self.draw_robot()
        self.after(100, self.update_display)  # Update every 100ms

    def on_click(self, event):
        """Handle map click - send waypoint"""
        # Convert screen coordinates to world coordinates
        world_x = (event.x - self.center_x) / self.scale
        world_y = -(event.y - self.center_y) / self.scale  # Y is inverted

        # Convert to GPS
        lat, lon = self.ros_node.xy_to_gps(world_x, world_y)

        # Draw waypoint marker
        self.create_oval(event.x - 5, event.y - 5, event.x + 5, event.y + 5,
                        fill='red', outline='black', width=2)
        self.create_text(event.x, event.y - 15,
                        text=f"({world_x:.1f}, {world_y:.1f})",
                        fill='red', font=('Arial', 9, 'bold'))

        # Send to robot via HTTP
        self.send_waypoint(lat, lon)

        self.ros_node.get_logger().info(f'Clicked: ({world_x:.1f}, {world_y:.1f}) -> GPS: ({lat:.6f}, {lon:.6f})')

    def send_waypoint(self, lat, lon):
        """Send waypoint to robot via HTTP API"""
        try:
            response = requests.post(self.api_url, json={
                'latitude': lat,
                'longitude': lon
            }, timeout=2.0)

            if response.status_code == 200:
                self.ros_node.get_logger().info(f'Waypoint sent: {lat:.6f}, {lon:.6f}')
            else:
                self.ros_node.get_logger().error(f'Failed to send waypoint: {response.status_code}')
        except Exception as e:
            self.ros_node.get_logger().error(f'Error sending waypoint: {e}')


class InteractiveMapApp:
    """Main application"""

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Interactive Waypoint Map")

        # Instructions
        instructions = tk.Label(self.root,
                               text="Click anywhere on the map to send a GPS waypoint to the rover",
                               font=('Arial', 12, 'bold'), bg='yellow', pady=10)
        instructions.pack(fill=tk.X)

        # Info panel
        info_frame = tk.Frame(self.root)
        info_frame.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(info_frame, text="Orange Triangle = Rover", font=('Arial', 10)).pack(side=tk.LEFT, padx=10)
        tk.Label(info_frame, text="Red Dot = Waypoint", font=('Arial', 10)).pack(side=tk.LEFT, padx=10)
        tk.Label(info_frame, text="Grid = 5m spacing", font=('Arial', 10)).pack(side=tk.LEFT, padx=10)

        # Initialize ROS node
        rclpy.init()

        # Create map widget first (needed for ROS node)
        self.map_widget = MapWidget(self.root, None, width=800, height=600)
        self.map_widget.pack()

        # Create ROS node with map widget reference
        self.ros_node = InteractiveMapNode(self.map_widget)
        self.map_widget.ros_node = self.ros_node

        # Start ROS spinning in background
        import threading
        self.ros_thread = threading.Thread(target=self.spin_ros, daemon=True)
        self.ros_thread.start()

    def spin_ros(self):
        """Spin ROS in background thread"""
        rclpy.spin(self.ros_node)

    def run(self):
        """Run the application"""
        self.root.mainloop()
        self.ros_node.destroy_node()
        rclpy.shutdown()


def main():
    """Main function"""
    app = InteractiveMapApp()
    app.run()


if __name__ == '__main__':
    main()
