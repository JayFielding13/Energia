#!/usr/bin/env python3
"""
MQTT-ROS2 Bridge for Energia Rover

Connects to the RTK system's MQTT broker and bridges messages to ROS2 topics.
This allows the simulation to receive real GPS data from the Mobile RTK Control Module.

MQTT Topics Subscribed:
  - beacon/terminal/position: Mobile RTK position updates
  - robot/baby-tessla/command/waypoint: Waypoint commands from control terminal
  - rtk/base/status: Base station status

ROS2 Topics Published:
  - /energia/gps/position (geometry_msgs/PoseStamped): Current GPS position in local coords
  - /energia/gps/waypoint (geometry_msgs/PoseStamped): Target waypoint in local coords
  - /energia/rtk/status (std_msgs/String): RTK system status
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import paho.mqtt.client as mqtt
import json
import math
from datetime import datetime


class MqttRos2Bridge(Node):
    """Bridge between MQTT RTK system and ROS2."""

    # WGS84 ellipsoid constants
    WGS84_A = 6378137.0  # Semi-major axis (meters)
    WGS84_B = 6356752.314245  # Semi-minor axis (meters)
    WGS84_E2 = 0.00669437999014  # First eccentricity squared

    def __init__(self):
        super().__init__('mqtt_ros2_bridge')

        # Declare parameters
        self.declare_parameter('mqtt_broker', '100.66.67.11')  # rtkpi Tailscale IP
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('base_latitude', 45.4296496)  # RTK base station GPS
        self.declare_parameter('base_longitude', -122.8398184)
        self.declare_parameter('robot_name', 'baby-tessla')
        # Local offset from GPS origin to map origin (base station position on map)
        self.declare_parameter('local_offset_x', -2.09)  # meters east
        self.declare_parameter('local_offset_y', 15.84)  # meters north

        # Get parameters
        self.mqtt_broker = self.get_parameter('mqtt_broker').value
        self.mqtt_port = self.get_parameter('mqtt_port').value
        self.base_lat = self.get_parameter('base_latitude').value
        self.base_lon = self.get_parameter('base_longitude').value
        self.robot_name = self.get_parameter('robot_name').value
        self.local_offset_x = self.get_parameter('local_offset_x').value
        self.local_offset_y = self.get_parameter('local_offset_y').value

        # Precompute base station ECEF coordinates
        self.base_ecef = self.geodetic_to_ecef(self.base_lat, self.base_lon, 0.0)

        # ROS2 Publishers
        self.position_pub = self.create_publisher(PoseStamped, '/energia/gps/position', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/energia/gps/waypoint', 10)
        self.status_pub = self.create_publisher(String, '/energia/rtk/status', 10)

        # MQTT Client setup
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, "energia_ros2_bridge")
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect

        # MQTT Topics
        self.topics = [
            ('beacon/terminal/position', 0),  # Mobile RTK position
            (f'robot/{self.robot_name}/command/waypoint', 0),  # Waypoint commands
            ('rtk/base/status', 0),  # Base station status
        ]

        # Connect to MQTT broker
        self.connect_mqtt()

        # Timer for connection monitoring
        self.create_timer(5.0, self.check_connection)

        self.get_logger().info(f'MQTT-ROS2 Bridge initialized')
        self.get_logger().info(f'  MQTT Broker: {self.mqtt_broker}:{self.mqtt_port}')
        self.get_logger().info(f'  Base Station GPS: {self.base_lat:.7f}, {self.base_lon:.7f}')
        self.get_logger().info(f'  Base Station Map Offset: ({self.local_offset_x:.2f}, {self.local_offset_y:.2f}) m')

    def connect_mqtt(self):
        """Connect to MQTT broker."""
        try:
            self.get_logger().info(f'Connecting to MQTT broker at {self.mqtt_broker}:{self.mqtt_port}...')
            self.mqtt_client.connect(self.mqtt_broker, self.mqtt_port, 60)
            self.mqtt_client.loop_start()
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT broker: {e}')

    def on_mqtt_connect(self, client, userdata, flags, rc, properties=None):
        """Callback when connected to MQTT broker."""
        if rc == 0:
            self.get_logger().info('Connected to MQTT broker')
            # Subscribe to topics
            for topic, qos in self.topics:
                client.subscribe(topic, qos)
                self.get_logger().info(f'  Subscribed to: {topic}')
        else:
            self.get_logger().error(f'MQTT connection failed with code: {rc}')

    def on_mqtt_disconnect(self, client, userdata, rc, properties=None):
        """Callback when disconnected from MQTT broker."""
        self.get_logger().warning(f'Disconnected from MQTT broker (code: {rc})')
        if rc != 0:
            self.get_logger().info('Attempting to reconnect...')

    def on_mqtt_message(self, client, userdata, msg):
        """Callback when MQTT message received."""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')

            if topic == 'beacon/terminal/position':
                self.handle_position_message(payload)
            elif topic == f'robot/{self.robot_name}/command/waypoint':
                self.handle_waypoint_command(payload)
            elif topic == 'rtk/base/status':
                self.handle_status_message(payload)
            else:
                self.get_logger().debug(f'Unknown topic: {topic}')

        except Exception as e:
            self.get_logger().error(f'Error processing MQTT message: {e}')

    def handle_position_message(self, payload):
        """Handle position update from mobile RTK terminal."""
        try:
            data = json.loads(payload)
            lat = data.get('latitude')
            lon = data.get('longitude')
            heading = data.get('heading', 0.0)

            if lat is not None and lon is not None:
                # Convert GPS to local coordinates
                x, y = self.gps_to_local(lat, lon)

                # Create and publish PoseStamped
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0

                # Convert heading to quaternion (yaw only)
                yaw_rad = math.radians(heading)
                pose.pose.orientation.z = math.sin(yaw_rad / 2.0)
                pose.pose.orientation.w = math.cos(yaw_rad / 2.0)

                self.position_pub.publish(pose)
                self.get_logger().debug(f'Position: ({lat:.6f}, {lon:.6f}) -> local ({x:.2f}, {y:.2f})')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse position JSON: {e}')

    def handle_waypoint_command(self, payload):
        """Handle waypoint command from control terminal."""
        try:
            data = json.loads(payload)
            lat = data.get('latitude')
            lon = data.get('longitude')
            waypoint_id = data.get('waypoint_id', 'unknown')

            if lat is not None and lon is not None:
                # Convert GPS to local coordinates
                x, y = self.gps_to_local(lat, lon)

                # Create and publish PoseStamped
                pose = PoseStamped()
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.header.frame_id = 'map'
                pose.pose.position.x = x
                pose.pose.position.y = y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0

                self.waypoint_pub.publish(pose)
                self.get_logger().info(f'Waypoint {waypoint_id}: ({lat:.6f}, {lon:.6f}) -> local ({x:.2f}, {y:.2f})')

        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse waypoint JSON: {e}')

    def handle_status_message(self, payload):
        """Handle RTK base station status."""
        status_msg = String()
        status_msg.data = payload
        self.status_pub.publish(status_msg)

    def geodetic_to_ecef(self, lat, lon, alt):
        """Convert geodetic coordinates (lat, lon, alt) to ECEF (X, Y, Z)."""
        lat_rad = math.radians(lat)
        lon_rad = math.radians(lon)

        # Radius of curvature in the prime vertical
        N = self.WGS84_A / math.sqrt(1 - self.WGS84_E2 * math.sin(lat_rad) ** 2)

        x = (N + alt) * math.cos(lat_rad) * math.cos(lon_rad)
        y = (N + alt) * math.cos(lat_rad) * math.sin(lon_rad)
        z = (N * (1 - self.WGS84_E2) + alt) * math.sin(lat_rad)

        return (x, y, z)

    def gps_to_local(self, lat, lon, alt=0.0):
        """
        Convert GPS coordinates to local ENU (East-North-Up) coordinates.
        Uses the base station location as the origin, then applies local offset
        to align with the satellite imagery map.

        Returns (x, y) where:
          x = East (meters in map frame)
          y = North (meters in map frame)
        """
        # Convert target point to ECEF
        target_ecef = self.geodetic_to_ecef(lat, lon, alt)

        # Vector from base to target in ECEF
        dx = target_ecef[0] - self.base_ecef[0]
        dy = target_ecef[1] - self.base_ecef[1]
        dz = target_ecef[2] - self.base_ecef[2]

        # Rotation matrix from ECEF to ENU at base station
        lat_rad = math.radians(self.base_lat)
        lon_rad = math.radians(self.base_lon)

        sin_lat = math.sin(lat_rad)
        cos_lat = math.cos(lat_rad)
        sin_lon = math.sin(lon_rad)
        cos_lon = math.cos(lon_rad)

        # ENU coordinates relative to base station
        east = -sin_lon * dx + cos_lon * dy
        north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
        # up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz

        # Apply local offset to convert from GPS origin to map origin
        # The base station is located at (local_offset_x, local_offset_y) on the map
        map_x = east + self.local_offset_x
        map_y = north + self.local_offset_y

        return (map_x, map_y)

    def check_connection(self):
        """Periodic check of MQTT connection."""
        if not self.mqtt_client.is_connected():
            self.get_logger().warning('MQTT connection lost, attempting reconnect...')
            try:
                self.mqtt_client.reconnect()
            except Exception as e:
                self.get_logger().error(f'Reconnection failed: {e}')

    def destroy_node(self):
        """Clean up on shutdown."""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttRos2Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
