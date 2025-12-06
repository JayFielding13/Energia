#!/usr/bin/env python3
"""
GPS Bridge Node
Converts Gazebo GPS pose to NavSatFix format for compatibility
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, NavSatStatus
import math


class GPSBridgeNode(Node):
    """Converts /gps/pose to /gps/fix in NavSatFix format"""

    def __init__(self):
        super().__init__('gps_bridge')

        # Reference latitude/longitude for Gazebo world origin
        # You can adjust these to match your actual test location
        self.origin_lat = 37.7749  # San Francisco default
        self.origin_lon = -122.4194

        # Meters per degree latitude/longitude (approximate)
        self.meters_per_degree_lat = 111320.0
        self.meters_per_degree_lon = 111320.0 * math.cos(math.radians(self.origin_lat))

        # Publisher
        self.gps_fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # Subscriber
        self.gps_pose_sub = self.create_subscription(
            PoseStamped,
            '/gps/pose',
            self.gps_pose_callback,
            10
        )

        self.get_logger().info(f'GPS Bridge started - Origin: {self.origin_lat:.6f}, {self.origin_lon:.6f}')

    def gps_pose_callback(self, pose_msg):
        """Convert PoseStamped to NavSatFix"""

        # Convert local XY coordinates to lat/lon offset from origin
        x_meters = pose_msg.pose.position.x
        y_meters = pose_msg.pose.position.y
        z_meters = pose_msg.pose.position.z

        # Convert to lat/lon
        lat_offset = y_meters / self.meters_per_degree_lat
        lon_offset = x_meters / self.meters_per_degree_lon

        latitude = self.origin_lat + lat_offset
        longitude = self.origin_lon + lon_offset
        altitude = z_meters

        # Create NavSatFix message
        fix_msg = NavSatFix()
        fix_msg.header = pose_msg.header
        fix_msg.header.frame_id = 'gps'

        # GPS Status
        fix_msg.status.status = NavSatStatus.STATUS_FIX  # Simulated GPS always has fix
        fix_msg.status.service = NavSatStatus.SERVICE_GPS

        # Position
        fix_msg.latitude = latitude
        fix_msg.longitude = longitude
        fix_msg.altitude = altitude

        # Accuracy (simulated RTK-quality)
        fix_msg.position_covariance[0] = 0.01  # East variance (m^2) - 0.1m std dev
        fix_msg.position_covariance[4] = 0.01  # North variance (m^2)
        fix_msg.position_covariance[8] = 0.04  # Up variance (m^2) - 0.2m std dev
        fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        # Publish
        self.gps_fix_pub.publish(fix_msg)


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = GPSBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
