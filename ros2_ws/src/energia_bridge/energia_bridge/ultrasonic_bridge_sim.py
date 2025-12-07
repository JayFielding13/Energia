#!/usr/bin/env python3
"""
Ultrasonic Bridge (Simulation Mode)

This is a modified version of ultrasonic_bridge.py for use in Gazebo simulation.
Instead of reading from serial port, it subscribes to /esp32/ultrasonic_json topic
which contains the same JSON format as the real ESP32.

This ensures identical data processing pipeline in both simulation and real robot.
"""
import json
import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Range, PointCloud2, PointField
from std_msgs.msg import Header, String
import struct


class UltrasonicBridgeSim(Node):
    def __init__(self):
        super().__init__('ultrasonic_bridge_sim')

        # Declare parameters (same as real bridge)
        self.declare_parameter('filter_window', 5)
        self.declare_parameter('outlier_threshold', 0.5)

        # Get parameters
        self.filter_window = self.get_parameter('filter_window').value
        self.outlier_threshold = self.get_parameter('outlier_threshold').value

        # Sensor configuration (same as real bridge)
        self.sensors = {
            'front': 0,
            'corner_left': -45,
            'corner_right': 45,
            'side_left': -90,
            'side_right': 90,
            'rear': 180
        }

        # Moving average filters
        self.filters = {name: deque(maxlen=self.filter_window) for name in self.sensors.keys()}

        # QoS profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribe to simulated ESP32 JSON output
        self.json_sub = self.create_subscription(
            String,
            '/esp32/ultrasonic_json',
            self.json_callback,
            10
        )

        # Create Range publishers for each sensor
        self.range_publishers = {}
        for name in self.sensors.keys():
            topic = f'/ultrasonic/{name}'
            self.range_publishers[name] = self.create_publisher(Range, topic, qos_profile)

        # Create PointCloud2 publisher
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/ultrasonic/pointcloud',
            qos_profile
        )

        # Statistics
        self.message_count = 0
        self.error_count = 0

        self.get_logger().info('Ultrasonic bridge (simulation mode) started')
        self.get_logger().info('Subscribed to /esp32/ultrasonic_json')
        self.get_logger().info(f'Filter window: {self.filter_window} samples')

    def json_callback(self, msg):
        """Process JSON message from simulated ESP32"""
        try:
            # Parse JSON (same format as real ESP32)
            data = json.loads(msg.data)

            # Validate message format
            if 'timestamp' not in data or 'sensors' not in data:
                return

            # Process sensor readings (SAME CODE as real bridge)
            sensor_data = {}
            for sensor in data['sensors']:
                name = sensor['name']
                distance = sensor['distance']
                valid = sensor['valid']
                angle = sensor['angle']

                # Apply filtering
                filtered_distance = self.filter_reading(name, distance, valid)

                sensor_data[name] = {
                    'distance': filtered_distance,
                    'angle': angle,
                    'valid': filtered_distance is not None
                }

            # Publish Range messages
            stamp = self.get_clock().now().to_msg()
            for name, reading in sensor_data.items():
                self.publish_range(name, reading, stamp)

            # Publish PointCloud2
            self.publish_pointcloud(sensor_data, stamp)

            # Update statistics
            self.message_count += 1

        except json.JSONDecodeError as e:
            self.error_count += 1
            if self.error_count % 10 == 0:
                self.get_logger().warn(f'JSON decode error: {e}')

    def filter_reading(self, sensor_name, distance, valid):
        """Apply moving average filter with outlier rejection (SAME as real bridge)"""
        if not valid:
            return None

        filter_queue = self.filters[sensor_name]

        # Outlier rejection
        if len(filter_queue) > 0:
            avg = sum(filter_queue) / len(filter_queue)
            if abs(distance - avg) > self.outlier_threshold:
                return avg

        # Add to filter queue
        filter_queue.append(distance)

        # Return moving average
        return sum(filter_queue) / len(filter_queue)

    def publish_range(self, sensor_name, reading, stamp):
        """Publish Range message (SAME as real bridge)"""
        msg = Range()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = f'ultrasonic_{sensor_name}'

        # Range parameters for AJ-SR04M
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = math.radians(15)
        msg.min_range = 0.02
        msg.max_range = 6.0

        if reading['valid']:
            msg.range = reading['distance']
        else:
            msg.range = msg.max_range

        self.range_publishers[sensor_name].publish(msg)

    def publish_pointcloud(self, sensor_data, stamp):
        """Publish PointCloud2 for RViz (SAME as real bridge)"""
        points = []

        for name, reading in sensor_data.items():
            if not reading['valid']:
                continue

            angle_deg = reading['angle']
            distance = reading['distance']

            # Convert to radians
            angle_rad = math.radians(angle_deg)

            # Cartesian coordinates
            x = distance * math.cos(angle_rad)
            y = distance * math.sin(angle_rad)
            z = 0.0

            # Add point with intensity
            intensity = 1.0 - (distance / 6.0)
            points.append([x, y, z, intensity])

        if not points:
            return

        # Create PointCloud2 message
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = stamp
        msg.header.frame_id = 'base_link'

        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * len(points)
        msg.is_dense = True
        msg.width = len(points)
        msg.height = 1

        # Pack point data
        msg.data = b''.join([struct.pack('ffff', *point) for point in points])

        self.pointcloud_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicBridgeSim()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
