#!/usr/bin/env python3
"""
Simple sensor monitoring script
Subscribes to all ultrasonic sensors and prints distances

Usage:
    python3 sensor_monitor.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')

        # Subscribe to all ultrasonic sensors
        self.sensors = ['front', 'corner_left', 'corner_right',
                       'side_left', 'side_right', 'rear']

        self.distances = {sensor: float('inf') for sensor in self.sensors}

        for sensor in self.sensors:
            self.create_subscription(
                Range,
                f'/ultrasonic/{sensor}',
                lambda msg, s=sensor: self.sensor_callback(msg, s),
                10
            )

        # Print status every second
        self.create_timer(1.0, self.print_status)

        self.get_logger().info('Sensor monitor started!')
        self.get_logger().info('Monitoring all 6 ultrasonic sensors...')

    def sensor_callback(self, msg, sensor_name):
        """Update distance for specific sensor"""
        self.distances[sensor_name] = msg.range

    def print_status(self):
        """Print current sensor status"""
        self.get_logger().info('=' * 50)
        self.get_logger().info('Ultrasonic Sensor Status')
        self.get_logger().info('=' * 50)

        for sensor, distance in self.distances.items():
            # Determine status
            if distance < 0.3:
                status = '🚨 CRITICAL'
            elif distance < 0.5:
                status = '⚠️  WARNING'
            elif distance < 1.0:
                status = '⚡ CAUTION'
            else:
                status = '✅ CLEAR'

            # Format output
            self.get_logger().info(
                f'{sensor:15s}: {distance:5.2f}m  {status}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
