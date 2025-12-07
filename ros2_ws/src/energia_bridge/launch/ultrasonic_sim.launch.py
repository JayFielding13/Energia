#!/usr/bin/env python3
"""
Launch file for ultrasonic sensor simulation

This launch file starts the ESP32-style ultrasonic simulation system:
1. ultrasonic_gazebo_simulator - Simulates ESP32 publishing JSON sensor data
2. ultrasonic_bridge_sim - Processes JSON and publishes Range messages (same as real robot)

This architecture matches the real robot exactly:
Real:  ESP32 → JSON over serial → ultrasonic_bridge.py → Range messages
Sim:   Simulator → JSON over ROS → ultrasonic_bridge_sim.py → Range messages
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ESP32 simulator - publishes JSON to /esp32/ultrasonic_json
        Node(
            package='jetson_rover_bridge',
            executable='ultrasonic_gazebo_simulator',
            name='esp32_ultrasonic_simulator',
            output='screen',
            parameters=[],
        ),

        # Ultrasonic bridge (simulation mode) - reads JSON, publishes Range messages
        Node(
            package='jetson_rover_bridge',
            executable='ultrasonic_bridge_sim',
            name='ultrasonic_bridge_sim',
            output='screen',
            parameters=[
                {'filter_window': 5},
                {'outlier_threshold': 0.5},
            ],
        ),
    ])
