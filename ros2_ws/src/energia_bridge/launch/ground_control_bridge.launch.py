#!/usr/bin/env python3
"""
Ground Control Bridge Launch File
Launches HTTP bridge and GPS bridge for Mobile RTK Control Module integration
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with bridge nodes"""

    # GPS Bridge Node - converts /gps/pose to /gps/fix
    gps_bridge = Node(
        package='jetson_rover_bridge',
        executable='gps_bridge',
        name='gps_bridge',
        output='screen',
        parameters=[]
    )

    # HTTP Bridge Node - provides REST API for ground control
    http_bridge = Node(
        package='jetson_rover_bridge',
        executable='http_bridge',
        name='http_bridge',
        output='screen',
        parameters=[]
    )

    return LaunchDescription([
        gps_bridge,
        http_bridge
    ])
