#!/usr/bin/env python3
"""
Ground Control Bridge Launch File
Launches HTTP bridge, GPS bridge, waypoint navigator, and obstacle avoidance
for Mobile RTK Control Module integration with autonomous navigation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with bridge and navigation nodes"""

    # Launch arguments
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='true',
        description='Set to false for real hardware deployment'
    )

    simulation_mode = LaunchConfiguration('simulation_mode')

    # GPS Bridge Node - converts /gps/pose to /gps/fix
    gps_bridge = Node(
        package='energia_bridge',
        executable='gps_bridge',
        name='gps_bridge',
        output='screen',
        parameters=[]
    )

    # HTTP Bridge Node - provides REST API for ground control
    http_bridge = Node(
        package='energia_bridge',
        executable='http_bridge',
        name='http_bridge',
        output='screen',
        parameters=[]
    )

    # Reactive Obstacle Avoidance - VFH-based steering
    reactive_avoidance = Node(
        package='energia_bridge',
        executable='reactive_obstacle_avoidance',
        name='reactive_obstacle_avoidance',
        output='screen',
        parameters=[]
    )

    # Waypoint Navigator - autonomous GPS waypoint navigation
    waypoint_navigator = Node(
        package='energia_bridge',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'simulation_mode': simulation_mode,
            'arrival_distance': 0.5,
            'heading_tolerance': 15.0,
            'max_linear_speed': 0.5,
            'max_angular_speed': 0.8,
            'origin_lat': 37.7749,
            'origin_lon': -122.4194,
        }]
    )

    return LaunchDescription([
        simulation_mode_arg,
        gps_bridge,
        http_bridge,
        reactive_avoidance,
        waypoint_navigator,
    ])
