#!/usr/bin/env python3
"""
RTK Navigation Launch File for Energia Rover

Launches the MQTT-ROS2 bridge and GPS waypoint navigator to enable
real-world GPS waypoint navigation in simulation.

Usage:
  ros2 launch energia_bridge rtk_navigation.launch.py

With custom MQTT broker:
  ros2 launch energia_bridge rtk_navigation.launch.py mqtt_broker:=192.168.254.165

Enable Nav2 navigation:
  ros2 launch energia_bridge rtk_navigation.launch.py use_nav2:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('energia_bridge')
    config_file = os.path.join(pkg_dir, 'config', 'rtk_navigation.yaml')

    # Declare launch arguments
    mqtt_broker_arg = DeclareLaunchArgument(
        'mqtt_broker',
        default_value='100.66.67.11',
        description='MQTT broker IP address (rtkpi Tailscale: 100.66.67.11, home: 192.168.254.165)'
    )

    mqtt_port_arg = DeclareLaunchArgument(
        'mqtt_port',
        default_value='1883',
        description='MQTT broker port'
    )

    base_latitude_arg = DeclareLaunchArgument(
        'base_latitude',
        default_value='45.4296496',
        description='Base station latitude (decimal degrees) - at RTK base station cone'
    )

    base_longitude_arg = DeclareLaunchArgument(
        'base_longitude',
        default_value='-122.8398184',
        description='Base station longitude (decimal degrees) - at RTK base station cone'
    )

    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='false',
        description='Use Nav2 for navigation (true) or simple proportional control (false)'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.5',
        description='Distance tolerance for reaching waypoint (meters)'
    )

    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Enable waypoint visualization in Gazebo/RViz'
    )

    # MQTT-ROS2 Bridge Node
    mqtt_bridge_node = Node(
        package='energia_bridge',
        executable='mqtt_ros2_bridge',
        name='mqtt_ros2_bridge',
        output='screen',
        parameters=[{
            'mqtt_broker': LaunchConfiguration('mqtt_broker'),
            'mqtt_port': LaunchConfiguration('mqtt_port'),
            'base_latitude': LaunchConfiguration('base_latitude'),
            'base_longitude': LaunchConfiguration('base_longitude'),
            'robot_name': 'baby-tessla',
        }],
    )

    # GPS Waypoint Navigator Node
    waypoint_navigator_node = Node(
        package='energia_bridge',
        executable='gps_waypoint_navigator',
        name='gps_waypoint_navigator',
        output='screen',
        parameters=[{
            'use_nav2': LaunchConfiguration('use_nav2'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'max_linear_speed': 0.5,
            'max_angular_speed': 1.0,
            'linear_gain': 0.5,
            'angular_gain': 2.0,
        }],
    )

    # Waypoint Visualizer Node (for Gazebo/RViz)
    waypoint_visualizer_node = Node(
        package='energia_bridge',
        executable='waypoint_visualizer',
        name='waypoint_visualizer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('visualize')),
        parameters=[{
            'waypoint_marker_scale': 0.5,
            'path_line_width': 0.05,
            'max_path_points': 500,
            'path_sample_distance': 0.1,
        }],
    )

    return LaunchDescription([
        mqtt_broker_arg,
        mqtt_port_arg,
        base_latitude_arg,
        base_longitude_arg,
        use_nav2_arg,
        goal_tolerance_arg,
        visualize_arg,
        mqtt_bridge_node,
        waypoint_navigator_node,
        waypoint_visualizer_node,
    ])
