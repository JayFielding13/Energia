#!/usr/bin/env python3
"""
SLAM Toolbox Launch File for Jetson Rover

This launch file works on both:
1. Simulation (Gazebo) - Uses /scan from simulated LiDAR
2. Real Hardware (Jetson Orin Nano) - Uses /scan from RPLiDAR A1

The SLAM node subscribes to /scan and /odom topics which are published
by both Gazebo plugins (simulation) and real hardware drivers.

Usage:
  Simulation: ros2 launch jetson_rover_bridge slam_toolbox.launch.py
  Real Robot: ros2 launch jetson_rover_bridge slam_toolbox.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Get the path to the config file
    pkg_share = get_package_share_directory('jetson_rover_bridge')
    slam_params_file = os.path.join(pkg_share, 'config', 'mapper_params_online_async.yaml')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true, system clock if false'
    )

    # SLAM Toolbox node
    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time)

    # Add nodes
    ld.add_action(start_async_slam_toolbox_node)

    return ld
