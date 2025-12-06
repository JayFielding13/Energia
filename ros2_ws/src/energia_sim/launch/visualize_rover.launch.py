#!/usr/bin/env python3

"""
RViz2 Visualization Launch File for Energia Rover
Displays all sensors: LiDAR, Camera, GPS, Ultrasonics, Odometry
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('energia_sim')
    rviz_config_file = os.path.join(pkg_share, 'config', 'rover_visualization.rviz')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world', default='empty_world')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Gazebo world to load'
    )

    # Include the spawn_rover launch file
    spawn_rover_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_share, 'launch', 'spawn_rover.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world_name
        }.items()
    )

    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_world,
        spawn_rover_launch,
        rviz_node
    ])
