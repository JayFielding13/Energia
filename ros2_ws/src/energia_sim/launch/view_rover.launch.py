#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # Package name
    pkg_name = 'energia_sim'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'energia_rover.urdf.xacro')
    rviz_config_file = os.path.join(pkg_share, 'config', 'view_rover.rviz')

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file])
        }]
    )

    # Joint State Publisher GUI (to manually move joints)
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else []
    )

    # Launch Description
    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
