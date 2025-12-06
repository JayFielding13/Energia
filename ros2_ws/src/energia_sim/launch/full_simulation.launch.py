#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import xacro


def generate_launch_description():

    # Package name
    pkg_name = 'energia_sim'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'energia_rover_gazebo.xacro')
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')

    # RViz config - use the one in the package config folder
    rviz_config_file = os.path.join(pkg_share, 'config', 'rover_visualization.rviz')

    # Xbox controller config
    xbox_config_file = os.path.join(pkg_share, 'config', 'xbox_controller.yaml')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world_name = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty_world',
        description='Gazebo world file to load (without .world extension)'
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='X position of the robot'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Y position of the robot'
    )

    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.2',
        description='Z position of the robot'
    )

    # Build world file path
    world_path = os.path.join(pkg_share, 'worlds', 'test_yard.world')

    # ==================== Gazebo ====================

    # Start Gazebo server
    start_gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_path}.items()
    )

    # Start Gazebo client
    start_gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gzclient.launch.py')
        )
    )

    # ==================== Robot State Publisher ====================

    # Process the URDF file with xacro
    robot_description_content = xacro.process_file(urdf_file).toxml()

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content
        }]
    )

    # ==================== Spawn Robot ====================

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'energia_rover',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    # ==================== Xbox Controller Teleop ====================

    # Joy node for Xbox 360 controller
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'use_sim_time': use_sim_time,
            'device_id': 0,
            'deadzone': 0.3,
            'autorepeat_rate': 20.0
        }]
    )

    # Teleop twist joy with custom config
    teleop_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[xbox_config_file, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel', '/cmd_vel')]
    )

    # ==================== RViz ====================

    # RViz2 with saved configuration
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # ==================== Launch Description ====================

    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)

    # Add Gazebo
    ld.add_action(start_gazebo_server)
    ld.add_action(start_gazebo_client)

    # Add robot
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    # Add controller
    ld.add_action(joy_node)
    ld.add_action(teleop_node)

    # Add RViz
    ld.add_action(rviz_node)

    return ld
