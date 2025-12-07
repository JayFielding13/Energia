#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
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

    # Build world file path - handle default
    # For empty_world, use Gazebo default, for test_yard use our world
    world_path = os.path.join(pkg_share, 'worlds', 'test_yard.world')

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

    # Launch Description
    ld = LaunchDescription()

    # Declare launch options
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)

    # Add nodes
    ld.add_action(start_gazebo_server)
    ld.add_action(start_gazebo_client)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    return ld
