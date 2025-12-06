#!/usr/bin/env python3
"""
Maze Simulation Launch File

Launches the Energia Rover in a maze environment designed for:
- SLAM algorithm development and testing
- Obstacle avoidance using ultrasonic sensors and LiDAR
- AprilTag detection and search/navigation tasks
- Robot vision and autonomous navigation

The maze includes:
- 12m x 12m area with 1.5m wide corridors
- Multiple dead ends and challenging passages
- 8 AprilTags (ID 0-7) distributed throughout
- Various obstacles (boxes, cylinders) for sensor testing

Usage:
    ros2 launch energia_sim maze_simulation.launch.py
    ros2 launch energia_sim maze_simulation.launch.py x_pose:=-5.0 y_pose:=5.0
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
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

    # Maze world file
    world_path = os.path.join(pkg_share, 'worlds', 'maze.world')

    # RViz config - use the one in the package if available, fallback to project root
    rviz_config_file = os.path.join(pkg_share, 'config', 'rover_visualization.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = os.path.expanduser('~/Desktop/Mini Rover Development/Jetson Cube Orange Outdoor Rover/rover_sensors.rviz')

    # Set Gazebo model path to include our custom models
    models_path = os.path.join(pkg_share, 'models')
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_model_path:
        gazebo_model_path = models_path + ':' + gazebo_model_path
    else:
        gazebo_model_path = models_path

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose',
        default_value='1.0',  # Upper-right quadrant, moved left to avoid obstacle
        description='X position of the robot'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose',
        default_value='8.0',   # Away from internal walls
        description='Y position of the robot'
    )

    declare_z_pose = DeclareLaunchArgument(
        'z_pose',
        default_value='0.2',
        description='Z position of the robot'
    )

    # Set Gazebo model path environment variable
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        gazebo_model_path
    )

    # ==================== Gazebo ====================

    # Start Gazebo server with maze world
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
            '-z', z_pose,
            '-Y', '0.0'  # Facing east (into the maze)
        ],
        output='screen'
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

    # Set environment
    ld.add_action(set_gazebo_model_path)

    # Declare launch options
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)

    # Add Gazebo
    ld.add_action(start_gazebo_server)
    ld.add_action(start_gazebo_client)

    # Add robot
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity)

    # Add RViz
    ld.add_action(rviz_node)

    return ld
