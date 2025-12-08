#!/usr/bin/env python3
"""
Full Simulation Launch File for Gazebo Harmonic

Launches the Energia Rover simulation with:
- Gazebo Harmonic (gz sim)
- ROS-Gazebo bridge for topic communication
- Robot State Publisher
- RViz2 visualization

Usage:
    ros2 launch energia_sim full_simulation.launch.py
    ros2 launch energia_sim full_simulation.launch.py world:=maze
    ros2 launch energia_sim full_simulation.launch.py world:=satellite_portland

Notes on timing:
    This launch file uses simulation time (use_sim_time=true) with the /clock
    topic bridged from Gazebo. ROS2 nodes are delayed 5 seconds after Gazebo
    starts to allow the clock to stabilize. Some "jump back in time" warnings
    may appear at startup but are transient.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():

    # Package name
    pkg_name = 'energia_sim'
    pkg_share = FindPackageShare(package=pkg_name).find(pkg_name)

    # Paths
    urdf_file = os.path.join(pkg_share, 'urdf', 'energia_rover_v2.urdf.xacro')
    worlds_dir = os.path.join(pkg_share, 'worlds')

    # RViz config
    rviz_config_file = os.path.join(pkg_share, 'config', 'rover_visualization.rviz')

    # Set Gazebo resource path for models and textures
    models_path = os.path.join(pkg_share, 'models')
    gz_resource_path = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    # Add both models path and package share (for textures directory)
    paths_to_add = [models_path, pkg_share]
    if gz_resource_path:
        gz_resource_path = ':'.join(paths_to_add) + ':' + gz_resource_path
    else:
        gz_resource_path = ':'.join(paths_to_add)

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
        default_value='test_yard',
        description='World to load (test_yard, maze, satellite_portland)'
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

    # Set Gazebo resource path environment variable
    set_gz_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        gz_resource_path
    )

    # ==================== Gazebo Harmonic ====================

    # Function to launch Gazebo with the selected world
    def launch_gazebo(context):
        world = context.launch_configurations['world']
        world_path = os.path.join(worlds_dir, f'{world}.world')
        return [ExecuteProcess(
            cmd=['gz', 'sim', '-r', world_path],
            output='screen'
        )]

    start_gazebo = OpaqueFunction(function=launch_gazebo)

    # ==================== ROS-Gazebo Bridge ====================

    # Bridge Gazebo topics to ROS2
    # Clock is bridged first for use_sim_time to work
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock - MUST be bridged first for use_sim_time to work
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Cmd vel (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # Odometry (Gazebo -> ROS)
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            # LiDAR scan (Gazebo -> ROS)
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            # IMU (Gazebo -> ROS)
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # NOTE: TF and joint_states are NOT bridged from Gazebo
            # Instead, odom_to_tf converts /odom to TF, and robot_state_publisher
            # handles robot links. This avoids timestamp ordering issues.
        ],
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ==================== Robot State Publisher ====================

    # Process the URDF file with xacro
    robot_description_content = xacro.process_file(urdf_file).toxml()

    # Robot State Publisher - publishes TF for robot links
    # Using use_sim_time=False to avoid timestamp conflicts with Gazebo
    # This means TF uses wall clock time, which avoids "jump back in time" errors
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'robot_description': robot_description_content
        }]
    )

    # ==================== Spawn Robot ====================

    # Spawn the robot in Gazebo Harmonic using ros_gz_sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'energia_rover',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    # ==================== Odom to TF ====================

    # Convert odometry to TF (odom -> base_footprint)
    # Uses wall clock time (use_sim_time=False) to avoid timestamp conflicts
    # This avoids timestamp issues from bridging TF directly from Gazebo
    odom_to_tf = Node(
        package='energia_bridge',
        executable='odom_to_tf',
        name='odom_to_tf',
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # ==================== Static Transforms ====================

    # Static transform: map -> odom (identity for simulation)
    # Uses wall clock time to avoid timestamp conflicts
    static_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # ==================== RViz ====================

    # RViz2 with saved configuration
    # Uses wall clock time for TF visualization to avoid timestamp conflicts
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        output='screen'
    )

    # ==================== Delayed Startup ====================

    # Delay ROS nodes to ensure Gazebo clock is available first
    # This gives time for the /clock topic to be established
    # The "jump back in time" warnings during startup are transient and clear quickly
    delayed_nodes = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to start and clock to stabilize
        actions=[
            robot_state_publisher,
            spawn_entity,
            odom_to_tf,
            static_map_to_odom,
            rviz_node,
        ]
    )

    # ==================== Launch Description ====================

    ld = LaunchDescription()

    # Set environment
    ld.add_action(set_gz_resource_path)

    # Declare launch options
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_world)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)

    # Start Gazebo and bridge immediately
    ld.add_action(start_gazebo)
    ld.add_action(ros_gz_bridge)

    # Start other nodes after delay to ensure clock is available
    ld.add_action(delayed_nodes)

    return ld
