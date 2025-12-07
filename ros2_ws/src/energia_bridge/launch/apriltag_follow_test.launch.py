from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import launch

def generate_launch_description():
    """
    Launch file for AprilTag following with obstacle avoidance
    Tests the complete system in simulation
    """

    # Launch arguments
    enable_ultrasonic = LaunchConfiguration('enable_ultrasonic', default='true')
    enable_lidar = LaunchConfiguration('enable_lidar', default='true')
    enable_follower = LaunchConfiguration('enable_follower', default='false')  # Start disabled for safety
    debug_images = LaunchConfiguration('debug_images', default='true')

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'enable_ultrasonic',
            default_value='true',
            description='Enable ultrasonic obstacle detection'
        ),
        DeclareLaunchArgument(
            'enable_lidar',
            default_value='true',
            description='Enable LiDAR obstacle detection'
        ),
        DeclareLaunchArgument(
            'enable_follower',
            default_value='false',
            description='Enable AprilTag follower (use /follow_me/enable topic to enable at runtime)'
        ),
        DeclareLaunchArgument(
            'debug_images',
            default_value='true',
            description='Publish debug images with tag detections'
        ),

        # Ultrasonic obstacle detector
        Node(
            package='jetson_rover_bridge',
            executable='obstacle_detector_ultrasonic',
            name='ultrasonic_detector',
            parameters=[{
                'danger_distance': 0.5,
                'warning_distance': 1.0,
                'min_valid_range': 0.2,
                'max_valid_range': 6.0,
            }],
            output='screen',
            condition=IfCondition(enable_ultrasonic)
        ),

        # LiDAR obstacle detector
        Node(
            package='jetson_rover_bridge',
            executable='obstacle_detector_lidar',
            name='lidar_detector',
            parameters=[{
                'danger_distance': 0.8,
                'warning_distance': 2.0,
                'field_of_view': 120.0,  # degrees
                'min_valid_range': 0.1,
                'max_valid_range': 12.0,
            }],
            output='screen',
            condition=IfCondition(enable_lidar)
        ),

        # Reactive obstacle avoidance (Vector Field Histogram)
        Node(
            package='jetson_rover_bridge',
            executable='reactive_obstacle_avoidance',
            name='reactive_obstacle_avoidance',
            parameters=[{
                'critical_distance': 0.4,     # Absolute minimum (m)
                'influence_distance': 2.5,    # Avoidance influence range (m)
                'obstacle_weight': 2.0,       # Histogram weighting
                'sector_angle': 5.0,          # degrees per sector
                'num_sectors': 72,            # 360 / 5 = 72 sectors
            }],
            output='screen'
        ),

        # AprilTag detector
        Node(
            package='jetson_rover_bridge',
            executable='apriltag_detector',
            name='apriltag_detector',
            parameters=[{
                'target_tag_id': 0,
                'camera_fx': 600.0,  # Will auto-update from camera_info
                'camera_fy': 600.0,
                'camera_cx': 320.0,
                'camera_cy': 240.0,
                'tag_size': 0.165,  # 6.5 inches in meters
                'debug_image': debug_images,
            }],
            output='screen'
        ),

        # AprilTag follower (Reactive version - never stops!)
        Node(
            package='jetson_rover_bridge',
            executable='apriltag_follower_reactive',
            name='apriltag_follower_reactive',
            parameters=[{
                'target_distance': 2.0,         # meters
                'distance_tolerance': 0.3,      # meters
                'max_linear_speed': 0.6,        # m/s
                'max_angular_speed': 1.0,       # rad/s
                'linear_kp': 0.5,               # P gain for distance
                'angular_kp': 1.2,              # P gain for angle
                'avoidance_blend': 0.7,         # How much to weight avoidance (0-1)
                'min_forward_speed': 0.1,       # Minimum speed to maintain (never stop!)
                'search_angular_speed': 0.4,    # rad/s for searching
                'search_timeout': 15.0,         # seconds
                'lost_tag_timeout': 1.5,        # seconds
            }],
            output='screen'
        ),
    ])
