from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'energia_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jay',
    maintainer_email='JayFielding13@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'http_bridge = energia_bridge.http_bridge:main',
            'gps_bridge = energia_bridge.gps_bridge:main',
            'interactive_map = energia_bridge.interactive_map:main',
            'obstacle_detector_ultrasonic = energia_bridge.obstacle_detector_ultrasonic:main',
            'obstacle_detector_lidar = energia_bridge.obstacle_detector_lidar:main',
            'obstacle_fusion = energia_bridge.obstacle_fusion:main',
            'apriltag_detector = energia_bridge.apriltag_detector:main',
            'apriltag_follower = energia_bridge.apriltag_follower:main',
            'reactive_obstacle_avoidance = energia_bridge.reactive_obstacle_avoidance:main',
            'apriltag_follower_reactive = energia_bridge.apriltag_follower_reactive:main',
            'waypoint_navigator = energia_bridge.waypoint_navigator:main',
            'odom_to_tf = energia_bridge.odom_to_tf:main',
            'static_joint_publisher = energia_bridge.static_joint_publisher:main',
            'timestamp_republisher = energia_bridge.timestamp_republisher:main',
        ],
    },
)
