#!/usr/bin/env python3
"""
Ultrasonic Gazebo Simulator - ESP32 JSON Emulator

This node simulates the ESP32 ultrasonic sensor behavior by publishing
JSON-formatted sensor data to /esp32/ultrasonic_json topic, exactly
mimicking the real ESP32's serial JSON output format.

This allows us to use the same ultrasonic_bridge.py in simulation as on the real robot.

This version subscribes to Gazebo model states and calculates real distances
to obstacles in the simulation environment.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
import json
import time
import random
import math


class UltrasonicGazeboSimulator(Node):
    """
    Simulates ESP32 behavior by reading Gazebo model states and
    calculating real distances to obstacles, then outputting JSON
    messages in the same format as the real ESP32.
    """
    def __init__(self):
        super().__init__('ultrasonic_gazebo_simulator')

        # Publisher for JSON strings (mimics ESP32 serial output)
        self.json_pub = self.create_publisher(
            String,
            '/esp32/ultrasonic_json',
            10
        )

        # Subscribe to Gazebo model states
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        # Create timer to publish simulated JSON at 10 Hz (matches ESP32)
        self.create_timer(0.1, self.publish_simulated_data)

        # Sensor configuration (angles in degrees)
        self.sensor_angles = {
            'front': 0,
            'corner_left': -45,
            'corner_right': 45,
            'side_left': -90,
            'side_right': 90,
            'rear': 180
        }

        # Current rover pose
        self.rover_x = 0.0
        self.rover_y = 0.0
        self.rover_yaw = 0.0

        # Obstacle positions (updated from Gazebo)
        self.obstacles = []

        # Sensor parameters (AJ-SR04M specs)
        self.max_range = 6.0  # meters
        self.min_range = 0.2  # meters
        self.beam_width = math.radians(15)  # 15 degree cone

        self.get_logger().info('Ultrasonic Gazebo Simulator started')
        self.get_logger().info('Publishing ESP32-format JSON to /esp32/ultrasonic_json')
        self.get_logger().info('Subscribing to /gazebo/model_states for obstacle detection')

    def model_states_callback(self, msg):
        """
        Update rover pose and obstacle positions from Gazebo
        """
        try:
            # Find rover model
            if 'jetson_rover' in msg.name:
                idx = msg.name.index('jetson_rover')
                pose = msg.pose[idx]

                # Extract position
                self.rover_x = pose.position.x
                self.rover_y = pose.position.y

                # Extract yaw from quaternion
                quat = pose.orientation
                siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
                cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
                self.rover_yaw = math.atan2(siny_cosp, cosy_cosp)

            # Extract obstacle positions (static models starting with "obstacle_")
            self.obstacles = []
            for i, name in enumerate(msg.name):
                if name.startswith('obstacle_'):
                    pose = msg.pose[i]
                    self.obstacles.append({
                        'name': name,
                        'x': pose.position.x,
                        'y': pose.position.y
                    })
        except Exception as e:
            self.get_logger().warn(f'Error processing model states: {e}')

    def calculate_distance_to_obstacles(self, sensor_angle_deg):
        """
        Calculate distance to nearest obstacle in sensor's direction.

        Args:
            sensor_angle_deg: Sensor angle in degrees (0=front, 90=right, -90=left, 180=rear)

        Returns:
            Distance in meters, or max_range if no obstacle detected
        """
        # Convert sensor angle to world frame
        sensor_angle_rad = math.radians(sensor_angle_deg)
        world_angle = self.rover_yaw + sensor_angle_rad

        # Ray direction
        ray_dx = math.cos(world_angle)
        ray_dy = math.sin(world_angle)

        min_distance = self.max_range

        # Check distance to each obstacle
        for obs in self.obstacles:
            # Vector from rover to obstacle
            obs_dx = obs['x'] - self.rover_x
            obs_dy = obs['y'] - self.rover_y

            # Distance to obstacle
            distance = math.sqrt(obs_dx**2 + obs_dy**2)

            # Angle to obstacle (in world frame)
            angle_to_obs = math.atan2(obs_dy, obs_dx)

            # Angle difference from sensor direction
            angle_diff = abs(angle_to_obs - world_angle)
            # Normalize to [-pi, pi]
            while angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            while angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            angle_diff = abs(angle_diff)

            # Check if obstacle is within sensor beam
            if angle_diff <= self.beam_width / 2:
                # Approximate obstacle as circle with radius 0.3m (conservative)
                obstacle_radius = 0.3
                effective_distance = max(self.min_range, distance - obstacle_radius)

                if effective_distance < min_distance:
                    min_distance = effective_distance

        return min_distance

    def publish_simulated_data(self):
        """
        Publish sensor data in ESP32 JSON format.
        Format matches esp32_ultrasonic_sketch.ino line 252-271

        Calculates real distances to Gazebo obstacles and adds realistic noise
        to simulate real AJ-SR04M sensors.
        """
        # Create JSON message matching ESP32 format
        json_data = {
            'timestamp': int(time.time() * 1000),  # Milliseconds since epoch
            'sensors': []
        }

        for name in ['front', 'corner_left', 'corner_right', 'side_left', 'side_right', 'rear']:
            angle = self.sensor_angles[name]

            # Calculate real distance to obstacles
            distance = self.calculate_distance_to_obstacles(angle)

            # Add realistic Gaussian noise (±1cm standard deviation, matching real sensors)
            noise = random.gauss(0, 0.01)
            distance = distance + noise

            # Clamp to valid range (0.2m - 6.0m for AJ-SR04M)
            distance = max(self.min_range, min(self.max_range, distance))

            sensor_entry = {
                'name': name,
                'angle': angle,
                'distance': round(distance, 3),
                'valid': True
            }
            json_data['sensors'].append(sensor_entry)

        # Convert to JSON string
        json_string = json.dumps(json_data)

        # Publish as String message
        msg = String()
        msg.data = json_string
        self.json_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicGazeboSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
