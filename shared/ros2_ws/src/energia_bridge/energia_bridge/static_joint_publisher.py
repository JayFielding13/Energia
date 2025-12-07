#!/usr/bin/env python3
"""
Static Joint State Publisher for RViz visualization.

Publishes constant (zero position) joint states for wheel joints to enable
robot_state_publisher to compute wheel transforms. This is needed because
the Gazebo Harmonic JointStatePublisher plugin publishes gz.msgs.Model which
doesn't convert properly to sensor_msgs/JointState via ros_gz_bridge.

Wheel rotation is visual-only in RViz and doesn't affect navigation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class StaticJointPublisher(Node):
    def __init__(self):
        super().__init__('static_joint_publisher')

        # List of wheel joints that need static transforms
        self.joint_names = [
            'front_left_wheel_joint',
            'front_right_wheel_joint',
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
        ]

        # Use same QoS as robot_state_publisher expects
        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE

        self.publisher = self.create_publisher(
            JointState,
            '/joint_states',
            qos
        )

        # Publish at 10 Hz (enough for visualization)
        self.timer = self.create_timer(0.1, self.publish_joint_states)

        self.get_logger().info('Static joint publisher started for wheel visualization')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        # All positions at zero (wheels in neutral position)
        msg.position = [0.0] * len(self.joint_names)
        msg.velocity = [0.0] * len(self.joint_names)
        msg.effort = [0.0] * len(self.joint_names)

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticJointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
