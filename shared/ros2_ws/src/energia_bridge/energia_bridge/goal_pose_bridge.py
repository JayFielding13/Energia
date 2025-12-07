#!/usr/bin/env python3
"""
Goal Pose Bridge Node (SIMULATION ONLY)

Converts RViz 2D Goal Pose clicks to waypoint commands for the navigation system.
This node is intended for simulation testing only and should NOT be used on real hardware.

Subscribes to:
    /goal_pose (PoseStamped) - RViz 2D Goal Pose clicks

Publishes to:
    /waypoint/target (PointStamped) - Local XY waypoint for navigator
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PointStamped
from std_msgs.msg import Bool
import math


class GoalPoseBridge(Node):
    """
    SIMULATION ONLY: Bridges RViz 2D Goal Pose to waypoint navigator.

    This allows clicking in RViz to set navigation waypoints during simulation.
    Do NOT use this node on real hardware - use the HTTP API instead.
    """

    def __init__(self):
        super().__init__('goal_pose_bridge')

        # Declare simulation_mode parameter - MUST be true to function
        self.declare_parameter('simulation_mode', False)
        self.simulation_mode = self.get_parameter('simulation_mode').value

        if not self.simulation_mode:
            self.get_logger().error('=' * 60)
            self.get_logger().error('GOAL POSE BRIDGE: simulation_mode is FALSE')
            self.get_logger().error('This node is for SIMULATION ONLY!')
            self.get_logger().error('Set simulation_mode:=true to enable, or use HTTP API')
            self.get_logger().error('=' * 60)
            return

        # Publisher for waypoint target (local XY coordinates)
        self.waypoint_pub = self.create_publisher(
            PointStamped, '/waypoint/target_local', 10)

        # Publisher to arm motors automatically when goal is set
        self.arm_pub = self.create_publisher(
            Bool, '/navigation/arm', 10)

        # Subscriber for RViz 2D Goal Pose
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_pose_callback, 10)

        self.get_logger().info('=' * 60)
        self.get_logger().info('GOAL POSE BRIDGE [SIMULATION ONLY]')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Click "2D Goal Pose" in RViz to set waypoints')
        self.get_logger().info('Subscribing to: /goal_pose')
        self.get_logger().info('Publishing to: /waypoint/target_local')
        self.get_logger().info('=' * 60)

    def goal_pose_callback(self, msg: PoseStamped):
        """Convert RViz goal pose to local waypoint"""

        if not self.simulation_mode:
            return

        # Extract XY position from goal pose
        x = msg.pose.position.x
        y = msg.pose.position.y

        # Extract yaw from quaternion (for logging)
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        yaw_deg = math.degrees(yaw)

        self.get_logger().info(f'Goal received: x={x:.2f}m, y={y:.2f}m, yaw={yaw_deg:.1f}°')

        # Publish local waypoint
        waypoint = PointStamped()
        waypoint.header.stamp = self.get_clock().now().to_msg()
        waypoint.header.frame_id = 'odom'
        waypoint.point.x = x
        waypoint.point.y = y
        waypoint.point.z = 0.0

        self.waypoint_pub.publish(waypoint)

        # Auto-arm motors
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)

        self.get_logger().info(f'Waypoint sent: ({x:.2f}, {y:.2f}) - Motors ARMED')


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    node = GoalPoseBridge()

    if not node.simulation_mode:
        node.get_logger().warn('Node disabled - not in simulation mode')
        node.destroy_node()
        rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
