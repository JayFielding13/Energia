#!/usr/bin/env python3
"""
AprilTag Follower with Smart Re-Acquisition
Follows an AprilTag while avoiding obstacles
If tag is lost (e.g., during obstacle avoidance), performs search pattern to re-acquire
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
import numpy as np
import time

class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower')

        # Following parameters
        self.declare_parameter('target_distance', 2.0)     # meters
        self.declare_parameter('distance_tolerance', 0.3)  # meters
        self.declare_parameter('max_linear_speed', 0.5)    # m/s
        self.declare_parameter('max_angular_speed', 0.8)   # rad/s
        self.declare_parameter('linear_kp', 0.5)           # P gain for distance
        self.declare_parameter('angular_kp', 1.0)          # P gain for angle

        # Re-acquisition parameters
        self.declare_parameter('search_angular_speed', 0.3)  # rad/s for searching
        self.declare_parameter('search_timeout', 10.0)       # seconds before giving up
        self.declare_parameter('lost_tag_timeout', 1.0)      # seconds before starting search

        self.target_dist = self.get_parameter('target_distance').value
        self.dist_tolerance = self.get_parameter('distance_tolerance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.search_speed = self.get_parameter('search_angular_speed').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.lost_timeout = self.get_parameter('lost_tag_timeout').value

        # State machine states
        self.STATE_IDLE = 0
        self.STATE_FOLLOWING = 1
        self.STATE_AVOIDING = 2
        self.STATE_SEARCHING = 3
        self.STATE_TAG_LOST = 4

        self.state = self.STATE_IDLE
        self.prev_state = self.STATE_IDLE

        # State tracking
        self.target_pose = None
        self.target_detected = False
        self.obstacle_danger = False
        self.obstacle_warning = False
        self.enabled = False

        # Timing for re-acquisition
        self.last_tag_seen = None
        self.search_start_time = None
        self.last_known_lateral_sign = 0  # -1 for left, +1 for right

        # Subscribers
        self.create_subscription(PoseStamped, '/apriltag/target_pose', self.pose_callback, 10)
        self.create_subscription(Bool, '/apriltag/detected', self.detected_callback, 10)
        self.create_subscription(Bool, '/obstacles/danger', self.obstacle_danger_callback, 10)
        self.create_subscription(Bool, '/obstacles/warning', self.obstacle_warning_callback, 10)
        self.create_subscription(Bool, '/follow_me/enable', self.enable_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(Bool, '/follow_me/active', 10)

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('AprilTag Follower started')
        self.get_logger().info(f'  Target distance: {self.target_dist}m')
        self.get_logger().info(f'  Re-acquisition enabled (search timeout: {self.search_timeout}s)')

    def pose_callback(self, msg):
        self.target_pose = msg
        self.last_tag_seen = self.get_clock().now()

        # Remember which side the tag was on (for re-acquisition)
        lateral = msg.pose.position.y
        if abs(lateral) > 0.1:  # Only update if significant lateral offset
            self.last_known_lateral_sign = 1 if lateral > 0 else -1

    def detected_callback(self, msg):
        prev_detected = self.target_detected
        self.target_detected = msg.data

        # Tag re-acquired
        if not prev_detected and msg.data:
            self.get_logger().info('✓ Tag re-acquired!')

        # Tag just lost
        if prev_detected and not msg.data:
            self.get_logger().warn('Tag lost from view')

    def obstacle_danger_callback(self, msg):
        prev_danger = self.obstacle_danger
        self.obstacle_danger = msg.data

        # Just entered danger zone
        if not prev_danger and msg.data:
            self.get_logger().warn('OBSTACLE DANGER - Stopping!')

    def obstacle_warning_callback(self, msg):
        self.obstacle_warning = msg.data

    def enable_callback(self, msg):
        prev_enabled = self.enabled
        self.enabled = msg.data

        if self.enabled and not prev_enabled:
            self.get_logger().info('Follow-me ENABLED')
            self.state = self.STATE_IDLE
        elif not self.enabled and prev_enabled:
            self.get_logger().info('Follow-me DISABLED')
            self.state = self.STATE_IDLE
            self.cmd_vel_pub.publish(Twist())  # Stop

    def update_state(self):
        """State machine for follower behavior"""
        now = self.get_clock().now()

        if not self.enabled:
            self.state = self.STATE_IDLE
            return

        # Check for obstacle danger (highest priority)
        if self.obstacle_danger:
            if self.state != self.STATE_AVOIDING:
                self.get_logger().warn('→ STATE: AVOIDING OBSTACLE')
            self.state = self.STATE_AVOIDING
            return

        # Check if we can see the tag
        if self.target_detected:
            # Tag visible - follow it
            if self.state != self.STATE_FOLLOWING:
                self.get_logger().info('→ STATE: FOLLOWING TAG')
            self.state = self.STATE_FOLLOWING
            self.search_start_time = None  # Reset search timer
            return

        # Tag not visible - determine if lost or searching
        if self.last_tag_seen is not None:
            time_since_seen = (now - self.last_tag_seen).nanoseconds / 1e9

            if time_since_seen < self.lost_timeout:
                # Just lost, wait a moment
                if self.state != self.STATE_TAG_LOST:
                    self.get_logger().warn('→ STATE: TAG LOST (waiting...)')
                self.state = self.STATE_TAG_LOST
            elif time_since_seen < self.search_timeout:
                # Start searching
                if self.state != self.STATE_SEARCHING:
                    self.get_logger().warn('→ STATE: SEARCHING FOR TAG')
                    self.search_start_time = now
                self.state = self.STATE_SEARCHING
            else:
                # Search timeout - give up
                if self.state != self.STATE_IDLE:
                    self.get_logger().error('Search timeout - tag not found')
                self.state = self.STATE_IDLE
                self.search_start_time = None
        else:
            self.state = self.STATE_IDLE

    def control_loop(self):
        """Main control loop - executes state machine"""
        self.update_state()

        cmd = Twist()

        if self.state == self.STATE_IDLE:
            # Do nothing
            pass

        elif self.state == self.STATE_AVOIDING:
            # EMERGENCY STOP for obstacle
            pass  # cmd already zero

        elif self.state == self.STATE_TAG_LOST:
            # Tag just lost - stop and wait
            pass  # cmd already zero

        elif self.state == self.STATE_SEARCHING:
            # Rotate to search for tag
            # Use last known direction to search intelligently
            cmd.angular.z = self.search_speed * self.last_known_lateral_sign
            if self.last_known_lateral_sign == 0:
                cmd.angular.z = self.search_speed  # Default to right

            self.get_logger().info(
                f'Searching... (rotating {"left" if cmd.angular.z > 0 else "right"})',
                throttle_duration_sec=2.0
            )

        elif self.state == self.STATE_FOLLOWING:
            # Follow the tag!
            if self.target_pose is not None:
                distance = self.target_pose.pose.position.x  # Forward
                lateral = self.target_pose.pose.position.y   # Left/right

                # Calculate errors
                distance_error = distance - self.target_dist
                angle_error = np.arctan2(lateral, distance)

                # P controller
                linear_vel = self.linear_kp * distance_error
                angular_vel = self.angular_kp * angle_error

                # Clamp velocities
                linear_vel = np.clip(linear_vel, -self.max_linear, self.max_linear)
                angular_vel = np.clip(angular_vel, -self.max_angular, self.max_angular)

                # Stop if close enough
                if abs(distance_error) < self.dist_tolerance:
                    linear_vel = 0.0
                    angular_vel = 0.0  # Also stop rotating if at target distance

                cmd.linear.x = linear_vel
                cmd.angular.z = angular_vel

                self.get_logger().info(
                    f'Following: {distance:.2f}m, lat={lateral:.2f}m | '
                    f'v={linear_vel:.2f}, ω={angular_vel:.2f}',
                    throttle_duration_sec=1.0
                )

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Publish active state
        self.state_pub.publish(Bool(data=(self.state != self.STATE_IDLE)))

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop rover on exit
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
