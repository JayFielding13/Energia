#!/usr/bin/env python3
"""
AprilTag Follower with Reactive Obstacle Avoidance
Blends tag-following with intelligent obstacle steering
NEVER stops - always tries to find a way around
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Vector3
from std_msgs.msg import Bool
import numpy as np

class AprilTagFollowerReactive(Node):
    def __init__(self):
        super().__init__('apriltag_follower_reactive')

        # Following parameters
        self.declare_parameter('target_distance', 2.0)     # meters
        self.declare_parameter('distance_tolerance', 0.3)  # meters
        self.declare_parameter('max_linear_speed', 0.6)    # m/s
        self.declare_parameter('max_angular_speed', 1.0)   # rad/s
        self.declare_parameter('linear_kp', 0.5)           # P gain for distance
        self.declare_parameter('angular_kp', 1.2)          # P gain for angle

        # Avoidance blending
        self.declare_parameter('avoidance_blend', 0.7)     # How much to weight avoidance (0-1)
        self.declare_parameter('min_forward_speed', 0.1)   # Minimum speed to maintain

        # Re-acquisition parameters
        self.declare_parameter('search_angular_speed', 0.4)  # rad/s for searching
        self.declare_parameter('search_timeout', 15.0)       # seconds
        self.declare_parameter('lost_tag_timeout', 1.5)      # seconds

        self.target_dist = self.get_parameter('target_distance').value
        self.dist_tolerance = self.get_parameter('distance_tolerance').value
        self.max_linear = self.get_parameter('max_linear_speed').value
        self.max_angular = self.get_parameter('max_angular_speed').value
        self.linear_kp = self.get_parameter('linear_kp').value
        self.angular_kp = self.get_parameter('angular_kp').value
        self.avoidance_blend = self.get_parameter('avoidance_blend').value
        self.min_speed = self.get_parameter('min_forward_speed').value
        self.search_speed = self.get_parameter('search_angular_speed').value
        self.search_timeout = self.get_parameter('search_timeout').value
        self.lost_timeout = self.get_parameter('lost_tag_timeout').value

        # State machine
        self.STATE_IDLE = 0
        self.STATE_FOLLOWING = 1
        self.STATE_AVOIDING_AND_SEEKING = 2
        self.STATE_SEARCHING = 3
        self.STATE_TAG_LOST = 4

        self.state = self.STATE_IDLE

        # State tracking
        self.target_pose = None
        self.target_detected = False
        self.avoidance_vector = None
        self.avoidance_speed_scale = 1.0
        self.critical_danger = False
        self.enabled = False

        # Timing
        self.last_tag_seen = None
        self.search_start_time = None
        self.last_known_lateral_sign = 0

        # Subscribers
        self.create_subscription(PoseStamped, '/apriltag/target_pose', self.pose_callback, 10)
        self.create_subscription(Bool, '/apriltag/detected', self.detected_callback, 10)
        self.create_subscription(Vector3, '/avoidance/steering_vector', self.avoidance_vector_callback, 10)
        self.create_subscription(Vector3, '/avoidance/speed_scale', self.speed_scale_callback, 10)
        self.create_subscription(Bool, '/avoidance/critical_danger', self.critical_danger_callback, 10)
        self.create_subscription(Bool, '/follow_me/enable', self.enable_callback, 10)

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.state_pub = self.create_publisher(Bool, '/follow_me/active', 10)

        # Timer
        self.create_timer(0.1, self.control_loop)  # 10 Hz

        self.get_logger().info('AprilTag Follower (Reactive) started')
        self.get_logger().info('  Strategy: Never stop, always find a way!')

    def pose_callback(self, msg):
        self.target_pose = msg
        self.last_tag_seen = self.get_clock().now()

        lateral = msg.pose.position.y
        if abs(lateral) > 0.1:
            self.last_known_lateral_sign = 1 if lateral > 0 else -1

    def detected_callback(self, msg):
        prev = self.target_detected
        self.target_detected = msg.data

        if not prev and msg.data:
            self.get_logger().info('✓ Tag re-acquired!')

    def avoidance_vector_callback(self, msg):
        self.avoidance_vector = msg

    def speed_scale_callback(self, msg):
        self.avoidance_speed_scale = msg.x

    def critical_danger_callback(self, msg):
        prev = self.critical_danger
        self.critical_danger = msg.data

        if not prev and msg.data:
            self.get_logger().warn('Critical danger - evasive maneuvers!')

    def enable_callback(self, msg):
        prev = self.enabled
        self.enabled = msg.data

        if self.enabled and not prev:
            self.get_logger().info('Follow-me ENABLED (reactive mode)')
            self.state = self.STATE_IDLE
        elif not self.enabled and prev:
            self.get_logger().info('Follow-me DISABLED')
            self.state = self.STATE_IDLE
            self.cmd_vel_pub.publish(Twist())

    def update_state(self):
        """State machine"""
        now = self.get_clock().now()

        if not self.enabled:
            self.state = self.STATE_IDLE
            return

        # Can we see the tag?
        if self.target_detected:
            # Tag visible
            if self.avoidance_speed_scale < 0.5:
                # Obstacles present - blend avoidance with seeking
                if self.state != self.STATE_AVOIDING_AND_SEEKING:
                    self.get_logger().info('→ STATE: AVOIDING & SEEKING')
                self.state = self.STATE_AVOIDING_AND_SEEKING
            else:
                # Clear path - normal following
                if self.state != self.STATE_FOLLOWING:
                    self.get_logger().info('→ STATE: FOLLOWING')
                self.state = self.STATE_FOLLOWING

            self.search_start_time = None
            return

        # Tag not visible
        if self.last_tag_seen is not None:
            time_since_seen = (now - self.last_tag_seen).nanoseconds / 1e9

            if time_since_seen < self.lost_timeout:
                # Just lost
                if self.state != self.STATE_TAG_LOST:
                    self.get_logger().warn('→ STATE: TAG LOST')
                self.state = self.STATE_TAG_LOST
            elif time_since_seen < self.search_timeout:
                # Searching
                if self.state != self.STATE_SEARCHING:
                    self.get_logger().warn('→ STATE: SEARCHING')
                    self.search_start_time = now
                self.state = self.STATE_SEARCHING
            else:
                # Timeout
                if self.state != self.STATE_IDLE:
                    self.get_logger().error('Search timeout')
                self.state = self.STATE_IDLE
        else:
            self.state = self.STATE_IDLE

    def compute_tag_following_command(self):
        """Compute velocity command to follow tag"""
        if self.target_pose is None:
            return 0.0, 0.0

        distance = self.target_pose.pose.position.x  # Forward
        lateral = self.target_pose.pose.position.y   # Left/right

        # Calculate errors
        distance_error = distance - self.target_dist
        angle_error = np.arctan2(lateral, distance)

        # P controller
        linear_vel = self.linear_kp * distance_error
        angular_vel = self.angular_kp * angle_error

        # Stop if at target distance
        if abs(distance_error) < self.dist_tolerance:
            linear_vel = 0.0

        return linear_vel, angular_vel

    def blend_with_avoidance(self, desired_linear, desired_angular):
        """Blend tag-following with obstacle avoidance"""
        if self.avoidance_vector is None:
            return desired_linear, desired_angular

        # Extract avoidance steering direction
        avoid_angle = np.arctan2(self.avoidance_vector.y, self.avoidance_vector.x)

        # Blend angular velocity
        # If obstacles nearby, blend more toward avoidance
        blend = self.avoidance_blend
        if self.avoidance_speed_scale < 0.5:
            blend = 0.9  # Strong avoidance

        # Convert desired direction to angle
        # (desired_angular already represents turn rate)

        # Add avoidance steering
        final_angular = (1 - blend) * desired_angular + blend * avoid_angle * 2.0

        # Scale linear velocity by avoidance speed scale
        final_linear = desired_linear * self.avoidance_speed_scale

        # Ensure minimum forward progress (unless critical)
        if not self.critical_danger and final_linear < self.min_speed and desired_linear > 0:
            final_linear = self.min_speed

        return final_linear, final_angular

    def control_loop(self):
        """Main control loop"""
        self.update_state()

        cmd = Twist()

        if self.state == self.STATE_IDLE:
            # Do nothing
            pass

        elif self.state == self.STATE_TAG_LOST:
            # Tag just lost - keep last command briefly
            # But reduce speed
            if self.target_pose is not None:
                linear, angular = self.compute_tag_following_command()
                cmd.linear.x = linear * 0.3  # Slow down
                cmd.angular.z = angular * 0.3

        elif self.state == self.STATE_SEARCHING:
            # Rotate to search
            # If obstacles present, use avoidance direction
            if self.avoidance_vector is not None and self.avoidance_speed_scale < 0.8:
                # Steer in safe direction
                avoid_angle = np.arctan2(self.avoidance_vector.y, self.avoidance_vector.x)
                cmd.angular.z = avoid_angle * 1.5
                cmd.linear.x = self.min_speed  # Keep moving forward slowly
            else:
                # Clear - just rotate
                cmd.angular.z = self.search_speed * self.last_known_lateral_sign
                if self.last_known_lateral_sign == 0:
                    cmd.angular.z = self.search_speed

            self.get_logger().info('Searching...', throttle_duration_sec=2.0)

        elif self.state == self.STATE_FOLLOWING:
            # Normal following - clear path
            linear, angular = self.compute_tag_following_command()

            # Still apply avoidance blend (minor)
            linear, angular = self.blend_with_avoidance(linear, angular)

            cmd.linear.x = linear
            cmd.angular.z = angular

            if self.target_pose is not None:
                dist = self.target_pose.pose.position.x
                self.get_logger().info(
                    f'Following: {dist:.2f}m | v={linear:.2f}, ω={angular:.2f}',
                    throttle_duration_sec=1.0
                )

        elif self.state == self.STATE_AVOIDING_AND_SEEKING:
            # Obstacles present while following tag
            linear, angular = self.compute_tag_following_command()

            # Strong blend with avoidance
            linear, angular = self.blend_with_avoidance(linear, angular)

            cmd.linear.x = linear
            cmd.angular.z = angular

            if self.target_pose is not None:
                dist = self.target_pose.pose.position.x
                self.get_logger().info(
                    f'Avoiding & Seeking: {dist:.2f}m | v={linear:.2f}, ω={angular:.2f} | scale={self.avoidance_speed_scale:.2f}',
                    throttle_duration_sec=0.5
                )

        # Clamp velocities
        cmd.linear.x = np.clip(cmd.linear.x, -self.max_linear, self.max_linear)
        cmd.angular.z = np.clip(cmd.angular.z, -self.max_angular, self.max_angular)

        # Publish
        self.cmd_vel_pub.publish(cmd)
        self.state_pub.publish(Bool(data=(self.state != self.STATE_IDLE)))

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollowerReactive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
