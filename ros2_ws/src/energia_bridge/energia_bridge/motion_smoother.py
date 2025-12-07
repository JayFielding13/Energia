#!/usr/bin/env python3
"""
Motion Smoother - Realistic Acceleration/Deceleration

This node provides smooth, realistic motion dynamics for the rover:
1. Limits acceleration/deceleration rates
2. Prevents instant speed changes
3. Mimics real-world motor physics

Use Xbox controller to tune until motion "feels right" like the real robot.

Controls:
- Use LEFT STICK on Xbox controller for manual driving
- Adjust parameters below to match real robot response
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time


class MotionSmoother(Node):
    def __init__(self):
        super().__init__('motion_smoother')

        # ===== TUNABLE PARAMETERS =====
        # Adjust these to match real robot feel!

        # Linear acceleration (m/s²)
        self.declare_parameter('max_linear_accel', 1.0)  # How fast it speeds up
        self.declare_parameter('max_linear_decel', 2.0)  # How fast it slows down (can be faster)

        # Angular acceleration (rad/s²) - DOUBLED for simulation
        self.declare_parameter('max_angular_accel', 4.0)  # Rotation speed-up rate
        self.declare_parameter('max_angular_decel', 6.0)  # Rotation slow-down rate

        # Update rate (Hz)
        self.declare_parameter('update_rate', 50.0)  # 50 Hz = smooth

        # Maximum velocities (safety limits)
        self.declare_parameter('max_linear_vel', 2.0)   # m/s
        self.declare_parameter('max_angular_vel', 2.0)  # rad/s

        # Get parameters
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_linear_decel = self.get_parameter('max_linear_decel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.max_angular_decel = self.get_parameter('max_angular_decel').value
        self.update_rate = self.get_parameter('update_rate').value
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value

        # Current commanded velocities (from controller/autonomy)
        self.target_linear_x = 0.0
        self.target_angular_z = 0.0

        # Current actual velocities (smoothed output)
        self.current_linear_x = 0.0
        self.current_angular_z = 0.0

        # Timing
        self.last_time = time.time()

        # Subscribe to commanded velocity (from Xbox controller or autonomous behaviors)
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Publish smoothed velocity
        self.smooth_vel_pub = self.create_publisher(Twist, '/cmd_vel_smooth', 10)

        # Control loop timer
        update_period = 1.0 / self.update_rate
        self.create_timer(update_period, self.smooth_loop)

        self.get_logger().info('Motion Smoother started')
        self.get_logger().info(f'  Linear accel:  {self.max_linear_accel} m/s²')
        self.get_logger().info(f'  Linear decel:  {self.max_linear_decel} m/s²')
        self.get_logger().info(f'  Angular accel: {self.max_angular_accel} rad/s²')
        self.get_logger().info(f'  Angular decel: {self.max_angular_decel} rad/s²')
        self.get_logger().info(f'  Update rate:   {self.update_rate} Hz')
        self.get_logger().info('')
        self.get_logger().info('Subscribing to: /cmd_vel')
        self.get_logger().info('Publishing to:  /cmd_vel_smooth')

    def cmd_vel_callback(self, msg):
        """Receive target velocity commands"""
        # Clamp to safety limits
        self.target_linear_x = max(-self.max_linear_vel, min(self.max_linear_vel, msg.linear.x))
        self.target_angular_z = max(-self.max_angular_vel, min(self.max_angular_vel, msg.angular.z))

    def smooth_loop(self):
        """Apply acceleration limits to create smooth motion"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0:
            return  # Skip first iteration

        # === Linear velocity smoothing ===
        linear_error = self.target_linear_x - self.current_linear_x

        if abs(linear_error) < 0.001:
            # Close enough - use target
            self.current_linear_x = self.target_linear_x
        else:
            # Apply acceleration/deceleration limits
            if linear_error > 0:
                # Speeding up or reducing backwards speed
                max_change = self.max_linear_accel * dt
            else:
                # Slowing down or reducing forward speed
                max_change = -self.max_linear_decel * dt

            # Clamp change to not overshoot
            if abs(linear_error) < abs(max_change):
                self.current_linear_x = self.target_linear_x
            else:
                self.current_linear_x += max_change

        # === Angular velocity smoothing ===
        angular_error = self.target_angular_z - self.current_angular_z

        if abs(angular_error) < 0.001:
            # Close enough - use target
            self.current_angular_z = self.target_angular_z
        else:
            # Apply acceleration/deceleration limits
            if angular_error > 0:
                # Speeding up rotation
                max_change = self.max_angular_accel * dt
            else:
                # Slowing down rotation
                max_change = -self.max_angular_decel * dt

            # Clamp change to not overshoot
            if abs(angular_error) < abs(max_change):
                self.current_angular_z = self.target_angular_z
            else:
                self.current_angular_z += max_change

        # Publish smoothed command
        smooth_cmd = Twist()
        smooth_cmd.linear.x = self.current_linear_x
        smooth_cmd.angular.z = self.current_angular_z
        self.smooth_vel_pub.publish(smooth_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MotionSmoother()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the rover
        stop_cmd = Twist()
        node.smooth_vel_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
