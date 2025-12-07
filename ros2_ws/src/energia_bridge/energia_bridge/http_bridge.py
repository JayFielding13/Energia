#!/usr/bin/env python3
"""
HTTP Bridge Server for Mobile RTK Control Module
Translates HTTP REST API calls to ROS2 topics for simulation
Compatible with robot_controller.py API interface
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32
from flask import Flask, request, jsonify
import threading
import time
import math

class HTTPBridgeNode(Node):
    """ROS2 node that bridges HTTP REST API to ROS2 topics"""

    def __init__(self):
        super().__init__('http_bridge')

        # Robot state
        self.armed = False
        self.mode = "manual"
        self.current_gps = None
        self.current_odom = None
        self.target_waypoint = None

        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/waypoint/goto', 10)
        self.arm_pub = self.create_publisher(Bool, '/rover/armed', 10)

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.gps_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info('HTTP Bridge Node initialized')

    def gps_callback(self, msg):
        """Store latest GPS data"""
        self.current_gps = msg

    def odom_callback(self, msg):
        """Store latest odometry data"""
        self.current_odom = msg

    def arm_motors(self):
        """ARM motors"""
        self.armed = True
        msg = Bool()
        msg.data = True
        self.arm_pub.publish(msg)
        self.get_logger().info('Motors ARMED')
        return True

    def disarm_motors(self):
        """DISARM motors"""
        self.armed = False
        msg = Bool()
        msg.data = False
        self.arm_pub.publish(msg)

        # Stop the rover
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        self.get_logger().info('Motors DISARMED')
        return True

    def send_velocity_command(self, linear_x, angular_z):
        """Send velocity command to rover"""
        if not self.armed:
            self.get_logger().warn('Cannot move - motors not armed')
            return False

        cmd = Twist()
        cmd.linear.x = float(linear_x)
        cmd.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(cmd)
        return True

    def send_waypoint(self, lat, lon):
        """Send GPS waypoint to rover"""
        self.target_waypoint = (lat, lon)

        # For now, just log it
        # In full implementation, this would convert GPS to local coordinates
        # and publish a goal pose
        self.get_logger().info(f'Waypoint set: {lat:.6f}, {lon:.6f}')
        return True

    def emergency_stop(self):
        """Emergency stop - disarm and halt"""
        self.disarm_motors()
        self.get_logger().warn('EMERGENCY STOP executed')
        return True

    def get_status(self):
        """Get current robot status"""
        status = {
            'success': True,
            'armed': self.armed,
            'mode': self.mode,
            'timestamp': time.time()
        }

        # Add GPS data if available
        if self.current_gps:
            status['gps'] = {
                'latitude': self.current_gps.latitude,
                'longitude': self.current_gps.longitude,
                'altitude': self.current_gps.altitude,
                'fix_type': int(self.current_gps.status.status),
                'satellites': 12  # Simulated
            }

        # Add odometry data if available
        if self.current_odom:
            status['position'] = {
                'x': self.current_odom.pose.pose.position.x,
                'y': self.current_odom.pose.pose.position.y,
                'heading': self._get_heading_from_quaternion(self.current_odom.pose.pose.orientation)
            }
            status['velocity'] = {
                'linear': self.current_odom.twist.twist.linear.x,
                'angular': self.current_odom.twist.twist.angular.z
            }

        return status

    def _get_heading_from_quaternion(self, q):
        """Convert quaternion to heading in degrees"""
        # Simplified - assumes rotation around Z axis
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        heading_rad = math.atan2(siny_cosp, cosy_cosp)
        heading_deg = math.degrees(heading_rad)
        return heading_deg


# Flask application
app = Flask(__name__)
ros_node = None

@app.route('/api/health', methods=['GET'])
def health_check():
    """Health check endpoint"""
    return jsonify({
        'success': True,
        'status': 'ok',
        'message': 'HTTP Bridge Server running',
        'simulation': True
    })

@app.route('/api/status', methods=['GET'])
def get_status():
    """Get robot status"""
    if ros_node:
        return jsonify(ros_node.get_status())
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/arm', methods=['POST'])
def arm_motors():
    """ARM motors"""
    if ros_node:
        success = ros_node.arm_motors()
        return jsonify({
            'success': success,
            'armed': ros_node.armed,
            'message': 'Motors ARMED' if success else 'ARM failed'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/disarm', methods=['POST'])
def disarm_motors():
    """DISARM motors"""
    if ros_node:
        success = ros_node.disarm_motors()
        return jsonify({
            'success': success,
            'armed': ros_node.armed,
            'message': 'Motors DISARMED' if success else 'DISARM failed'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/target', methods=['POST'])
def send_target():
    """Send target GPS position"""
    data = request.json

    if not data or 'latitude' not in data or 'longitude' not in data:
        return jsonify({
            'success': False,
            'message': 'Missing latitude or longitude'
        }), 400

    if ros_node:
        success = ros_node.send_waypoint(
            data['latitude'],
            data['longitude']
        )
        return jsonify({
            'success': success,
            'message': 'Waypoint set' if success else 'Failed to set waypoint'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/stop', methods=['POST'])
def emergency_stop():
    """Emergency stop"""
    if ros_node:
        success = ros_node.emergency_stop()
        return jsonify({
            'success': success,
            'message': 'Emergency stop executed' if success else 'Emergency stop failed'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/pause', methods=['POST'])
def pause():
    """Pause rover movement"""
    if ros_node:
        # Send zero velocity
        success = ros_node.send_velocity_command(0.0, 0.0)
        return jsonify({
            'success': success,
            'message': 'Rover paused' if success else 'Pause failed'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/cancel', methods=['POST'])
def cancel_mission():
    """Cancel current mission"""
    if ros_node:
        ros_node.target_waypoint = None
        success = ros_node.send_velocity_command(0.0, 0.0)
        return jsonify({
            'success': success,
            'message': 'Mission cancelled' if success else 'Cancel failed'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})


def run_flask_app():
    """Run Flask app in background thread"""
    app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)


def main(args=None):
    """Main function"""
    global ros_node

    # Initialize ROS2
    rclpy.init(args=args)
    ros_node = HTTPBridgeNode()

    # Start Flask in background thread
    flask_thread = threading.Thread(target=run_flask_app, daemon=True)
    flask_thread.start()

    ros_node.get_logger().info('HTTP Bridge Server started on http://0.0.0.0:5001')
    ros_node.get_logger().info('API endpoints:')
    ros_node.get_logger().info('  GET  /api/health  - Health check')
    ros_node.get_logger().info('  GET  /api/status  - Robot status')
    ros_node.get_logger().info('  POST /api/arm     - ARM motors')
    ros_node.get_logger().info('  POST /api/disarm  - DISARM motors')
    ros_node.get_logger().info('  POST /api/target  - Send GPS waypoint')
    ros_node.get_logger().info('  POST /api/stop    - Emergency stop')
    ros_node.get_logger().info('  POST /api/pause   - Pause movement')
    ros_node.get_logger().info('  POST /api/cancel  - Cancel mission')

    # Spin ROS2 node
    try:
        rclpy.spin(ros_node)
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
