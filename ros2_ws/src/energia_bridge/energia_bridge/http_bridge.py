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

        # Navigation/Follow state (for status reporting)
        self.navigating = False
        self.following = False
        self.paused = False

        # Follow-Me mode state
        self.follow_standoff_distance = 2.0  # Default distance to maintain
        self.mobile_position = None  # Latest mobile terminal position (PoseStamped)
        self.last_mobile_update = 0.0

        # Follow-Me control parameters
        self.follow_linear_gain = 0.5
        self.follow_angular_gain = 2.0
        self.follow_max_linear = 0.5
        self.follow_max_angular = 1.0

        # Mission state
        self.mission_waypoints = []  # List of (lat, lon) tuples
        self.mission_id = None
        self.current_waypoint_index = 0
        self.mission_running = False
        self.waypoint_tolerance = 1.0  # meters

        # Geofence state
        self.geofence_points = []  # List of (lat, lon) tuples defining polygon
        self.geofence_id = None
        self.geofence_enabled = False
        self.geofence_action = 'stop'  # Action when geofence violated

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

        # Subscribe to mobile terminal position from MQTT bridge
        self.mobile_pos_sub = self.create_subscription(
            PoseStamped,
            '/energia/gps/position',
            self.mobile_position_callback,
            10
        )

        # Follow-Me control loop timer (10 Hz)
        self.follow_timer = self.create_timer(0.1, self.follow_control_loop)

        self.get_logger().info('HTTP Bridge Node initialized')

    def gps_callback(self, msg):
        """Store latest GPS data"""
        self.current_gps = msg

    def odom_callback(self, msg):
        """Store latest odometry data"""
        self.current_odom = msg

    def mobile_position_callback(self, msg):
        """Store latest mobile terminal position from MQTT bridge"""
        self.mobile_position = msg
        self.last_mobile_update = time.time()

    def follow_control_loop(self):
        """Control loop for Follow-Me mode - runs at 10Hz"""
        if not self.following or not self.armed:
            return

        if self.mobile_position is None or self.current_odom is None:
            return

        # Check if mobile position is stale (>2 seconds old)
        if time.time() - self.last_mobile_update > 2.0:
            self.get_logger().warn('Mobile position stale - stopping')
            self.send_velocity_command(0.0, 0.0)
            return

        # Get current rover position
        rover_x = self.current_odom.pose.pose.position.x
        rover_y = self.current_odom.pose.pose.position.y

        # Get mobile position (already in local coords from MQTT bridge)
        mobile_x = self.mobile_position.pose.position.x
        mobile_y = self.mobile_position.pose.position.y

        # Calculate distance and angle to mobile
        dx = mobile_x - rover_x
        dy = mobile_y - rover_y
        distance = math.sqrt(dx * dx + dy * dy)

        # If within standoff distance, stop
        if distance < self.follow_standoff_distance:
            self.send_velocity_command(0.0, 0.0)
            return

        # Calculate angle to mobile
        target_angle = math.atan2(dy, dx)

        # Get current rover heading from quaternion
        q = self.current_odom.pose.pose.orientation
        current_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                  1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        # Calculate angular error (normalized to [-pi, pi])
        angle_error = target_angle - current_yaw
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi

        # Proportional control
        angular_vel = self.follow_angular_gain * angle_error
        angular_vel = max(-self.follow_max_angular, min(self.follow_max_angular, angular_vel))

        # Reduce linear speed when turning sharply
        turn_factor = 1.0 - min(abs(angle_error) / math.pi, 1.0)
        effective_distance = distance - self.follow_standoff_distance
        linear_vel = self.follow_linear_gain * effective_distance * turn_factor
        linear_vel = max(0.0, min(self.follow_max_linear, linear_vel))

        self.send_velocity_command(linear_vel, angular_vel)

    def enable_follow_me(self, standoff_distance=2.0):
        """Enable Follow-Me mode"""
        self.follow_standoff_distance = standoff_distance
        self.following = True
        self.navigating = False
        self.paused = False
        self.get_logger().info(f'Follow-Me mode ENABLED (standoff: {standoff_distance}m)')
        return True

    def disable_follow_me(self):
        """Disable Follow-Me mode"""
        self.following = False
        self.send_velocity_command(0.0, 0.0)
        self.get_logger().info('Follow-Me mode DISABLED')
        return True

    def get_follow_status(self):
        """Get Follow-Me mode status"""
        status = {
            'enabled': self.following,
            'standoff_distance': self.follow_standoff_distance,
        }
        if self.mobile_position is not None and self.current_odom is not None:
            rover_x = self.current_odom.pose.pose.position.x
            rover_y = self.current_odom.pose.pose.position.y
            mobile_x = self.mobile_position.pose.position.x
            mobile_y = self.mobile_position.pose.position.y
            dx = mobile_x - rover_x
            dy = mobile_y - rover_y
            status['distance_to_mobile'] = math.sqrt(dx * dx + dy * dy)
            status['mobile_position_age'] = time.time() - self.last_mobile_update
        return status

    # ==================== Mission Methods ====================

    def upload_mission(self, waypoints, mission_id=None):
        """Upload a mission with multiple waypoints"""
        self.mission_waypoints = []
        for wp in waypoints:
            lat = wp.get('latitude') or wp.get('lat')
            lon = wp.get('longitude') or wp.get('lon')
            if lat is not None and lon is not None:
                self.mission_waypoints.append((lat, lon))

        self.mission_id = mission_id or f'mission_{int(time.time())}'
        self.current_waypoint_index = 0
        self.get_logger().info(f'Mission uploaded: {self.mission_id} with {len(self.mission_waypoints)} waypoints')
        return True

    def start_mission(self):
        """Start executing the uploaded mission"""
        if not self.mission_waypoints:
            self.get_logger().warn('No mission waypoints to execute')
            return False

        if not self.armed:
            self.get_logger().warn('Cannot start mission - motors not armed')
            return False

        self.mission_running = True
        self.navigating = True
        self.following = False
        self.paused = False
        self.current_waypoint_index = 0

        # Send first waypoint
        lat, lon = self.mission_waypoints[0]
        self.send_waypoint(lat, lon)
        self.get_logger().info(f'Mission started: {self.mission_id}')
        return True

    def stop_mission(self):
        """Stop and clear the current mission"""
        self.mission_running = False
        self.navigating = False
        self.mission_waypoints = []
        self.current_waypoint_index = 0
        self.send_velocity_command(0.0, 0.0)
        self.get_logger().info('Mission stopped')
        return True

    def get_mission_status(self):
        """Get current mission status"""
        status = {
            'mission_id': self.mission_id,
            'running': self.mission_running,
            'total_waypoints': len(self.mission_waypoints),
            'current_waypoint': self.current_waypoint_index,
            'waypoints_remaining': len(self.mission_waypoints) - self.current_waypoint_index if self.mission_waypoints else 0,
        }
        if self.mission_waypoints and self.current_waypoint_index < len(self.mission_waypoints):
            lat, lon = self.mission_waypoints[self.current_waypoint_index]
            status['current_target'] = {'latitude': lat, 'longitude': lon}
        return status

    # ==================== Geofence Methods ====================

    def upload_geofence(self, points, geofence_id=None, action='stop'):
        """Upload a geofence polygon"""
        self.geofence_points = []
        for pt in points:
            lat = pt.get('latitude') or pt.get('lat')
            lon = pt.get('longitude') or pt.get('lon')
            if lat is not None and lon is not None:
                self.geofence_points.append((lat, lon))

        self.geofence_id = geofence_id or f'geofence_{int(time.time())}'
        self.geofence_action = action
        self.get_logger().info(f'Geofence uploaded: {self.geofence_id} with {len(self.geofence_points)} points')
        return True

    def enable_geofence(self):
        """Enable geofence enforcement"""
        if not self.geofence_points:
            self.get_logger().warn('No geofence points defined')
            return False
        self.geofence_enabled = True
        self.get_logger().info('Geofence ENABLED')
        return True

    def disable_geofence(self):
        """Disable geofence enforcement"""
        self.geofence_enabled = False
        self.get_logger().info('Geofence DISABLED')
        return True

    def get_geofence_status(self):
        """Get geofence status"""
        return {
            'geofence_id': self.geofence_id,
            'enabled': self.geofence_enabled,
            'num_points': len(self.geofence_points),
            'action': self.geofence_action,
            'points': [{'latitude': lat, 'longitude': lon} for lat, lon in self.geofence_points]
        }

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
        """Get current robot status in format compatible with Mobile RTK Control Module"""
        status = {
            'success': True,
            'armed': self.armed,
            'mode': self.mode,
            'timestamp': time.time(),
            # State flags for Mayak compatibility
            'navigating': self.navigating,
            'following': self.following,
            'paused': self.paused,
        }

        # Add GPS data at top level (flat format for Mayak compatibility)
        if self.current_gps:
            status['latitude'] = self.current_gps.latitude
            status['longitude'] = self.current_gps.longitude
            status['altitude'] = self.current_gps.altitude
            status['fix_type'] = int(self.current_gps.status.status)
            status['satellites'] = 12  # Simulated
            status['rtk_status'] = 'simulation'  # Indicate we're in simulation
        else:
            # Default values when no GPS
            status['latitude'] = 0.0
            status['longitude'] = 0.0
            status['altitude'] = 0.0
            status['fix_type'] = 0
            status['satellites'] = 0
            status['rtk_status'] = 'none'

        # Add heading and speed from odometry
        if self.current_odom:
            status['heading'] = self._get_heading_from_quaternion(self.current_odom.pose.pose.orientation)
            status['speed'] = abs(self.current_odom.twist.twist.linear.x)
            # Also include local position for simulation use
            status['local_x'] = self.current_odom.pose.pose.position.x
            status['local_y'] = self.current_odom.pose.pose.position.y
        else:
            status['heading'] = 0.0
            status['speed'] = 0.0

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

@app.route('/api/velocity', methods=['POST'])
def send_velocity():
    """Send velocity command for manual control"""
    data = request.json

    if not data:
        return jsonify({
            'success': False,
            'message': 'Missing velocity data'
        }), 400

    linear = data.get('linear', 0.0)
    angular = data.get('angular', 0.0)

    if ros_node:
        success = ros_node.send_velocity_command(linear, angular)
        return jsonify({
            'success': success,
            'message': 'Velocity command sent' if success else 'Velocity command failed'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

# ==================== Follow-Me Endpoints ====================

@app.route('/api/follow', methods=['GET'])
def get_follow_status():
    """Get Follow-Me mode status"""
    if ros_node:
        status = ros_node.get_follow_status()
        status['success'] = True
        return jsonify(status)
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/follow/enable', methods=['POST'])
def enable_follow_me():
    """Enable Follow-Me mode"""
    data = request.json or {}
    distance = data.get('distance', 2.0)

    if ros_node:
        success = ros_node.enable_follow_me(standoff_distance=distance)
        return jsonify({
            'success': success,
            'message': f'Follow-Me mode enabled (standoff: {distance}m)' if success else 'Failed to enable'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/follow/disable', methods=['POST'])
def disable_follow_me():
    """Disable Follow-Me mode"""
    if ros_node:
        success = ros_node.disable_follow_me()
        return jsonify({
            'success': success,
            'message': 'Follow-Me mode disabled' if success else 'Failed to disable'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

# ==================== Mission Endpoints ====================

@app.route('/api/mission/upload', methods=['POST'])
def upload_mission():
    """Upload a waypoint mission"""
    data = request.json

    if not data or 'waypoints' not in data:
        return jsonify({
            'success': False,
            'message': 'Missing waypoints data'
        }), 400

    waypoints = data['waypoints']
    mission_id = data.get('mission_id')

    if ros_node:
        success = ros_node.upload_mission(waypoints, mission_id)
        return jsonify({
            'success': success,
            'message': f'Mission uploaded with {len(waypoints)} waypoints' if success else 'Upload failed',
            'mission_id': ros_node.mission_id
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/mission/start', methods=['POST'])
def start_mission():
    """Start the uploaded mission"""
    if ros_node:
        success = ros_node.start_mission()
        return jsonify({
            'success': success,
            'message': 'Mission started' if success else 'Failed to start mission'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/mission/stop', methods=['POST'])
def stop_mission():
    """Stop the current mission"""
    if ros_node:
        success = ros_node.stop_mission()
        return jsonify({
            'success': success,
            'message': 'Mission stopped' if success else 'Failed to stop mission'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/mission/status', methods=['GET'])
def get_mission_status():
    """Get current mission status"""
    if ros_node:
        status = ros_node.get_mission_status()
        status['success'] = True
        return jsonify(status)
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

# ==================== Geofence Endpoints ====================

@app.route('/api/geofence/upload', methods=['POST'])
def upload_geofence():
    """Upload a geofence polygon"""
    data = request.json

    if not data or 'points' not in data:
        return jsonify({
            'success': False,
            'message': 'Missing geofence points'
        }), 400

    points = data['points']
    geofence_id = data.get('geofence_id')
    action = data.get('action', 'stop')

    if ros_node:
        success = ros_node.upload_geofence(points, geofence_id, action)
        return jsonify({
            'success': success,
            'message': f'Geofence uploaded with {len(points)} points' if success else 'Upload failed',
            'geofence_id': ros_node.geofence_id
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/geofence/enable', methods=['POST'])
def enable_geofence():
    """Enable geofence enforcement"""
    if ros_node:
        success = ros_node.enable_geofence()
        return jsonify({
            'success': success,
            'message': 'Geofence enabled' if success else 'Failed to enable geofence'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/geofence/disable', methods=['POST'])
def disable_geofence():
    """Disable geofence enforcement"""
    if ros_node:
        success = ros_node.disable_geofence()
        return jsonify({
            'success': success,
            'message': 'Geofence disabled' if success else 'Failed to disable geofence'
        })
    return jsonify({'success': False, 'message': 'ROS node not initialized'})

@app.route('/api/geofence/status', methods=['GET'])
def get_geofence_status():
    """Get geofence status"""
    if ros_node:
        status = ros_node.get_geofence_status()
        status['success'] = True
        return jsonify(status)
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
    ros_node.get_logger().info('  GET  /api/health         - Health check')
    ros_node.get_logger().info('  GET  /api/status         - Robot status')
    ros_node.get_logger().info('  POST /api/arm            - ARM motors')
    ros_node.get_logger().info('  POST /api/disarm         - DISARM motors')
    ros_node.get_logger().info('  POST /api/target         - Send GPS waypoint')
    ros_node.get_logger().info('  POST /api/velocity       - Send velocity command')
    ros_node.get_logger().info('  POST /api/stop           - Emergency stop')
    ros_node.get_logger().info('  POST /api/pause          - Pause movement')
    ros_node.get_logger().info('  POST /api/cancel         - Cancel mission')
    ros_node.get_logger().info('  GET  /api/follow         - Follow-Me status')
    ros_node.get_logger().info('  POST /api/follow/enable  - Enable Follow-Me')
    ros_node.get_logger().info('  POST /api/follow/disable - Disable Follow-Me')
    ros_node.get_logger().info('  POST /api/mission/upload  - Upload mission waypoints')
    ros_node.get_logger().info('  POST /api/mission/start   - Start mission')
    ros_node.get_logger().info('  POST /api/mission/stop    - Stop mission')
    ros_node.get_logger().info('  GET  /api/mission/status  - Mission status')
    ros_node.get_logger().info('  POST /api/geofence/upload - Upload geofence polygon')
    ros_node.get_logger().info('  POST /api/geofence/enable - Enable geofence')
    ros_node.get_logger().info('  POST /api/geofence/disable- Disable geofence')
    ros_node.get_logger().info('  GET  /api/geofence/status - Geofence status')

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
