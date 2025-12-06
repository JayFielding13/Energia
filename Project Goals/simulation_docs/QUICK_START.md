# Jetson Rover Simulation - Quick Start

## Ready to Test!

Your complete rover simulation is ready with all sensors and a test environment. Here's how to get started:

## Launch Commands

### 1. Basic Simulation (Empty World)

```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

### 2. Test World with Obstacles (Recommended)

```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard
```

This launches:
- Gazebo with test yard (grass, obstacles, waypoint markers, geofence)
- Rover with all sensors active
- RViz2 with pre-configured sensor visualization

### 3. Manual Control

In a new terminal:

```bash
source ~/Desktop/Mini\ Rover\ Development/ros2_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controls: `i` (forward), `k` (stop), `,` (back), `j` (left), `l` (right)

## What's Included

### Rover Configuration
- **Size**: 27" × 23.75" × 12" chassis + 12" × 12" × 8" electronics box
- **Weight**: 200 lbs (90.7 kg)
- **Motors**: 4× wheelchair motors (24V, 4WD skid-steer)
- **Battery**: 2× 12V lead acid (series = 24V)
- **Top Speed**: ~3 mph (1.5 m/s)

### Sensors
- **RP-LIDAR A1**: 360° laser scanner (0.15-12m range) → `/scan`
- **Logitech C920X**: 1080p camera @ 30fps → `/camera/image_raw`
- **HERE 3+ GPS**: RTK-capable GPS → `/gps/fix`
- **6× Ultrasonics**: AJ-SR04M sensors (0.2-6m) → `/ultrasonic/*`
- **Odometry**: From wheel encoders → `/odom`

### Test Environment
- **Terrain**: Grass (0.8 friction) + pavement section (1.0 friction)
- **Obstacles**: Tree, post, curb, wall, rocks
- **GPS Waypoints**: 4 markers for navigation testing
- **Geofence**: 30m × 20m boundary (yellow spheres at corners)

## Verify Everything Works

```bash
# Check all topics are publishing
ros2 topic list

# Monitor sensor data
ros2 topic hz /scan          # Should be ~10 Hz
ros2 topic hz /camera/image_raw  # Should be ~30 Hz
ros2 topic hz /gps/fix       # Should be ~10 Hz

# View LiDAR data
ros2 topic echo /scan --once

# View camera feed
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Check GPS position
ros2 topic echo /gps/fix --once
```

## Control Interface

### Subscribe to Sensors (Python Example)

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, NavSatFix, Image
from geometry_msgs.msg import Twist

class RoverController(Node):
    def __init__(self):
        super().__init__('rover_controller')

        # Subscribe to sensors
        self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def lidar_callback(self, msg):
        # msg.ranges[0] = distance straight ahead
        pass

    def gps_callback(self, msg):
        # msg.latitude, msg.longitude
        pass

    def camera_callback(self, msg):
        # msg.data = image data
        pass

    def move_forward(self, speed):
        cmd = Twist()
        cmd.linear.x = speed  # m/s
        cmd.angular.z = 0.0
        self.cmd_pub.publish(cmd)
```

## Test Scenarios

### 1. Sensor Validation (5 minutes)
- Drive toward obstacles
- Verify LiDAR detects them
- Check ultrasonics activate at close range
- Confirm camera shows view

### 2. Obstacle Avoidance (15 minutes)
- Implement simple avoidance logic
- Test with tree obstacle at (5, 0)
- Verify rover stops/turns before collision

### 3. GPS Waypoint Following (20 minutes)
- Navigate to waypoint at (10, 0)
- Then to (10, 5)
- Return to origin (0, 0)
- Track accuracy (<0.5m error)

### 4. Geofence Testing (10 minutes)
- Drive toward yellow boundary markers
- Implement geofence check
- Verify rover stops at boundary

### 5. Multi-Sensor Fusion (30 minutes)
- Combine GPS navigation + obstacle avoidance
- Navigate to waypoint while avoiding obstacles
- Test robustness

## Hardware Deployment Notes

When you're ready to deploy to the real rover:

**Your code works identically** - same ROS 2 topics, same message types!

**On Real Jetson Orin Nano:**
1. MAVROS connects to Cube Orange → provides `/mavros/imu/data`, `/mavros/global_position/global`
2. LiDAR driver → publishes to `/scan` (same topic!)
3. Camera driver → publishes to `/camera/image_raw` (same!)
4. ESP32 ultrasonic node → publishes to `/ultrasonic/*` (same!)

**Only difference:**
- Change `use_sim_time:=false` in launch files
- Cube Orange handles motor PWM (you send `/cmd_vel`, it converts to motor commands)

## Documentation

- **[README.md](README.md)** - Overview and basic usage
- **[DIMENSIONS.md](DIMENSIONS.md)** - Physical specifications
- **[MOTORS.md](MOTORS.md)** - Drive system and battery info
- **[SENSORS.md](SENSORS.md)** - Ultrasonic sensors
- **[LIDAR_CAMERA_GPS.md](LIDAR_CAMERA_GPS.md)** - Main navigation sensors
- **[TESTING_GUIDE.md](TESTING_GUIDE.md)** - Complete testing procedures (start here!)

## Support

If you encounter issues:

1. Check Gazebo is running: `ps aux | grep gazebo`
2. Verify ROS topics: `ros2 topic list`
3. Check for errors: Look at terminal output
4. Restart simulation if needed: `Ctrl+C` and relaunch

## Next Steps

1. **Read [TESTING_GUIDE.md](TESTING_GUIDE.md)** for detailed test procedures
2. **Run sensor validation** to confirm everything works
3. **Implement your control logic** using the ROS 2 interface
4. **Test progressively** - start simple, add complexity
5. **Validate thoroughly** before deploying to 200 lb real robot!

---

**You're all set!** Your simulation is complete and ready for autonomous navigation testing. Good luck! 🚀
