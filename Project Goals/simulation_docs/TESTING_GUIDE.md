# Jetson Rover Simulation Testing Guide

## Overview

This guide covers testing your rover in simulation before deploying to the real 200 lb robot. The simulation environment allows you to validate:

- **Autonomous Navigation**: Waypoint following with GPS
- **Obstacle Avoidance**: LiDAR and ultrasonic sensor fusion
- **Geofence Logic**: Boundary enforcement
- **Control Architecture**: Jetson Orin Nano + Cube Orange integration

## Control Architecture

### Hardware Stack (Real Robot)

```
┌─────────────────────────────────────────┐
│         Jetson Orin Nano                │
│  - High-level mission planning          │
│  - Obstacle avoidance logic             │
│  - GPS waypoint navigation              │
│  - LiDAR processing (SLAM)              │
│  - Camera vision processing             │
│  - Geofence enforcement                 │
│  - WiFi telemetry                       │
└─────────────┬───────────────────────────┘
              │ Serial/UART
              ▼
┌─────────────────────────────────────────┐
│         Cube Orange                      │
│  - Low-level motor control              │
│  - IMU data acquisition                 │
│  - GPS data from HERE 3+                │
│  - Sensor fusion (IMU + GPS)            │
│  - Sends data to Jetson via MAVROS     │
└─────────┬───────────────────────────────┘
          │ PWM signals
          ▼
┌─────────────────────────────────────────┐
│    2× MDDS30 Motor Drivers              │
│  - Driver 1: Front wheels               │
│  - Driver 2: Rear wheels                │
└─────────┬───────────────────────────────┘
          │
          ▼
    4× Wheelchair Motors (24V)
```

### Software Stack

**On Jetson Orin Nano:**
- Your custom ground control software (Python/ROS 2)
- Processes sensor data from ROS 2 topics
- Sends velocity commands to `/cmd_vel`
- Receives IMU/GPS data from MAVROS (from Cube Orange)

**In Simulation:**
- All sensors publish to same ROS 2 topics as real hardware
- Gazebo simulates physics, motors, and sensors
- Your ground control software works identically in sim and real world

## Test Environment

### Test World: `test_yard.world`

The test yard simulates a typical residential front yard with:

**Terrain:**
- **Grass** (default surface, friction 0.8)
- **Pavement section** (higher friction 1.0, at x=8m)

**Obstacles:**
- Large object (tree/box) at (5, 0) - 0.5m × 0.5m × 1m
- Narrow post at (3, 2) - 10cm diameter cylinder
- Low curb at (0, -3) - 5m long, 10cm high
- Wall section at (-5, 3) - simulates fence
- Scattered rocks at (2, -2) and (-2, -1)

**GPS Waypoints (green markers):**
1. Waypoint 1: (10, 0) - Start of navigation course
2. Waypoint 2: (10, 5) - Turn point
3. Waypoint 3: (5, 5) - Obstacle zone
4. Waypoint 4: (0, 0) - Return home (red marker)

**Geofence Boundaries (yellow spheres):**
- NE corner: (15, 10)
- NW corner: (-15, 10)
- SE corner: (15, -10)
- SW corner: (-15, -10)
- **Geofence area**: 30m × 20m rectangle

## Quick Start

### 1. Launch Simulation with Visualization

```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source install/setup.bash

# Launch rover in test world with RViz2
ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard
```

This starts:
- Gazebo with `test_yard.world`
- Rover model with all sensors
- RViz2 with pre-configured visualization

### 2. Verify All Sensors

In a new terminal:

```bash
source ~/Desktop/Mini\ Rover\ Development/ros2_ws/install/setup.bash

# Check all topics are publishing
ros2 topic list

# Expected topics:
# /scan                    - LiDAR
# /camera/image_raw        - Camera
# /gps/fix                 - GPS
# /ultrasonic/front        - Ultrasonics
# /ultrasonic/corner_left
# /ultrasonic/corner_right
# /ultrasonic/side_left
# /ultrasonic/side_right
# /ultrasonic/rear
# /odom                    - Odometry
# /cmd_vel                 - Velocity command (input)
```

### 3. Manual Control Test

Test basic movement with keyboard teleoperation:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Forward
- `k` - Stop
- `,` - Backward
- `j` - Turn left
- `l` - Turn right
- `q/z` - Increase/decrease max speed
- `w/x` - Increase/decrease turn speed

**Validation:**
- Rover should move smoothly in Gazebo
- Watch RViz2 odometry update
- LiDAR scan should detect obstacles as you approach them

## Testing Scenarios

### Test 1: Sensor Validation

**Objective**: Verify all sensors are working correctly

**Procedure:**

1. **LiDAR Test:**
```bash
# Monitor scan data
ros2 topic echo /scan --once

# Check for obstacles
# Drive toward obstacle_tree at (5, 0)
# LiDAR should show decreasing distance
```

2. **Ultrasonic Test:**
```bash
# Monitor front ultrasonic
ros2 topic echo /ultrasonic/front

# Approach obstacle slowly
# Range should decrease from 6m to ~0.2m
```

3. **Camera Test:**
```bash
# View camera feed
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# Should see Gazebo world from rover's perspective
```

4. **GPS Test:**
```bash
# Check GPS fix
ros2 topic echo /gps/fix --once

# Verify:
# - status.status >= 1 (fix acquired)
# - latitude/longitude match Gazebo position
```

**Expected Results:**
- ✅ All sensors publish at expected rates
- ✅ LiDAR detects obstacles at correct distances
- ✅ Ultrasonics provide 360° coverage
- ✅ Camera shows clear view forward
- ✅ GPS provides accurate position

### Test 2: Obstacle Avoidance

**Objective**: Validate LiDAR + Ultrasonic fusion for obstacle avoidance

**Procedure:**

1. Position rover facing obstacle_tree (5, 0)
2. Implement simple obstacle avoidance logic:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist

class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')

        # Subscribe to sensors
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.ultrasonic_sub = self.create_subscription(
            Range, '/ultrasonic/front', self.ultrasonic_callback, 10)

        # Publish velocity commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.front_distance = 10.0  # meters

    def scan_callback(self, msg):
        # Get distance straight ahead (index 0)
        if len(msg.ranges) > 0:
            front_scan = msg.ranges[0]
            if front_scan > msg.range_min and front_scan < msg.range_max:
                self.front_distance = min(self.front_distance, front_scan)

    def ultrasonic_callback(self, msg):
        if msg.range < msg.max_range:
            self.front_distance = min(self.front_distance, msg.range)

    def avoid_obstacles(self):
        cmd = Twist()

        # Safety thresholds
        STOP_DISTANCE = 0.5  # meters
        SLOW_DISTANCE = 1.0  # meters

        if self.front_distance < STOP_DISTANCE:
            # Emergency stop
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5  # Turn to find clear path
            self.get_logger().warn(f'STOP! Obstacle at {self.front_distance:.2f}m')

        elif self.front_distance < SLOW_DISTANCE:
            # Slow down
            cmd.linear.x = 0.2
            self.get_logger().info(f'Slowing: obstacle at {self.front_distance:.2f}m')

        else:
            # Normal speed
            cmd.linear.x = 0.5

        self.cmd_pub.publish(cmd)
        self.front_distance = 10.0  # Reset for next cycle

def main():
    rclpy.init()
    node = ObstacleAvoidance()

    # Run avoidance at 10 Hz
    timer = node.create_timer(0.1, node.avoid_obstacles)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. Run the test:
```bash
python3 obstacle_avoidance_test.py
```

**Expected Results:**
- ✅ Rover slows down at 1m from obstacle
- ✅ Rover stops at 0.5m from obstacle
- ✅ Rover turns to find clear path
- ✅ No collisions occur

### Test 3: GPS Waypoint Navigation

**Objective**: Follow GPS waypoints accurately

**Test Route:**
- Start: (0, 0)
- Waypoint 1: (10, 0)
- Waypoint 2: (10, 5)
- Waypoint 3: (5, 5)
- Return: (0, 0)

**Procedure:**

1. Create waypoint list with GPS coordinates
2. Implement waypoint follower (example):

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import math

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Waypoints (lat, lon) - convert from Gazebo coordinates
        self.waypoints = [
            (0.0, 0.0),   # Start
            (10.0, 0.0),  # Waypoint 1
            (10.0, 5.0),  # Waypoint 2
            (5.0, 5.0),   # Waypoint 3
            (0.0, 0.0)    # Return home
        ]
        self.current_wp = 0
        self.current_pos = (0.0, 0.0)

    def gps_callback(self, msg):
        # In simulation, GPS lat/lon matches Gazebo XY
        self.current_pos = (msg.latitude, msg.longitude)
        self.navigate_to_waypoint()

    def navigate_to_waypoint(self):
        if self.current_wp >= len(self.waypoints):
            self.get_logger().info('All waypoints reached!')
            return

        target = self.waypoints[self.current_wp]

        # Calculate distance to waypoint
        dx = target[0] - self.current_pos[0]
        dy = target[1] - self.current_pos[1]
        distance = math.sqrt(dx**2 + dy**2)

        # Calculate heading to waypoint
        target_heading = math.atan2(dy, dx)

        cmd = Twist()

        if distance < 0.5:  # Reached waypoint
            self.get_logger().info(f'Reached waypoint {self.current_wp + 1}')
            self.current_wp += 1
            cmd.linear.x = 0.0
        else:
            # Simple proportional control
            cmd.linear.x = min(0.5, distance * 0.3)
            # TODO: Add heading correction with angular.z

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Expected Results:**
- ✅ Rover navigates to each waypoint
- ✅ Accuracy within 0.5m of each waypoint
- ✅ Smooth transitions between waypoints
- ✅ Completes full circuit and returns home

### Test 4: Geofence Enforcement

**Objective**: Prevent rover from exiting defined boundary

**Geofence Definition:**
- Rectangular boundary: ±15m in X, ±10m in Y
- Yellow spheres mark corners in simulation

**Procedure:**

1. Implement geofence check:

```python
class GeofenceMonitor(Node):
    def __init__(self):
        super().__init__('geofence_monitor')

        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Geofence boundaries
        self.fence_x_min = -15.0
        self.fence_x_max = 15.0
        self.fence_y_min = -10.0
        self.fence_y_max = 10.0

        self.in_fence = True

    def gps_callback(self, msg):
        x, y = msg.latitude, msg.longitude

        # Check if inside fence
        if (x < self.fence_x_min or x > self.fence_x_max or
            y < self.fence_y_min or y > self.fence_y_max):

            if self.in_fence:
                self.get_logger().error('GEOFENCE BREACH! Stopping rover.')
                self.in_fence = False

            # Stop rover
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)
        else:
            if not self.in_fence:
                self.get_logger().info('Back inside geofence.')
                self.in_fence = True
```

2. Test by commanding rover toward boundary

**Expected Results:**
- ✅ Rover stops when approaching fence
- ✅ Warning logged when fence approached
- ✅ Rover prevented from exiting boundary
- ✅ Normal operation resumes when inside fence

### Test 5: Multi-Sensor Fusion

**Objective**: Combine LiDAR, ultrasonics, and GPS for robust navigation

**Scenario:** Navigate through obstacle course to waypoint

**Procedure:**

1. Set target waypoint: (10, 0) - requires passing obstacle at (5, 0)
2. Implement fusion logic:
   - Use GPS for long-range waypoint guidance
   - Use LiDAR for obstacle detection (2-10m)
   - Use ultrasonics for close-range safety (0-2m)
   - Combine all for decision making

3. Test algorithm handles:
   - Following waypoint when path is clear
   - Detecting and avoiding obstacles
   - Resuming waypoint following after avoidance

**Expected Results:**
- ✅ Rover navigates toward GPS waypoint
- ✅ Detects obstacle with LiDAR ahead of time
- ✅ Executes avoidance maneuver
- ✅ Returns to waypoint route
- ✅ Reaches target successfully

## Performance Metrics

Track these metrics during testing:

| Metric | Target | How to Measure |
|--------|--------|----------------|
| **GPS Accuracy** | < 0.5m error | Distance from waypoint when "reached" |
| **Obstacle Detection Distance** | > 2m (LiDAR) | Check `/scan` topic ranges |
| **Reaction Time** | < 500ms | Time from obstacle detect to velocity change |
| **Waypoint Completion** | 100% | All waypoints reached successfully |
| **Geofence Violations** | 0 | No breaches during testing |
| **Collision Avoidance** | 100% | No contacts with obstacles |

## Debugging Tools

### View All Sensor Data

```bash
# Create multi-window terminal view
ros2 topic echo /scan | head -20 &
ros2 topic echo /gps/fix | grep -E 'latitude|longitude' &
ros2 topic echo /odom | grep -E 'position' &
```

### Record Test Data

```bash
# Record all topics for playback
ros2 bag record -a -o test_run_1

# Playback later
ros2 bag play test_run_1
```

### Visualize Navigation

In RViz2 (already configured):
- **Red points**: LiDAR scan data
- **Colored cones**: Ultrasonic sensor ranges
- **Robot model**: Current rover position
- **TF frames**: All sensor coordinate frames
- **Path**: Planned trajectory (if using Nav2)

## Common Issues

### Rover doesn't move

- Check `/cmd_vel` topic is publishing: `ros2 topic echo /cmd_vel`
- Verify Gazebo physics is running (real-time factor in GUI)
- Ensure no emergency stop in your control logic

### Sensors not publishing

- Check Gazebo plugins loaded: Look for sensor messages in Gazebo terminal
- Verify topics exist: `ros2 topic list`
- Check update rates: `ros2 topic hz /scan`

### GPS position incorrect

- GPS uses Gazebo world coordinates in simulation
- Origin (0, 0) in Gazebo = GPS (0, 0)
- Coordinate transform same as real GPS (for now)

### Rover tips over / unstable

- Check mass distribution in URDF (CoM should be 4" above bottom)
- Verify wheel friction (should be 1.0 for good grip)
- Reduce max acceleration if too aggressive

## Next Steps

After validating in simulation:

1. **Transfer code to real Jetson**: Your ROS 2 nodes should work identically
2. **Configure MAVROS**: Connect to Cube Orange for IMU/GPS data
3. **Test sensors individually**: Verify each sensor on real hardware
4. **Start with manual control**: Test motors before autonomous mode
5. **Begin with small geofence**: Test in safe, controlled area first
6. **Gradually increase complexity**: Add features as you gain confidence

## Safety Reminders

Before deploying to real 200 lb rover:

- ✅ Test emergency stop in simulation
- ✅ Validate geofence logic thoroughly
- ✅ Ensure obstacle avoidance works reliably
- ✅ Have manual override ready (RC controller or E-stop)
- ✅ Test in safe area away from people/property
- ✅ Start slow - limit max speed initially
- ✅ Monitor battery voltage
- ✅ Keep WiFi connection stable for telemetry

---

**Good luck with your testing!** This simulation environment allows you to validate all your logic safely before putting your rover in the real world.
