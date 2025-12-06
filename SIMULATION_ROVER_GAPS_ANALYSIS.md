# Simulation vs Real Rover - Gap Analysis

**Date**: November 25, 2025 (Updated)
**Last Rover Update**: November 23-24, 2025 (Heltec Gen 2 killswitch integration)
**Purpose**: Identify differences between simulation environment and physical Jetson rover
**Goal**: Enable girlfriend to develop and test rover code in simulation that will work on real hardware

📋 **See Also:** [LATEST_ROVER_UPDATES_COMPARISON.md](LATEST_ROVER_UPDATES_COMPARISON.md) for recent rover developments

---

## Executive Summary

The simulation environment is **well-structured** with good sensor coverage (ultrasonic, LiDAR, camera, GPS), but there are **critical gaps** in safety systems and software integration that prevent seamless code transfer from simulation to real rover.

### Most Critical Gaps (November 2025)

1. 🔴 **CRITICAL: No Heltec LoRa killswitch simulation** - Two-layer safety system (added Nov 2025)
2. 🔴 **CRITICAL: No safety mode enforcement** - Real rover requires ARM/DISARM checks
3. 🟡 **HIGH: No MAVLink/Cube Orange integration** - Real rover uses MAVLink for motor control
4. 🟡 **HIGH: Different control architectures** - Sim uses `cmd_vel`, real uses HTTP API + MAVLink
5. 🟢 **MEDIUM: Missing system health monitoring** - Battery, CPU temp, heartbeat service

---

## Hardware Components Comparison

| Component | Real Rover | Simulation | Status |
|-----------|-----------|------------|--------|
| **Chassis** | 27" x 23.75" x 12" | 27" x 23.75" x 12" | ✅ Match |
| **Wheels** | 10" diameter, skid-steer | 10" diameter, skid-steer | ✅ Match |
| **Flight Controller** | Cube Orange (MAVLink) | Differential drive plugin | ❌ Different |
| **Ultrasonic Sensors** | 6x AJ-SR04M via ESP32 | 6x Gazebo ray sensors | ✅ Match (topics) |
| **LiDAR** | RPLidar A1 | Simulated RPLidar | ✅ Match |
| **Camera** | USB camera 640x480 | Simulated camera | ✅ Match |
| **GPS** | HERE 3+ RTK GPS | Simulated GPS | ✅ Match |
| **IMU** | Cube Orange built-in | Simulated IMU | ✅ Match |
| **Killswitch** | Heltec LoRa wireless | None | ❌ Missing |
| **Jetson Computer** | Orin Nano | Simulated in host | ✅ N/A |

---

## Software Architecture Comparison

### Real Rover Stack

```
Mobile App / External Control
         ↓
   HTTP REST API (Port 5000)
    energia_rover_server.py
         ↓
    MAVLink Protocol
         ↓
   Cube Orange Flight Controller
         ↓
    Motor Control (PWM)
         ↓
      4 DC Motors
```

**Supporting Services:**
- `jetson-rplidar.service` - ROS2 node for LiDAR
- `jetson-camera.service` - ROS2 node for camera
- `jetson-ultrasonic-bridge.service` - ROS2 node for ultrasonic sensors
- `jetson-sensor-bridge.service` - HTTP API for sensor data (Port 5001)

### Simulation Stack

```
Xbox Controller / Keyboard
         ↓
  Joy Node / Teleop Twist
         ↓
     /cmd_vel topic
         ↓
Gazebo Differential Drive Plugin
         ↓
   Simulated Motors
         ↓
   Wheel Joints
```

**All sensors publish to ROS2 topics** (no HTTP bridge needed)

---

## Control Interface Gaps

### Motor Control

**Real Rover:**
- Uses HTTP REST API endpoints:
  - `POST /api/arm` - ARM motors
  - `POST /api/disarm` - DISARM motors
  - `POST /api/target` - Send GPS waypoint
  - `POST /api/stop` - Emergency stop
- Motor commands sent via MAVLink to Cube Orange
- Requires ARM/DISARM sequence for safety

**Simulation:**
- Uses standard ROS2 `/cmd_vel` topic (geometry_msgs/Twist)
- No ARM/DISARM concept
- No MAVLink layer
- Direct motor control via Gazebo plugin

**Impact:** Code written for simulation using `/cmd_vel` will NOT work on real rover without adapter

---

### Sensor Data Access

**Real Rover:**
- ROS2 topics available: `/scan`, `/image_raw/compressed`, `/ultrasonic/*`
- Also available via HTTP API on port 5001:
  - `GET /api/lidar` - Latest LiDAR scan (JSON)
  - `GET /api/camera` - Latest camera image (base64 JPEG)
  - `GET /api/sensors/status` - Sensor status

**Simulation:**
- ROS2 topics only: `/scan`, `/camera/image_raw`, `/ultrasonic/*`
- No HTTP bridge (not needed in simulation)

**Impact:** ✅ ROS2 sensor topics match - code using ROS2 will work on both

---

## Critical Missing Features in Simulation

### 1. Heltec Killswitch (CRITICAL SAFETY FEATURE)

**Real Rover:**
- Heltec LoRa wireless killswitch (transmitter/receiver pair)
- Provides emergency stop capability
- Monitors battery voltage on transmitter
- Heartbeat monitoring to detect signal loss
- New service: `heltec_heartbeat_service.py`

**Simulation:**
- ❌ No killswitch simulation
- ❌ No safety/emergency stop mechanism

**Recommendation:** Add simulated killswitch that publishes to `/killswitch/status` topic

---

### 2. MAVLink / Cube Orange Integration

**Real Rover:**
- Cube Orange runs ArduPilot firmware
- Communicates via MAVLink protocol
- Provides GPS, IMU, compass, motor control
- ARM/DISARM safety checks

**Simulation:**
- ❌ No MAVLink layer
- ❌ No ArduPilot integration
- Uses simple differential drive plugin

**Recommendation:** Add MAVROS2 to simulation for MAVLink compatibility

---

### 3. HTTP REST API Server

**Real Rover:**
- `energia_rover_server.py` on port 5000 - Motor control API
- `ros2_sensor_bridge.py` on port 5001 - Sensor data API

**Simulation:**
- ❌ No HTTP API servers

**Recommendation:** Run the same server scripts in simulation to test API integration

---

### 4. Ultrasonic ESP32 Serial Processing

**Real Rover:**
- ESP32 sends JSON packets via USB serial
- `ultrasonic_bridge.py` parses serial data and publishes to ROS2
- Handles filtering, outlier rejection, moving averages

**Simulation:**
- Gazebo sensors directly publish clean data to ROS2
- No serial parsing needed

**Impact:** ⚠️ Simulation won't test serial communication bugs or ESP32 failures

---

### 5. Systemd Auto-Start Services

**Real Rover:**
- Four systemd services auto-start on boot
- Service dependencies configured
- Automatic restart on failure

**Simulation:**
- Launch files used instead
- No systemd integration testing

**Impact:** ⚠️ Can't test deployment/service configuration in simulation

---

## ROS2 Topic Compatibility

### ✅ Topics That Match

| Topic | Message Type | Purpose |
|-------|--------------|---------|
| `/scan` | sensor_msgs/LaserScan | RPLidar A1 360° scan |
| `/ultrasonic/front` | sensor_msgs/Range | Front ultrasonic sensor |
| `/ultrasonic/corner_left` | sensor_msgs/Range | Front-left ultrasonic |
| `/ultrasonic/corner_right` | sensor_msgs/Range | Front-right ultrasonic |
| `/ultrasonic/side_left` | sensor_msgs/Range | Left side ultrasonic |
| `/ultrasonic/side_right` | sensor_msgs/Range | Right side ultrasonic |
| `/ultrasonic/rear` | sensor_msgs/Range | Rear ultrasonic sensor |
| `/odom` | nav_msgs/Odometry | Wheel odometry |
| `/tf` | tf2_msgs/TFMessage | Transform tree |

### ⚠️ Topics That Differ

| Topic | Real Rover | Simulation | Issue |
|-------|-----------|------------|-------|
| Camera | `/image_raw/compressed` | `/camera/image_raw` | Different topic names |
| GPS | `/mavros/global_position/global` | `/gps/fix` | Different topic names |
| IMU | `/mavros/imu/data` | `/imu/data` | Different topic names |
| Motor Control | N/A (uses HTTP API) | `/cmd_vel` | Different control method |

---

## Configuration Files Comparison

### Real Rover Config

**Location:** [/rover/scripts/config.py](../rover/scripts/config.py)

Key parameters:
```python
# Distance Thresholds (centimeters)
CRITICAL_THRESHOLD = 10         # Emergency stop
OBSTACLE_THRESHOLD = 50         # Start reacting
SAFE_DISTANCE = 100             # Distance considered clear

# Movement Speeds (0-255 PWM range)
DEFAULT_SPEED = 150
SLOW_SPEED = 75
TURN_SPEED = 120
```

**Simulation Config:**
- No equivalent config file
- Speed/threshold values hardcoded in launch files
- Uses ROS2 parameters instead

**Recommendation:** Create `simulation_config.py` that mirrors real rover config

---

## Code Portability Issues

### Problem 1: Motor Control Abstraction Missing

**Current State:**
- Simulation code uses: `cmd_vel_pub.publish(twist_msg)`
- Real rover uses: `requests.post('http://localhost:5000/api/arm')`

**Solution Needed:**
- Create motor control abstraction layer that works on both
- Example:
  ```python
  class MotorController:
      def __init__(self, use_simulation=False):
          if use_simulation:
              self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
          else:
              self.api_url = 'http://localhost:5000/api'

      def move_forward(self, speed):
          if self.use_simulation:
              # Publish to cmd_vel
          else:
              # HTTP POST to API
  ```

### Problem 2: Sensor Topic Names Inconsistent

**Solution:**
- Standardize topic names in simulation to match real rover
- Or use topic remapping in launch files

### Problem 3: No ARM/DISARM Concept in Simulation

**Solution:**
- Add simulated ARM/DISARM state machine to simulation
- Publish to `/rover/armed` topic
- Reject motor commands when disarmed

---

## Recommendations for Simulation Improvements

### High Priority (Critical for Development)

1. **Add MAVLink/MAVROS2 Integration**
   - Install `ros-humble-mavros`
   - Connect simulation to PX4 SITL (Software In The Loop)
   - Test MAVLink commands in simulation

2. **Create Motor Control Abstraction Layer**
   - Write wrapper class that works in both sim and real
   - Use ROS2 parameter to switch between modes
   - Document in [DEVELOPERS_GUIDE.md](DEVELOPERS_GUIDE_FOR_GIRLFRIEND.md)

3. **Standardize Topic Names**
   - Update simulation topics to match real rover exactly
   - Update camera topic from `/camera/image_raw` to `/image_raw/compressed`
   - Update GPS topic to `/mavros/global_position/global`

4. **Add Simulated Killswitch**
   - Publish to `/killswitch/status` topic
   - Add keyboard shortcut or service call to trigger
   - Emergency stop behavior when triggered

5. **Run HTTP API Servers in Simulation**
   - Copy `energia_rover_server.py` to simulation workspace
   - Modify to work with `/cmd_vel` instead of MAVLink
   - Test HTTP API integration

### Medium Priority (Nice to Have)

6. **Create Simulation-Specific Config File**
   - Mirror structure of `rover/scripts/config.py`
   - Use same parameter names
   - Document differences

7. **Add Serial Port Simulation**
   - Simulate ESP32 serial JSON output
   - Test `ultrasonic_bridge.py` in simulation
   - Catch serial parsing bugs before hardware

8. **Docker Container for Simulation**
   - Package entire sim environment in Docker
   - Easy setup for new developers (girlfriend!)
   - Consistent environment across machines

### Low Priority (Future Enhancements)

9. **Add Battery Simulation**
   - Simulate battery drain over time
   - Test low-battery behaviors
   - Matches real rover battery monitoring

10. **Simulate Network Latency**
    - Add realistic delays to sensor data
    - Test control systems under latency
    - Matches real-world network conditions

---

## Immediate Action Items for Girlfriend's Setup

### Phase 1: Get Simulation Running (Week 1)

1. ✅ Install Ubuntu 22.04 (if not already)
2. ✅ Install ROS2 Humble
3. ✅ Clone this repository
4. ✅ Build simulation workspace
5. ✅ Test basic simulation launch
6. ✅ Connect Xbox controller (if available)

**See:** [GETTING_STARTED_GUIDE_FOR_GIRLFRIEND.md](GETTING_STARTED_GUIDE_FOR_GIRLFRIEND.md)

### Phase 2: Learn ROS2 Basics (Week 2)

1. Complete ROS2 beginner tutorials
2. Understand topics, nodes, publishers, subscribers
3. Write simple sensor monitoring script
4. Test in simulation

### Phase 3: Develop Portable Code (Week 3+)

1. Use motor control abstraction layer
2. Subscribe to standardized sensor topics
3. Test in simulation
4. Deploy to real rover (when available)

---

## Testing Strategy

### Code That Can Be Tested in Simulation

✅ **Sensor Processing:**
- Ultrasonic obstacle detection algorithms
- LiDAR data interpretation
- Camera image processing
- Sensor fusion logic

✅ **Navigation Behaviors:**
- Obstacle avoidance
- Path planning
- Waypoint following (with GPS simulation)

✅ **Control Algorithms:**
- PID controllers
- State machines
- Decision logic

### Code That CANNOT Be Fully Tested in Simulation

❌ **Hardware Integration:**
- ESP32 serial communication
- MAVLink protocol edge cases
- Real motor response/latency
- Cube Orange failsafes

❌ **Real-World Conditions:**
- GPS accuracy/drift
- Sensor noise characteristics
- Motor current draw
- Battery performance

❌ **Network Issues:**
- HTTP API timeout handling
- WiFi connectivity drops
- ROS2 network partitions

---

## Conclusion

The simulation environment is **excellent for algorithm development and testing**, but requires **several additions** to truly match the real rover's software architecture.

**For your girlfriend to be successful:**

1. Start with simulation for learning and algorithm development
2. Focus on ROS2 sensor topics (these match between sim and real)
3. Use the motor control abstraction layer (need to create this)
4. Be aware that HTTP API integration must be tested on real hardware
5. Understand that MAVLink/Cube Orange behavior differs from simulation

**Next Steps:**
1. Create developer's guide specifically for girlfriend
2. Build motor control abstraction layer
3. Update simulation topic names to match real rover
4. Add simulated killswitch
5. Create example projects demonstrating portable code

---

**Status**: Analysis complete - Ready for developer guide creation
**Next Document**: [GETTING_STARTED_GUIDE_FOR_GIRLFRIEND.md](GETTING_STARTED_GUIDE_FOR_GIRLFRIEND.md)
