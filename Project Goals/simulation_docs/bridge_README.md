# Jetson Rover Bridge - Ground Control Integration

HTTP REST API bridge for connecting Mobile RTK Control Module to simulated rover in Gazebo/RViz.

## Overview

This package provides a bidirectional bridge between your Mobile RTK Control Module (ground control software) and the ROS2 rover simulation. Your ground control software can send commands via HTTP REST API and receive telemetry data, exactly as if it were communicating with the real Jetson Orin Nano rover.

## Architecture

```
Mobile RTK Control Module (Python/Tkinter)
           ↓ HTTP REST API (localhost:5001)
     HTTP Bridge Node (Flask + ROS2)
           ↓ ROS2 Topics
     Simulated Rover (Gazebo + RViz)
           ↓ ROS2 Topics
      GPS Bridge Node (converts formats)
           ↓ /gps/fix (NavSatFix)
     HTTP Bridge Node → Mobile RTK Module
```

## Features

### HTTP Bridge Server
- **Port**: 5001 (matches your robot_controller.py configuration)
- **Protocol**: HTTP REST API (compatible with robot_controller.py)
- **Endpoints**: All standard endpoints (health, status, arm, disarm, target, stop, pause, cancel)

### GPS Bridge
- Converts Gazebo GPS (`/gps/pose`) to NavSatFix format (`/gps/fix`)
- Simulates RTK-quality GPS accuracy (0.1m horizontal, 0.2m vertical)
- Configurable origin coordinates (default: San Francisco)

## Quick Start

### 1. Terminal 1: Start Gazebo Simulation

```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard
```

This starts:
- Gazebo physics simulation
- RViz2 visualization
- Simulated rover with sensors

### 2. Terminal 2: Start Ground Control Bridge

```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source install/setup.bash
ros2 launch jetson_rover_bridge ground_control_bridge.launch.py
```

This starts:
- HTTP Bridge Server on port 5001
- GPS format converter

You should see:
```
[http_bridge-1] HTTP Bridge Server started on http://0.0.0.0:5001
[http_bridge-1] API endpoints:
[http_bridge-1]   GET  /api/health  - Health check
[http_bridge-1]   GET  /api/status  - Robot status
[http_bridge-1]   POST /api/arm     - ARM motors
[http_bridge-1]   POST /api/disarm  - DISARM motors
[http_bridge-1]   POST /api/target  - Send GPS waypoint
[http_bridge-1]   POST /api/stop    - Emergency stop
[gps_bridge-2] GPS Bridge started - Origin: 37.774900, -122.419400
```

### 3. Terminal 3: Run Mobile RTK Control Module

```bash
cd ~/Desktop/Mini\ Rover\ Development/Mobile\ RTK\ Control\ Module
python3 09_dashboard_enhanced.py
```

In the dashboard, configure robot connection:
- **Robot IP**: `127.0.0.1` (localhost)
- **Robot Port**: `5001`

Click "Connect" - you should see the connection establish!

## API Endpoints

All endpoints match the interface expected by `robot_controller.py`:

### GET /api/health
Health check endpoint
**Response:**
```json
{
  "success": true,
  "status": "ok",
  "message": "HTTP Bridge Server running",
  "simulation": true
}
```

### GET /api/status
Get current robot status including GPS and odometry
**Response:**
```json
{
  "success": true,
  "armed": false,
  "mode": "manual",
  "timestamp": 1234567890.123,
  "gps": {
    "latitude": 37.774900,
    "longitude": -122.419400,
    "altitude": 0.2,
    "fix_type": 0,
    "satellites": 12
  },
  "position": {
    "x": 0.0,
    "y": 0.0,
    "heading": 0.0
  },
  "velocity": {
    "linear": 0.0,
    "angular": 0.0
  }
}
```

### POST /api/arm
ARM motors to enable movement
**Response:**
```json
{
  "success": true,
  "armed": true,
  "message": "Motors ARMED"
}
```

### POST /api/disarm
DISARM motors and stop movement
**Response:**
```json
{
  "success": true,
  "armed": false,
  "message": "Motors DISARMED"
}
```

### POST /api/target
Send GPS waypoint target
**Request:**
```json
{
  "latitude": 37.775000,
  "longitude": -122.419500,
  "distance_offset": 0.0
}
```
**Response:**
```json
{
  "success": true,
  "message": "Waypoint set"
}
```

### POST /api/stop
Emergency stop - disarm and halt immediately
**Response:**
```json
{
  "success": true,
  "message": "Emergency stop executed"
}
```

### POST /api/pause
Pause rover movement (keep armed)
**Response:**
```json
{
  "success": true,
  "message": "Rover paused"
}
```

### POST /api/cancel
Cancel current mission and clear waypoints
**Response:**
```json
{
  "success": true,
  "message": "Mission cancelled"
}
```

## ROS2 Topics

### Published by Bridge
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands to rover
- `/waypoint/goto` (geometry_msgs/PoseStamped) - Waypoint goals
- `/rover/armed` (std_msgs/Bool) - ARM/DISARM state

### Subscribed by Bridge
- `/gps/pose` (geometry_msgs/PoseStamped) - Gazebo GPS position
- `/odom` (nav_msgs/Odometry) - Wheel odometry
- `/gps/fix` (sensor_msgs/NavSatFix) - Converted GPS data

## Testing Commands

### Test HTTP Bridge Connection
```bash
# Health check
curl http://localhost:5001/api/health

# Get status
curl http://localhost:5001/api/status

# ARM motors
curl -X POST http://localhost:5001/api/arm

# Send waypoint
curl -X POST http://localhost:5001/api/target \
  -H "Content-Type: application/json" \
  -d '{"latitude": 37.775, "longitude": -122.419}'

# DISARM
curl -X POST http://localhost:5001/api/disarm
```

### Monitor ROS2 Topics
```bash
# Check all topics
ros2 topic list

# Monitor velocity commands (from ground control → simulation)
ros2 topic echo /cmd_vel

# Monitor GPS data (from simulation → ground control)
ros2 topic echo /gps/fix

# Monitor odometry
ros2 topic echo /odom
```

### Test Waypoint Following
1. ARM the rover via ground control or curl
2. Send a GPS waypoint via ground control
3. Watch the rover move in Gazebo/RViz
4. Monitor `/cmd_vel` to see velocity commands

## Configuration

### GPS Origin Coordinates
Edit [jetson_rover_bridge/gps_bridge.py:18-19](jetson_rover_bridge/gps_bridge.py#L18-L19):
```python
self.origin_lat = 37.7749  # Your location
self.origin_lon = -122.4194
```

### HTTP Server Port
Edit [jetson_rover_bridge/http_bridge.py:260](jetson_rover_bridge/http_bridge.py#L260):
```python
app.run(host='0.0.0.0', port=5001, debug=False, use_reloader=False)
```

### Mobile RTK Control Module Robot IP
In your dashboard, set:
- Robot IP: `127.0.0.1` (localhost for simulation)
- Robot Port: `5001` (matches HTTP bridge)

For real rover, change to:
- Robot IP: `192.168.x.x` (your Jetson's WiFi IP)
- Robot Port: `5001`

## Usage Workflow

### Typical Testing Session

1. **Launch Simulation**
   ```bash
   ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard
   ```

2. **Start Bridge**
   ```bash
   ros2 launch jetson_rover_bridge ground_control_bridge.launch.py
   ```

3. **Run Ground Control**
   ```bash
   cd Mobile\ RTK\ Control\ Module
   python3 09_dashboard_enhanced.py
   ```

4. **Test Connection**
   - Click "Connect" in dashboard
   - Verify connection status shows "CONNECTED"
   - Check GPS position updates

5. **ARM and Control**
   - Click "ARM" button
   - Record waypoints or send target positions
   - Watch rover move in simulation
   - Monitor sensor data in RViz

6. **DISARM When Done**
   - Click "DISARM" or Emergency Stop
   - Verify rover halts

### Transition to Real Hardware

When deploying to real Jetson Orin Nano:

**Your ground control software needs ZERO changes!**

Just update the robot IP:
- Simulation: `127.0.0.1:5001`
- Real Robot: `192.168.x.x:5001` (Jetson's IP)

The HTTP bridge runs on the Jetson instead of localhost, but the API is identical.

## Troubleshooting

### Bridge won't start
```bash
# Check if port 5001 is already in use
sudo netstat -tlnp | grep 5001

# Kill existing process if needed
sudo kill <PID>
```

### No GPS data
```bash
# Verify Gazebo is publishing GPS
ros2 topic echo /gps/pose --once

# Verify bridge is converting
ros2 topic echo /gps/fix --once

# Check GPS bridge logs
ros2 node list
ros2 node info /gps_bridge
```

### Can't ARM motors
- Check HTTP bridge logs for errors
- Verify `/rover/armed` topic is being published:
  ```bash
  ros2 topic echo /rover/armed
  ```

### Rover doesn't move
- Verify rover is ARMED
- Check if `/cmd_vel` is being published:
  ```bash
  ros2 topic echo /cmd_vel
  ```
- Ensure Gazebo physics is running (not paused)

### Ground control can't connect
- Verify bridge is running: `curl http://localhost:5001/api/health`
- Check firewall settings
- Confirm port 5001 in robot_controller.py matches

## Dependencies

- ROS 2 Humble
- Python 3.10+
- Flask (for HTTP server)
- rclpy (ROS2 Python client)
- sensor_msgs, geometry_msgs, nav_msgs (ROS2 message types)

Install Flask:
```bash
pip3 install flask
```

## Integration Testing Checklist

- [ ] Bridge starts without errors
- [ ] Health endpoint responds
- [ ] Status endpoint returns GPS data
- [ ] ARM/DISARM commands work
- [ ] Velocity commands appear on `/cmd_vel`
- [ ] Ground control connects successfully
- [ ] GPS position updates in dashboard
- [ ] Waypoints can be sent
- [ ] Emergency stop works
- [ ] Rover moves in simulation when commanded

## Future Enhancements

- [ ] GPS waypoint navigation logic (convert lat/lon to local goals)
- [ ] MQTT bridge support (for beacon_prefix topics)
- [ ] Battery simulation
- [ ] Obstacle detection status
- [ ] WiFi RSSI simulation
- [ ] Mission planning interface
- [ ] Geofence enforcement

## License

MIT

## Credits

Developer: Jay
Development Assistant: Claude Code
Hardware: Jetson Orin Nano, Cube Orange, Mobile RTK Terminal
