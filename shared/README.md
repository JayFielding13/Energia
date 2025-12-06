# Shared Code

This directory contains ROS2 packages that work with both simulation and real hardware.

## Directory Structure

```
shared/
└── ros2_ws/src/energia_bridge/   # HTTP API bridge package
    ├── energia_bridge/           # Python modules
    │   ├── http_bridge.py             # REST API server (port 5001)
    │   ├── gps_bridge.py              # GPS format converter
    │   ├── apriltag_detector.py       # Visual tag detection
    │   ├── apriltag_follower.py       # Waypoint following
    │   ├── apriltag_follower_reactive.py  # Reactive mode
    │   ├── reactive_obstacle_avoidance.py # VFH steering
    │   ├── obstacle_detector_lidar.py # LiDAR processing
    │   ├── obstacle_detector_ultrasonic.py # Ultrasonic processing
    │   └── obstacle_fusion.py         # Multi-sensor fusion
    └── launch/
        ├── ground_control_bridge.launch.py  # HTTP + GPS bridge
        └── apriltag_follow_test.launch.py   # Full test suite
```

## Why Shared?

The bridge package provides a **unified HTTP REST API** that works identically whether connected to:
- **Simulation:** Gazebo simulation on desktop
- **Real Hardware:** Jetson rover in the field

This means ground control software (Mobile RTK Control Module) needs zero changes when switching between simulation and real hardware - only the IP address changes.

## Building

```bash
cd shared/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Running with Simulation

```bash
# Terminal 1: Start simulation (from simulation/ directory)
ros2 launch energia_sim spawn_rover.launch.py

# Terminal 2: Start bridge
ros2 launch energia_bridge ground_control_bridge.launch.py
```

## API Endpoints

All endpoints on `http://localhost:5001/api/`:

| Endpoint | Method | Description |
|----------|--------|-------------|
| /health | GET | Health check |
| /status | GET | Robot status + GPS |
| /arm | POST | ARM motors |
| /disarm | POST | DISARM motors |
| /target | POST | Send GPS waypoint |
| /stop | POST | Emergency stop |
