# Hardware ROS 2 Packages

This directory contains ROS 2 packages for interfacing with the real rover hardware (as opposed to the Gazebo simulation).

## Directory Structure

```
hardware/
├── ros2_ws_src/
│   └── rplidar_driver/     # RP LiDAR A1 driver
│       ├── rplidar_driver/
│       │   ├── rplidar_node.py    # Main ROS 2 node
│       │   └── rplidar_test.py    # Hardware test utility
│       ├── launch/
│       │   ├── rplidar.launch.py              # Basic LiDAR launch
│       │   └── lidar_with_avoidance.launch.py # LiDAR + obstacle avoidance
│       ├── package.xml
│       └── setup.py
├── setup_workspace.sh      # Build script
└── README.md              # This file
```

## Quick Start

### 1. Install Dependencies

```bash
pip install rplidar-roboticia
```

### 2. Build the Workspace

```bash
cd ~/Git\ Sandbox/Jetson\ Outdoor\ Rover/hardware
./setup_workspace.sh
```

Or manually:
```bash
cd hardware
mkdir -p ros2_ws/src
ln -s ../../ros2_ws_src/* ros2_ws/src/
cd ros2_ws
colcon build
source install/setup.bash
```

### 3. Test LiDAR Connection

```bash
python3 -m rplidar_driver.rplidar_test
```

### 4. Run the Driver

```bash
ros2 launch rplidar_driver rplidar.launch.py
```

## Available Packages

### rplidar_driver

Publishes `/scan` topic from the RP LiDAR A1 sensor.

**Topics Published:**
- `/scan` (sensor_msgs/LaserScan) - Standard laser scan data
- `/scan_reliable` (sensor_msgs/LaserScan) - Same data with RELIABLE QoS

**Launch Files:**
- `rplidar.launch.py` - Basic LiDAR driver
- `lidar_with_avoidance.launch.py` - LiDAR + reactive obstacle avoidance

See [rplidar_driver/README.md](ros2_ws_src/rplidar_driver/README.md) for details.

## Differences from Simulation

The hardware packages interface with real sensors and don't require the workarounds used in simulation (like LiDAR sanitization for inf/nan values).

See [../simulation/SIMULATION_VS_REAL_WORLD.md](../simulation/SIMULATION_VS_REAL_WORLD.md) for detailed differences.
