# RP LiDAR A1 ROS 2 Driver

ROS 2 driver for the RP LiDAR A1 (and compatible models) that publishes to the `/scan` topic for obstacle avoidance and mapping applications.

## Hardware Specifications

| Specification | Value |
|--------------|-------|
| Model | RP LiDAR A1 |
| Range | 0.15m - 12.0m |
| Scan Rate | 5.5 Hz (default) |
| Samples | 360 per revolution (1° resolution) |
| Interface | USB (CP2102 UART bridge) |
| Power | 5V via USB |

## Installation

### 1. Install Python Dependencies

```bash
pip install rplidar-roboticia
```

### 2. Build the ROS 2 Package

```bash
cd ~/ros2_ws
colcon build --packages-select rplidar_driver
source install/setup.bash
```

### 3. Configure USB Permissions

The LiDAR appears as `/dev/ttyUSB0` (or similar). You need read/write access:

**Option A: Temporary (resets on reboot)**
```bash
sudo chmod 666 /dev/ttyUSB0
```

**Option B: Permanent (add user to dialout group)**
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for this to take effect
```

**Option C: Create udev rule (recommended)**
```bash
echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0666", SYMLINK+="rplidar"' | sudo tee /etc/udev/rules.d/99-rplidar.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```

This creates a symlink `/dev/rplidar` for easy access.

## Usage

### Test Hardware Connection (No ROS)

First, verify the LiDAR is working:

```bash
python3 -m rplidar_driver.rplidar_test
# Or specify port:
python3 -m rplidar_driver.rplidar_test /dev/ttyUSB0
```

### Run the ROS 2 Node

**Simple launch:**
```bash
ros2 launch rplidar_driver rplidar.launch.py
```

**With custom port:**
```bash
ros2 launch rplidar_driver rplidar.launch.py serial_port:=/dev/ttyUSB1
```

**With obstacle avoidance:**
```bash
ros2 launch rplidar_driver lidar_with_avoidance.launch.py
```

### Run Node Directly

```bash
ros2 run rplidar_driver rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0
```

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | 360° laser scan data (BEST_EFFORT QoS) |
| `/scan_reliable` | `sensor_msgs/LaserScan` | Same data with RELIABLE QoS |

The `/scan_reliable` topic is provided for compatibility with Python subscribers in ROS 2 Humble that may have issues with BEST_EFFORT QoS.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `serial_port` | string | `/dev/ttyUSB0` | USB serial port |
| `serial_baudrate` | int | `115200` | Baud rate |
| `frame_id` | string | `rplidar_link` | TF frame for scan data |
| `inverted` | bool | `false` | Flip scan direction |
| `range_min` | float | `0.15` | Minimum valid range (m) |
| `range_max` | float | `12.0` | Maximum valid range (m) |

## Integration with Obstacle Avoidance

The existing `reactive_obstacle_avoidance` node in `jetson_rover_bridge` can subscribe to `/scan` directly:

```bash
# Launch LiDAR + avoidance stack
ros2 launch rplidar_driver lidar_with_avoidance.launch.py
```

This publishes:
- `/avoidance/steering_vector` - Suggested steering direction
- `/avoidance/safe_direction` - Safest heading
- `/avoidance/critical_danger` - Emergency stop flag
- `/avoidance/speed_scale` - Speed reduction factor

## Troubleshooting

### LiDAR Not Found

```
ERROR: Failed to connect to RP LiDAR
```

1. Check USB connection
2. Verify port: `ls -la /dev/ttyUSB*`
3. Check permissions (see Installation section)
4. Try different USB port

### No Scan Data

```
Scan rate: 0.0 Hz
```

1. Ensure motor is spinning (you should hear it)
2. Check LiDAR health: `ros2 run rplidar_driver rplidar_test`
3. Clean the sensor window

### Permission Denied

```
[Errno 13] Permission denied: '/dev/ttyUSB0'
```

Run:
```bash
sudo chmod 666 /dev/ttyUSB0
# Or permanently add to dialout group (see Installation)
```

## Differences from Simulation

In simulation (Gazebo), the `/scan` topic comes from `libgazebo_ros_ray_sensor.so` and may contain `inf`/`nan` values requiring sanitization. The real RP LiDAR A1 does not have this issue - you can subscribe directly to `/scan`.

See [SIMULATION_VS_REAL_WORLD.md](../../simulation/SIMULATION_VS_REAL_WORLD.md) for details.
