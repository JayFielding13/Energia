# Jetson Rover Sensor Configuration

## Ultrasonic Sensor Array

### Hardware Specifications

**Sensor Model:** AJ-SR04M Waterproof Ultrasonic Sensors
**Quantity:** 6 sensors
**Controller:** ESP32 DevKit (connects to Jetson via USB)

#### AJ-SR04M Specifications
| Parameter | Value |
|-----------|-------|
| Operating Voltage | 3.0V - 5.5V |
| Current Draw | 30mA max per sensor |
| Measurement Range | 20cm - 600cm (0.2m - 6m) |
| Detection Angle | 70° cone |
| Accuracy | ±1cm |
| Waterproof Rating | IP67 |
| Operating Temp | -10°C to +70°C |

### Physical Layout

#### Sensor Positions (Top-Down View)

```
                    FRONT

      [Corner L] [Front] [Corner R]
          45°      0°       45°
           \       |       /
            \      |      /
  [Side L]   \     |     /    [Side R]
   90°         \   |   /        90°
                └──┴──┘
                   |
                 [Rear]
                  180°
```

### Mounting Specifications

| Sensor | Position | Angle | Height from Ground | Purpose |
|--------|----------|-------|-------------------|---------|
| **Front** | Front center | 0° (straight ahead) | 11 inches (0.28m) | Forward obstacle detection |
| **Corner Left** | Front-left corner | 45° left | 11 inches (0.28m) | Diagonal left coverage |
| **Corner Right** | Front-right corner | 45° right | 11 inches (0.28m) | Diagonal right coverage |
| **Side Left** | Left side | 90° left | 11 inches (0.28m) | Perpendicular left |
| **Side Right** | Right side | 90° right | 11 inches (0.28m) | Perpendicular right |
| **Rear** | Rear center | 180° (straight back) | 11 inches (0.28m) | Rear obstacle detection |

### Simulated Sensor Characteristics

In Gazebo, each ultrasonic sensor is simulated as:
- **Type:** Ray sensor (simulates ultrasonic pulses)
- **Range:** 0.2m - 6.0m
- **Field of View:** 70° cone (1.22 radians)
- **Resolution:** 1cm
- **Update Rate:** 20 Hz
- **Noise:** Gaussian with ±1cm std dev (matches real sensor accuracy)

### ROS 2 Topics (Simulation)

Each sensor publishes to its own topic:

| Sensor | Topic | Message Type | Frame ID |
|--------|-------|--------------|----------|
| Front | `/ultrasonic/front` | `sensor_msgs/Range` | `front_link` |
| Corner Left | `/ultrasonic/corner_left` | `sensor_msgs/Range` | `corner_left_link` |
| Corner Right | `/ultrasonic/corner_right` | `sensor_msgs/Range` | `corner_right_link` |
| Side Left | `/ultrasonic/side_left` | `sensor_msgs/Range` | `side_left_link` |
| Side Right | `/ultrasonic/side_right` | `sensor_msgs/Range` | `side_right_link` |
| Rear | `/ultrasonic/rear` | `sensor_msgs/Range` | `rear_link` |

### sensor_msgs/Range Message Format

```yaml
radiation_type: 0  # ULTRASOUND
field_of_view: 1.22  # 70° in radians
min_range: 0.2  # meters
max_range: 6.0  # meters
range: <measured_distance>  # meters
```

### Coverage Map

```
        360° Obstacle Detection

    315°    0°/360°    45°
      \       |       /
       \      |      /
    ────[CL][Front][CR]────
   |                        |
270°[SL]      ESP32     [SR]90°
   |                        |
    ────────[Rear]──────────
            180°

Legend:
Front = Front Center
CL = Corner Left
CR = Corner Right
SL = Side Left
SR = Side Right
```

## Integration with Real Hardware

### ESP32 Connection

The real rover uses an ESP32 DevKit connected to the Jetson via USB:

**Serial Communication:**
- **Port:** `/dev/ttyUSB0` or `/dev/ttyACM0`
- **Baud Rate:** 115200
- **Format:** JSON packets at 10-20 Hz

**Data Packet Format:**
```json
{
  "timestamp": 1234567890,
  "sensors": {
    "front": 150,
    "corner_left": 200,
    "corner_right": 180,
    "side_left": 120,
    "side_right": 110,
    "rear": 300
  },
  "valid": {
    "front": true,
    "corner_left": true,
    "corner_right": true,
    "side_left": true,
    "side_right": true,
    "rear": true
  }
}
```

### ROS 2 Node (Real Robot)

A ROS 2 node on the Jetson reads serial data from the ESP32 and publishes to the same topics as the simulation, ensuring code compatibility:

```bash
# On real robot
ros2 run jetson_rover_bringup ultrasonic_serial_node

# Topics will match simulation exactly
ros2 topic list | grep ultrasonic
/ultrasonic/front
/ultrasonic/corner_left
/ultrasonic/corner_right
/ultrasonic/side_left
/ultrasonic/side_right
/ultrasonic/rear
```

## Using Ultrasonic Data in Simulation

### Viewing Sensor Data

```bash
# Echo a specific sensor
ros2 topic echo /ultrasonic/front

# List all ultrasonic topics
ros2 topic list | grep ultrasonic

# Get sensor info
ros2 topic info /ultrasonic/front

# Monitor all sensors in real-time
ros2 topic echo /ultrasonic/front &
ros2 topic echo /ultrasonic/corner_left &
ros2 topic echo /ultrasonic/corner_right &
ros2 topic echo /ultrasonic/side_left &
ros2 topic echo /ultrasonic/side_right &
ros2 topic echo /ultrasonic/rear &
```

### Visualizing in RViz2

While Gazebo is running with the rover:

```bash
# Launch RViz2
rviz2
```

**In RViz2:**
1. Set **Fixed Frame** to `base_link` or `odom`
2. Click **Add** → **By topic**
3. Expand `/ultrasonic/` and add all **Range** topics
4. Adjust **Range** display:
   - **Color:** Different color per sensor
   - **Alpha:** 0.3-0.5 for transparency
   - **Queue Size:** 1

You'll see colored cones representing each sensor's detection range and measured obstacles.

### Sample Python Code

Read ultrasonic data in your code:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class UltrasonicMonitor(Node):
    def __init__(self):
        super().__init__('ultrasonic_monitor')

        # Subscribe to all ultrasonic sensors
        self.front_sub = self.create_subscription(
            Range, '/ultrasonic/front', self.front_callback, 10)
        self.corner_left_sub = self.create_subscription(
            Range, '/ultrasonic/corner_left', self.corner_left_callback, 10)
        self.corner_right_sub = self.create_subscription(
            Range, '/ultrasonic/corner_right', self.corner_right_callback, 10)
        self.side_left_sub = self.create_subscription(
            Range, '/ultrasonic/side_left', self.side_left_callback, 10)
        self.side_right_sub = self.create_subscription(
            Range, '/ultrasonic/side_right', self.side_right_callback, 10)
        self.rear_sub = self.create_subscription(
            Range, '/ultrasonic/rear', self.rear_callback, 10)

        self.distances = {
            'front': float('inf'),
            'corner_left': float('inf'),
            'corner_right': float('inf'),
            'side_left': float('inf'),
            'side_right': float('inf'),
            'rear': float('inf')
        }

    def front_callback(self, msg):
        self.distances['front'] = msg.range
        self.check_obstacles()

    def corner_left_callback(self, msg):
        self.distances['corner_left'] = msg.range

    def corner_right_callback(self, msg):
        self.distances['corner_right'] = msg.range

    def side_left_callback(self, msg):
        self.distances['side_left'] = msg.range

    def side_right_callback(self, msg):
        self.distances['side_right'] = msg.range

    def rear_callback(self, msg):
        self.distances['rear'] = msg.range

    def check_obstacles(self):
        # Example: Check if any obstacle is within 50cm
        OBSTACLE_THRESHOLD = 0.5  # meters

        for sensor, distance in self.distances.items():
            if distance < OBSTACLE_THRESHOLD:
                self.get_logger().warn(
                    f'Obstacle detected by {sensor}: {distance:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    monitor = UltrasonicMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Obstacle Avoidance Strategy

### Detection Zones

```
                    FRONT ZONE
         ┌─────────────────────────────┐
         │  [CL]   [Front]   [CR]      │
         │   \       |       /         │
         │    \      |      /          │
         │     \     |     /           │
LEFT     │      \    |    /            │    RIGHT
ZONE  [SL]       \   |   /          [SR] ZONE
         │         \ | /               │
         │          \|/                │
         │        ROVER                │
         │           |                 │
         │         [Rear]              │
         │           |                 │
         └───────────┴─────────────────┘
                  REAR ZONE
```

### Recommended Safety Thresholds

| Scenario | Threshold | Action |
|----------|-----------|--------|
| **Critical** | < 0.3m | Emergency stop |
| **Warning** | 0.3m - 0.5m | Slow down |
| **Caution** | 0.5m - 1.0m | Proceed with caution |
| **Clear** | > 1.0m | Normal operation |

### Multi-Sensor Fusion

Combine sensors for better awareness:

```python
def get_direction_safety(self):
    """Returns safety status for each direction"""
    return {
        'forward': min(self.distances['front'],
                      self.distances['corner_left'],
                      self.distances['corner_right']),
        'left': min(self.distances['side_left'],
                   self.distances['corner_left']),
        'right': min(self.distances['side_right'],
                    self.distances['corner_right']),
        'backward': self.distances['rear']
    }

def can_move_forward(self):
    """Check if safe to move forward"""
    forward_clearance = self.get_direction_safety()['forward']
    return forward_clearance > 0.5  # 50cm threshold

def find_clear_direction(self):
    """Find the safest direction to move"""
    safety = self.get_direction_safety()
    return max(safety, key=safety.get)  # Direction with most clearance
```

## Testing Ultrasonic Sensors

### In Simulation

1. **Spawn rover in Gazebo**
```bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

2. **Check sensor topics**
```bash
ros2 topic list | grep ultrasonic
```

3. **Place obstacles in Gazebo**
   - Click **Insert** tab in Gazebo
   - Add boxes, cylinders, or other objects
   - Move them near the rover

4. **Monitor sensor readings**
```bash
ros2 topic echo /ultrasonic/front
```

5. **Drive rover toward obstacle**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Watch the distance decrease as you approach!

### Expected Behavior

- **No obstacles:** Range = max_range (6.0m)
- **Object at 1m:** Range ≈ 1.0m ± 0.01m
- **Object < 0.2m:** Range = min_range (0.2m) - sensor saturation
- **Object > 6m:** Range = max_range (6.0m) - out of range

## Future Enhancements

### Planned Additions

- [ ] Combined sensor fusion node (all 6 sensors → unified obstacle map)
- [ ] LaserScan conversion (360° scan from ultrasonic array)
- [ ] Integration with Nav2 costmap
- [ ] Automatic obstacle avoidance behaviors
- [ ] Data logging and visualization tools
- [ ] Sensor health monitoring (detect faulty sensors)

### Integration with Other Sensors

The ultrasonic array complements other sensors:

- **Camera:** Visual identification + ultrasonic distance
- **GPS:** Outdoor navigation + local obstacle avoidance
- **IMU:** Orientation + obstacle detection during turns
- **LiDAR** (if added): Long-range detection + short-range ultrasonics

## Troubleshooting

### Simulation Issues

**No sensor topics appearing:**
```bash
# Check if sensors loaded
ros2 param list | grep ultrasonic

# Verify URDF includes sensors
ros2 param get /robot_state_publisher robot_description | grep ultrasonic
```

**Sensors always return max_range:**
- Check Gazebo is rendering sensor rays (**View** → **Contacts** in Gazebo)
- Ensure obstacles are within sensor range
- Verify sensor orientations (use RViz2 TF display)

**Incorrect sensor angles:**
- Check `yaw` parameter in ultrasonic_sensors.xacro
- Verify coordinate frames in RViz2

### Real Hardware Integration

See [ESP32_ULTRASONIC_WIRING_GUIDE.md](../../Jetson%20Cube%20Orange%20Outdoor%20Rover/ESP32_ULTRASONIC_WIRING_GUIDE.md) for:
- Complete wiring diagrams
- ESP32 pin assignments
- Level shifter configuration
- Serial communication protocol

## References

- Hardware Wiring: `ESP32_ULTRASONIC_WIRING_GUIDE.md`
- ESP32 Pinout: `ESP32_PINOUT_DIAGRAM.md`
- Firmware Code: `esp32_ultrasonic_raw.ino`
- ROS 2 sensor_msgs/Range: https://docs.ros.org/en/humble/p/sensor_msgs/
- Gazebo Ray Sensor: https://classic.gazebosim.org/tutorials?tut=ros2_installing

---

**Created:** November 8, 2025
**Status:** Simulation ready, hardware integration pending
**Next Step:** Create ROS 2 node to read ESP32 serial data and publish to matching topics
