# Simulation Example Scripts

This directory contains example Python scripts demonstrating how to interact with the rover simulation.

## Prerequisites

Make sure the simulation is running:
```bash
cd ~/rover_simulation/simulation/ros2_ws
source install/setup.bash
ros2 launch energia_sim full_simulation.launch.py
```

## Available Examples

### 1. Sensor Monitor (`sensor_monitor.py`)

**Purpose:** Monitors all 6 ultrasonic sensors and displays their readings.

**Run:**
```bash
python3 sensor_monitor.py
```

**What it does:**
- Subscribes to all ultrasonic sensor topics
- Prints sensor distances every second
- Shows status indicators (CLEAR, CAUTION, WARNING, CRITICAL)

**Learning objectives:**
- How to subscribe to ROS2 topics
- Working with sensor_msgs/Range messages
- Creating timer callbacks

---

### 2. Simple Obstacle Avoidance (`simple_avoid.py`)

**Purpose:** Basic autonomous obstacle avoidance behavior.

**Run:**
```bash
python3 simple_avoid.py
```

**What it does:**
- Moves forward when path is clear
- Turns away from obstacles
- Stops if obstacle is too close

**Learning objectives:**
- Publishing to `/cmd_vel` for motor control
- Decision-making based on sensor data
- Simple reactive behavior

---

## Creating Your Own Scripts

### Template Script

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range

class MyRoverNode(Node):
    def __init__(self):
        super().__init__('my_rover_node')

        # Subscribe to sensor
        self.create_subscription(Range, '/ultrasonic/front',
                                self.sensor_callback, 10)

        # Publish motor commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.create_timer(0.1, self.control_loop)

    def sensor_callback(self, msg):
        # Handle sensor data
        distance = msg.range
        self.get_logger().info(f'Distance: {distance}m')

    def control_loop(self):
        # Your control logic here
        twist = Twist()
        twist.linear.x = 0.3  # Move forward
        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = MyRoverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Next Steps

1. **Experiment with parameters** - Change speeds, thresholds in the examples
2. **Combine sensors** - Use LiDAR + ultrasonics together
3. **Add camera vision** - Subscribe to `/camera/image_raw`
4. **Try GPS navigation** - Subscribe to `/gps/fix` for waypoint following
5. **Integrate LLMs** - Use AI for decision-making

See [GETTING_STARTED_GUIDE.md](../GETTING_STARTED_GUIDE.md) for LLM integration ideas!
