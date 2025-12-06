# Getting Started with Rover Simulation

**Welcome!** This guide will help you get the rover simulation up and running so you can experiment with robotics, ROS2, and potentially integrate LLMs into the rover's decision-making system.

**Target Audience:** Developers without access to physical rover hardware
**Prerequisites:** Basic comfort with Linux command line
**Time to Complete:** 2-3 hours for full setup

---

## Table of Contents

1. [System Requirements](#system-requirements)
2. [Installation](#installation)
3. [First Launch](#first-launch)
4. [Understanding the Simulation](#understanding-the-simulation)
5. [Controlling the Rover](#controlling-the-rover)
6. [Sensor Data Access](#sensor-data-access)
7. [Writing Your First Script](#writing-your-first-script)
8. [LLM Integration Ideas](#llm-integration-ideas)
9. [Next Steps](#next-steps)
10. [Troubleshooting](#troubleshooting)

---

## System Requirements

### Minimum Requirements
- **OS:** Ubuntu 22.04 LTS (recommended) or Ubuntu 20.04
- **RAM:** 8GB minimum, 16GB recommended
- **GPU:** Not required, but helps with Gazebo rendering
- **Disk Space:** 10GB free for ROS2 + packages
- **CPU:** Any modern multi-core processor

### Recommended Setup
- Ubuntu 22.04 LTS (native install or VM)
- 16GB RAM
- SSD for faster simulation
- Dedicated graphics card for smooth visualization

---

## Installation

### Step 1: Install Ubuntu 22.04

If you don't already have Ubuntu 22.04, you can:
- **Dual boot** with your existing OS
- **Virtual Machine** (VMware, VirtualBox)
- **WSL2** (Windows Subsystem for Linux) - works but slower graphics

### Step 2: Install ROS2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe

# Add ROS2 apt repository
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop-full
```

### Step 3: Install Additional ROS2 Packages

```bash
# Install Gazebo Classic (for simulation)
sudo apt install ros-humble-gazebo-ros-pkgs

# Install robot state publisher
sudo apt install ros-humble-robot-state-publisher

# Install xacro for processing robot descriptions
sudo apt install ros-humble-xacro

# Install RViz2 plugins
sudo apt install ros-humble-rviz2

# Install teleop packages for keyboard control
sudo apt install ros-humble-teleop-twist-keyboard

# Install joy for Xbox controller support (optional)
sudo apt install ros-humble-joy ros-humble-teleop-twist-joy

# Install other useful tools
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep
```

### Step 4: Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

### Step 5: Setup ROS2 Environment

Add this to your `~/.bashrc` file:

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# Auto-source workspace if it exists
if [ -f ~/rover_simulation/ros2_ws/install/setup.bash ]; then
    source ~/rover_simulation/ros2_ws/install/setup.bash
fi

# Set ROS domain ID (important for isolation)
export ROS_DOMAIN_ID=42
```

Apply changes:
```bash
source ~/.bashrc
```

### Step 6: Clone the Repository

```bash
# Create directory for rover project
mkdir -p ~/rover_simulation
cd ~/rover_simulation

# Clone the repository
git clone <REPOSITORY_URL> .

# Navigate to simulation workspace
cd simulation/ros2_ws
```

### Step 7: Build the Simulation Workspace

```bash
cd ~/rover_simulation/simulation/ros2_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

**Expected output:** You should see:
```
Starting >>> energia_sim
Finished <<< energia_sim [10.5s]

Summary: 1 package finished [11.2s]
```

---

## First Launch

### Option 1: Full Simulation (Gazebo + RViz + Controller)

This launches everything - the simulated world, the rover, visualization, and Xbox controller support.

```bash
cd ~/rover_simulation/simulation/ros2_ws
source install/setup.bash
ros2 launch energia_sim full_simulation.launch.py
```

**You should see:**
1. **Gazebo** window with the rover in a test yard
2. **RViz2** window showing sensor data visualization
3. Terminal output showing nodes starting

### Option 2: Gazebo Only (No RViz)

If RViz is too heavy for your system:

```bash
ros2 launch energia_sim spawn_rover.launch.py
```

### Option 3: View Robot Model Only (No Physics)

Just want to see what the rover looks like?

```bash
ros2 launch energia_sim view_rover.launch.py
```

---

## Understanding the Simulation

### What's Running?

When you launch the full simulation, several ROS2 nodes start:

| Node | Purpose |
|------|---------|
| `gazebo` | Physics simulation engine |
| `robot_state_publisher` | Publishes robot's coordinate frames |
| `spawn_entity` | Places rover in Gazebo world |
| `joy_node` | Reads Xbox controller input (if connected) |
| `teleop_node` | Converts joystick to velocity commands |
| `rviz2` | 3D visualization of sensor data |

### Available Sensors

The simulated rover has these sensors:

| Sensor | Topic | Message Type | Update Rate |
|--------|-------|--------------|-------------|
| **Ultrasonic (Front)** | `/ultrasonic/front` | sensor_msgs/Range | 20 Hz |
| **Ultrasonic (Corner Left)** | `/ultrasonic/corner_left` | sensor_msgs/Range | 20 Hz |
| **Ultrasonic (Corner Right)** | `/ultrasonic/corner_right` | sensor_msgs/Range | 20 Hz |
| **Ultrasonic (Side Left)** | `/ultrasonic/side_left` | sensor_msgs/Range | 20 Hz |
| **Ultrasonic (Side Right)** | `/ultrasonic/side_right` | sensor_msgs/Range | 20 Hz |
| **Ultrasonic (Rear)** | `/ultrasonic/rear` | sensor_msgs/Range | 20 Hz |
| **LiDAR** | `/scan` | sensor_msgs/LaserScan | 10 Hz |
| **Camera** | `/camera/image_raw` | sensor_msgs/Image | 30 Hz |
| **GPS** | `/gps/fix` | sensor_msgs/NavSatFix | 5 Hz |
| **IMU** | `/imu/data` | sensor_msgs/Imu | 50 Hz |
| **Odometry** | `/odom` | nav_msgs/Odometry | 50 Hz |

---

## Controlling the Rover

### Method 1: Keyboard Control

In a new terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Controls:**
- `i` - Move forward
- `k` - Stop
- `,` - Move backward
- `j` - Turn left
- `l` - Turn right
- `u` - Forward + Left
- `o` - Forward + Right
- `m` - Backward + Left
- `.` - Backward + Right
- `q` - Increase speed
- `z` - Decrease speed

### Method 2: Xbox Controller

If you have an Xbox 360 or compatible controller:

1. Connect controller via USB
2. Launch simulation with `full_simulation.launch.py` (it includes joy nodes)
3. Use left stick for steering, right trigger for forward, left trigger for backward

### Method 3: Command Line (Direct Topic Publishing)

```bash
# Move forward at 0.5 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" --once

# Turn left at 0.5 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.5}}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

### Method 4: Python Script (Your Custom Code!)

See [Writing Your First Script](#writing-your-first-script) below.

---

## Sensor Data Access

### Viewing Topics

List all available topics:
```bash
ros2 topic list
```

Get info about a specific topic:
```bash
ros2 topic info /scan
```

### Echoing Sensor Data

View live sensor data in terminal:

```bash
# View front ultrasonic sensor
ros2 topic echo /ultrasonic/front

# View LiDAR data (lots of output!)
ros2 topic echo /scan

# View GPS position
ros2 topic echo /gps/fix

# View IMU data
ros2 topic echo /imu/data
```

### Measuring Update Rates

Check how fast sensors are publishing:

```bash
# Check LiDAR rate
ros2 topic hz /scan

# Check camera rate
ros2 topic hz /camera/image_raw

# Check ultrasonic rate
ros2 topic hz /ultrasonic/front
```

### Visualizing in RViz2

If you didn't launch with `full_simulation.launch.py`, you can start RViz2 manually:

```bash
rviz2
```

**Setup in RViz2:**
1. Set **Fixed Frame** to `odom` or `base_link`
2. Click **Add** button (bottom left)
3. Add displays:
   - **RobotModel** - Shows 3D rover model
   - **LaserScan** - Shows LiDAR points (topic: `/scan`)
   - **Range** - Shows ultrasonic sensors (add all 6)
   - **Camera** - Shows camera feed (topic: `/camera/image_raw`)
   - **TF** - Shows coordinate frames
   - **Odometry** - Shows path traveled

---

## Writing Your First Script

### Example 1: Simple Sensor Monitor

Create a file: `~/rover_simulation/examples/sensor_monitor.py`

```python
#!/usr/bin/env python3
"""
Simple sensor monitoring script
Subscribes to all ultrasonic sensors and prints distances
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class SensorMonitor(Node):
    def __init__(self):
        super().__init__('sensor_monitor')

        # Subscribe to all ultrasonic sensors
        self.sensors = ['front', 'corner_left', 'corner_right',
                       'side_left', 'side_right', 'rear']

        self.distances = {sensor: float('inf') for sensor in self.sensors}

        for sensor in self.sensors:
            self.create_subscription(
                Range,
                f'/ultrasonic/{sensor}',
                lambda msg, s=sensor: self.sensor_callback(msg, s),
                10
            )

        # Print status every second
        self.create_timer(1.0, self.print_status)

        self.get_logger().info('Sensor monitor started!')

    def sensor_callback(self, msg, sensor_name):
        self.distances[sensor_name] = msg.range

    def print_status(self):
        self.get_logger().info('=== Ultrasonic Sensors ===')
        for sensor, distance in self.distances.items():
            status = '⚠️ OBSTACLE' if distance < 0.5 else '✓ Clear'
            self.get_logger().info(f'{sensor:15s}: {distance:5.2f}m  {status}')


def main(args=None):
    rclpy.init(args=args)
    node = SensorMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Run it:**
```bash
chmod +x ~/rover_simulation/examples/sensor_monitor.py
python3 ~/rover_simulation/examples/sensor_monitor.py
```

### Example 2: Simple Obstacle Avoidance

Create a file: `~/rover_simulation/examples/simple_avoid.py`

```python
#!/usr/bin/env python3
"""
Simple obstacle avoidance using ultrasonic sensors
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider')

        # Subscribe to front sensors
        self.create_subscription(Range, '/ultrasonic/front',
                                self.front_callback, 10)
        self.create_subscription(Range, '/ultrasonic/corner_left',
                                self.corner_left_callback, 10)
        self.create_subscription(Range, '/ultrasonic/corner_right',
                                self.corner_right_callback, 10)

        # Publisher for motor control
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Sensor distances
        self.front = float('inf')
        self.left = float('inf')
        self.right = float('inf')

        # Control loop at 10 Hz
        self.create_timer(0.1, self.control_loop)

        self.get_logger().info('Obstacle avoider started!')

    def front_callback(self, msg):
        self.front = msg.range

    def corner_left_callback(self, msg):
        self.left = msg.range

    def corner_right_callback(self, msg):
        self.right = msg.range

    def control_loop(self):
        twist = Twist()

        OBSTACLE_THRESHOLD = 1.0  # meters
        SPEED = 0.3  # m/s
        TURN_SPEED = 0.5  # rad/s

        if self.front < OBSTACLE_THRESHOLD:
            # Obstacle ahead - turn towards clearer side
            if self.left > self.right:
                self.get_logger().info('Obstacle ahead! Turning left')
                twist.angular.z = TURN_SPEED
            else:
                self.get_logger().info('Obstacle ahead! Turning right')
                twist.angular.z = -TURN_SPEED
        else:
            # Path clear - move forward
            twist.linear.x = SPEED

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoider()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Run it:**
```bash
python3 ~/rover_simulation/examples/simple_avoid.py
```

Place obstacles in Gazebo (Insert tab → drag boxes) and watch the rover avoid them!

---

## LLM Integration Ideas

Since you're interested in integrating LLMs into the rover, here are some project ideas:

### Beginner: Natural Language Control

Use an LLM to convert natural language commands to robot actions.

**Example:**
- Input: "Move forward slowly and stop if you see an obstacle"
- LLM translates to: Subscribe to `/ultrasonic/front`, publish to `/cmd_vel`
- Execute the generated code

**Tools:** OpenAI API, Anthropic Claude API, or local LLM (llama.cpp)

### Intermediate: Sensor Data Interpretation

Feed sensor data to LLM for scene understanding.

**Example:**
- Collect LiDAR scan data
- Convert to text description
- Send to LLM: "What obstacles are around me?"
- LLM responds: "There's a wall 2 meters to your left and a box 1 meter ahead"

### Advanced: Autonomous Decision Making

LLM acts as high-level planner for autonomous navigation.

**Example:**
- LLM receives: GPS position, sensor data, mission goal
- LLM decides: "Navigate to waypoint, but there's obstacle ahead - go around left side"
- LLM generates waypoints, rover executes

### Expert: Multi-Modal Perception

Combine camera, LiDAR, and language for complex understanding.

**Example:**
- Camera captures image
- LiDAR provides 3D structure
- LLM with vision model identifies: "Red barrel 3 meters ahead, safe to pass on right"

### Code Example: LLM Natural Language Control

```python
#!/usr/bin/env python3
"""
LLM-based natural language rover control
Requires: pip install anthropic  (or openai)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import anthropic
import json


class LLMController(Node):
    def __init__(self):
        super().__init__('llm_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.client = anthropic.Anthropic(api_key='YOUR_API_KEY')

        self.get_logger().info('LLM Controller ready!')

    def execute_command(self, natural_language_cmd):
        """Convert natural language to robot command using LLM"""

        prompt = f"""You are a robot control system. Convert this command to JSON:
        Command: {natural_language_cmd}

        Output JSON with 'linear_x' (forward speed m/s) and 'angular_z' (turn speed rad/s).
        Example: {{"linear_x": 0.5, "angular_z": 0.0}} for moving forward.
        """

        response = self.client.messages.create(
            model="claude-3-5-sonnet-20241022",
            max_tokens=256,
            messages=[{"role": "user", "content": prompt}]
        )

        # Parse LLM response
        cmd_json = json.loads(response.content[0].text)

        # Execute
        twist = Twist()
        twist.linear.x = cmd_json.get('linear_x', 0.0)
        twist.angular.z = cmd_json.get('angular_z', 0.0)
        self.cmd_pub.publish(twist)

        self.get_logger().info(f'Executed: {cmd_json}')


def main(args=None):
    rclpy.init(args=args)
    node = LLMController()

    # Example commands
    node.execute_command("move forward slowly")
    rclpy.spin_once(node, timeout_sec=2.0)

    node.execute_command("turn left")
    rclpy.spin_once(node, timeout_sec=2.0)

    node.execute_command("stop")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Next Steps

### Week 1: Get Comfortable with Simulation
- ✅ Launch simulation successfully
- ✅ Control rover with keyboard
- ✅ View sensor data in RViz2
- ✅ Run example sensor monitor script

### Week 2: Learn ROS2 Fundamentals
- Complete [ROS2 tutorials](https://docs.ros.org/en/humble/Tutorials.html)
- Understand publishers, subscribers, topics
- Write custom sensor processing node
- Experiment with obstacle avoidance

### Week 3: Integrate LLMs
- Choose LLM API (OpenAI, Anthropic, or local)
- Implement natural language control
- Test sensor data interpretation
- Build simple autonomous agent

### Week 4: Advanced Projects
- Multi-sensor fusion
- Waypoint navigation with LLM planning
- Camera-based object detection with LLM
- Prepare code for deployment to real rover

---

## Troubleshooting

### Gazebo won't start
```bash
# Kill existing Gazebo processes
pkill gzserver
pkill gzclient

# Try again
ros2 launch energia_sim full_simulation.launch.py
```

### Rover falls through ground
```bash
# Spawn higher
ros2 launch energia_sim spawn_rover.launch.py z_pose:=0.5
```

### "No executable found" error
```bash
# Rebuild workspace
cd ~/rover_simulation/simulation/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### RViz2 is too slow
Launch without RViz:
```bash
ros2 launch energia_sim spawn_rover.launch.py
```

### Can't find /cmd_vel topic
```bash
# Check if differential drive plugin loaded
ros2 topic list | grep cmd_vel

# Check for errors in Gazebo terminal
```

### Python script can't find modules
```bash
# Make sure ROS2 is sourced
source /opt/ros/humble/setup.bash
source ~/rover_simulation/simulation/ros2_ws/install/setup.bash

# Install missing packages
pip3 install <package-name>
```

---

## Getting Help

### Documentation
- **ROS2 Humble Docs:** https://docs.ros.org/en/humble/
- **Gazebo Tutorials:** https://classic.gazebosim.org/tutorials
- **RViz2 User Guide:** https://github.com/ros2/rviz

### Community
- **ROS Discourse:** https://discourse.ros.org/
- **Robotics Stack Exchange:** https://robotics.stackexchange.com/

### Project-Specific
- Check [README.md](README.md) for simulation overview
- See [SIMULATION_ROVER_GAPS_ANALYSIS.md](SIMULATION_ROVER_GAPS_ANALYSIS.md) for sim vs real differences
- Review example scripts in `/examples` directory

---

## Summary

You now have:
- ✅ Fully functional rover simulation
- ✅ Understanding of available sensors
- ✅ Multiple ways to control the rover
- ✅ Example scripts to learn from
- ✅ Ideas for LLM integration
- ✅ Path forward for development

**Start experimenting and have fun!** The simulation is your safe sandbox - you can't break anything, so try wild ideas and see what works.

When you're ready to test on the real rover, review [SIMULATION_ROVER_GAPS_ANALYSIS.md](SIMULATION_ROVER_GAPS_ANALYSIS.md) to understand what will need to change.

**Happy coding!** 🤖🚀
