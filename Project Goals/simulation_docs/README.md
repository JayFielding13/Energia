# Jetson Cube Orange Rover Simulation

A ROS 2 Humble simulation package for the Jetson Cube Orange outdoor rover.

## Rover Specifications

### Physical Dimensions
- **Chassis**: 27" L x 23.75" W x 12" H (0.69m x 0.60m x 0.30m)
- **Wheels**: 10" diameter x 3" width (0.25m x 0.076m)
- **Wheelbase**: 17" (0.43m)
- **Track Width**: 23.75" (0.60m)
- **Ground Clearance**: 3" (0.076m)
- **Drive Type**: 4-wheel skid-steer (differential drive)

### Electronics (Simulated)
- Jetson Nano/Orin (mounted inside chassis)
- Cube Orange flight controller (mounted on top)
- Future sensors to be added

## Installation

### Build the Package

```bash
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select jetson_rover_sim
source install/setup.bash
```

## Usage

### Option 1: View Rover in RViz2 (No Simulation)

View the 3D model without physics simulation:

```bash
source ~/Desktop/Mini\ Rover\ Development/ros2_ws/install/setup.bash
ros2 launch jetson_rover_sim view_rover.launch.py
```

This will open:
- **RViz2**: 3D visualization
- **Joint State Publisher GUI**: Manually rotate wheels

### Option 2: Full Gazebo Simulation

Launch the rover in Gazebo with physics:

```bash
source ~/Desktop/Mini\ Rover\ Development/ros2_ws/install/setup.bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

This will:
- Start Gazebo simulator
- Spawn the rover at origin (0, 0, 0.2)
- Enable differential drive controller
- Publish odometry on `/odom` topic
- Listen for velocity commands on `/cmd_vel` topic

### Control the Rover

#### Using Keyboard Teleoperation

Install teleop_twist_keyboard (if not already installed):
```bash
sudo apt install ros-humble-teleop-twist-keyboard
```

In a new terminal:
```bash
source /opt/ros/humble/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Controls:
- `i` - Forward
- `k` - Stop
- `,` - Backward
- `j` - Turn left
- `l` - Turn right
- `q` - Increase speed
- `z` - Decrease speed

#### Using Command Line

Publish velocity commands directly:

```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "linear: {x: 0.5}" --once

# Turn left
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "angular: {z: 0.5}" --once

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once
```

### Visualize in RViz2 During Simulation

While Gazebo is running, launch RViz2 in another terminal:

```bash
source ~/Desktop/Mini\ Rover\ Development/ros2_ws/install/setup.bash
rviz2
```

In RViz2:
1. Set **Fixed Frame** to `odom`
2. Add displays:
   - **RobotModel** - Shows the rover
   - **TF** - Shows coordinate frames
   - **Odometry** - Shows path traveled

## ROS 2 Topics

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Robot odometry (position, velocity) |
| `/tf` | `tf2_msgs/TFMessage` | Transform tree |
| `/joint_states` | `sensor_msgs/JointState` | Wheel joint positions |

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | Velocity commands (linear.x, angular.z) |

## File Structure

```
jetson_rover_sim/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/              # RViz configurations (future)
├── launch/
│   ├── spawn_rover.launch.py    # Launch in Gazebo
│   └── view_rover.launch.py     # View in RViz2 only
├── meshes/              # 3D mesh files (future)
└── urdf/
    ├── jetson_rover.urdf.xacro        # Base robot model
    └── jetson_rover_gazebo.xacro      # Model + Gazebo plugins
```

## Customization

### Adjust Rover Position

Spawn at custom location:
```bash
ros2 launch jetson_rover_sim spawn_rover.launch.py x_pose:=5.0 y_pose:=2.0 z_pose:=0.2
```

### Modify Robot Properties

Edit `urdf/jetson_rover.urdf.xacro`:
- **Mass**: Change `chassis_mass` property (currently 15kg)
- **Wheel friction**: Modify `mu1` and `mu2` in Gazebo tags
- **Colors**: Edit materials in the URDF

### Add Sensors

Future sensors will be added as separate xacro files:
- Camera (`urdf/sensors/camera.xacro`)
- GPS (`urdf/sensors/gps.xacro`)
- IMU (`urdf/sensors/imu.xacro`)
- LiDAR (`urdf/sensors/lidar.xacro`)

## Troubleshooting

### Build Errors

```bash
# Clean and rebuild
cd ~/Desktop/Mini\ Rover\ Development/ros2_ws
rm -rf build install log
colcon build --packages-select jetson_rover_sim
```

### Gazebo Not Starting

```bash
# Kill existing Gazebo processes
pkill gzserver
pkill gzclient

# Restart
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

### Robot Falls Through Ground

If the rover falls through the ground in Gazebo:
- Increase `z_pose` when spawning
- Check collision geometries in URDF

### Robot Doesn't Move

Check topics:
```bash
# Verify cmd_vel is being published
ros2 topic echo /cmd_vel

# Check if differential drive plugin loaded
ros2 topic list | grep odom
```

## Next Steps

1. **Add Sensors**:
   - Camera for computer vision
   - GPS for outdoor navigation
   - IMU (already in Cube Orange)
   - LiDAR for obstacle detection

2. **Integrate MAVROS2**:
   - Connect to PX4 SITL
   - Test autopilot integration

3. **Navigation Stack**:
   - Add Nav2 for autonomous navigation
   - Create outdoor test worlds

4. **Hardware Integration**:
   - Match simulation topics to real robot
   - Test code in sim before deploying

## Resources

- [ROS 2 URDF Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/URDF/URDF-Main.html)
- [Gazebo-ROS Integration](http://classic.gazebosim.org/tutorials?tut=ros2_overview)
- [Differential Drive Plugin](https://github.com/ros-simulation/gazebo_ros_pkgs/wiki/ROS-2-Migration:-Gazebo-ROS-Paths)
