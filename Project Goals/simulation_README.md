# Simulation Environment

This directory contains everything needed for Gazebo/RViz simulation of the rover on a desktop workstation.

## 🚀 Quick Start for New Developers

**New to this project?** Read this first: 📋 [**DEVELOPER_ONBOARDING.md**](DEVELOPER_ONBOARDING.md)

Then dive into:
- 📘 [Getting Started Guide](GETTING_STARTED_GUIDE.md) - Complete setup and tutorial
- 🔍 [Simulation vs Real Rover Gaps](SIMULATION_ROVER_GAPS_ANALYSIS.md) - Understanding differences
- 💻 [Example Scripts](examples/) - Working code examples to learn from

## Directory Structure

```
simulation/
├── ros2_ws/src/jetson_rover_sim/   # ROS2 simulation package
│   ├── urdf/                        # Robot URDF models
│   ├── launch/                      # Launch files
│   ├── worlds/                      # Gazebo world files
│   ├── models/                      # Custom Gazebo models
│   └── config/                      # RViz configs
├── examples/                        # Example Python scripts
│   ├── sensor_monitor.py           # Monitor all sensors
│   ├── simple_avoid.py             # Basic obstacle avoidance
│   └── README.md                   # Example script docs
├── launch_local_sim.sh              # Launch simulation locally
├── launch_desktop_sim.sh            # Launch via SSH to desktop
├── launch_desktop_sim_headless.sh   # Headless simulation
├── run_xbox_controller.sh           # Xbox controller teleop
├── rover_sensors.rviz               # RViz visualization config
├── GETTING_STARTED_GUIDE.md         # 📘 START HERE for new devs
└── SIMULATION_ROVER_GAPS_ANALYSIS.md # Sim vs real differences
```

## Quick Start

### Build the workspace
```bash
cd simulation/ros2_ws
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### Launch Options

**Full simulation with Gazebo + RViz:**
```bash
ros2 launch jetson_rover_sim full_simulation.launch.py
```

**View robot model only (no physics):**
```bash
ros2 launch jetson_rover_sim view_rover.launch.py
```

**Spawn in Gazebo:**
```bash
ros2 launch jetson_rover_sim spawn_rover.launch.py
```

## Simulated Sensors

- RPLidar A1 (360 degree 2D laser scan)
- USB Camera
- HERE 3+ RTK GPS
- 6x Ultrasonic sensors
- IMU

## Requirements

- Ubuntu 22.04
- ROS2 Humble
- Gazebo Classic (ros-humble-gazebo-ros-pkgs)
- RViz2

**Full installation instructions:** See [GETTING_STARTED_GUIDE.md](GETTING_STARTED_GUIDE.md)
