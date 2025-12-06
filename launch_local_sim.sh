#!/bin/bash
# Launch Rover Simulation Locally on This Machine
# No network, no SSH, just local Gazebo!

echo "========================================="
echo "Launching Rover Simulation (LOCAL)"
echo "========================================="
echo ""
echo "Running on: $(hostname)"
echo "CPU: AMD Ryzen 5 PRO 5650U"
echo "RAM: 28GB"
echo "GPU: AMD Radeon Vega Graphics"
echo ""
echo "Expected performance: 40-60 fps in Gazebo"
echo "========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
# Get the directory where this script lives
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source simulation workspace
source "$PROJECT_ROOT/simulation/ros2_ws/install/setup.bash" 2>/dev/null || {
    echo "Workspace not built yet. Building..."
    cd "$PROJECT_ROOT/simulation/ros2_ws"
    colcon build
    source install/setup.bash
}

# Launch simulation
echo "Starting Gazebo simulation..."
ros2 launch jetson_rover_sim spawn_rover.launch.py
