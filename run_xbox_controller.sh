#!/bin/bash
# Run Xbox 360 Controller for Rover Control
# Make sure simulation is running first!

echo "========================================="
echo "Xbox 360 Controller Setup for Rover"
echo "========================================="
echo ""
echo "Make sure simulation is running first!"
echo "  Run: ./launch_local_sim.sh"
echo ""
echo "Controller mapping:"
echo "  Left Stick Up/Down    - Forward/Backward"
echo "  Left Stick Left/Right - Turn Left/Right"
echo "  Right Bumper (RB)     - Turbo speed (optional)"
echo ""
echo "NO DEADMAN SWITCH - Stick controls directly!"
echo ""
echo "Plug in your Xbox 360 controller now!"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash

# Check if controller is connected
if [ ! -e /dev/input/js0 ]; then
    echo "ERROR: No joystick detected at /dev/input/js0"
    echo "Please plug in your Xbox 360 controller"
    exit 1
fi

echo "Controller detected at /dev/input/js0"
echo ""

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

# Launch joy node and teleop with custom config
ros2 launch teleop_twist_joy teleop-launch.py \
    config_filepath:="${PROJECT_ROOT}/rover/config/xbox_rover.config.yaml"
