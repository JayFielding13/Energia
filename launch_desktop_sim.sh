#!/bin/bash
# Launch Gazebo simulation on desktop from Jetson
# This script SSHs to the desktop and launches the rover simulation

# Tailscale IP for Ubuntu Desktop (jay-desktop)
DESKTOP_IP="100.73.129.15"
SSH_KEY="$HOME/.ssh/jetson_to_desktop"

echo "========================================="
echo "Launching Gazebo Simulation on Desktop"
echo "========================================="
echo "Desktop IP: $DESKTOP_IP"
echo ""
echo "This will:"
echo "  1. Configure distributed ROS2 on desktop"
echo "  2. Launch Gazebo with the rover"
echo "  3. Launch RViz for visualization"
echo ""
echo "On the desktop, Gazebo and RViz will open."
echo "On this Jetson, you can monitor topics:"
echo "  source ros2_distributed_setup.sh"
echo "  ros2 topic list"
echo "  ros2 topic hz /scan"
echo ""
echo "Press Ctrl+C here to stop the simulation."
echo "========================================="
echo ""

# Launch simulation on desktop
ssh -i "$SSH_KEY" -X jay@$DESKTOP_IP "
    source ~/ros2_distributed_setup.sh
    cd ~/ros2_ws
    source install/setup.bash
    echo 'Launching rover simulation...'
    ros2 launch jetson_rover_sim visualize_rover.launch.py world:=test_yard
"
