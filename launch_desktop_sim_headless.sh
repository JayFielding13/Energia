#!/bin/bash
# Launch Gazebo simulation on desktop (headless mode - no GUI)
# This is useful if you want to run Gazebo on desktop but view topics from Jetson

# Tailscale IP for Ubuntu Desktop (jay-desktop)
DESKTOP_IP="100.73.129.15"
SSH_KEY="$HOME/.ssh/jetson_to_desktop"

echo "========================================="
echo "Launching Headless Simulation on Desktop"
echo "========================================="
echo "Desktop IP: $DESKTOP_IP"
echo ""
echo "This launches Gazebo in headless mode (no GUI)"
echo "All sensor data will be published via ROS2"
echo ""
echo "To view the data on Jetson:"
echo "  source ros2_distributed_setup.sh"
echo "  ros2 topic list"
echo "  ros2 topic echo /scan --once"
echo "  rviz2  # Open RViz locally"
echo ""
echo "Press Ctrl+C to stop the simulation."
echo "========================================="
echo ""

# Launch headless simulation on desktop
ssh -i "$SSH_KEY" jay@$DESKTOP_IP "
    source ~/ros2_distributed_setup.sh
    cd ~/ros2_ws
    source install/setup.bash
    echo 'Launching headless rover simulation...'
    LIBGL_ALWAYS_SOFTWARE=1 ros2 launch jetson_rover_sim spawn_rover.launch.py use_sim_time:=true
"
