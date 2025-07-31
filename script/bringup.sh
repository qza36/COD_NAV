#!/bin/bash
# Set password variable directly in the current script if needed
PASSWORD="1"

# Change directory
cd /home/cod-sentry/qza_ws/cod_nav

# Source ROS files
source /opt/ros/humble/setup.bash
source install/setup.bash

# Script to launch various ROS2 packages for the robot system
echo "Starting ROS2 packages for robot operation..."

# Launch rm_bringup
echo "Launching rm_bringup..."
ros2 launch rm_bringup bringup.launch.py &
RM_BRINGUP_PID=$!

# Wait a bit for initialization
sleep 3

# Launch nav_bringup bringup
echo "Launching nav_bringup bringup..."
ros2 launch nav_bringup bringup.launch.py &
NAV_BRINGUP_PID=$!

# Wait a bit for initialization
sleep 3

# Launch nav_bringup nav_bringup
echo "Launching nav_bringup nav_bringup..."
#ros2 launch nav_bringup nav_bringup.launch.py &
ros2 launch nav_bringup nav_bringup.launch.py map:=/home/cod-sentry/qza_ws/cod_nav/src/cod_nav/nav_bringup/map/map.yaml &
NAV_NAV_BRINGUP_PID=$!

# Wait a bit for initialization
sleep 3

# Launch cod_behavior
#echo "Launching cod_behavior..."
ros2 launch cod_behavior bringup_behavior.launch.py & rviz2
#COD_BEHAVIOR_PID=$!

echo "All packages have been launched successfully!"
echo "Processes will keep running in the background."
echo "PIDs:"
echo "rm_bringup: $RM_BRINGUP_PID"
echo "nav_bringup: $NAV_BRINGUP_PID"
echo "nav_bringup (nav): $NAV_NAV_BRINGUP_PID" 
echo "cod_behavior: $COD_BEHAVIOR_PID"
echo ""
echo "Use 'kill <PID>' to terminate a specific process if needed."

# Keep the script running to maintain the processes
# This way the processes won't be terminated when the script exits
echo "Press Ctrl+C to exit this script (processes will continue running in background)."
wait
