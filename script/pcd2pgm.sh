#!/bin/bash
# Change directory
cd /home/cod-sentry/qza_ws/cod_nav

# Source ROS files
source /opt/ros/humble/setup.bash
source install/setup.bash
cd /home/cod-sentry/qza_ws/cod_nav/src/cod_nav/nav_bringup/map/

ros2 launch pcd2pgm pcd2pgm_launch.py &

read

ros2 run nav2_map_server map_saver_cli -f map
