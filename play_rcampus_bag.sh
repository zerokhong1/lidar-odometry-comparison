#!/bin/bash
# Play R-Campus bag - run AFTER LIMOncello is launched

WS=/home/thailuu/LIMOncello_ws
BAG=/home/thailuu/datasets/R_Campus

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

echo "Playing R-Campus bag (~19.5 min, 1400m campus route)..."
echo "Topics: /livox/lidar (livox_interfaces/CustomMsg) | /livox/imu (sensor_msgs/Imu)"
echo ""
echo "Press SPACE to pause/resume, Ctrl+C to stop"
echo ""

ros2 bag play $BAG -r 1.0
