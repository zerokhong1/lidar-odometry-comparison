#!/bin/bash
# Run FAST-LIO2 on R-Campus dataset
unset GDK_PIXBUF_MODULE_FILE GTK_EXE_PREFIX GSETTINGS_SCHEMA_DIR GTK_PATH

WS=/home/thailuu/LIMOncello_ws

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

echo "================================================================"
echo "  FAST-LIO2 - Fast Direct LiDAR-Inertial Odometry"
echo "  Dataset: R-Campus (Livox Avia, 1400m campus route)"
echo "================================================================"
echo ""
echo "After RViz opens, open a NEW terminal and run:"
echo "  bash bash scripts/play_rcampus_bag.sh"
echo ""

ros2 launch fast_lio mapping.launch.py \
    config_file:=r_campus.yaml \
    rviz:=true
