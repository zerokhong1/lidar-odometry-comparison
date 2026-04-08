#!/bin/bash
# Run RESPLE on R-Campus dataset
# Fix VS Code snap/GTK conflict with RViz2
unset GDK_PIXBUF_MODULE_FILE GTK_EXE_PREFIX GSETTINGS_SCHEMA_DIR GTK_PATH

WS=/home/thailuu/LIMOncello_ws

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

echo "================================================================"
echo "  RESPLE - Recursive Spline Estimation for LiDAR Odometry"
echo "  Dataset: R-Campus (Livox Avia, 1400m campus route)"
echo "================================================================"
echo ""
echo "After RViz opens, open a NEW terminal and run:"
echo "  bash bash scripts/play_rcampus_bag.sh"
echo ""

ros2 launch resple resple_r_campus.launch.py
