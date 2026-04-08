#!/bin/bash
# LIMOncello - R-Campus Dataset Runner
# Fix VS Code snap/GTK environment conflict with RViz2
unset GDK_PIXBUF_MODULE_FILE GTK_EXE_PREFIX GSETTINGS_SCHEMA_DIR GTK_PATH

WS=/home/thailuu/LIMOncello_ws
BAG=/home/thailuu/datasets/R_Campus

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

echo "================================================================"
echo "  LIMOncello - SGal(3) LiDAR-Inertial Odometry"
echo "  Dataset: R-Campus (Livox Avia, 1400m campus route)"
echo "================================================================"
echo ""
echo "  Topics published:"
echo "    /limoncello/state     - pose (nav_msgs/Odometry)"
echo "    /limoncello/full_pcl  - current scan in world frame"
echo "    /limoncello/debug/*   - deskewed/filtered/downsampled"
echo ""
echo "  After RViz opens, open a NEW terminal and run:"
echo "    bash /home/thailuu/play_rcampus_bag.sh"
echo ""

ros2 launch limoncello limoncello.launch.py \
    config_name:=r_campus \
    rviz:=true \
    use_sim_time:=false
