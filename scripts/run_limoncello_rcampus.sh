#!/bin/bash
# LIMOncello - R-Campus Dataset Runner
# Fix VS Code snap/GTK environment conflict with RViz2
unset GDK_PIXBUF_MODULE_FILE GTK_EXE_PREFIX GSETTINGS_SCHEMA_DIR GTK_PATH

WS=/home/thailuu/LIMOncello_ws
SCRIPTS_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

mkdir -p ~/results

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
echo "Poses will be saved to: ~/results/limoncello_poses.txt"
echo ""
echo "After RViz opens, open a NEW terminal and run:"
echo "  bash scripts/play_rcampus_bag.sh"
echo ""

# Start pose saver in background
python3 "$SCRIPTS_DIR/save_odometry_tum.py" ~/results/limoncello_poses.txt /limoncello/state &
SAVER_PID=$!

cleanup() {
    kill $SAVER_PID 2>/dev/null
    wait $SAVER_PID 2>/dev/null
    echo ""
    echo "Poses saved to: ~/results/limoncello_poses.txt"
    NLINES=$(wc -l < ~/results/limoncello_poses.txt 2>/dev/null || echo 0)
    echo "Total poses: $NLINES"
}
trap cleanup EXIT

ros2 launch limoncello limoncello.launch.py \
    config_name:=r_campus \
    rviz:=true \
    use_sim_time:=false
