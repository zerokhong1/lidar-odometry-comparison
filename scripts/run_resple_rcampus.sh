#!/bin/bash
# Run RESPLE on R-Campus dataset
unset GDK_PIXBUF_MODULE_FILE GTK_EXE_PREFIX GSETTINGS_SCHEMA_DIR GTK_PATH

WS=/home/thailuu/LIMOncello_ws
SCRIPTS_DIR="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

mkdir -p ~/results

echo "================================================================"
echo "  RESPLE - Recursive Spline Estimation for LiDAR Odometry"
echo "  Dataset: R-Campus (Livox Avia, 1400m campus route)"
echo "  Mode: LiDAR-only (if_lidar_only: true in config)"
echo "================================================================"
echo ""
echo "Poses will be saved to: ~/results/resple_poses.txt"
echo ""
echo "After RViz opens, open a NEW terminal and run:"
echo "  bash scripts/play_rcampus_bag.sh"
echo ""

# Start pose saver in background
python3 "$SCRIPTS_DIR/save_odometry_tum.py" ~/results/resple_poses.txt /odometry &
SAVER_PID=$!

cleanup() {
    kill $SAVER_PID 2>/dev/null
    wait $SAVER_PID 2>/dev/null
    echo ""
    echo "Poses saved to: ~/results/resple_poses.txt"
    NLINES=$(wc -l < ~/results/resple_poses.txt 2>/dev/null || echo 0)
    echo "Total poses: $NLINES"
}
trap cleanup EXIT

ros2 launch resple resple_r_campus.launch.py
