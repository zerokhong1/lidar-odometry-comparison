#!/bin/bash
# Run LIMOncello on KITTI Odometry Sequence 00
# Velodyne HDL-64E (type=1) — proper ATE available!
set -e
unset GDK_PIXBUF_MODULE_FILE GTK_EXE_PREFIX GSETTINGS_SCHEMA_DIR GTK_PATH

WS=/home/thailuu/LIMOncello_ws
REPO="$(cd "$(dirname "$0")/.." && pwd)"
KITTI_DIR="${KITTI_DIR:-$HOME/datasets/kitti_seq00}"
BAG="$KITTI_DIR/kitti_seq00_ros2"
RESULT="$HOME/results/limoncello_kitti_poses.txt"

if [ ! -d "$BAG" ]; then
    echo "[ERROR] ROS2 bag not found: $BAG"
    echo "Run: bash $REPO/scripts/download_kitti_seq00.sh"
    echo "     python3 $REPO/scripts/convert_kitti_to_ros2bag.py"
    exit 1
fi

# Install KITTI config into LIMOncello config directory
SRC_CFG="$REPO/configs/limoncello/kitti_seq00.yaml"
DST_CFG="$WS/src/LIMOncello/config/kitti_seq00.yaml"
if [ ! -f "$DST_CFG" ] || ! diff -q "$SRC_CFG" "$DST_CFG" > /dev/null 2>&1; then
    cp "$SRC_CFG" "$DST_CFG"
    echo "Copied KITTI config → $DST_CFG"
fi

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

mkdir -p "$HOME/results"

echo "================================================================"
echo "  LIMOncello - SGal(3) LiDAR-Inertial Odometry"
echo "  Dataset: KITTI Odometry Sequence 00"
echo "  Sensor : Velodyne HDL-64E (type=1) @ 10 Hz"
echo "  IMU    : KITTI OXTS @ 100 Hz"
echo "  GT     : poses/00.txt → ATE computable!"
echo "================================================================"
echo ""
echo "Poses will be saved to: $RESULT"
echo ""
echo "After RViz opens, in a NEW terminal:"
echo "  ros2 bag play $BAG --clock"
echo ""

SCRIPTS_DIR="$(dirname "$0")"
python3 "$SCRIPTS_DIR/save_odometry_tum.py" "$RESULT" /limoncello/state &
SAVER_PID=$!

cleanup() {
    kill $SAVER_PID 2>/dev/null
    wait $SAVER_PID 2>/dev/null
    echo ""
    NLINES=$(wc -l < "$RESULT" 2>/dev/null || echo 0)
    echo "Poses saved: $NLINES  →  $RESULT"
}
trap cleanup EXIT

ros2 launch limoncello limoncello.launch.py \
    config_name:=kitti_seq00 \
    rviz:=true \
    use_sim_time:=false
