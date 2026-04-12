#!/bin/bash
# Run FAST-LIO2 on KITTI Odometry Sequence 00
# Velodyne HDL-64E — has ground truth → proper ATE computable!
set -e
unset GDK_PIXBUF_MODULE_FILE GTK_EXE_PREFIX GSETTINGS_SCHEMA_DIR GTK_PATH

WS=/home/thailuu/LIMOncello_ws
REPO="$(cd "$(dirname "$0")/.." && pwd)"
KITTI_DIR="${KITTI_DIR:-$HOME/datasets/kitti_seq00}"
BAG="$KITTI_DIR/kitti_seq00_ros2"
RESULT="$HOME/results/fastlio2_kitti_poses.txt"

# ── Pre-flight checks ────────────────────────────────────────────────────
if [ ! -d "$BAG" ]; then
    echo "[ERROR] ROS2 bag not found: $BAG"
    echo "Run first:"
    echo "  bash $REPO/scripts/download_kitti_seq00.sh"
    echo "  python3 $REPO/scripts/convert_kitti_to_ros2bag.py"
    exit 1
fi

# Copy KITTI config into workspace (if different from installed version)
SRC_CFG="$REPO/configs/fast_lio2/kitti_seq00.yaml"
DST_CFG="$WS/src/FAST_LIO2/config/kitti_seq00.yaml"
if [ ! -f "$DST_CFG" ] || ! diff -q "$SRC_CFG" "$DST_CFG" > /dev/null 2>&1; then
    cp "$SRC_CFG" "$DST_CFG"
    echo "Copied KITTI config → $DST_CFG"
fi

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

mkdir -p "$HOME/results"

echo "================================================================"
echo "  FAST-LIO2 - KITTI Odometry Sequence 00"
echo "  Sensor: Velodyne HDL-64E @ 10 Hz, 4541 frames, ~4.5 km"
echo "  IMU   : KITTI OXTS @ 100 Hz"
echo "  GT    : poses/00.txt  →  ATE computable!"
echo "================================================================"
echo ""
echo "Poses will be saved to: $RESULT"
echo ""
echo "Bag will be played in a SECOND TERMINAL after RViz opens:"
echo "  ros2 bag play $BAG --clock"
echo ""

# Pose saver
SCRIPTS_DIR="$(dirname "$0")"
python3 "$SCRIPTS_DIR/save_odometry_tum.py" "$RESULT" /Odometry &
SAVER_PID=$!

cleanup() {
    kill $SAVER_PID 2>/dev/null
    wait $SAVER_PID 2>/dev/null
    echo ""
    NLINES=$(wc -l < "$RESULT" 2>/dev/null || echo 0)
    echo "Poses saved: $NLINES  →  $RESULT"
}
trap cleanup EXIT

ros2 launch fast_lio mapping.launch.py \
    config_file:=kitti_seq00.yaml \
    rviz:=true
