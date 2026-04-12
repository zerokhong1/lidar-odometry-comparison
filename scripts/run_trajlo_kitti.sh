#!/bin/bash
# Run Traj-LO (headless) on KITTI Odometry Sequence 00 (ROS1 bag)
# Traj-LO is LiDAR-only (no IMU needed) — proper ATE available!
set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
TRAJLO_BIN=/home/thailuu/LIMOncello_ws/src/Traj-LO/build/trajlo_headless
CONFIG="$REPO/configs/trajlo/config_kitti_seq00.yaml"
KITTI_DIR="${KITTI_DIR:-$HOME/datasets/kitti_seq00}"
RESULT="$HOME/results/trajlo_kitti_poses.txt"

# ── Pre-flight checks ────────────────────────────────────────────────────
if [ ! -f "$TRAJLO_BIN" ]; then
    echo "[ERROR] trajlo_headless not built: $TRAJLO_BIN"
    echo "Build it first:"
    echo "  cd ~/LIMOncello_ws/src/Traj-LO/build && make -j\$(nproc)"
    exit 1
fi

BAG="$KITTI_DIR/kitti_seq00_ros1.bag"
if [ ! -f "$BAG" ]; then
    echo "[ERROR] ROS1 bag not found: $BAG"
    echo "Run: bash $REPO/scripts/download_kitti_seq00.sh"
    echo "     python3 $REPO/scripts/convert_kitti_to_ros2bag.py"
    exit 1
fi

# Patch bag path in config (create a temp config with the correct path)
TMP_CFG=$(mktemp /tmp/trajlo_kitti_XXXXXX.yaml)
sed "s|path:.*|path: \"$BAG\"|" "$CONFIG" | \
sed "s|pose_file_path:.*|pose_file_path: \"$RESULT\"|" > "$TMP_CFG"

mkdir -p "$HOME/results"

echo "================================================================"
echo "  Traj-LO - Continuous-Time LiDAR Odometry (headless)"
echo "  Dataset: KITTI Odometry Sequence 00"
echo "  Sensor : Velodyne HDL-64E @ 10 Hz (LiDAR-only, no IMU)"
echo "  GT     : poses/00.txt → ATE computable!"
echo "  Output : $RESULT"
echo "================================================================"
echo ""

"$TRAJLO_BIN" "$TMP_CFG"
rm -f "$TMP_CFG"

echo ""
echo "Done! Evaluate ATE:"
GT="$KITTI_DIR/kitti_seq00_gt_tum.txt"
if [ -f "$GT" ]; then
    echo "  evo_ape tum $GT $RESULT --align --plot"
else
    echo "  [!] GT not converted yet — run first:"
    echo "      python3 $REPO/scripts/convert_kitti_gt_to_tum.py"
fi
echo ""
echo "Or run the full comparison:"
echo "  bash $REPO/scripts/evaluate_ate_kitti.sh"
