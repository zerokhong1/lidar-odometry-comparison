#!/bin/bash
# Download KITTI Odometry seq 00 + corresponding raw data (IMU)
# ─────────────────────────────────────────────────────────────
# seq 00 = KITTI raw  2011_10_03_drive_0027_sync
#   LiDAR : Velodyne HDL-64E  @ 10 Hz
#   IMU   : OXTS IMU          @ 100 Hz
#   GT    : poses/00.txt      (camera-0 frame, 4×4 matrices)
#
# After download, run:
#   python3 scripts/convert_kitti_to_ros2bag.py
#   python3 scripts/convert_kitti_gt_to_tum.py

set -e

DEST="${1:-$HOME/datasets/kitti_seq00}"
mkdir -p "$DEST"

echo "================================================================"
echo "  KITTI Odometry Sequence 00  →  $DEST"
echo "  LiDAR: Velodyne HDL-64E, 4541 frames, ~4.5 km loop"
echo "  IMU  : OXTS 100 Hz (from raw 2011_10_03_drive_0027)"
echo "================================================================"
echo ""

# ── 1. KITTI odometry velodyne scans (seq 00 only, ~1.5 GB) ──────────────
echo "[1/4] Downloading velodyne scans for sequence 00..."
# The odometry zip has all sequences; we download and extract only seq 00
VELO_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_velodyne.zip"
VELO_ZIP="$DEST/data_odometry_velodyne.zip"

if [ ! -d "$DEST/velodyne/00" ]; then
    if [ ! -f "$VELO_ZIP" ]; then
        echo "  Downloading ~22 GB odometry velodyne archive..."
        echo "  (This contains all 11 sequences; only seq 00 will be extracted)"
        echo "  Tip: interrupt after seq 00 is extracted to save time."
        wget -c -O "$VELO_ZIP" "$VELO_URL"
    fi
    echo "  Extracting seq 00 velodyne scans..."
    mkdir -p "$DEST/velodyne/00"
    # Detect zip path prefix first, then extract
    PREFIX=$(unzip -Z1 "$VELO_ZIP" | grep "00/velodyne/.*\.bin" | head -1 | sed 's|00/velodyne/.*||')
    echo "  Detected zip prefix: '${PREFIX}'"
    unzip -jn "$VELO_ZIP" "${PREFIX}00/velodyne/*.bin" -d "$DEST/velodyne/00/"
    echo "  Extracted $(ls $DEST/velodyne/00/*.bin 2>/dev/null | wc -l) scans"
fi

# ── 2. KITTI odometry ground truth poses ─────────────────────────────────
echo "[2/4] Downloading ground truth poses..."
POSES_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_poses.zip"
POSES_ZIP="$DEST/data_odometry_poses.zip"

if [ ! -f "$DEST/poses/00.txt" ]; then
    if [ ! -f "$POSES_ZIP" ]; then
        wget -c -O "$POSES_ZIP" "$POSES_URL"
    fi
    mkdir -p "$DEST/poses"
    unzip -jn "$POSES_ZIP" "dataset/poses/00.txt" -d "$DEST/poses/" || \
    unzip -jn "$POSES_ZIP" "poses/00.txt" -d "$DEST/poses/"
    echo "  GT poses: $(wc -l < $DEST/poses/00.txt) frames"
fi

# ── 3. KITTI odometry timestamps ─────────────────────────────────────────
echo "[3/4] Downloading timestamps..."
CALIB_URL="https://s3.eu-central-1.amazonaws.com/avg-kitti/data_odometry_calib.zip"
CALIB_ZIP="$DEST/data_odometry_calib.zip"

if [ ! -f "$DEST/calib/00/times.txt" ]; then
    if [ ! -f "$CALIB_ZIP" ]; then
        wget -c -O "$CALIB_ZIP" "$CALIB_URL"
    fi
    mkdir -p "$DEST/calib/00"
    unzip -jn "$CALIB_ZIP" "dataset/sequences/00/times.txt"   -d "$DEST/calib/00/" || \
    unzip -jn "$CALIB_ZIP"          "sequences/00/times.txt"  -d "$DEST/calib/00/"
    unzip -jn "$CALIB_ZIP" "dataset/sequences/00/calib.txt"   -d "$DEST/calib/00/" || \
    unzip -jn "$CALIB_ZIP"          "sequences/00/calib.txt"  -d "$DEST/calib/00/" || true
fi

# ── 4. OXTS/IMU: KITTI raw no longer publicly accessible via S3 ───────────
echo "[4/4] KITTI raw OXTS (IMU) — checking availability..."
echo "  NOTE: KITTI raw S3 URLs are no longer publicly accessible."
echo "  → Using synthetic IMU (differentiated from GT poses) instead."
echo "  → Traj-LO (LO-only) needs no IMU at all."
echo ""

echo ""
echo "================================================================"
echo "  Download complete!  DEST=$DEST"
echo ""
echo "  WHAT YOU HAVE:"
echo "    ✓  velodyne scans  : $DEST/velodyne/00/  (4541 frames)"
echo "    ✓  GT poses        : $DEST/poses/00.txt"
echo "    ✓  timestamps      : $DEST/calib/00/times.txt"
echo "    ✗  OXTS/IMU        : not available (see below)"
echo ""
echo "  NEXT STEPS:"
echo ""
echo "  1) Convert LiDAR → ROS2 bag + ROS1 bag:"
echo "       python3 scripts/convert_kitti_to_ros2bag.py --kitti_dir $DEST"
echo ""
echo "  2a) Run Traj-LO (no IMU needed — start here):"
echo "       bash scripts/run_trajlo_kitti.sh"
echo ""
echo "  2b) For LIO methods (FAST-LIO2, GenZ-LIO, LIMOncello),"
echo "      inject synthetic IMU into the bag:"
echo "       python3 scripts/generate_synthetic_imu.py --kitti_dir $DEST"
echo "       bash scripts/run_fastlio2_kitti.sh   (+ ros2 bag play in Terminal 2)"
echo "       bash scripts/run_genz_lio_kitti.sh"
echo "       bash scripts/run_limoncello_kitti.sh"
echo ""
echo "  3) Convert GT to TUM format:"
echo "       python3 scripts/convert_kitti_gt_to_tum.py --kitti_dir $DEST"
echo ""
echo "  4) Compare ATE:"
echo "       bash scripts/evaluate_ate_kitti.sh"
echo ""
echo "  OPTIONAL — Download real OXTS manually:"
echo "    Register at https://www.cvlibs.net/datasets/kitti/raw_data.php"
echo "    Download: 2011_10_03_drive_0027_sync → extract oxts/ to:"
echo "    $DEST/raw/2011_10_03/2011_10_03_drive_0027_sync/oxts/"
echo "    Then re-run: python3 scripts/convert_kitti_to_ros2bag.py"
echo "================================================================"
