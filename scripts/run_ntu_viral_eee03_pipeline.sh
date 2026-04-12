#!/usr/bin/env bash
# Pipeline: run all 5 LiDAR-odometry algorithms on NTU VIRAL eee_03
# Dataset:  https://ntu-viral.github.io/  (eee_03 sequence)
# LiDAR:    Ouster OS1-16 (horizontal), /os1_cloud_node1/points
# IMU:      VN100, /imu/imu @ ~389 Hz
set -e

WS=~/LIMOncello_ws
BAG_ROS1=~/datasets/ntu_viral/eee_03/eee_03.bag
BAG_ROS2=~/datasets/ntu_viral/eee_03/eee_03_ros2
GT=~/datasets/ntu_viral/gt/eee_03_gt_tum.txt
RESULTS=~/results

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

mkdir -p $RESULTS/logs

# ── Helper: clean up background processes on exit ───────────────────────────
cleanup() { kill $(jobs -p) 2>/dev/null || true; sleep 1; }
trap cleanup EXIT

# ── Helper: run one ros2-based algorithm ────────────────────────────────────
run_ros2_algo() {
    local NAME=$1 ALGO_CMD=$2 ODOM_TOPIC=$3
    local OUT=$RESULTS/${NAME}_ntu_eee03_poses.txt
    echo ""; echo "=== $NAME ==="; rm -f $OUT
    eval "$ALGO_CMD" > $RESULTS/logs/${NAME}.log 2>&1 &
    local ALGO_PID=$!; sleep 6
    python3 ~/scripts/record_poses_tum.py $ODOM_TOPIC $OUT > /dev/null 2>&1 &
    local REC_PID=$!; sleep 2
    ros2 bag play $BAG_ROS2 --clock -r 1.0 > /dev/null 2>&1
    sleep 5
    kill $REC_PID $ALGO_PID 2>/dev/null || true
    wait $REC_PID $ALGO_PID 2>/dev/null || true
    echo "  Saved $(wc -l < $OUT) poses -> $OUT"
}

# 1. FAST-LIO2
run_ros2_algo "fastlio2" \
    "ros2 run fast_lio fastlio_mapping --ros-args \
     --params-file $WS/src/FAST_LIO2/config/ntu_viral_eee03.yaml" \
    "/Odometry"

# 2. GenZ-LIO (saves TUM internally to ~/results/genz_lio_poses.txt; remap ts afterward)
echo ""; echo "=== GenZ-LIO ==="
ros2 run genz_lio genz_lio_node --ros-args \
    --params-file $WS/src/genz_lio/config/ntu_viral_eee03.yaml \
    > $RESULTS/logs/genz_lio.log 2>&1 &
GENZ_PID=$!; sleep 6
ros2 bag play $BAG_ROS2 --clock -r 1.0 > /dev/null 2>&1
sleep 5; kill $GENZ_PID 2>/dev/null || true
# Remap wall-time timestamps to bag time using offset
python3 - <<'PYEOF'
import numpy as np
fastlio = np.loadtxt('/home/thailuu/results/fastlio2_ntu_eee03_poses.txt')
genz    = np.loadtxt('/home/thailuu/results/genz_lio_poses.txt')
genz[:,0] += fastlio[0,0] - genz[0,0]
np.savetxt('/home/thailuu/results/genz_lio_ntu_eee03_poses.txt', genz, fmt='%.9f')
print(f"  GenZ-LIO remapped {len(genz)} poses")
PYEOF

# 3. LIMOncello
run_ros2_algo "limoncello" \
    "ros2 launch limoncello limoncello.launch.py \
     config_name:=ntu_viral_eee03 rviz:=false use_sim_time:=false" \
    "/limoncello/state"

# 4. RESPLE
CFG_RESPLE=$WS/src/RESPLE/resple/config/config_ntu_viral_eee03.yaml
OUT_RESPLE=$RESULTS/resple_ntu_eee03_poses.txt
echo ""; echo "=== RESPLE ==="; rm -f $OUT_RESPLE
ros2 run resple RESPLE --ros-args --params-file $CFG_RESPLE > $RESULTS/logs/resple.log 2>&1 &
R1=$!
ros2 run resple Mapping --ros-args --params-file $CFG_RESPLE > $RESULTS/logs/resple_mapping.log 2>&1 &
R2=$!; sleep 8
python3 ~/scripts/record_poses_tum.py /odometry $OUT_RESPLE > /dev/null 2>&1 &
RR=$!; sleep 2
ros2 bag play $BAG_ROS2 --clock -r 1.0 > /dev/null 2>&1
sleep 5; kill $RR $R1 $R2 2>/dev/null || true
echo "  Saved $(wc -l < $OUT_RESPLE) poses -> $OUT_RESPLE"

# 5. Traj-LO (uses ROS1 bag directly, no ROS2 needed)
echo ""; echo "=== Traj-LO ==="
$WS/src/Traj-LO/build/trajlo_headless \
    $WS/src/Traj-LO/data/config_ntu_viral_eee03.yaml \
    2>&1 | tail -5

# ── ATE Evaluation ─────────────────────────────────────────────────────────
echo ""; echo "=== ATE RMSE (Sim3 alignment) ==="
for pair in \
    "FAST-LIO2:$RESULTS/fastlio2_ntu_eee03_poses.txt" \
    "GenZ-LIO:$RESULTS/genz_lio_ntu_eee03_poses.txt" \
    "LIMOncello:$RESULTS/limoncello_ntu_eee03_poses.txt" \
    "RESPLE:$RESULTS/resple_ntu_eee03_poses.txt" \
    "Traj-LO:$RESULTS/trajlo_ntu_eee03_poses.txt"; do
    NAME="${pair%%:*}"; FILE="${pair##*:}"
    CLEANED=/tmp/${NAME}_clean.txt
    grep -v "^#" $FILE | awk 'NF==8' > $CLEANED
    RMSE=$(evo_ape tum $GT $CLEANED --align --correct_scale 2>&1 | grep "rmse" | awk '{print $NF}')
    printf "  %-14s ATE RMSE = %s m\n" "$NAME" "$RMSE"
done
