#!/bin/bash
# Run Traj-LO (headless) on R-Campus dataset (ROS1 bag)
# Traj-LO is LiDAR-only, continuous-time piecewise-linear trajectory

TRAJLO_BIN=/home/thailuu/LIMOncello_ws/src/Traj-LO/build/trajlo_headless
CONFIG=/home/thailuu/LIMOncello_ws/src/Traj-LO/data/config_r_campus.yaml
RESULTS=/home/thailuu/results/trajlo_r_campus_poses.txt

echo "================================================================"
echo "  Traj-LO - LiDAR-Only Continuous-Time Odometry"
echo "  Dataset: R-Campus (Livox Avia, 1400m campus route)"
echo "  Mode: Headless (no GUI) - poses saved to:"
echo "    $RESULTS"
echo "================================================================"
echo ""

mkdir -p /home/thailuu/results
$TRAJLO_BIN $CONFIG

echo ""
echo "Done! Evaluate with evo:"
echo "  evo_ape tum <ground_truth.txt> $RESULTS --align --correct_scale"
