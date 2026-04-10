#!/bin/bash
# Run GenZ-LIO on R-Campus dataset
# GenZ-LIO: Generalizable LiDAR-Inertial Odometry Beyond Indoor-Outdoor Boundaries
# Three key contributions over FAST-LIO2:
#   1. Scale-aware adaptive voxelization (PD controller, Algorithm 1)
#   2. Hybrid-metric state update (point-to-plane + point-to-point L2)
#   3. Voxel-pruned correspondence search (Algorithm 2)

unset GDK_PIXBUF_MODULE_FILE GTK_EXE_PREFIX GSETTINGS_SCHEMA_DIR GTK_PATH

WS=/home/thailuu/LIMOncello_ws

source /opt/ros/jazzy/setup.bash
source $WS/install/setup.bash

echo "================================================================"
echo "  GenZ-LIO: Generalizable LiDAR-Inertial Odometry"
echo "  Dataset: R-Campus (Livox Avia, 1400m campus route)"
echo "  Features:"
echo "    - Scale-aware adaptive voxelization (PD control)"
echo "    - Hybrid-metric: point-to-plane + point-to-point"
echo "    - Voxel-pruned correspondence search"
echo "================================================================"
echo ""
echo "After RViz opens, open a NEW terminal and run:"
echo "  bash /home/thailuu/play_rcampus_bag.sh"
echo ""
echo "Watch console for GenZ-LIO diagnostics:"
echo "  [GenZ-LIO] m_bar=<scene_scale>m  N_des=<target_pts>  N_down=<actual_pts>  d_vox=<voxel_size>m"
echo ""

ros2 launch genz_lio mapping.launch.py \
    config_file:=r_campus.yaml \
    rviz:=true
