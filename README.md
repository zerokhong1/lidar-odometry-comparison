# LiDAR Odometry Comparison

Comparison of 4 LiDAR odometry systems on the **R-Campus dataset** (Livox Avia, 1400m campus route).

## Systems

| System | Paper | Type | Script |
|--------|-------|------|--------|
| [RESPLE](https://github.com/ASIG-X/RESPLE) | RA-L 2025 | LIO | `scripts/run_resple_rcampus.sh` |
| **FAST-LIO2** (modified) | T-RO 2022 | LIO | `scripts/run_fastlio2_rcampus.sh` |
| [LIMOncello](https://github.com/fetty31/LIMOncello) | arXiv 2024 | LIO | `scripts/run_limoncello_rcampus.sh` |
| **Traj-LO** (modified) | RA-L 2024 | LO | `scripts/run_trajlo_rcampus.sh` |

## Repo Structure

```
lidar-odometry-comparison/
├── scripts/          # Run scripts for all systems
├── configs/          # Config files per system
├── fast_lio2/        # FAST-LIO2 (modified: livox_interfaces + C++17 + R-Campus config)
└── traj_lo/          # Traj-LO (modified: headless mode + R-Campus config)
```

## Modifications

### FAST-LIO2
- Replaced `livox_ros_driver2` → `livox_interfaces` (same message structure, matches bag format)
- Fixed C++14 → C++17 for ROS2 Jazzy compatibility
- Removed unused `pcl_ros` dependency
- Added `config/r_campus.yaml` for Livox Avia

### Traj-LO
- Added `-DTRAJLO_HEADLESS=ON` cmake option (builds without GLFW/OpenGL)
- Added `run_trajlo_headless.cpp`: runs odometry headless, saves poses to file
- Added `data/config_r_campus.yaml` for Livox Avia R-Campus bag (ROS1 format)

## Setup

**Requirements**: ROS2 Jazzy, Livox interfaces

```bash
git clone https://github.com/zerokhong1/lidar-odometry-comparison.git
cd lidar-odometry-comparison

# Build FAST-LIO2 (copy fast_lio2/ into your ROS2 workspace)
cp -r fast_lio2 ~/your_ws/src/fast_lio
cd ~/your_ws && colcon build --packages-select fast_lio

# Build Traj-LO (standalone CMake)
git clone --recursive https://github.com/kevin2431/Traj-LO.git
cp traj_lo/CMakeLists.txt Traj-LO/
cp traj_lo/thirdparty/CMakeLists.txt Traj-LO/thirdparty/
cp traj_lo/run_trajlo_headless.cpp Traj-LO/
cp traj_lo/data/config_r_campus.yaml Traj-LO/data/
cd Traj-LO && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DTRAJLO_HEADLESS=ON && make -j$(nproc)
```

## Running

### ROS2 systems (RESPLE / FAST-LIO2 / LIMOncello)

```bash
# Terminal 1
bash scripts/run_resple_rcampus.sh   # or fastlio2 / limoncello

# Terminal 2 (after RViz opens)
bash scripts/play_rcampus_bag.sh
```

### Traj-LO (headless, LiDAR-only)

```bash
# Convert ROS2 bag -> ROS1 first
pip install rosbags
rosbags-convert --src /path/to/R_Campus --dst R_Campus_ros1.bag

bash scripts/run_trajlo_rcampus.sh
# Poses saved to ~/results/trajlo_r_campus_poses.txt
```

## Expected Results (R-Campus, end-to-end error)

| System | Error |
|--------|-------|
| RESPLE-LIO | ~0.27 m |
| RESPLE-LO  | ~0.28 m |
| FAST-LIO2  | ~2.70 m |
| Traj-LO    | fail (~80 m) |
