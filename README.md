# LiDAR Odometry Comparison

Comparison of 4 LiDAR odometry systems on the **R-Campus dataset** (Livox Avia, 1400m campus route).

## Systems

| System | Paper | Type | Script |
|--------|-------|------|--------|
| [RESPLE](https://github.com/ASIG-X/RESPLE) | RA-L 2025 | LIO | `run_resple_rcampus.sh` |
| [FAST-LIO2](https://github.com/zerokhong1/FAST_LIO) | T-RO 2022 | LIO | `run_fastlio2_rcampus.sh` |
| [LIMOncello](https://github.com/fetty31/LIMOncello) | arXiv 2024 | LIO | `run_limoncello_rcampus.sh` |
| [Traj-LO](https://github.com/zerokhong1/Traj-LO) | RA-L 2024 | LO | `run_trajlo_rcampus.sh` |

## Setup

**Requirements**: ROS2 Jazzy, Livox SDK

```bash
# Clone workspace
git clone <this-repo>

# Build all systems in LIMOncello_ws
cd ~/LIMOncello_ws
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Running

### ROS2 systems (RESPLE / FAST-LIO2 / LIMOncello)

```bash
# Terminal 1 - launch system + RViz
bash run_resple_rcampus.sh        # or fast_lio2 / limoncello

# Terminal 2 - play bag (after RViz opens)
bash play_rcampus_bag.sh
```

### Traj-LO (headless, LiDAR-only)

```bash
bash run_trajlo_rcampus.sh
# Poses saved to ~/results/trajlo_r_campus_poses.txt
```

## Expected Results (R-Campus, end-to-end error)

| System | Error |
|--------|-------|
| RESPLE-LIO | ~0.27 m |
| RESPLE-LO  | ~0.28 m |
| FAST-LIO2  | ~2.70 m |
| Traj-LO    | fail   |

## Dataset

R-Campus: bipedal wheeled robot (DIABLO), Livox Avia LiDAR + IMU, ~1400m campus route.

Topics: `/livox/lidar` (`livox_interfaces/CustomMsg`), `/livox/imu` (`sensor_msgs/Imu`)

## Notes

- **FAST-LIO2** modified to use `livox_interfaces` instead of `livox_ros_driver2`
- **Traj-LO** modified with headless mode (no GLFW/OpenGL required)
- Traj-LO uses ROS1 bag format — convert with `rosbags`: `rosbags-convert --src <ros2_bag> --dst <output.bag>`
