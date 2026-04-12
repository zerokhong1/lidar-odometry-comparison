#!/usr/bin/env python3
"""
Generate synthetic IMU from KITTI GT poses (differentiation).

KITTI raw OXTS is not publicly downloadable anymore (requires registration).
This script numerically differentiates GT camera poses → angular velocity +
linear acceleration, then injects them into the existing ROS2 bag.

Key correctness requirements:
  1. IMU timestamps must START before the first LiDAR frame (LEAD_TIME=2s)
     so LIO algorithms can propagate state to the first scan.
  2. CDR serialization done via rosbags (correct alignment/padding).

Usage:
  python3 scripts/generate_synthetic_imu.py --kitti_dir ~/datasets/kitti_seq00
"""

import argparse
import math
import os
import sqlite3
from pathlib import Path

import numpy as np

try:
    from rosbags.typesys import get_typestore, Stores
except ImportError:
    raise SystemExit("[ERROR] pip install rosbags")


# ── Rotation utilities ────────────────────────────────────────────────────

def rot_to_quat(R):
    trace = R[0,0] + R[1,1] + R[2,2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2,1]-R[1,2])*s; y = (R[0,2]-R[2,0])*s; z = (R[1,0]-R[0,1])*s
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2*math.sqrt(1+R[0,0]-R[1,1]-R[2,2])
        w=(R[2,1]-R[1,2])/s; x=0.25*s; y=(R[0,1]+R[1,0])/s; z=(R[0,2]+R[2,0])/s
    elif R[1,1] > R[2,2]:
        s = 2*math.sqrt(1+R[1,1]-R[0,0]-R[2,2])
        w=(R[0,2]-R[2,0])/s; x=(R[0,1]+R[1,0])/s; y=0.25*s; z=(R[1,2]+R[2,1])/s
    else:
        s = 2*math.sqrt(1+R[2,2]-R[0,0]-R[1,1])
        w=(R[1,0]-R[0,1])/s; x=(R[0,2]+R[2,0])/s; y=(R[1,2]+R[2,1])/s; z=0.25*s
    return np.array([x, y, z, w])


def quat_to_rot(q):
    x, y, z, w = q
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)  ],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)  ],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)],
    ])


def angular_velocity_body(R1, R2, dt):
    dR = R1.T @ R2
    cos_a = np.clip((np.trace(dR) - 1.0) / 2.0, -1.0, 1.0)
    angle = math.acos(cos_a)
    if abs(angle) < 1e-9:
        return np.zeros(3)
    axis = np.array([dR[2,1]-dR[1,2], dR[0,2]-dR[2,0], dR[1,0]-dR[0,1]])
    axis /= (2.0 * math.sin(angle))
    return axis * angle / dt


# ── rosbags IMU message builder ───────────────────────────────────────────

def make_imu_cdr(typestore, ts_ns, wx, wy, wz, ax, ay, az):
    Header = typestore.types['std_msgs/msg/Header']
    Stamp  = typestore.types['builtin_interfaces/msg/Time']
    Imu    = typestore.types['sensor_msgs/msg/Imu']
    Vec3   = typestore.types['geometry_msgs/msg/Vector3']
    Quat   = typestore.types['geometry_msgs/msg/Quaternion']

    sec  = int(ts_ns // 1_000_000_000)
    nsec = int(ts_ns  % 1_000_000_000)

    msg = Imu(
        header=Header(stamp=Stamp(sec=sec, nanosec=nsec), frame_id='imu_link'),
        orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=np.array([-1.0]+[0.0]*8, dtype=np.float64),
        angular_velocity=Vec3(x=wx, y=wy, z=wz),
        angular_velocity_covariance=np.full(9, 0.001, dtype=np.float64),
        linear_acceleration=Vec3(x=ax, y=ay, z=az),
        linear_acceleration_covariance=np.full(9, 0.01, dtype=np.float64),
    )
    return typestore.serialize_cdr(msg, 'sensor_msgs/msg/Imu')


# ── metadata writer ───────────────────────────────────────────────────────

QOS = (
    r"- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n"
    r"  deadline:\n    sec: 9223372036\n    nsec: 854775807\n"
    r"  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n"
    r"  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n"
    r"    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
)


def rewrite_metadata(bag_dir, lidar_count, imu_count, start_ns, dur_ns):
    total = lidar_count + imu_count
    meta = (
        f"rosbag2_bagfile_information:\n"
        f"  version: 5\n"
        f"  storage_identifier: sqlite3\n"
        f"  duration:\n    nanoseconds: {dur_ns}\n"
        f"  starting_time:\n    nanoseconds_since_epoch: {start_ns}\n"
        f"  message_count: {total}\n"
        f"  topics_with_message_count:\n"
        f"    - topic_metadata:\n"
        f"        name: /velodyne_points\n"
        f"        type: sensor_msgs/msg/PointCloud2\n"
        f"        serialization_format: cdr\n"
        f"        offered_qos_profiles: \"{QOS}\"\n"
        f"      message_count: {lidar_count}\n"
        f"    - topic_metadata:\n"
        f"        name: /imu/data\n"
        f"        type: sensor_msgs/msg/Imu\n"
        f"        serialization_format: cdr\n"
        f"        offered_qos_profiles: \"{QOS}\"\n"
        f"      message_count: {imu_count}\n"
        f"  compression_format: \"\"\n"
        f"  compression_mode: \"\"\n"
        f"  relative_file_paths:\n    - kitti_seq00_0.db3\n"
        f"  files:\n"
        f"    - path: kitti_seq00_0.db3\n"
        f"      starting_time:\n        nanoseconds_since_epoch: {start_ns}\n"
        f"      duration:\n        nanoseconds: {dur_ns}\n"
        f"      message_count: {total}\n"
    )
    (bag_dir / "metadata.yaml").write_text(meta)


# ══════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kitti_dir",
                        default=os.path.expanduser("~/datasets/kitti_seq00"))
    parser.add_argument("--imu_hz", type=float, default=100.0)
    args = parser.parse_args()

    kitti_dir  = Path(args.kitti_dir)
    poses_file = kitti_dir / "poses" / "00.txt"
    times_file = kitti_dir / "calib" / "00" / "times.txt"
    bag_dir    = kitti_dir / "kitti_seq00_ros2"
    db_path    = bag_dir / "kitti_seq00_0.db3"

    for f in [poses_file, times_file, db_path]:
        if not f.exists():
            raise SystemExit(f"[ERROR] Not found: {f}")

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # ── Load GT poses ─────────────────────────────────────────────────────
    BASE_NS   = 1_317_600_000_000_000_000
    times_rel = np.loadtxt(str(times_file))          # (N,) relative seconds
    lidar_ts  = (BASE_NS + times_rel * 1e9).astype(np.int64)

    lines  = poses_file.read_text().strip().split("\n")
    N      = len(lines)
    Rs     = np.zeros((N, 3, 3))
    ts_pos = np.zeros((N, 3))
    for i, line in enumerate(lines):
        v = list(map(float, line.split()))
        Rs[i]     = np.array(v).reshape(3, 4)[:3, :3]
        ts_pos[i] = np.array(v).reshape(3, 4)[:3,  3]
    print(f"Loaded {N} GT poses")

    # ── Load Tr (Velodyne→camera) from calib.txt ─────────────────────────
    # The KITTI odometry calib.txt contains Tr = R_velo_to_cam | t
    # We need R_cam_to_velo = R_velo_to_cam.T to express IMU in Velo/IMU frame
    calib_file = kitti_dir / "calib" / "00" / "calib.txt"
    R_cam_to_velo = np.eye(3)   # fallback: identity
    if calib_file.exists():
        for line in calib_file.read_text().splitlines():
            if line.startswith("Tr:"):
                vals = list(map(float, line.split()[1:]))
                R_velo_to_cam = np.array(vals).reshape(3, 4)[:3, :3]
                R_cam_to_velo = R_velo_to_cam.T
                print(f"Loaded R_cam_to_velo from {calib_file}")
                break
    else:
        print("[WARN] calib.txt not found — using identity R_cam_to_velo")

    # Gravity FIELD in KITTI camera frame (x=right, y=down, z=forward)
    # Points DOWNWARD = +y direction
    g_gravity_cam = np.array([0.0, 9.81, 0.0])

    # ── Build IMU timeline (start 2 s BEFORE first LiDAR) ────────────────
    LEAD_TIME = 2.0   # seconds
    dt_imu    = 1.0 / args.imu_hz
    t_lidar   = times_rel
    t_imu     = np.arange(t_lidar[0] - LEAD_TIME, t_lidar[-1], dt_imu)
    n_imu     = len(t_imu)

    # ── Estimate initial velocity / angular rate for backward extrapolation ──
    # KITTI seq 00 starts with the car already moving. We back-project the
    # trajectory using the average velocity and angular rate from the first
    # few LiDAR frames, avoiding an artificial acceleration spike at t=t0.
    N_EXTRAP = min(5, N - 1)
    dt_first = t_lidar[N_EXTRAP] - t_lidar[0]
    v0_cam   = (ts_pos[N_EXTRAP] - ts_pos[0]) / dt_first   # initial velocity in cam world

    # Initial angular velocity (axis-angle from R[0]→R[N_EXTRAP])
    quats = np.array([rot_to_quat(R) for R in Rs])
    omega0_cam = angular_velocity_body(Rs[0], Rs[N_EXTRAP], dt_first)

    # Extended time axis: lead frames use backward extrapolation
    t_full = t_imu.copy()  # full IMU timeline

    # For each IMU sample, compute position and rotation in cam world frame
    pos_full   = np.zeros((n_imu, 3))
    Rs_full    = np.zeros((n_imu, 3, 3))

    for i, t in enumerate(t_full):
        if t < t_lidar[0]:
            # Backward extrapolation: constant v and omega from t0
            dt_back = t - t_lidar[0]           # negative
            pos_full[i] = ts_pos[0] + v0_cam * dt_back
            # Backward rotation (undo omega0 for dt_back seconds)
            angle  = np.linalg.norm(omega0_cam) * (-dt_back)
            if angle < 1e-9:
                Rs_full[i] = Rs[0].copy()
            else:
                ax_v = omega0_cam / np.linalg.norm(omega0_cam)
                # Rodrigues rotation for backward step
                c, s = math.cos(-angle), math.sin(-angle)
                K = np.array([[0,-ax_v[2],ax_v[1]],
                               [ax_v[2],0,-ax_v[0]],
                               [-ax_v[1],ax_v[0],0]])
                dR = np.eye(3)*c + (1-c)*np.outer(ax_v,ax_v) + s*K
                Rs_full[i] = dR @ Rs[0]
        else:
            t_c = np.clip(t, t_lidar[0], t_lidar[-1])
            pos_full[i] = np.array([np.interp(t_c, t_lidar, ts_pos[:, d]) for d in range(3)])
            q = np.array([np.interp(t_c, t_lidar, quats[:, j]) for j in range(4)])
            q /= np.linalg.norm(q)
            Rs_full[i] = quat_to_rot(q)

    # Velocity at LiDAR rate (for smooth acceleration in body frame)
    vel_lidar   = np.gradient(ts_pos, t_lidar, axis=0)              # (N,3) in cam world
    # Interpolate vel to IMU rate (gives piecewise-linear vel → constant accel within each LiDAR period)
    vel_imu_cam = np.stack([
        np.interp(np.clip(t_full, t_lidar[0], t_lidar[-1]), t_lidar, vel_lidar[:, d])
        for d in range(3)
    ], axis=1)
    # For lead frames: use constant v0
    for i, t in enumerate(t_full):
        if t < t_lidar[0]:
            vel_imu_cam[i] = v0_cam
    # Acceleration from smoothed velocity
    accel_world = np.gradient(vel_imu_cam, dt_imu, axis=0)

    # ── Serialize IMU messages ────────────────────────────────────────────
    print(f"Generating {n_imu} IMU msgs ({LEAD_TIME}s lead + {t_lidar[-1]-t_lidar[0]:.1f}s sequence) ...")
    imu_msgs = []
    for i in range(n_imu):
        ts_ns = int(BASE_NS + t_imu[i] * 1e9)
        R     = Rs_full[i]

        # Angular velocity in camera body frame → Velodyne/IMU frame
        if i == 0:
            omega_cam = omega0_cam   # first sample: use initial rate
        else:
            omega_cam = angular_velocity_body(Rs_full[i-1], R, dt_imu)
        omega_imu = R_cam_to_velo @ omega_cam
        wx, wy, wz = omega_imu

        # Specific force: f = a_real - g_gravity_field, expressed in Velodyne body frame
        # Correct sign: g_gravity_cam = [0, +9.81, 0] points DOWN (+y in camera frame)
        a_body_cam = R.T @ (accel_world[i] - g_gravity_cam)
        a_body_imu = R_cam_to_velo @ a_body_cam
        ax, ay, az = a_body_imu

        cdr = make_imu_cdr(typestore, ts_ns, wx, wy, wz, ax, ay, az)
        imu_msgs.append((ts_ns, cdr))

    # ── Inject into ROS2 bag ──────────────────────────────────────────────
    con = sqlite3.connect(str(db_path))
    cur = con.cursor()

    cur.execute("SELECT id FROM topics WHERE name='/imu/data'")
    row = cur.fetchone()
    if row is None:
        cur.execute("INSERT INTO topics VALUES (2,'/imu/data',"
                    "'sensor_msgs/msg/Imu','cdr','')")
        con.commit()
        topic_id = 2
    else:
        topic_id = row[0]
        cur.execute("DELETE FROM messages WHERE topic_id=?", (topic_id,))
        con.commit()

    print(f"Writing {len(imu_msgs)} IMU messages to bag …")
    cur.executemany(
        "INSERT INTO messages(topic_id,timestamp,data) VALUES(?,?,?)",
        [(topic_id, ts, data) for ts, data in imu_msgs]
    )
    con.commit()

    cur.execute("SELECT COUNT(*) FROM messages WHERE topic_id=1")
    lidar_count = cur.fetchone()[0]
    cur.execute("SELECT MIN(timestamp) FROM messages")
    start_ns = cur.fetchone()[0]
    cur.execute("SELECT MAX(timestamp) FROM messages")
    end_ns = cur.fetchone()[0]
    total = cur.execute("SELECT COUNT(*) FROM messages").fetchone()[0]
    con.close()

    rewrite_metadata(bag_dir, lidar_count, len(imu_msgs), start_ns, end_ns - start_ns)

    # Verify
    lead_count = sum(1 for ts, _ in imu_msgs if ts < int(lidar_ts[0]))
    print(f"\n  IMU lead frames before first LiDAR: {lead_count} ({lead_count*dt_imu:.1f}s)")
    print(f"  Total bag messages: {total}")
    print(f"  Bag start: {start_ns/1e9:.3f}s  (first LiDAR: {lidar_ts[0]/1e9:.3f}s)")
    print(f"\n================================================================")
    print(f"  Done!  IMU starts {LEAD_TIME}s BEFORE first LiDAR scan.")
    print(f"  LIO algorithms should now initialize correctly.")
    print(f"\n  Play bag (NO --clock flag):")
    print(f"    ros2 bag play ~/datasets/kitti_seq00/kitti_seq00_ros2 -r 1.0")
    print(f"================================================================")


if __name__ == "__main__":
    main()
