#!/usr/bin/env python3
"""
Convert KITTI seq 00 to ROS2 bag (.db3) + ROS1 bag (.bag)
using rosbags library for correct CDR serialization.

Topics:
  /velodyne_points  → sensor_msgs/msg/PointCloud2   (PointXYZIRT, 10 Hz)
  /imu/data         → sensor_msgs/msg/Imu           (100 Hz, from OXTS if available)

Usage:
  python3 scripts/convert_kitti_to_ros2bag.py [--kitti_dir DIR] [--max_frames N] [--no_ros1]
"""

import argparse
import math
import os
import sys
from pathlib import Path

import numpy as np

# ── rosbags imports ───────────────────────────────────────────────────────
try:
    from rosbags.rosbag2 import Writer as Ros2Writer
    from rosbags.rosbag1 import Writer as Ros1Writer
    from rosbags.typesys import get_typestore, Stores
except ImportError:
    print("[ERROR] rosbags not installed. Run: pip install rosbags")
    sys.exit(1)

# ── KITTI HDL-64E elevation angles ───────────────────────────────────────
_ELEV_MIN_DEG = -24.8
_ELEV_MAX_DEG =   2.0
_ELEV_RANGE   = _ELEV_MAX_DEG - _ELEV_MIN_DEG  # 26.8°


def elevation_to_ring(z, dist):
    elev_deg = np.degrees(np.arctan2(z, dist))
    ring = np.round((elev_deg - _ELEV_MIN_DEG) / _ELEV_RANGE * 63.0).astype(np.uint16)
    return np.clip(ring, 0, 63)


def azimuth_to_time(x, y, scan_period=0.1):
    az = (np.arctan2(y, x) + 2 * math.pi) % (2 * math.pi)
    return (az / (2 * math.pi) * scan_period).astype(np.float32)


def read_kitti_bin(path):
    return np.fromfile(path, dtype=np.float32).reshape(-1, 4)


def parse_oxts_line(line):
    v = list(map(float, line.strip().split()))
    return dict(ax=v[11], ay=v[12], az=v[13], wx=v[17], wy=v[18], wz=v[19])


# ══════════════════════════════════════════════════════════════════════════
#  Build ROS2 message objects
# ══════════════════════════════════════════════════════════════════════════

def make_pc2_msg(ts, pts, ring, toff, ts_ns):
    """Build a sensor_msgs/msg/PointCloud2 for one KITTI scan."""
    Header     = ts.types['std_msgs/msg/Header']
    Stamp      = ts.types['builtin_interfaces/msg/Time']
    PC2        = ts.types['sensor_msgs/msg/PointCloud2']
    PF         = ts.types['sensor_msgs/msg/PointField']

    sec  = int(ts_ns // 1_000_000_000)
    nsec = int(ts_ns  % 1_000_000_000)
    N    = pts.shape[0]

    # Fields: x(f32) y(f32) z(f32) intensity(f32) ring(u16) _pad(u16) time(f32)
    # point_step = 4+4+4+4+2+2+4 = 24 bytes
    POINT_STEP = 24
    fields = [
        PF(name='x',         offset=0,  datatype=7, count=1),  # FLOAT32=7
        PF(name='y',         offset=4,  datatype=7, count=1),
        PF(name='z',         offset=8,  datatype=7, count=1),
        PF(name='intensity', offset=12, datatype=7, count=1),
        PF(name='ring',      offset=16, datatype=4, count=1),  # UINT16=4
        PF(name='time',      offset=20, datatype=7, count=1),
    ]

    # Pack point data
    pdata = np.zeros(N, dtype=[
        ('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
        ('ring', '<u2'), ('_pad', '<u2'), ('time', '<f4'),
    ])
    pdata['x']         = pts[:, 0]
    pdata['y']         = pts[:, 1]
    pdata['z']         = pts[:, 2]
    pdata['intensity'] = pts[:, 3]
    pdata['ring']      = ring
    pdata['time']      = toff

    return PC2(
        header=Header(stamp=Stamp(sec=sec, nanosec=nsec), frame_id='velodyne'),
        height=1,
        width=N,
        fields=fields,
        is_bigendian=False,
        point_step=POINT_STEP,
        row_step=POINT_STEP * N,
        data=pdata.view(np.uint8),
        is_dense=True,
    )


def make_imu_msg(ts, oxts, ts_ns):
    """Build a sensor_msgs/msg/Imu from OXTS data."""
    Header = ts.types['std_msgs/msg/Header']
    Stamp  = ts.types['builtin_interfaces/msg/Time']
    Imu    = ts.types['sensor_msgs/msg/Imu']
    Vec3   = ts.types['geometry_msgs/msg/Vector3']
    Quat   = ts.types['geometry_msgs/msg/Quaternion']

    sec  = int(ts_ns // 1_000_000_000)
    nsec = int(ts_ns  % 1_000_000_000)

    return Imu(
        header=Header(stamp=Stamp(sec=sec, nanosec=nsec), frame_id='imu_link'),
        orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0),
        orientation_covariance=np.array([-1.0] + [0.0]*8, dtype=np.float64),
        angular_velocity=Vec3(x=oxts['wx'], y=oxts['wy'], z=oxts['wz']),
        angular_velocity_covariance=np.full(9, 0.001, dtype=np.float64),
        linear_acceleration=Vec3(x=oxts['ax'], y=oxts['ay'], z=oxts['az']),
        linear_acceleration_covariance=np.full(9, 0.01, dtype=np.float64),
    )


# ══════════════════════════════════════════════════════════════════════════
#  QOS string for metadata
# ══════════════════════════════════════════════════════════════════════════
# Escaped newlines — must appear as literal \n in the YAML file (not real newlines)
QOS_STR = (
    r"- history: 3\n  depth: 0\n  reliability: 1\n  durability: 2\n"
    r"  deadline:\n    sec: 9223372036\n    nsec: 854775807\n"
    r"  lifespan:\n    sec: 9223372036\n    nsec: 854775807\n"
    r"  liveliness: 1\n  liveliness_lease_duration:\n    sec: 9223372036\n"
    r"    nsec: 854775807\n  avoid_ros_namespace_conventions: false"
)


def write_metadata(out_dir, lidar_count, imu_count, start_ns, dur_ns):
    total = lidar_count + imu_count
    imu_entry = ""
    if imu_count > 0:
        imu_entry = (
            f"    - topic_metadata:\n"
            f"        name: /imu/data\n"
            f"        type: sensor_msgs/msg/Imu\n"
            f"        serialization_format: cdr\n"
            f"        offered_qos_profiles: \"{QOS_STR}\"\n"
            f"      message_count: {imu_count}\n"
        )
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
        f"        offered_qos_profiles: \"{QOS_STR}\"\n"
        f"      message_count: {lidar_count}\n"
        f"{imu_entry}"
        f"  compression_format: \"\"\n"
        f"  compression_mode: \"\"\n"
        f"  relative_file_paths:\n    - kitti_seq00_0.db3\n"
        f"  files:\n"
        f"    - path: kitti_seq00_0.db3\n"
        f"      starting_time:\n        nanoseconds_since_epoch: {start_ns}\n"
        f"      duration:\n        nanoseconds: {dur_ns}\n"
        f"      message_count: {total}\n"
    )
    (out_dir / "metadata.yaml").write_text(meta)


# ══════════════════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kitti_dir", default=os.path.expanduser("~/datasets/kitti_seq00"))
    parser.add_argument("--max_frames", type=int, default=0)
    parser.add_argument("--no_ros1", action="store_true")
    args = parser.parse_args()

    kitti_dir  = Path(args.kitti_dir)
    velo_dir   = kitti_dir / "velodyne" / "00"
    times_file = kitti_dir / "calib" / "00" / "times.txt"
    oxts_dir   = kitti_dir / "raw" / "2011_10_03" / "2011_10_03_drive_0027_sync" / "oxts" / "data"

    for d in [velo_dir, times_file]:
        if not d.exists():
            print(f"[ERROR] Not found: {d}")
            sys.exit(1)

    has_oxts = oxts_dir.exists()
    if not has_oxts:
        print("[WARN] OXTS/IMU not found — LiDAR only. Run generate_synthetic_imu.py after.")

    BASE_NS  = 1_317_600_000_000_000_000
    times_rel = np.loadtxt(str(times_file))
    lidar_ts  = (BASE_NS + times_rel * 1e9).astype(np.int64)

    bin_files = sorted(velo_dir.glob("*.bin"))
    if args.max_frames > 0:
        bin_files = bin_files[:args.max_frames]
    n_frames = len(bin_files)
    print(f"Converting {n_frames} LiDAR frames …")

    typestore = get_typestore(Stores.ROS2_HUMBLE)

    # ── Build all messages in memory ──────────────────────────────────────
    lidar_msgs = []   # (ts_ns, cdr_bytes)
    for i, bf in enumerate(bin_files):
        if i % 200 == 0:
            print(f"  frame {i}/{n_frames} …")
        pts  = read_kitti_bin(str(bf))
        xy   = np.sqrt(pts[:, 0]**2 + pts[:, 1]**2)
        ring = elevation_to_ring(pts[:, 2], xy)
        toff = azimuth_to_time(pts[:, 0], pts[:, 1])
        ts_ns = int(lidar_ts[i])
        msg  = make_pc2_msg(typestore, pts, ring, toff, ts_ns)
        cdr  = typestore.serialize_cdr(msg, 'sensor_msgs/msg/PointCloud2')
        lidar_msgs.append((ts_ns, cdr))

    imu_msgs = []
    if has_oxts:
        oxts_files = sorted(oxts_dir.glob("*.txt"))
        print(f"Converting {len(oxts_files)} OXTS/IMU frames …")
        for j, of in enumerate(oxts_files):
            line = of.read_text().strip()
            if not line:
                continue
            oxts  = parse_oxts_line(line)
            ts_ns = int(lidar_ts[0] + j * 10_000_000)
            msg   = make_imu_msg(typestore, oxts, ts_ns)
            cdr   = typestore.serialize_cdr(msg, 'sensor_msgs/msg/Imu')
            imu_msgs.append((ts_ns, cdr))

    # ── Write ROS2 bag (sqlite3) ──────────────────────────────────────────
    import sqlite3
    out_ros2 = kitti_dir / "kitti_seq00_ros2"
    out_ros2.mkdir(parents=True, exist_ok=True)
    db_path = out_ros2 / "kitti_seq00_0.db3"
    if db_path.exists():
        db_path.unlink()

    con = sqlite3.connect(str(db_path))
    cur = con.cursor()
    cur.executescript("""
        CREATE TABLE topics (
            id INTEGER PRIMARY KEY,
            name TEXT NOT NULL,
            type TEXT NOT NULL,
            serialization_format TEXT NOT NULL,
            offered_qos_profiles TEXT NOT NULL
        );
        CREATE TABLE messages (
            id INTEGER PRIMARY KEY,
            topic_id INTEGER NOT NULL,
            timestamp INTEGER NOT NULL,
            data BLOB NOT NULL
        );
        CREATE INDEX messages_timestamp ON messages (timestamp ASC);
    """)
    cur.execute("INSERT INTO topics VALUES (1,'/velodyne_points',"
                "'sensor_msgs/msg/PointCloud2','cdr','')")
    if imu_msgs:
        cur.execute("INSERT INTO topics VALUES (2,'/imu/data',"
                    "'sensor_msgs/msg/Imu','cdr','')")
    con.commit()

    print(f"  Writing {len(lidar_msgs)} LiDAR msgs …")
    cur.executemany("INSERT INTO messages(topic_id,timestamp,data) VALUES(1,?,?)",
                    lidar_msgs)
    if imu_msgs:
        print(f"  Writing {len(imu_msgs)} IMU msgs …")
        cur.executemany("INSERT INTO messages(topic_id,timestamp,data) VALUES(2,?,?)",
                        imu_msgs)
    con.commit()
    con.close()

    start_ns = lidar_msgs[0][0]
    end_ns   = lidar_msgs[-1][0]
    write_metadata(out_ros2, len(lidar_msgs), len(imu_msgs), start_ns, end_ns - start_ns)
    print(f"  ROS2 bag → {out_ros2}")

    # ── Write ROS1 bag ────────────────────────────────────────────────────
    if not args.no_ros1:
        out_ros1 = kitti_dir / "kitti_seq00_ros1.bag"
        if out_ros1.exists():
            out_ros1.unlink()
        ts1 = get_typestore(Stores.ROS1_NOETIC)
        print(f"  Writing ROS1 bag → {out_ros1}")
        with Ros1Writer(str(out_ros1)) as writer:
            vc = writer.add_connection('/velodyne_points',
                                       'sensor_msgs/msg/PointCloud2', typestore=ts1)
            ic = writer.add_connection('/imu/data',
                                       'sensor_msgs/msg/Imu', typestore=ts1) if imu_msgs else None
            for ts_ns, data in lidar_msgs:
                writer.write(vc, ts_ns, data)
            if ic:
                for ts_ns, data in imu_msgs:
                    writer.write(ic, ts_ns, data)
        print(f"  ROS1 bag done.")

    print("\n================================================================")
    print(f"  Done!  LiDAR: {len(lidar_msgs)}  IMU: {len(imu_msgs)}")
    if not imu_msgs:
        print("  Run generate_synthetic_imu.py to inject IMU for LIO methods.")
    print("================================================================")


if __name__ == "__main__":
    main()
