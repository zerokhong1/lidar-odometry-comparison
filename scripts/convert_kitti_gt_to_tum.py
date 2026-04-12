#!/usr/bin/env python3
"""
Convert KITTI odometry ground truth (poses/00.txt) to TUM format.

KITTI GT:  12-value row per frame = [R | t] (3×4, camera-0 frame, row-major)
TUM format: "timestamp tx ty tz qx qy qz qw"  (one pose per line)

The KITTI GT is in the camera-0 coordinate frame.  For ATE comparison we
use SE(3) alignment (--align flag in evo_ape), so the reference frame does
NOT need to match the odometry output frame — evo handles the rigid alignment.

Usage:
  python3 scripts/convert_kitti_gt_to_tum.py [--kitti_dir DIR] [--seq SEQ]

Output:
  ~/datasets/kitti_seq00/kitti_seq00_gt_tum.txt
"""

import argparse
import math
import os
from pathlib import Path

import numpy as np


def rot_to_quat(R: np.ndarray):
    """3×3 rotation matrix → quaternion (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kitti_dir", default=os.path.expanduser("~/datasets/kitti_seq00"))
    parser.add_argument("--seq", default="00", help="Odometry sequence ID (e.g. 00)")
    args = parser.parse_args()

    kitti_dir  = Path(args.kitti_dir)
    poses_file = kitti_dir / "poses" / f"{args.seq}.txt"
    times_file = kitti_dir / "calib" / args.seq / "times.txt"
    out_file   = kitti_dir / f"kitti_seq{args.seq}_gt_tum.txt"

    if not poses_file.exists():
        print(f"[ERROR] GT not found: {poses_file}")
        print("Run scripts/download_kitti_seq00.sh first.")
        raise SystemExit(1)

    # Load timestamps (relative seconds from seq start)
    BASE_NS = 1_317_600_000_000_000_000  # 2011-10-03 00:00:00 UTC (approx)
    if times_file.exists():
        times_rel = np.loadtxt(str(times_file))
        timestamps = BASE_NS / 1e9 + times_rel   # absolute seconds
    else:
        print("[WARN] times.txt not found — using frame indices as timestamps")
        timestamps = None

    # Parse KITTI poses file (one 3×4 matrix per line, row-major)
    lines = poses_file.read_text().strip().split("\n")
    print(f"Loaded {len(lines)} GT poses from {poses_file}")

    if timestamps is None:
        timestamps = np.arange(len(lines), dtype=float) * 0.1  # 10 Hz

    out_lines = []
    for i, line in enumerate(lines):
        vals = list(map(float, line.split()))
        assert len(vals) == 12, f"Expected 12 values per pose, got {len(vals)}"
        T = np.array(vals).reshape(3, 4)
        R = T[:3, :3]
        t = T[:3,  3]
        qx, qy, qz, qw = rot_to_quat(R)
        ts = timestamps[i] if i < len(timestamps) else i * 0.1
        out_lines.append(
            f"{ts:.9f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
            f"{qx:.6f} {qy:.6f} {qz:.6f} {qw:.6f}"
        )

    out_file.write_text("\n".join(out_lines) + "\n")
    print(f"GT (TUM format) written → {out_file}")
    print(f"  Frames: {len(out_lines)}")
    print(f"  Time range: {timestamps[0]:.3f} – {timestamps[min(len(out_lines)-1, len(timestamps)-1)]:.3f} s")
    print("")
    print("Evaluate with evo_ape:")
    print(f"  evo_ape tum {out_file} ~/results/fastlio2_kitti_poses.txt --align --plot")
    print(f"  evo_ape tum {out_file} ~/results/genz_lio_kitti_poses.txt --align --plot")
    print(f"  evo_ape tum {out_file} ~/results/limoncello_kitti_poses.txt --align --plot")
    print(f"  evo_ape tum {out_file} ~/results/trajlo_kitti_poses.txt --align --plot")
    print("")
    print("Or run the full comparison script:")
    print("  bash scripts/evaluate_ate_kitti.sh")


if __name__ == "__main__":
    main()
