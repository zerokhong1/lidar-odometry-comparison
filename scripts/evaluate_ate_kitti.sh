#!/bin/bash
# Compare ATE for all LiDAR odometry methods on KITTI Sequence 00
# Uses evo (pip install evo)
#
# Usage:
#   bash scripts/evaluate_ate_kitti.sh [--plot] [--save_plots]

set -e

REPO="$(cd "$(dirname "$0")/.." && pwd)"
KITTI_DIR="${KITTI_DIR:-$HOME/datasets/kitti_seq00}"
RESULTS="$HOME/results"
GT="$KITTI_DIR/kitti_seq00_gt_tum.txt"

PLOT=false
SAVE_PLOTS=false
for arg in "$@"; do
    case "$arg" in
        --plot)       PLOT=true ;;
        --save_plots) SAVE_PLOTS=true; PLOT=true ;;
    esac
done

echo "================================================================"
echo "  ATE Comparison — KITTI Odometry Sequence 00"
echo "  Ground truth: $GT"
echo "================================================================"
echo ""

# ── Check evo ────────────────────────────────────────────────────────────
if ! command -v evo_ape &>/dev/null; then
    echo "[ERROR] evo not installed. Run: pip install evo"
    exit 1
fi

# ── Check GT ─────────────────────────────────────────────────────────────
if [ ! -f "$GT" ]; then
    echo "[!] GT file not found: $GT"
    echo "    Converting now..."
    python3 "$REPO/scripts/convert_kitti_gt_to_tum.py" --kitti_dir "$KITTI_DIR"
    echo ""
fi

# ── Define methods ────────────────────────────────────────────────────────
declare -A METHOD_POSE
METHOD_POSE["FAST-LIO2"]="$RESULTS/fastlio2_kitti_poses.txt"
METHOD_POSE["GenZ-LIO"]="$RESULTS/genz_lio_kitti_poses.txt"
METHOD_POSE["LIMOncello"]="$RESULTS/limoncello_kitti_poses.txt"
METHOD_POSE["Traj-LO"]="$RESULTS/trajlo_kitti_poses.txt"

declare -A METHOD_TYPE   # LIO or LO
METHOD_TYPE["FAST-LIO2"]="LIO"
METHOD_TYPE["GenZ-LIO"]="LIO"
METHOD_TYPE["LIMOncello"]="LIO"
METHOD_TYPE["Traj-LO"]="LO"

# ── Run evo_ape for each method ──────────────────────────────────────────
declare -A ATE_RMSE
declare -A ATE_MEAN
declare -A ATE_MAX

FOUND=0
PLOT_DIR="$RESULTS/kitti_plots"
[ "$SAVE_PLOTS" = true ] && mkdir -p "$PLOT_DIR"

for METHOD in "FAST-LIO2" "GenZ-LIO" "LIMOncello" "Traj-LO"; do
    POSE="${METHOD_POSE[$METHOD]}"
    TYPE="${METHOD_TYPE[$METHOD]}"

    if [ ! -f "$POSE" ]; then
        echo "  [$METHOD] SKIPPED — pose file not found: $POSE"
        echo "           Run: bash scripts/run_${METHOD,,}_kitti.sh"
        continue
    fi

    echo "──────────────────────────────────────────────────────────────"
    echo "  $METHOD ($TYPE)"
    echo "  Pose file: $POSE ($(wc -l < "$POSE") poses)"

    EVO_ARGS="tum $GT $POSE --align --correct_scale"
    [ "$PLOT" = true ] && EVO_ARGS="$EVO_ARGS --plot"
    [ "$SAVE_PLOTS" = true ] && EVO_ARGS="$EVO_ARGS --save_plot $PLOT_DIR/${METHOD// /_}_ape.pdf"

    OUTPUT=$(evo_ape $EVO_ARGS 2>&1)
    echo "$OUTPUT"

    # Parse RMSE/mean/max from evo output
    RMSE=$(echo "$OUTPUT" | grep -i "rmse" | grep -oP '[\d.]+' | head -1)
    MEAN=$(echo "$OUTPUT" | grep -i "mean" | grep -oP '[\d.]+' | head -1)
    MAX_V=$(echo "$OUTPUT" | grep -i "max"  | grep -oP '[\d.]+' | head -1)

    ATE_RMSE[$METHOD]="${RMSE:-N/A}"
    ATE_MEAN[$METHOD]="${MEAN:-N/A}"
    ATE_MAX[$METHOD]="${MAX_V:-N/A}"
    FOUND=$((FOUND+1))
done

# ── Summary table ─────────────────────────────────────────────────────────
echo ""
echo "================================================================"
echo "  SUMMARY — ATE on KITTI Odometry Seq 00 (SE3 alignment)"
echo "  All values in metres (lower is better)"
echo "================================================================"
printf "  %-14s  %-6s  %-8s  %-8s  %-8s\n" "Method" "Type" "RMSE [m]" "Mean [m]" "Max [m]"
printf "  %-14s  %-6s  %-8s  %-8s  %-8s\n" "--------------" "------" "--------" "--------" "-------"
for METHOD in "FAST-LIO2" "GenZ-LIO" "LIMOncello" "Traj-LO"; do
    if [ -n "${ATE_RMSE[$METHOD]}" ]; then
        TYPE="${METHOD_TYPE[$METHOD]}"
        printf "  %-14s  %-6s  %-8s  %-8s  %-8s\n" \
            "$METHOD" "$TYPE" "${ATE_RMSE[$METHOD]}" "${ATE_MEAN[$METHOD]}" "${ATE_MAX[$METHOD]}"
    fi
done
echo "================================================================"
echo ""

if [ $FOUND -eq 0 ]; then
    echo "[!] No pose files found. Run the algorithms first:"
    echo "    bash scripts/run_fastlio2_kitti.sh    (+ bag play)"
    echo "    bash scripts/run_genz_lio_kitti.sh    (+ bag play)"
    echo "    bash scripts/run_limoncello_kitti.sh  (+ bag play)"
    echo "    bash scripts/run_trajlo_kitti.sh      (headless)"
fi

echo "Reference: KITTI seq 00 published ATE (ORB-SLAM2 ~0.7m, LeGO-LOAM ~1.1m)"
echo "For relative pose error (RPE): evo_rpe tum $GT <pose_file> --align"
echo ""

# ── Optional: multi-method comparison plot ────────────────────────────────
if [ $FOUND -ge 2 ] && [ "$PLOT" = true ]; then
    echo "Multi-method trajectory comparison..."
    TRAJ_ARGS=""
    LABELS=""
    for METHOD in "FAST-LIO2" "GenZ-LIO" "LIMOncello" "Traj-LO"; do
        POSE="${METHOD_POSE[$METHOD]}"
        if [ -f "$POSE" ]; then
            TRAJ_ARGS="$TRAJ_ARGS $POSE"
            LABELS="$LABELS $METHOD"
        fi
    done
    evo_traj tum $GT $TRAJ_ARGS --align --plot \
        --labels "GT $LABELS" 2>/dev/null || true
fi
