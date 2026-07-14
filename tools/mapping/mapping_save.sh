#!/bin/bash
# mapping_save.sh — dump 3D PCD + 2D PGM directly to maps/.
#
# Triggered either by mapping_launch.sh's SIGINT trap (one-shot save on
# Ctrl+C), or by a human in a separate window for an interim dump.
#
# Two save modes:
#   A) /map_save service (RECOMMENDED — requires pcd_save_en: false in mid360.yaml)
#      → fast_lio writes PCD on demand, process keeps running
#   B) SIGINT to fastlio_mapping (FALLBACK — when pcd_save_en: true)
#      → fast_lio auto-saves PCD on shutdown, then restarts
#
# map_saver_cli writes accumulated_grid.{pgm,yaml} on top of any existing
# files (previous maps are backed up as *.bak).
#
# Exit codes:
#   0  — PCD ok + 2D ok
#   1  — PCD save failed (2D not attempted)
#   2  — PCD ok but 2D save failed
#
# Pre-conditions:
#   - fast_lio running (mapping mode) + grid_accumulator publishing /accumulated_grid
#   - G1 driven around enough to cover the workspace
#
# Usage (inside fast_lio container):
#   docker exec -it g1_robot_fast_lio bash
#   source install/setup.bash
#   bash /botbrain_ws/tools/mapping/mapping_save.sh [scene_name]
#
#   scene_name: 场景名（如 floor1, office_A），决定输出文件名:
#     <scene_name>_scans.pcd / <scene_name>.pgm / <scene_name>.yaml
#   不传则默认使用 accumulated_grid / scans

set -euo pipefail

source /opt/ros/humble/setup.bash
source /botbrain_ws/install/setup.bash

SCENE="${1:-}"
MAPS="/botbrain_ws/src/g1_pkg/maps"

if [ -n "$SCENE" ]; then
    PCD="$MAPS/${SCENE}_scans.pcd"
    PGM="$MAPS/${SCENE}.pgm"
    YML="$MAPS/${SCENE}.yaml"
else
    PCD="$MAPS/scans.pcd"
    PGM="$MAPS/accumulated_grid.pgm"
    YML="$MAPS/accumulated_grid.yaml"
fi

MIN_PCD_BYTES=$((1 * 1024 * 1024))   # 1 MB floor

mkdir -p "$MAPS"
T0=$(date +%s)

# ── backup existing files ──────────────────────────
for f in "$PCD" "$PGM" "$YML"; do
    [ -f "$f" ] && cp "$f" "$f.bak" 2>/dev/null || true
done

# ── [1/3] dump 3D PCD ──────────────────────────────
echo "[1/3] dumping 3D PCD via fast_lio /map_save ..."

# Try /map_save service first (pcd_save_en: false mode)
SAVE_OK=false
if ros2 service call /map_save std_srvs/srv/Trigger 2>/dev/null; then
    sleep 2
    if [ -f "$PCD" ]; then
        SAVE_OK=true
    fi
fi

# Fallback: SIGINT to fastlio_mapping (pcd_save_en: true mode)
if [ "$SAVE_OK" = false ]; then
    echo "  /map_save unavailable, trying SIGINT to fastlio_mapping ..."
    PID=$(pgrep -f fastlio_mapping 2>/dev/null || true)
    if [ -n "$PID" ]; then
        kill -SIGINT "$PID" 2>/dev/null || true
        sleep 5
        if [ -f "$PCD" ]; then
            SAVE_OK=true
        else
            echo "  WARNING: PCD not found at $PCD after SIGINT"
            echo "  Check mid360.yaml pcd_save_en and map_file_path"
        fi
    else
        echo "  ERROR: fastlio_mapping process not found"
    fi
fi

# Validate PCD
if [ "$SAVE_OK" = false ] || [ ! -f "$PCD" ]; then
    echo "  FAIL: $PCD does not exist after save attempt." >&2
    exit 1
fi

PCD_SIZE=$(stat -c %s "$PCD" 2>/dev/null || echo 0)
PCD_MTIME=$(stat -c %Y "$PCD" 2>/dev/null || echo 0)
if [ "$PCD_SIZE" -lt "$MIN_PCD_BYTES" ]; then
    echo "  FAIL: $PCD size $PCD_SIZE < $MIN_PCD_BYTES (looks truncated)." >&2
    exit 1
fi
if [ "$PCD_MTIME" -lt "$T0" ]; then
    echo "  FAIL: $PCD mtime older than save start $T0 (stale file — map_file_path mismatch?)." >&2
    exit 1
fi
echo "  ok: $PCD ($PCD_SIZE bytes)"

# ── [2/3] dump 2D PGM ──────────────────────────────
echo "[2/3] dumping 2D PGM via map_saver_cli ..."

PREFIX="${PGM%.pgm}"  # strip .pgm suffix for map_saver_cli -f
if ! ros2 run nav2_map_server map_saver_cli \
        -t /accumulated_grid \
        --free 0.196 \
        --occ 0.65 \
        --mode trinary \
        -f "$PREFIX" 2>/tmp/map_saver_err.log; then
    echo "  FAIL: map_saver_cli returned non-zero." >&2
    cat /tmp/map_saver_err.log >&2 2>/dev/null || true
    exit 2
fi
if [ ! -f "$PGM" ] || [ ! -f "$YML" ]; then
    echo "  FAIL: map_saver_cli did not produce $PGM and $YML." >&2
    exit 2
fi
PGM_MTIME=$(stat -c %Y "$PGM" 2>/dev/null || echo 0)
if [ "$PGM_MTIME" -lt "$T0" ]; then
    echo "  FAIL: $PGM mtime older than save start $T0 (stale)." >&2
    exit 2
fi
echo "  ok: $PGM + $YML"

# ── [3/3] fixing yaml ──────────────────────────────
echo "[3/3] fixing yaml ..."
sed -i \
    -e "s|^image:.*|image: $PGM|" \
    -e "s|^free_thresh:.*|free_thresh: 0.196|" \
    -e "s|^occupied_thresh:.*|occupied_thresh: 0.65|" \
    -e "s|^mode:.*|mode: trinary|" \
    "$YML" 2>/dev/null || true
grep -q "^mode:"            "$YML" || echo "mode: trinary"        >> "$YML"
grep -q "^occupied_thresh:" "$YML" || echo "occupied_thresh: 0.65" >> "$YML"
grep -q "^free_thresh:"     "$YML" || echo "free_thresh: 0.196"   >> "$YML"

echo
echo "DONE. Files in $MAPS/ :"
ls -la "$PCD" "$PGM" "$YML" 2>&1 | sed 's|^| |'
echo ""
echo "Next steps:"
RAW_PCD="${PCD%.pcd}_raw.pcd"
echo "  1. PCD floor correction: follow run.md Step 5; back up to $RAW_PCD and shift $PCD exactly once."
echo "     If the raw backup already exists, stop and verify whether this PCD was already corrected."
echo "  2. 验证ICP fitness: docker compose logs localization | grep fitness"
echo "  3. 如需修图: scp到workstation → tools/host_side/map_edit/ 编辑 → scp回传"
exit 0
