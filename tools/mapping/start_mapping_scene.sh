#!/bin/bash
# Start a G1 mapping session and return when the data required to map in RViz
# is live. Run this on the robot from /data/unitree/botbrain_ws.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: bash tools/mapping/start_mapping_scene.sh <scene> [--overwrite] [--live-loop-correction]

Examples:
  bash tools/mapping/start_mapping_scene.sh floor1
  bash tools/mapping/start_mapping_scene.sh long_corridor --overwrite
  bash tools/mapping/start_mapping_scene.sh loop_tf_trial --live-loop-correction

The old `default`/`corridor` positional argument is accepted as a legacy alias
but ignored.  Mapping always uses the one validated default configuration.
`--live-loop-correction` is an explicit experiment: it publishes the loop
pose-graph's `map -> camera_init` TF and is not part of the stable raw-map run.
EOF
}

scene="${1:-}"
shift || true
overwrite=""
live_loop_correction=false
if [[ ! "$scene" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
    usage >&2
    echo "ERROR: scene must contain only letters, digits, '_' or '-'" >&2
    exit 2
fi
for arg in "$@"; do
    case "$arg" in
        --overwrite)
            if [ -n "$overwrite" ]; then
                usage >&2
                echo "ERROR: --overwrite was supplied more than once" >&2
                exit 2
            fi
            overwrite=--overwrite
            ;;
        default|corridor)
            echo "WARNING: legacy mapping profile '$arg' is ignored; using the stable default configuration" >&2
            ;;
        --live-loop-correction)
            live_loop_correction=true
            ;;
        *)
            usage >&2
            echo "ERROR: unknown option '$arg'" >&2
            exit 2
            ;;
    esac
done

repo="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$repo"
maps="$repo/botbrain_ws/src/g1_pkg/maps"
pcd="$maps/${scene}_scans.pcd"
fast_lio_raw_pcd="$maps/${scene}_fast_lio_raw.pcd"
yaml="$maps/${scene}.yaml"
pgm="$maps/${scene}.pgm"
mkdir -p "$maps"

existing=()
for file in "$pcd" "$fast_lio_raw_pcd" "$yaml" "$pgm"; do
    [ -e "$file" ] && existing+=("$file")
done
if [ "${#existing[@]}" -gt 0 ] && [ "$overwrite" != --overwrite ]; then
    printf 'ERROR: scene %s already has map files:\n' "$scene" >&2
    printf '  %s\n' "${existing[@]}" >&2
    echo "Choose a new scene name, or pass --overwrite to create backups first." >&2
    exit 3
fi
if [ "${#existing[@]}" -gt 0 ]; then
    stamp="$(date +%Y%m%d_%H%M%S)"
    backup="$maps/backup_${scene}_$stamp"
    mkdir -p "$backup"
    cp -a "${existing[@]}" "$backup/"
    echo "Existing map files backed up to $backup"
fi

# Used by the save instructions to prove that the PCD belongs to this run.
touch "$maps/.${scene}_mapping_started"

echo "Starting mapping scene '$scene' with the stable raw FAST-LIO configuration"
docker compose stop localization navigation
# A previous standalone loop diagnostic has the same node name and must not
# remain on the graph while the mapping launch starts its own loop node.
docker compose stop loop_closure
docker compose up -d zenoh bringup state_machine
FAST_LIO_START_DELAY_SEC=0 \
FAST_LIO_MAPPING_MODE=true \
FAST_LIO_MAPPING_SAVE=true \
FAST_LIO_MAP_FILE="/botbrain_ws/src/g1_pkg/maps/${scene}_fast_lio_raw.pcd" \
FAST_LIO_MAPPING_PROFILE=default \
FAST_LIO_LOOP_POSE_GRAPH=true \
FAST_LIO_LOOP_LIVE_CORRECTION="$live_loop_correction" \
docker compose up -d --force-recreate fast_lio

echo "Waiting up to 120s for IMU, world/body point clouds, grid and TF"
deadline=$((SECONDS + 120))
while [ "$SECONDS" -lt "$deadline" ]; do
    logs="$(docker logs g1_robot_fast_lio 2>&1 || true)"
    if grep -Fq 'IMU Initial Done' <<<"$logs" && \
       grep -Fq 'FAST_LIO_TIMING' <<<"$logs" && \
       docker exec g1_robot_fast_lio bash -lc '
         set -e
         source /opt/ros/humble/setup.bash
         source /botbrain_ws/install/setup.bash
         timeout 3 ros2 topic echo /cloud_registered_1 --once --field header >/dev/null 2>&1
         timeout 3 ros2 topic echo /cloud_registered_body_1 --once --field header >/dev/null 2>&1
         timeout 3 ros2 topic echo /accumulated_grid --once --field header >/dev/null 2>&1
         timeout 3 ros2 run tf2_ros tf2_echo camera_init body 2>/dev/null | grep -q "Translation:"
       '; then
        echo "MAPPING READY: scene=$scene loop_live_tf=$live_loop_correction"
        echo "RViz topics live: dense raw FAST-LIO cloud/grid in camera_init"
        if [ "$live_loop_correction" = true ]; then
            echo "Online loop TF test enabled: map->camera_init may move after a confirmed loop."
        fi
        echo "Navigation PCD target: /botbrain_ws/src/g1_pkg/maps/${scene}_scans.pcd"
        echo "FAST-LIO raw source: /botbrain_ws/src/g1_pkg/maps/${scene}_fast_lio_raw.pcd"
        echo "Loop diagnostic only: /botbrain_ws/src/g1_pkg/maps/${scene}_loop_optimized.pcd"
        exit 0
    fi
    sleep 3
done

echo "ERROR: mapping did not become ready within 120s; do not move the robot" >&2
docker logs --tail 160 g1_robot_fast_lio >&2 || true
exit 1
