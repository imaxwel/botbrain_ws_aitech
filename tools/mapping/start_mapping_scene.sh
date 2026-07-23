#!/bin/bash
# Start a G1 mapping session and return when the data required to map in RViz
# is live. Run this on the robot from /data/unitree/botbrain_ws.
set -euo pipefail

usage() {
    cat <<'EOF'
Usage: bash tools/mapping/start_mapping_scene.sh <scene> [default|corridor] [--overwrite]

Examples:
  bash tools/mapping/start_mapping_scene.sh floor1 default
  bash tools/mapping/start_mapping_scene.sh long_corridor corridor
  bash tools/mapping/start_mapping_scene.sh floor1 corridor --overwrite
EOF
}

scene="${1:-}"
profile="${2:-default}"
overwrite="${3:-}"
if [[ ! "$scene" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
    usage >&2
    echo "ERROR: scene must contain only letters, digits, '_' or '-'" >&2
    exit 2
fi
if [ "$profile" != default ] && [ "$profile" != corridor ]; then
    usage >&2
    echo "ERROR: profile must be default or corridor" >&2
    exit 2
fi
if [ -n "$overwrite" ] && [ "$overwrite" != --overwrite ]; then
    usage >&2
    echo "ERROR: unknown option '$overwrite'" >&2
    exit 2
fi

repo="$(cd "$(dirname "$0")/../.." && pwd)"
cd "$repo"
maps="$repo/botbrain_ws/src/g1_pkg/maps"
pcd="$maps/${scene}_scans.pcd"
yaml="$maps/${scene}.yaml"
pgm="$maps/${scene}.pgm"
mkdir -p "$maps"

existing=()
for file in "$pcd" "$yaml" "$pgm"; do
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

echo "Starting mapping scene '$scene' with profile '$profile'"
docker compose stop localization navigation
docker compose up -d zenoh bringup state_machine
FAST_LIO_START_DELAY_SEC=0 \
FAST_LIO_MAPPING_MODE=true \
FAST_LIO_MAPPING_SAVE=true \
FAST_LIO_MAP_FILE="/botbrain_ws/src/g1_pkg/maps/${scene}_scans.pcd" \
FAST_LIO_MAPPING_PROFILE="$profile" \
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
        echo "MAPPING READY: scene=$scene profile=$profile"
        echo "RViz topics live: /cloud_registered_1 /cloud_registered_body_1 /accumulated_grid /tf"
        echo "PCD target: /botbrain_ws/src/g1_pkg/maps/${scene}_scans.pcd"
        exit 0
    fi
    sleep 3
done

echo "ERROR: mapping did not become ready within 120s; do not move the robot" >&2
docker logs --tail 160 g1_robot_fast_lio >&2 || true
exit 1
