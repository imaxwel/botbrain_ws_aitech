#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 <scene> [--restart-fast-lio]" >&2
    echo "Same floor:   $0 aitech" >&2
    echo "Cross floor:  $0 floor4 --restart-fast-lio" >&2
}

if [ "$#" -lt 1 ] || [ "$#" -gt 2 ]; then
    usage
    exit 2
fi

scene=$1
restart_fast_lio=false
if [ "$#" -eq 2 ]; then
    if [ "$2" != "--restart-fast-lio" ]; then
        usage
        exit 2
    fi
    restart_fast_lio=true
fi
if [[ ! "$scene" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
    echo "ERROR: invalid scene '$scene'" >&2
    exit 2
fi

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
cd "$project_root"

maps=botbrain_ws/src/g1_pkg/maps
pcd="$maps/${scene}_scans.pcd"
yaml="$maps/${scene}.yaml"
pgm="$maps/${scene}.pgm"

for path in "$pcd" "$yaml" "$pgm"; do
    if [ ! -s "$path" ]; then
        echo "ERROR: missing or empty map asset: $path" >&2
        exit 1
    fi
done

image=$(sed -n 's/^[[:space:]]*image:[[:space:]]*//p' "$yaml" | head -n 1)
image=${image#\"}
image=${image%\"}
image=${image#\'}
image=${image%\'}
case "$image" in
    /*) image_path=$image ;;
    *) image_path="$(dirname "$yaml")/$image" ;;
esac
if [ "$(realpath -e "$image_path")" != "$(realpath -e "$pgm")" ]; then
    echo "ERROR: $yaml points to '$image', expected ${scene}.pgm" >&2
    exit 1
fi

echo "Selecting map scene '$scene'"
if [ "$restart_fast_lio" = true ]; then
    for config in \
        botbrain_ws/src/fast_lio/config/mid360.yaml \
        botbrain_ws/install/fast_lio/share/fast_lio/config/mid360.yaml; do
        if [ ! -f "$config" ]; then
            echo "ERROR: FAST-LIO config is missing: $config" >&2
            exit 1
        fi
        if ! grep -Eq \
                '^[[:space:]]*pcd_save_en:[[:space:]]*false([[:space:]]|$)' \
                "$config"; then
            echo "ERROR: $config must set pcd_save_en: false before navigation" >&2
            exit 1
        fi
    done

    docker compose stop navigation localization fast_lio
    docker compose rm -f navigation localization fast_lio
    docker compose up -d --force-recreate fast_lio

    echo "Waiting for FAST-LIO IMU initialization"
    for _ in $(seq 1 150); do
        if docker compose logs --no-color --tail 200 fast_lio 2>&1 |
                grep -Fq "IMU Initial Done"; then
            echo "FAST-LIO is initialized"
            break
        fi
        if ! docker inspect -f '{{.State.Running}}' g1_robot_fast_lio 2>/dev/null |
                grep -qx true; then
            echo "ERROR: FAST-LIO stopped during initialization" >&2
            docker compose logs --no-color --tail 200 fast_lio >&2 || true
            exit 1
        fi
        sleep 1
    done
    if ! docker compose logs --no-color --tail 200 fast_lio 2>&1 |
            grep -Fq "IMU Initial Done"; then
        echo "ERROR: FAST-LIO did not finish IMU initialization within 150 seconds" >&2
        docker compose logs --no-color --tail 200 fast_lio >&2 || true
        exit 1
    fi
else
    docker compose stop navigation localization
    docker compose rm -f navigation localization
    if ! docker inspect -f '{{.State.Running}}' g1_robot_fast_lio 2>/dev/null |
            grep -qx true; then
        echo "ERROR: FAST-LIO is not running; retry with --restart-fast-lio" >&2
        exit 1
    fi
fi

# `docker compose run localization ...` creates one-off containers that are not
# guaranteed to be removed by `docker compose rm`. They would keep publishing an
# old transient-local /map or /pcd_map on the host ROS graph.
stale_localization_ids=$(docker ps -aq \
    --filter label=com.docker.compose.service=localization)
if [ -n "$stale_localization_ids" ]; then
    echo "Removing stale localization containers"
    # shellcheck disable=SC2086
    docker rm -f $stale_localization_ids >/dev/null
fi

MAP_SCENE="$scene" docker compose --profile navigation \
    up -d --force-recreate --no-deps localization

actual_scene=$(docker inspect g1_robot_localization \
    --format '{{range .Config.Env}}{{println .}}{{end}}' |
    sed -n 's/^MAP_SCENE=//p' | tail -n 1)
if [ "$actual_scene" != "$scene" ]; then
    echo "ERROR: localization container has MAP_SCENE='$actual_scene', expected '$scene'" >&2
    exit 1
fi

expected_log="Map selection: scene=$scene "
for _ in $(seq 1 75); do
    if docker compose logs --no-color --tail 200 localization 2>&1 |
            grep -Fq "$expected_log"; then
        docker compose logs --no-color --tail 200 localization 2>&1 |
            grep -F "$expected_log" | tail -n 1

        runtime_verified=false
        for _ in $(seq 1 180); do
            pcd_param=$(docker exec g1_robot_localization bash -lc \
                'source /opt/ros/humble/setup.bash; source /botbrain_ws/install/setup.bash; ros2 param get /global_localization_node path_map' \
                2>/dev/null || true)
            yaml_param=$(docker exec g1_robot_localization bash -lc \
                'source /opt/ros/humble/setup.bash; source /botbrain_ws/install/setup.bash; ros2 param get /map_server yaml_filename' \
                2>/dev/null || true)
            map_info=$(docker exec g1_robot_localization bash -lc \
                'source /opt/ros/humble/setup.bash; source /botbrain_ws/install/setup.bash; ros2 topic info /map' \
                2>/dev/null || true)
            pcd_info=$(docker exec g1_robot_localization bash -lc \
                'source /opt/ros/humble/setup.bash; source /botbrain_ws/install/setup.bash; ros2 topic info /pcd_map' \
                2>/dev/null || true)
            if [[ "$pcd_param" == *"/${scene}_scans.pcd"* ]] &&
                    [[ "$yaml_param" == *"/${scene}.yaml"* ]] &&
                    grep -Fq "Publisher count: 1" <<<"$map_info" &&
                    grep -Fq "Publisher count: 1" <<<"$pcd_info"; then
                runtime_verified=true
                break
            fi
            sleep 1
        done
        if [ "$runtime_verified" != true ]; then
            echo "ERROR: runtime map verification failed for scene '$scene'" >&2
            echo "path_map: $pcd_param" >&2
            echo "yaml_filename: $yaml_param" >&2
            echo "/map: $map_info" >&2
            echo "/pcd_map: $pcd_info" >&2
            exit 1
        fi

        docker compose up -d --force-recreate foxglove >/dev/null
        echo "Scene '$scene' loaded. Foxglove recreated to discard stale transient map data."
        echo "Wait for 'Localization ready' before starting navigation."
        exit 0
    fi
    if ! docker inspect -f '{{.State.Running}}' g1_robot_localization 2>/dev/null |
            grep -qx true; then
        echo "ERROR: localization container stopped before loading '$scene'" >&2
        docker compose logs --no-color --tail 200 localization >&2 || true
        exit 1
    fi
    sleep 1
done

echo "ERROR: localization did not confirm scene '$scene' within 75 seconds" >&2
docker compose logs --no-color --tail 200 localization >&2 || true
exit 1
