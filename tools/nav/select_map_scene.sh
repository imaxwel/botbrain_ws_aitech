#!/usr/bin/env bash
set -euo pipefail

usage() {
    echo "Usage: $0 <scene> [--restart-fast-lio] [--wait-ready] [--ready-timeout SEC]" >&2
    echo "Same floor:   $0 aitech" >&2
    echo "Cross floor:  $0 floor4 --restart-fast-lio --wait-ready" >&2
}

if [ "$#" -lt 1 ]; then
    usage
    exit 2
fi

scene=$1
shift
restart_fast_lio=false
wait_ready=false
ready_timeout=300
while [ "$#" -gt 0 ]; do
    case "$1" in
        --restart-fast-lio)
            restart_fast_lio=true
            shift
            ;;
        --wait-ready)
            wait_ready=true
            shift
            ;;
        --ready-timeout)
            if [ "$#" -lt 2 ] || [[ ! "$2" =~ ^[1-9][0-9]*$ ]]; then
                echo "ERROR: --ready-timeout requires a positive integer" >&2
                exit 2
            fi
            ready_timeout=$2
            wait_ready=true
            shift 2
            ;;
        *)
            usage
            exit 2
            ;;
    esac
done
if [[ ! "$scene" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
    echo "ERROR: invalid scene '$scene'" >&2
    exit 2
fi

project_root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
cd "$project_root"

exec 9>/tmp/botbrain_map_scene_switch.lock
if ! flock -n 9; then
    echo "ERROR: another map scene switch is already running" >&2
    exit 75
fi

fast_lio_running() {
    docker inspect -f '{{.State.Running}}' g1_robot_fast_lio 2>/dev/null |
        grep -qx true
}

fast_lio_logs() {
    docker compose logs --no-color --tail 500 fast_lio 2>&1
}

fast_lio_active_logs() {
    awk '
        { all = all $0 ORS }
        /IMU Initial Done/ { found = 1; active = ""; next }
        found { active = active $0 ORS }
        END {
            if (found) printf "%s", active;
            else printf "%s", all;
        }
    '
}

verify_fast_lio_topics() {
    docker exec g1_robot_fast_lio bash -lc '
        set -e
        source /opt/ros/humble/setup.bash
        source /botbrain_ws/install/setup.bash
        timeout 10 ros2 topic echo /Odometry_loc --once >/dev/null
        timeout 10 ros2 topic echo /cloud_registered_1 --once >/dev/null
    ' >/dev/null 2>&1
}

topic_publisher_count() {
    local container=$1
    local topic=$2
    docker exec "$container" bash -lc '
        # ROS setup scripts legitimately probe unset AMENT_* variables. Enable
        # nounset only after both environments have been sourced.
        set +u
        set -o pipefail
        source /opt/ros/humble/setup.bash
        source /botbrain_ws/install/setup.bash
        set -u
        topic=$1
        status=0
        output=$(timeout 5 ros2 topic info "$topic" 2>&1) || status=$?
        if [ "$status" -ne 0 ]; then
            if grep -Fq "Unknown topic" <<<"$output"; then
                echo 0
                exit 0
            fi
            printf "%s\n" "$output" >&2
            exit "$status"
        fi
        count=$(sed -n "s/^[[:space:]]*Publisher count: //p" \
            <<<"$output" | tail -n 1)
        if [[ ! "$count" =~ ^[0-9]+$ ]]; then
            echo "Unable to parse publisher count for $topic" >&2
            exit 1
        fi
        echo "$count"
    ' _ "$topic"
}

wait_for_restarted_fast_lio() {
    echo "Waiting for FAST-LIO initialization and continuous sensor timing"
    local deadline=$((SECONDS + 180))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if ! fast_lio_running; then
            echo "ERROR: FAST-LIO stopped during initialization" >&2
            fast_lio_logs >&2 || true
            return 1
        fi

        logs=$(fast_lio_logs)
        after_last_init=$(fast_lio_active_logs <<<"$logs")
        if grep -Eq 'output (remains )?latched unhealthy' \
                <<<"$after_last_init"; then
            echo "ERROR: FAST-LIO entered a latched unhealthy state" >&2
            tail -n 80 <<<"$after_last_init" >&2
            return 1
        fi

        latest_timing=$(grep -E '\[FAST_LIO_TIMING\] ok=(true|false)' \
            <<<"$after_last_init" | tail -n 3 || true)
        timing_count=$(awk 'NF { count++ } END { print count + 0 }' \
            <<<"$latest_timing")
        if grep -Fq "IMU Initial Done" <<<"$logs" &&
                [ "$timing_count" -eq 3 ] &&
                ! grep -Fq "ok=false" <<<"$latest_timing" &&
                verify_fast_lio_topics; then
            echo "FAST-LIO is healthy: IMU initialized, timing stable, outputs live"
            return 0
        fi
        sleep 1
    done

    echo "ERROR: FAST-LIO did not become healthy within 180 seconds" >&2
    fast_lio_logs >&2 || true
    return 1
}

wait_for_localization_preflight() {
    local command_timeout=$((ready_timeout + 15))
    echo "Waiting up to ${ready_timeout}s for automatic localization and navigation inputs"
    if ! docker exec g1_robot_localization bash -lc "
        source /opt/ros/humble/setup.bash
        source /botbrain_ws/install/setup.bash
        timeout ${command_timeout} ros2 run bot_navigation navigation_preflight.py \\
            --ros-args -p timeout_sec:=${ready_timeout}.0
    "; then
        echo "ERROR: scene '$scene' loaded but automatic localization did not become ready" >&2
        docker compose logs --no-color --tail 200 localization >&2 || true
        return 1
    fi
    echo "Scene '$scene' localization is ready for navigation."
}

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
    FAST_LIO_START_DELAY_SEC=0 docker compose up -d --force-recreate fast_lio

    wait_for_restarted_fast_lio
else
    docker compose stop navigation localization
    docker compose rm -f navigation localization
    if ! fast_lio_running; then
        echo "ERROR: FAST-LIO is not running; retry with --restart-fast-lio" >&2
        exit 1
    fi
    current_fast_lio_logs=$(fast_lio_logs)
    current_fast_lio_logs=$(fast_lio_active_logs <<<"$current_fast_lio_logs")
    if grep -Eq 'output (remains )?latched unhealthy' \
            <<<"$current_fast_lio_logs" || ! verify_fast_lio_topics; then
        echo "ERROR: FAST-LIO outputs are not healthy; retry with --restart-fast-lio" >&2
        exit 1
    fi
fi

# `docker compose run localization ...` creates one-off containers that are not
# guaranteed to be removed by `docker compose rm`. They would keep publishing an
# old transient-local /map or /pcd_map on the host ROS graph.
stale_localization_ids=$(docker compose ps -aq --all localization)
if [ -n "$stale_localization_ids" ]; then
    echo "Removing stale localization containers"
    # shellcheck disable=SC2086
    docker rm -f $stale_localization_ids >/dev/null
fi

old_publishers_gone=false
old_publisher_deadline=$((SECONDS + 30))
publisher_query_error=""
publisher_query_succeeded=false
map_publishers=unknown
pcd_publishers=unknown
while [ "$SECONDS" -lt "$old_publisher_deadline" ]; do
    if ! map_publisher_result=$(topic_publisher_count \
            g1_robot_fast_lio /map 2>&1); then
        publisher_query_error=$map_publisher_result
        sleep 1
        continue
    fi
    map_publishers=$map_publisher_result
    if ! pcd_publisher_result=$(topic_publisher_count \
            g1_robot_fast_lio /pcd_map 2>&1); then
        publisher_query_error=$pcd_publisher_result
        sleep 1
        continue
    fi
    pcd_publishers=$pcd_publisher_result
    publisher_query_succeeded=true
    if [ "$map_publishers" -eq 0 ] && [ "$pcd_publishers" -eq 0 ]; then
        old_publishers_gone=true
        break
    fi
    sleep 1
done
if [ "$old_publishers_gone" != true ]; then
    if [ "$publisher_query_succeeded" != true ]; then
        echo "ERROR: unable to query old map publishers; refusing to switch scenes" >&2
    else
        echo "ERROR: old map publishers are still visible; refusing to mix scenes" >&2
    fi
    echo "/map publishers: ${map_publishers:-unknown}" >&2
    echo "/pcd_map publishers: ${pcd_publishers:-unknown}" >&2
    if [ -n "$publisher_query_error" ]; then
        echo "last ROS graph query error: $publisher_query_error" >&2
    fi
    if docker inspect -f '{{.State.Running}}' g1_robot_mapping 2>/dev/null |
            grep -qx true; then
        echo "legacy container g1_robot_mapping is running; inspect or stop it before retrying" >&2
    fi
    exit 1
fi

MAP_SCENE="$scene" LOCALIZATION_START_DELAY_SEC=0 \
    docker compose --profile navigation \
    up -d --force-recreate --no-deps localization

actual_scene=$(docker inspect g1_robot_localization \
    --format '{{range .Config.Env}}{{println .}}{{end}}' |
    sed -n 's/^MAP_SCENE=//p' | tail -n 1)
if [ "$actual_scene" != "$scene" ]; then
    echo "ERROR: localization container has MAP_SCENE='$actual_scene', expected '$scene'" >&2
    exit 1
fi

expected_log="Map selection: scene=$scene "
scene_log_deadline=$((SECONDS + 75))
while [ "$SECONDS" -lt "$scene_log_deadline" ]; do
    if docker compose logs --no-color --tail 200 localization 2>&1 |
            grep -Fq "$expected_log"; then
        docker compose logs --no-color --tail 200 localization 2>&1 |
            grep -F "$expected_log" | tail -n 1

        runtime_verified=false
        runtime_deadline=$((SECONDS + 180))
        while [ "$SECONDS" -lt "$runtime_deadline" ]; do
            current_fast_lio_logs=$(fast_lio_logs)
            current_fast_lio_logs=$(fast_lio_active_logs \
                <<<"$current_fast_lio_logs")
            if grep -Eq 'output (remains )?latched unhealthy' \
                    <<<"$current_fast_lio_logs"; then
                echo "ERROR: FAST-LIO became unhealthy while loading '$scene'" >&2
                tail -n 80 <<<"$current_fast_lio_logs" >&2
                exit 1
            fi
            localization_logs=$(docker compose logs --no-color --tail 200 \
                localization 2>&1)
            if ! grep -Fq "initialize finished" <<<"$localization_logs"; then
                sleep 2
                continue
            fi
            pcd_param=$(docker exec g1_robot_localization bash -lc \
                'source /opt/ros/humble/setup.bash; source /botbrain_ws/install/setup.bash; timeout 5 ros2 param get /global_localization_node path_map' \
                2>/dev/null || true)
            yaml_param=$(docker exec g1_robot_localization bash -lc \
                'source /opt/ros/humble/setup.bash; source /botbrain_ws/install/setup.bash; timeout 5 ros2 param get /map_server yaml_filename' \
                2>/dev/null || true)
            map_publishers=""
            pcd_publishers=""
            map_query_ok=false
            pcd_query_ok=false
            if map_publishers=$(topic_publisher_count \
                    g1_robot_localization /map 2>/dev/null); then
                map_query_ok=true
            fi
            if pcd_publishers=$(topic_publisher_count \
                    g1_robot_localization /pcd_map 2>/dev/null); then
                pcd_query_ok=true
            fi
            if [[ "$pcd_param" == *"/${scene}_scans.pcd"* ]] &&
                    [[ "$yaml_param" == *"/${scene}.yaml"* ]] &&
                    [ "$map_query_ok" = true ] && [ "$map_publishers" -eq 1 ] &&
                    [ "$pcd_query_ok" = true ] && [ "$pcd_publishers" -eq 1 ]; then
                runtime_verified=true
                break
            fi
            sleep 5
        done
        if [ "$runtime_verified" != true ]; then
            echo "ERROR: runtime map verification failed for scene '$scene'" >&2
            echo "path_map: $pcd_param" >&2
            echo "yaml_filename: $yaml_param" >&2
            echo "/map publishers: ${map_publishers:-unknown}" >&2
            echo "/pcd_map publishers: ${pcd_publishers:-unknown}" >&2
            exit 1
        fi

        if docker inspect -f '{{.State.Running}}' g1_robot_foxglove 2>/dev/null |
                grep -qx true; then
            echo "Scene '$scene' loaded. Foxglove connection was preserved."
        else
            docker compose up -d foxglove >/dev/null
            echo "Scene '$scene' loaded. Foxglove was started without forced recreation."
        fi
        if [ "$wait_ready" = true ]; then
            wait_for_localization_preflight
        else
            echo "Wait for 'Localization ready' before starting navigation."
        fi
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
