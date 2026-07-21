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
scene_state_file=botbrain_ws/.runtime/map_scene
readonly ICP_MIN_FITNESS=0.50
readonly ICP_MAX_RMSE=0.30
readonly PUBLISHER_STABLE_ROUNDS=3

rollback_armed=false
switch_completed=false
previous_fast_lio_running=false
previous_localization_running=false
previous_navigation_running=false
previous_scene=""

exec 9>/tmp/botbrain_map_scene_switch.lock
if ! flock -n 9; then
    echo "ERROR: another map scene switch is already running" >&2
    exit 75
fi

verify_navigation_install_matches_source() {
    local pairs=(
        "botbrain_ws/src/bot_navigation/scripts/navigation_preflight.py:botbrain_ws/install/bot_navigation/lib/bot_navigation/navigation_preflight.py"
        "botbrain_ws/src/bot_navigation/scripts/nav_odom_relay.py:botbrain_ws/install/bot_navigation/lib/bot_navigation/nav_odom_relay.py"
        "botbrain_ws/src/bot_navigation/scripts/localization_monitor.py:botbrain_ws/install/bot_navigation/lib/bot_navigation/localization_monitor.py"
        "botbrain_ws/src/bot_navigation/scripts/waypoint_navigator.py:botbrain_ws/install/bot_navigation/lib/bot_navigation/waypoint_navigator.py"
        "botbrain_ws/src/bot_navigation/scripts/waypoint_recorder.py:botbrain_ws/install/bot_navigation/lib/bot_navigation/waypoint_recorder.py"
        "botbrain_ws/src/bot_navigation/launch/nav_utils.launch.py:botbrain_ws/install/bot_navigation/share/bot_navigation/launch/nav_utils.launch.py"
        "botbrain_ws/src/bot_navigation/behavior_trees/g1_navigate_to_pose.xml:botbrain_ws/install/bot_navigation/share/bot_navigation/behavior_trees/g1_navigate_to_pose.xml"
        "botbrain_ws/src/g1_pkg/config/nav2_params.yaml:botbrain_ws/install/g1_pkg/share/g1_pkg/config/nav2_params.yaml"
        "botbrain_ws/src/g1_pkg/config/pointcloud_to_laserscan_params.yaml:botbrain_ws/install/g1_pkg/share/g1_pkg/config/pointcloud_to_laserscan_params.yaml"
        "botbrain_ws/src/g1_pkg/launch/localization_3d.launch.py:botbrain_ws/install/g1_pkg/share/g1_pkg/launch/localization_3d.launch.py"
        "botbrain_ws/src/g1_pkg/launch/robot_interface.launch.py:botbrain_ws/install/g1_pkg/share/g1_pkg/launch/robot_interface.launch.py"
        "botbrain_ws/src/g1_pkg/scripts/g1_read.py:botbrain_ws/install/g1_pkg/lib/g1_pkg/g1_read.py"
        "botbrain_ws/src/bot_bringup/config/twist_mux.yaml:botbrain_ws/install/bot_bringup/share/bot_bringup/config/twist_mux.yaml"
    )
    local pair
    local source_path
    local install_path
    local waypoint_store_install
    local stale=()

    for pair in "${pairs[@]}"; do
        source_path=${pair%%:*}
        install_path=${pair#*:}
        if [ ! -f "$install_path" ] ||
                ! cmp -s "$source_path" "$install_path"; then
            stale+=("$install_path")
        fi
    done
    waypoint_store_install=$(
        find -L botbrain_ws/install/bot_navigation \
            -type f -path '*/bot_navigation/waypoint_store.py' \
            -print -quit 2>/dev/null || true
    )
    if [ -z "$waypoint_store_install" ] ||
            ! cmp -s \
                botbrain_ws/src/bot_navigation/bot_navigation/waypoint_store.py \
                "$waypoint_store_install"; then
        stale+=("bot_navigation Python module waypoint_store.py")
    fi
    if [ ! -x botbrain_ws/install/open3d_loc/lib/open3d_loc/global_localization_node ] ||
            [ botbrain_ws/src/open3d_loc/src/global_localization.cpp -nt \
              botbrain_ws/install/open3d_loc/lib/open3d_loc/global_localization_node ]; then
        stale+=("botbrain_ws/install/open3d_loc/lib/open3d_loc/global_localization_node")
    fi
    if [ ! -x botbrain_ws/install/fast_lio/lib/fast_lio/fastlio_mapping ] ||
            [ botbrain_ws/src/fast_lio/src/laserMapping.cpp -nt \
              botbrain_ws/install/fast_lio/lib/fast_lio/fastlio_mapping ]; then
        stale+=("botbrain_ws/install/fast_lio/lib/fast_lio/fastlio_mapping")
    fi
    if [ ! -x botbrain_ws/install/g1_pkg/lib/g1_pkg/g1_write_node ] ||
            [ botbrain_ws/src/g1_pkg/src/g1_write.cpp -nt \
              botbrain_ws/install/g1_pkg/lib/g1_pkg/g1_write_node ]; then
        stale+=("botbrain_ws/install/g1_pkg/lib/g1_pkg/g1_write_node")
    fi
    if [ "${#stale[@]}" -eq 0 ]; then
        return 0
    fi

    echo "ERROR: navigation install space is missing or stale:" >&2
    printf '  %s\n' "${stale[@]}" >&2
    echo "Rebuild fast_lio, open3d_loc, g1_pkg, bot_navigation and bot_bringup before using --wait-ready." >&2
    return 1
}

persist_scene_state() {
    local selected_scene=$1
    local state_dir
    local temporary
    state_dir=$(dirname "$scene_state_file")
    mkdir -p "$state_dir"
    temporary=$(mktemp "$state_dir/.map_scene.XXXXXX")
    printf '%s\n' "$selected_scene" >"$temporary"
    mv -f "$temporary" "$scene_state_file"
}

persist_selected_scene() {
    persist_scene_state "$scene"
}

container_running() {
    local container=$1
    docker inspect -f '{{.State.Running}}' "$container" 2>/dev/null |
        grep -qx true
}

container_map_scene() {
    local container=$1
    docker inspect "$container" \
        --format '{{range .Config.Env}}{{println .}}{{end}}' 2>/dev/null |
        sed -n 's/^MAP_SCENE=//p' | tail -n 1
}

capture_previous_runtime() {
    if container_running g1_robot_fast_lio; then
        previous_fast_lio_running=true
    fi
    if container_running g1_robot_localization; then
        previous_localization_running=true
    fi
    if container_running g1_robot_navigation; then
        previous_navigation_running=true
    fi

    previous_scene=$(container_map_scene g1_robot_localization || true)
    if [ -z "$previous_scene" ] && [ -s "$scene_state_file" ]; then
        IFS= read -r previous_scene <"$scene_state_file" || true
    fi
    if [ -n "$previous_scene" ] &&
            [[ ! "$previous_scene" =~ ^[A-Za-z0-9][A-Za-z0-9_-]*$ ]]; then
        echo "WARNING: ignoring invalid previous map scene '$previous_scene'" >&2
        previous_scene=""
    fi
    if [ "$previous_localization_running" = true ] &&
            [ -z "$previous_scene" ]; then
        echo "ERROR: running localization scene cannot be identified; refusing a destructive switch" >&2
        return 1
    fi
}

restore_previous_runtime() {
    local restore_ok=true

    echo "Map switch failed; restoring the previous runtime state" >&2
    if ! docker compose stop navigation localization >/dev/null 2>&1; then
        restore_ok=false
    fi
    if ! docker compose rm -f navigation localization >/dev/null 2>&1; then
        restore_ok=false
    fi

    if [ "$restart_fast_lio" = true ]; then
        if ! docker compose stop fast_lio >/dev/null 2>&1; then
            restore_ok=false
        fi
        if ! docker compose rm -f fast_lio >/dev/null 2>&1; then
            restore_ok=false
        fi
        if [ "$previous_fast_lio_running" = true ] &&
                ! FAST_LIO_START_DELAY_SEC=0 docker compose up -d \
                    --force-recreate fast_lio >/dev/null 2>&1; then
            restore_ok=false
        fi
    fi

    if [ "$previous_localization_running" = true ]; then
        if [ -z "$previous_scene" ]; then
            restore_ok=false
        elif ! MAP_SCENE="$previous_scene" LOCALIZATION_START_DELAY_SEC=0 \
                docker compose --profile navigation up -d --force-recreate \
                    --no-deps localization >/dev/null 2>&1; then
            restore_ok=false
        fi
    fi
    if [ "$previous_navigation_running" = true ]; then
        if ! docker compose --profile navigation up -d --force-recreate \
                --no-deps navigation >/dev/null 2>&1; then
            restore_ok=false
        fi
    fi

    if [ -n "$previous_scene" ]; then
        if ! persist_scene_state "$previous_scene"; then
            restore_ok=false
        fi
    elif ! rm -f "$scene_state_file"; then
        restore_ok=false
    fi

    if [ "$restore_ok" = true ]; then
        echo "Previous runtime restored: scene=${previous_scene:-none} localization=${previous_localization_running} navigation=${previous_navigation_running}" >&2
        return 0
    fi
    echo "WARNING: automatic runtime restore was incomplete; inspect docker compose ps and logs" >&2
    return 1
}

handle_selector_exit() {
    local status=$1
    trap - EXIT INT TERM
    if [ "$status" -ne 0 ] && [ "$rollback_armed" = true ] &&
            [ "$switch_completed" != true ]; then
        restore_previous_runtime || true
    fi
    exit "$status"
}

arm_switch_rollback() {
    capture_previous_runtime
    rollback_armed=true
    trap 'handle_selector_exit "$?"' EXIT
    trap 'exit 130' INT
    trap 'exit 143' TERM
}

fast_lio_running() {
    container_running g1_robot_fast_lio
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

stable_fast_lio_timing_streak() {
    awk '
        /\[FAST_LIO_TIMING\] ok=false|timing discontinuity|abnormal timing|unsynchronized scan|invalid IMU rebase baseline/ {
            count = 0
            first = second = third = ""
            next
        }
        /\[FAST_LIO_TIMING\] ok=true/ {
            if (count == 0) first = $0
            else if (count == 1) second = $0
            else if (count == 2) third = $0
            else {
                first = second
                second = third
                third = $0
            }
            if (count < 3) count++
        }
        END {
            print count + 0
            if (count == 3) {
                print first
                print second
                print third
            }
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

        timing_streak=$(stable_fast_lio_timing_streak <<<"$after_last_init")
        timing_count=$(sed -n '1p' <<<"$timing_streak")
        if grep -Fq "IMU Initial Done" <<<"$logs" &&
                [ "$timing_count" -eq 3 ] &&
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

localization_active_logs() {
    local since=${1:-$localization_started_at}
    local current_container_id
    if ! current_container_id=$(docker inspect -f '{{.Id}}' \
            g1_robot_localization 2>/dev/null); then
        echo "ERROR: localization container is no longer available" >&2
        return 1
    fi
    if [ "$current_container_id" != "$localization_container_id" ]; then
        echo "ERROR: localization container changed while readiness was being checked" >&2
        return 1
    fi
    docker logs --since "$since" \
        "$localization_container_id" 2>&1
}

trailing_accepted_icp_streak() {
    local min_fitness=${ICP_MIN_FITNESS:-0.50}
    local max_rmse=${ICP_MAX_RMSE:-0.30}
    awk -v min_fitness="$min_fitness" -v max_rmse="$max_rmse" '
        function reset_streak() {
            count = 0
            first = second = third = ""
        }
        BEGIN {
            number_pattern = "^[-+]?[0-9]+([.][0-9]+)?([eE][-+]?[0-9]+)?$"
        }
        /Rejecting ICP|Skipping ICP|ICP: accepted=false|Holding large ICP correction|Discarding stale ICP result|Waiting for odometry history/ {
            reset_streak()
            next
        }
        /ICP: accepted=true/ {
            fitness_text = rmse_text = ""
            for (field = 1; field <= NF; field++) {
                if ($field ~ /^fitness=/)
                    fitness_text = substr($field, length("fitness=") + 1)
                if ($field ~ /^rmse=/)
                    rmse_text = substr($field, length("rmse=") + 1)
            }
            if (fitness_text !~ number_pattern || rmse_text !~ number_pattern) {
                reset_streak()
                next
            }
            fitness_value = fitness_text + 0
            rmse_value = rmse_text + 0
            if (fitness_value <= min_fitness || rmse_value > max_rmse) {
                reset_streak()
                next
            }
            if (count == 0) first = $0
            else if (count == 1) second = $0
            else if (count == 2) third = $0
            else {
                first = second
                second = third
                third = $0
            }
            if (count < 3) count++
        }
        END {
            print count + 0
            if (count == 3) {
                print first
                print second
                print third
            }
        }
    '
}

verify_navigation_topic_publishers() {
    local topics=(
        /Odometry_loc
        /cloud_registered_1
        /cloud_registered_body_1
        /scan
        /g1_robot/odom
        /g1_robot/nav_odom
    )
    local counts=()
    local errors=()
    local last_counts=()
    local last_errors=()
    local all_unique=true
    local stable_rounds=0
    # Keep the historical default (required_rounds=3) while centralizing the
    # value so every publisher check uses the same debounce requirement.
    local required_rounds=${PUBLISHER_STABLE_ROUNDS:-3}
    local deadline=$((SECONDS + 15))
    local result
    local topic

    while [ "$stable_rounds" -lt "$required_rounds" ] &&
            [ "$SECONDS" -lt "$deadline" ]; do
        counts=()
        errors=()
        all_unique=true
        for topic in "${topics[@]}"; do
            if result=$(topic_publisher_count \
                    g1_robot_localization "$topic" 2>&1); then
                counts+=("$result")
                if [[ ! "$result" =~ ^1$ ]]; then
                    all_unique=false
                fi
            else
                counts+=("unknown")
                errors+=("$topic: $result")
                all_unique=false
            fi
        done
        last_counts=("${counts[@]}")
        last_errors=("${errors[@]}")

        if [ "$all_unique" = true ]; then
            stable_rounds=$((stable_rounds + 1))
            echo "Navigation topic publisher check ${stable_rounds}/${required_rounds}"
            if [ "$stable_rounds" -eq "$required_rounds" ]; then
                echo "Navigation topic publishers are unique and stable."
                return 0
            fi
        else
            stable_rounds=0
            echo "Navigation topic publisher graph is not stable yet; retrying" >&2
        fi
        sleep 1
    done

    echo "ERROR: navigation topics did not reach stable single publishers; navigation topics must each have exactly 1 publisher for ${required_rounds} consecutive checks" >&2
    for index in "${!topics[@]}"; do
        echo "${topics[$index]} publishers: ${last_counts[$index]:-unknown}" >&2
    done
    if [ "${#last_errors[@]}" -gt 0 ]; then
        printf 'publisher query error: %s\n' "${last_errors[@]}" >&2
    fi
    return 1
}

print_icp_quality() {
    awk '
        {
            fitness = ""
            rmse = ""
            for (field = 1; field <= NF; field++) {
                if ($field ~ /^fitness=/) fitness = $field
                if ($field ~ /^rmse=/) rmse = $field
            }
            if (fitness != "" && rmse != "")
                printf "  ICP accepted: %s %s\n", fitness, rmse
        }
    '
}

print_unitree_twist_diagnostics() {
    echo "Unitree odometry diagnostics:" >&2
    if ! docker inspect -f '{{.State.Running}}' g1_robot_bringup 2>/dev/null |
            grep -qx true; then
        echo "  g1_robot_bringup is not running" >&2
        return
    fi
    docker exec g1_robot_bringup bash -lc '
        source /opt/ros/humble/setup.bash
        source /botbrain_ws/install/setup.bash
        ros2 lifecycle get /g1_robot/robot_read_node 2>&1 || true
        ros2 topic info /lf/odommodestate 2>&1 || true
        timeout 5 ros2 topic hz /lf/odommodestate 2>&1 || true
        ros2 topic info /g1_robot/odom 2>&1 || true
        timeout 5 ros2 topic hz /g1_robot/odom 2>&1 || true
        ros2 topic info /g1_robot/nav_odom 2>&1 || true
        timeout 5 ros2 topic hz /g1_robot/nav_odom 2>&1 || true
    ' >&2 || true
}

wait_for_localization_preflight() {
    local ready_deadline=$1
    local remaining=$((ready_deadline - SECONDS))
    if [ "$remaining" -le 0 ]; then
        echo "ERROR: --ready-timeout expired before navigation preflight" >&2
        return 1
    fi
    local command_timeout=$((remaining + 15))
    local preflight_output=""
    local preflight_log
    preflight_log=$(mktemp /tmp/botbrain_navigation_preflight.XXXXXX)
    if ! docker exec g1_robot_localization bash -lc "
        source /opt/ros/humble/setup.bash
        source /botbrain_ws/install/setup.bash
        timeout ${command_timeout} ros2 run bot_navigation navigation_preflight.py \\
            --ros-args -p timeout_sec:=${remaining}.0 \\
            -p min_confidence:=${ICP_MIN_FITNESS} \\
            -p allow_pose_derived_twist:=false
    " 2>&1 | tee "$preflight_log"; then
        preflight_output=$(<"$preflight_log")
        rm -f "$preflight_log"
        echo "ERROR: scene '$scene' loaded but automatic localization did not become ready" >&2
        print_unitree_twist_diagnostics
        docker compose logs --no-color --tail 200 localization >&2 || true
        return 1
    fi
    preflight_output=$(<"$preflight_log")
    rm -f "$preflight_log"
    if ! grep -Fq "twist_source=unitree" <<<"$preflight_output"; then
        echo "ERROR: navigation preflight passed without verified Unitree twist" >&2
        return 1
    fi
}

wait_for_consecutive_icp_accepts() {
    local ready_deadline=$1
    local localization_logs=""
    local latest_decision=""
    local printed_decision=""
    local recent_icp=""
    local streak=""
    local streak_count=0

    echo "Localization is running; waiting for 3 consecutive accepted ICP updates (fitness>${ICP_MIN_FITNESS}, rmse<=${ICP_MAX_RMSE})"
    while [ "$SECONDS" -lt "$ready_deadline" ]; do
        # Three accepted updates normally span about 4-6 seconds. Restrict the
        # window so old startup successes cannot mask a stopped ICP worker.
        if ! localization_logs=$(localization_active_logs 15s); then
            return 1
        fi
        streak=$(trailing_accepted_icp_streak <<<"$localization_logs")
        streak_count=$(sed -n '1p' <<<"$streak")
        latest_decision=$(grep -E \
            'ICP: accepted=(true|false)|Rejecting ICP|Skipping ICP|Holding large ICP correction|Discarding stale ICP result|Waiting for odometry history|Global registration seed=|Global initialization evaluation|LocalizationInitialize: (rejecting|holding)|localization initialization succeeded' \
            <<<"$localization_logs" | tail -n 1 || true)
        if [ -n "$latest_decision" ] && \
                [ "$latest_decision" != "$printed_decision" ]; then
            printed_decision=$latest_decision
            if [ "$streak_count" -gt 0 ] && \
                    [[ "$latest_decision" == *"ICP: accepted=true"* ]]; then
                echo "ICP stability progress: ${streak_count}/3"
                tail -n 1 <<<"$streak" | print_icp_quality
            elif [[ "$latest_decision" == *"ICP"* ]]; then
                echo "ICP stability progress reset:"
                printf '  %s\n' "$latest_decision"
            else
                echo "Localization matching update:"
                printf '  %s\n' "$latest_decision"
            fi
        fi
        if [ "$streak_count" -eq 3 ]; then
            echo "Localization ICP is stable:"
            tail -n 3 <<<"$streak" | print_icp_quality
            return 0
        fi
        sleep 1
    done

    echo "ERROR: localization did not produce 3 consecutive accepted ICP updates before --ready-timeout ${ready_timeout}s expired" >&2
    recent_icp=$(grep -E \
        'ICP: accepted=(true|false)|Rejecting ICP|Skipping ICP' \
        <<<"$localization_logs" | tail -n 12 || true)
    if [ -n "$recent_icp" ]; then
        echo "Latest ICP decisions from the current localization container:" >&2
        printf '%s\n' "$recent_icp" >&2
    else
        echo "No ICP decision was logged by the current localization container." >&2
    fi
    return 1
}

wait_for_localization_ready() {
    local ready_deadline=$((SECONDS + ready_timeout))
    echo "Waiting up to ${ready_timeout}s for navigation inputs and stable localization"
    # Check live inputs first, collect a current ICP streak, then recheck the
    # inputs so an early match cannot mask a later sensor outage.
    wait_for_localization_preflight "$ready_deadline" || return 1
    wait_for_consecutive_icp_accepts "$ready_deadline" || return 1
    wait_for_localization_preflight "$ready_deadline" || return 1
    verify_navigation_topic_publishers || return 1
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

if [ "$wait_ready" = true ]; then
    verify_navigation_install_matches_source
fi

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
    if ! cmp -s botbrain_ws/src/fast_lio/config/mid360.yaml \
            botbrain_ws/install/fast_lio/share/fast_lio/config/mid360.yaml; then
        echo "ERROR: installed FAST-LIO config is stale; rebuild fast_lio" >&2
        exit 1
    fi

    arm_switch_rollback
    docker compose stop navigation localization fast_lio
    docker compose rm -f navigation localization fast_lio
    FAST_LIO_START_DELAY_SEC=0 docker compose up -d --force-recreate fast_lio

    wait_for_restarted_fast_lio
else
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
    arm_switch_rollback
    docker compose stop navigation localization
    docker compose rm -f navigation localization
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
old_publisher_zero_rounds=0
publisher_query_error=""
publisher_query_succeeded=false
map_publishers=unknown
pcd_publishers=unknown
while [ "$SECONDS" -lt "$old_publisher_deadline" ]; do
    if ! map_publisher_result=$(topic_publisher_count \
            g1_robot_fast_lio /map 2>&1); then
        publisher_query_error=$map_publisher_result
        publisher_query_succeeded=false
        old_publisher_zero_rounds=0
        sleep 1
        continue
    fi
    map_publishers=$map_publisher_result
    if ! pcd_publisher_result=$(topic_publisher_count \
            g1_robot_fast_lio /pcd_map 2>&1); then
        publisher_query_error=$pcd_publisher_result
        publisher_query_succeeded=false
        old_publisher_zero_rounds=0
        sleep 1
        continue
    fi
    pcd_publishers=$pcd_publisher_result
    publisher_query_succeeded=true
    publisher_query_error=""
    if [ "$map_publishers" -eq 0 ] && [ "$pcd_publishers" -eq 0 ]; then
        old_publisher_zero_rounds=$((old_publisher_zero_rounds + 1))
        echo "Old map publisher removal check ${old_publisher_zero_rounds}/${PUBLISHER_STABLE_ROUNDS}"
        if [ "$old_publisher_zero_rounds" -eq "$PUBLISHER_STABLE_ROUNDS" ]; then
            old_publishers_gone=true
            break
        fi
    else
        old_publisher_zero_rounds=0
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

localization_container_id=$(docker inspect -f '{{.Id}}' \
    g1_robot_localization)
localization_started_at=$(docker inspect -f \
    '{{if .State.Running}}{{.State.StartedAt}}{{end}}' \
    "$localization_container_id")
if [ -z "$localization_container_id" ] || [ -z "$localization_started_at" ]; then
    echo "ERROR: unable to identify the current localization startup" >&2
    exit 1
fi
# Whole-second RFC3339 is accepted by older Docker versions as well. The
# immutable container ID still limits the query to this startup cycle.
case "$localization_started_at" in
    *.*Z) localization_started_at="${localization_started_at%%.*}Z" ;;
esac

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
        runtime_stable_rounds=0
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
                runtime_stable_rounds=$((runtime_stable_rounds + 1))
                echo "Runtime map publisher check ${runtime_stable_rounds}/${PUBLISHER_STABLE_ROUNDS}"
                if [ "$runtime_stable_rounds" -eq "$PUBLISHER_STABLE_ROUNDS" ]; then
                    runtime_verified=true
                    break
                fi
            else
                runtime_stable_rounds=0
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

        if [ "$wait_ready" = true ]; then
            wait_for_localization_ready
        else
            echo "Wait for 'Localization ready' before starting navigation."
        fi

        if docker inspect -f '{{.State.Running}}' g1_robot_foxglove 2>/dev/null |
                grep -qx true; then
            echo "Scene '$scene' loaded. Foxglove connection was preserved."
        else
            docker compose up -d foxglove >/dev/null
            echo "Scene '$scene' loaded. Foxglove was started without forced recreation."
        fi

        # All containers mount botbrain_ws at /botbrain_ws. Keep one atomic,
        # shared scene marker so waypoint commands can select the matching
        # scene section without changing their familiar CLI. Write it only
        # after all requested readiness checks pass, so rollback never leaves
        # a marker for a scene that was not accepted.
        persist_selected_scene
        switch_completed=true
        rollback_armed=false
        trap - EXIT INT TERM
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
