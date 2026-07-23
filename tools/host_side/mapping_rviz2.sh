#!/bin/bash
# tools/mapping_rviz2.sh — Launch RViz2 on the operator workstation,
# connected to the G1 Zenoh router for the mapping view.
#
# Pre-conditions:
#   - G1 has `docker compose up -d zenoh bringup state_machine fast_lio` running
#   - workstation has ros-humble-rmw-zenoh-cpp + ros-humble-rviz2
#
# Usage (from the cloned repo root or any cwd):
#   bash tools/host_side/mapping_rviz2.sh <G1_IP>
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG="$REPO/configs/g1_mapping_rviz2.rviz"
G1_IP="${1:-${G1_IP:-192.168.37.204}}"

if [ ! -f "$RVIZ_CFG" ]; then
    echo "ERROR: RViz config not found: $RVIZ_CFG" >&2
    exit 1
fi
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo "ERROR: ROS 2 Humble is not installed on this workstation" >&2
    exit 1
fi
if [[ ! "$G1_IP" =~ ^[A-Za-z0-9.-]+$ ]]; then
    echo "ERROR: invalid G1 IP/hostname '$G1_IP'" >&2
    exit 2
fi
if ! timeout 3 bash -c "exec 3<>/dev/tcp/${G1_IP}/7448" 2>/dev/null; then
    echo "ERROR: cannot reach Zenoh at ${G1_IP}:7448; start robot zenoh and check the network" >&2
    exit 1
fi

source /opt/ros/humble/setup.bash
if ! command -v rviz2 >/dev/null; then
    echo "ERROR: rviz2 is missing; install ros-humble-rviz2" >&2
    exit 1
fi
if ! command -v ros2 >/dev/null; then
    echo "ERROR: ros2 is missing after loading ROS 2 Humble" >&2
    exit 1
fi
if ! ros2 pkg prefix rmw_zenoh_cpp >/dev/null 2>&1; then
    echo "ERROR: rmw_zenoh_cpp is missing; install ros-humble-rmw-zenoh-cpp" >&2
    exit 1
fi
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${G1_IP}:7448\"]"
export ZENOH_ROUTER_CHECK_ATTEMPTS=10

echo "RViz2 mapping preset: $RVIZ_CFG"
echo "Checking that FAST-LIO world/body point clouds are visible from this workstation"
if ! timeout 15 ros2 topic echo /cloud_registered_1 --once --field header \
        --qos-reliability best_effort --no-daemon >/dev/null 2>&1; then
    echo "ERROR: no /cloud_registered_1 reached this workstation within 15s" >&2
    echo "Check g1_robot_fast_lio, Zenoh, and use the mapping launcher with FAST_LIO_MAPPING_MODE=true." >&2
    exit 1
fi

# Both FAST-LIO cloud publishers use BEST_EFFORT/volatile QoS. Zenoh discovery
# may expose the world topic before the diagnostic body topic, especially when
# each `ros2 topic echo` starts a fresh participant. Keep the world cloud as the
# real mapping gate, but do not reject a usable mapping view for this transient
# discovery race; RViz will continue discovering the preloaded body display.
body_cloud_ready=false
for _ in 1 2 3; do
    if timeout 3 ros2 topic echo /cloud_registered_body_1 --once --field header \
            --qos-reliability best_effort --no-daemon >/dev/null 2>&1; then
        body_cloud_ready=true
        break
    fi
    sleep 1
done
if [ "$body_cloud_ready" = true ]; then
    echo "FAST-LIO world/body point-cloud transport is ready. Opening RViz2."
else
    echo "WARNING: world cloud is live but Body cloud discovery is delayed; opening RViz2 and continuing to retry in the preloaded Body display." >&2
    echo "If Body remains Warm after 30s, check publish.scan_bodyframe_pub_en and FAST-LIO logs." >&2
fi

case "${RVIZ_RENDERING:-hardware}" in
    hardware) ;;
    software)
        export LIBGL_ALWAYS_SOFTWARE=1
        echo "RViz2 renderer: Mesa software fallback"
        ;;
    *)
        echo "ERROR: RVIZ_RENDERING must be 'hardware' or 'software'" >&2
        exit 2
        ;;
esac

echo "Preloaded displays: live/history /cloud_registered_1, /cloud_registered_body_1, /accumulated_grid, /Odometry_loc, /path_1 and TF"
echo "No manual Add is required. Fixed Frame must remain camera_init."
echo "If Intel/Mesa reports 'active samplers with a different type', retry with RVIZ_RENDERING=software."
if command -v pgrep >/dev/null 2>&1 && pgrep -x rviz2 >/dev/null 2>&1; then
    echo "WARNING: another RViz2 process is already open; close the old window so it is not mistaken for this updated preset." >&2
fi

exec rviz2 -d "$RVIZ_CFG"
