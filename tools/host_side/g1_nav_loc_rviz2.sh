#!/bin/bash
# tools/g1_nav_loc_rviz2.sh — Launch RViz2 on the operator workstation,
# connected to the G1 Zenoh router for the navigation + localization view.
#
# Renamed from the legacy `g1_track0_rviz2.sh`; "track0" was the old
# mapping/loc-track concept and no longer reflects what this view shows.
#
# Pre-conditions:
#   - G1 localization is ready and the navigation Compose service is running
#   - workstation has ros-humble-rmw-zenoh-cpp + ros-humble-rviz2
#
# Usage (from the cloned repo root or any cwd):
#   bash tools/host_side/g1_nav_loc_rviz2.sh <G1_IP>
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$SCRIPT_DIR/../.." && pwd)"
RVIZ_CFG="$REPO/configs/g1_nav_loc_rviz2.rviz"
G1_IP="${1:-${G1_IP:-192.168.100.30}}"

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

ros2 daemon stop  >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true
sleep 2

exec rviz2 -d "$RVIZ_CFG"
