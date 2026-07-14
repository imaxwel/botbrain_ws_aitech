#!/bin/bash
# start_map_edit.sh — launch ros_map_edit RViz panel for editing the
# G1 2D occupancy grid.
#
# Runs on the operator's *workstation*, not on G1. G1 is headless;
# map editing is GUI work. The yaml/pgm files must already be mounted
# into the container at MAPS_IN_CONT.
#
# Override via env vars when your setup differs from the defaults:
#   CONTAINER     = map_edit_rviz   (docker container name)
#   CATKIN_WS     = /catkin_ws      (workspace path inside container)
#   MAPS_IN_CONT  = /root/maps      (maps dir inside container)
#   LIBGL_ALWAYS_SOFTWARE = 1       (safe default; set 0 only when /dev/dri is mapped)
#   DISABLE_ROS1_EOL_WARNINGS = 1   (hide the ROS 1 end-of-life reminder dialog)
#
# Usage:
#   ./start_map_edit.sh
#   ./start_map_edit.sh /root/maps/accumulated_grid.yaml
#   CONTAINER=my_rviz ./start_map_edit.sh
#
# First-time setup (build the container) is in README.md.
set -e

CONTAINER="${CONTAINER:-map_edit_rviz}"
CATKIN_WS="${CATKIN_WS:-/catkin_ws}"
MAPS_IN_CONT="${MAPS_IN_CONT:-/root/maps}"
MAP_FILE="${1:-$MAPS_IN_CONT/accumulated_grid.yaml}"
HOST_DISPLAY="${DISPLAY:-:0}"
LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
LIBGL_DRI3_DISABLE="${LIBGL_DRI3_DISABLE:-1}"
DISABLE_ROS1_EOL_WARNINGS="${DISABLE_ROS1_EOL_WARNINGS:-1}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "ERROR: container '$CONTAINER' is not running." >&2
    echo "  - if it exists but is stopped:  docker start $CONTAINER" >&2
    echo "  - if it doesn't exist yet:      see README.md (Build the editor container)" >&2
    echo "  - if it's named differently:    CONTAINER=<name> $0" >&2
    exit 1
fi

DISPLAY="$HOST_DISPLAY" xhost +local:root >/dev/null 2>&1 || true

echo "[1/3] cleaning stale rviz/roscore in container ..."
docker exec "$CONTAINER" bash -c '
    # Bracketed process patterns avoid matching this cleanup shell itself.
    pkill -TERM -f "[r]oslaunch ros_map_edit map_edit.launch" 2>/dev/null || true
    pkill -TERM -x rviz                                  2>/dev/null || true
    pkill -TERM -x map_server                            2>/dev/null || true
    pkill -TERM -f "[r]oscore"                           2>/dev/null || true
    pkill -TERM -f "[r]osmaster"                         2>/dev/null || true
    sleep 1
' || true

echo "[2/3] verifying ros_map_edit is built ..."
docker exec "$CONTAINER" bash -c "
    test -f $CATKIN_WS/devel/lib/libros_map_edit.so || {
        echo '  libros_map_edit.so missing under $CATKIN_WS/devel/lib/' >&2
        echo '  rebuild:  docker exec $CONTAINER bash -c \"source /opt/ros/noetic/setup.bash && cd $CATKIN_WS && catkin_make\"' >&2
        exit 1
    }
    echo '  OK'
"

echo "[3/3] launching ros_map_edit with map=$MAP_FILE ..."
# The container may inherit ROS_MASTER_URI from prior cross-host work.
# Local map editing is fully self-contained — clear those vars so
# roslaunch starts its own local roscore on localhost:11311.
docker exec \
    -e DISPLAY="$HOST_DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e XDG_RUNTIME_DIR=/tmp/runtime-root \
    -e LIBGL_ALWAYS_SOFTWARE="$LIBGL_ALWAYS_SOFTWARE" \
    -e LIBGL_DRI3_DISABLE="$LIBGL_DRI3_DISABLE" \
    -e DISABLE_ROS1_EOL_WARNINGS="$DISABLE_ROS1_EOL_WARNINGS" \
    "$CONTAINER" bash -c "
    unset ROS_MASTER_URI ROS_IP ROS_HOSTNAME
    export ROS_MASTER_URI=http://localhost:11311
    install -d -m 700 /tmp/runtime-root
    source /opt/ros/noetic/setup.bash
    source $CATKIN_WS/devel/setup.bash
    rospack find ros_map_edit >/dev/null || {
        echo 'ros_map_edit not on ROS_PACKAGE_PATH after sourcing $CATKIN_WS/devel/setup.bash' >&2
        echo 'is the workspace built?' >&2
        exit 1
    }
    exec roslaunch ros_map_edit map_edit.launch map_file:='$MAP_FILE'
"
