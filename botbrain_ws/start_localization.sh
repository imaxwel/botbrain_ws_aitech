#!/usr/bin/env bash
sleep 30
source install/setup.bash
export LD_LIBRARY_PATH=/opt/open3d/lib:$LD_LIBRARY_PATH

# --- Camera auto-activation guard ---
# state_machine's bring_up scans lifecycle nodes once (event-triggered) and
# permanently skips any that are absent at scan time (if(!cur) continue).
# On boot / localization restart the realsense nodes appear ~30-60s late, so
# state_machine skips them and they stay "unconfigured" with no camera image.
# This background helper waits for the camera nodes to appear and drives them
# to active. It is a no-op if state_machine already activated them.
(
  NS=/g1_robot
  FRONT=$NS/front_camera
  COMP=$NS/realsense_compressed_node
  # Wait up to ~4min for the front camera lifecycle node to register.
  for i in $(seq 1 80); do
    if ros2 lifecycle get $FRONT >/dev/null 2>&1; then
      break
    fi
    sleep 3
  done
  activate_node() {
    node=$1
    st=$(ros2 lifecycle get $node 2>/dev/null | awk '{print $1}')
    case "$st" in
      unconfigured)
        ros2 lifecycle set $node configure >/dev/null 2>&1
        sleep 8
        ros2 lifecycle set $node activate  >/dev/null 2>&1
        ;;
      inactive)
        ros2 lifecycle set $node activate  >/dev/null 2>&1
        ;;
      active)
        : # already active, nothing to do
        ;;
    esac
  }
  # Give state_machine a chance first; then fill the gap if it did not activate.
  sleep 20
  for attempt in 1 2 3; do
    fs=$(ros2 lifecycle get $FRONT 2>/dev/null | awk '{print $1}')
    if [ "$fs" != "active" ]; then
      activate_node $FRONT
      sleep 3
    fi
    cs=$(ros2 lifecycle get $COMP 2>/dev/null | awk '{print $1}')
    if [ "$cs" != "active" ]; then
      activate_node $COMP
    fi
    fs=$(ros2 lifecycle get $FRONT 2>/dev/null | awk '{print $1}')
    cs=$(ros2 lifecycle get $COMP 2>/dev/null | awk '{print $1}')
    if [ "$fs" = "active" ] && [ "$cs" = "active" ]; then
      break
    fi
    sleep 15
  done
) &

python3 /botbrain_ws/cam_frame_writer.py &>/tmp/cam_frame_writer.log &
exec ros2 launch g1_pkg localization_3d.launch.py
