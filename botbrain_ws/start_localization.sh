#!/usr/bin/env bash
sleep 30
source install/setup.bash
export LD_LIBRARY_PATH=/opt/open3d/lib:$LD_LIBRARY_PATH
exec ros2 launch g1_pkg localization_3d.launch.py
