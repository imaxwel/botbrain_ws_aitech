#!/bin/bash
# RealSense 低帧率启动脚本 - 保留所有传感器，降低帧率

docker exec unitree-dex3-dev bash -c '
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py   depth_module.profile:=640x480x15   rgb_camera.profile:=640x480x15   align_depth.enable:=true   initial_reset:=true
'
