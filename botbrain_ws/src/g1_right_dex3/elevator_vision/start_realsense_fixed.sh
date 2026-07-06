#!/bin/bash
# RealSense 修正版启动脚本 - 正确的参数语法

docker exec unitree-dex3-dev bash -c '
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py   depth_module.depth_profile:=640x480x30   rgb_camera.color_profile:=640x480x30   align_depth.enable:=true   initial_reset:=true
'
