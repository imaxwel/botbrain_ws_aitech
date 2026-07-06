#!/bin/bash
# RealSense 简化启动脚本 - 仅启用彩色和深度流

docker exec unitree-dex3-dev bash -c '
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py   enable_gyro:=false   enable_accel:=false   enable_infra1:=false   enable_infra2:=false   enable_color:=true   enable_depth:=true   depth_module.profile:=640x480x15   rgb_camera.profile:=640x480x15   align_depth.enable:=true
'
