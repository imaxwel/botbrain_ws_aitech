#!/bin/bash
# RealSense 相机启动脚本
# 保留 IMU 传感器（即使有报错）

docker exec unitree-dex3-dev bash -c '
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py   depth_module.profile:=640x480x30   rgb_camera.profile:=640x480x30   align_depth.enable:=true   initial_reset:=true
'
