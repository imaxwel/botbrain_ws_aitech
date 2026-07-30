#!/bin/bash
TARGET_FLOOR="${1:-0}"
echo "目标楼层: ${TARGET_FLOOR}"
mkdir -p /data/botbrain_ws/botbrain_project-main/botbrain_ws/D405img/button_debug
docker exec g1_robot_dev_dex3 bash -c "export CYCLONEDDS_URI=file:///botbrain_ws/cyclonedds_config.xml && source /opt/ros/humble/setup.bash && mkdir -p /botbrain_ws/D405img/button_debug && python3 /botbrain_ws/src/g1_right_dex3/elevator_vision/scripts/button_detector_node.py --ros-args -p frozen_model_dir:=/botbrain_ws/src/g1_right_dex3/yolonas_ocr/frozen_model -p target_floor:=${TARGET_FLOOR} -p image_topic:=/g1_robot/front_camera/color/image_raw -p depth_topic:=/g1_robot/front_camera/depth/image_rect_raw -p info_topic:=/g1_robot/front_camera/color/camera_info -p camera_frame:=g1_robot/front_camera_color_optical_frame -p output_frame:=torso_link -p det_threshold:=0.3 -p save_debug_images:=true -p debug_image_dir:=/botbrain_ws/D405img/button_debug 2>&1 | grep -v ddsi_udp | grep -v tev:"
