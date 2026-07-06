#!/bin/bash
sleep 1
ros2 service call /g1_robot/arm_cmd bot_custom_interfaces/srv/ArmCmd \
    "{command: 6, name: '', names: ['pose_now_7','pose_now_6','pose_now_5','pose_now_4','pose_now_3','pose_now_2','pose_now_1']}"

ros2 service call /g1_robot/arm_cmd bot_custom_interfaces/srv/ArmCmd \
    "{command: 6, name: '', names: ['pose_now_2','pose_now_3','pose_now_4','pose_now_5','pose_now_6','pose_now_7']}"

ros2 service call /g1_robot/arm_cmd bot_custom_interfaces/srv/ArmCmd "{command: 3, name: ''}"
