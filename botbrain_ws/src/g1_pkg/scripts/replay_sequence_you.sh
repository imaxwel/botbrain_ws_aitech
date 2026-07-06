#!/bin/bash
sleep 5
ros2 service call /g1_robot/arm_cmd bot_custom_interfaces/srv/ArmCmd \
    "{command: 6, name: '', names: ['pose_now6','pose_now5','pose_now4','pose_now3','pose_now2','pose_now1']}"

ros2 service call /g1_robot/arm_cmd bot_custom_interfaces/srv/ArmCmd \
    "{command: 6, name: '', names: ['pose_now2','pose_now3','pose_now4','pose_now5','pose_now6']}"

ros2 service call /g1_robot/arm_cmd bot_custom_interfaces/srv/ArmCmd "{command: 3, name: ''}"
