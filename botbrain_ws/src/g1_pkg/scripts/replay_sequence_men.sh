#!/bin/bash
# 等待机器人稳定
sleep 8
# 顺序执行上前刷卡整套动作 door1~door6
ros2 service call /g1_robot/arm_cmd bot_custom_interfaces/srv/ArmCmd     "{command: 6, name: '', names: ['door_1','door_2','door_3','door_4','door_5','door_6']}"
# 顺序执行刷卡后收回复位动作 door_back1~door5
ros2 service call /g1_robot/arm_cmd bot_custom_interfaces/srv/ArmCmd     "{command: 6, name: '', names: ['door_back_1','door_back_2','door_back_3','door_back_4','door_back_5']}"
# 动作结束，手臂回到初始待机位
ros2 service call /g1_robot/arm_cmd bot_custom_interfaces/srv/ArmCmd "{command: 3, name: ''}"
