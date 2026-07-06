#!/usr/bin/env python3
"""
标定数据采集工具
用于采集相机坐标系和机器人坐标系的对应点对
"""
import json
import numpy as np
from pathlib import Path

class CalibrationDataCollector:
    def __init__(self, save_path='calibration_data.json'):
        self.save_path = Path(save_path)
        self.data = {'camera_points': [], 'robot_points': []}
        if self.save_path.exists():
            self.load()

    def add_point_pair(self, camera_point, robot_point):
        """添加一对对应点"""
        self.data['camera_points'].append(camera_point)
        self.data['robot_points'].append(robot_point)
        print(f"已添加第 {len(self.data['camera_points'])} 组数据")

    def save(self):
        """保存数据"""
        with open(self.save_path, 'w') as f:
            json.dump(self.data, f, indent=2)
        print(f"数据已保存到 {self.save_path}")

    def load(self):
        """加载数据"""
        with open(self.save_path, 'r') as f:
            self.data = json.load(f)
        print(f"已加载 {len(self.data['camera_points'])} 组数据")

if __name__ == '__main__':
    collector = CalibrationDataCollector()
    print("标定数据采集工具")
    print("请在ROS2环境中配合AprilTag检测和机器人控制使用")
