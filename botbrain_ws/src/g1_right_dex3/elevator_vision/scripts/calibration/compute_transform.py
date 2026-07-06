#!/usr/bin/env python3
"""
计算相机到机器人坐标系的变换矩阵
使用SVD求解刚体变换
"""
import json
import numpy as np
from pathlib import Path

def compute_transform(camera_points, robot_points):
    """计算从相机坐标系到机器人坐标系的4x4变换矩阵"""
    cam_pts = np.array(camera_points)
    rob_pts = np.array(robot_points)

    cam_mean = np.mean(cam_pts, axis=0)
    rob_mean = np.mean(rob_pts, axis=0)
    cam_centered = cam_pts - cam_mean
    rob_centered = rob_pts - rob_mean

    H = cam_centered.T @ rob_centered
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = rob_mean - R @ cam_mean

    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def main():
    data_file = Path(__file__).parent / 'calibration_data.json'
    output_file = Path(__file__).parent.parent.parent / 'transforms' / 'camera_to_robot.npy'

    if not data_file.exists():
        print(f"错误: 找不到 {data_file}")
        return

    with open(data_file, 'r') as f:
        data = json.load(f)

    if len(data['camera_points']) < 3:
        print(f"错误: 至少需要3组数据")
        return

    T = compute_transform(data['camera_points'], data['robot_points'])
    output_file.parent.mkdir(parents=True, exist_ok=True)
    np.save(output_file, T)
    print(f"变换矩阵已保存: {output_file}")
    print(T)

if __name__ == '__main__':
    main()
