#!/usr/bin/env python3
"""从URDF参数生成相机到机器人的变换矩阵"""
import numpy as np
from scipy.spatial.transform import Rotation as R

# URDF参数：torso_link → d435_link
xyz = [0.0576235, 0.01753, 0.42987]
rpy = [0, 0.8307767239493009, 0]

rotation = R.from_euler('xyz', rpy).as_matrix()
T_torso_to_d435 = np.eye(4)
T_torso_to_d435[:3, :3] = rotation
T_torso_to_d435[:3, 3] = xyz

# d435_link → camera_color_optical_frame
q = np.array([-0.5, 0.5, -0.5, 0.5])
R_color_to_optical = R.from_quat(q).as_matrix()
T_d435_to_optical = np.eye(4)
T_d435_to_optical[:3, :3] = R_color_to_optical

T_torso_to_camera = T_torso_to_d435 @ T_d435_to_optical
T_camera_to_torso = np.linalg.inv(T_torso_to_camera)

print("变换矩阵 (camera → torso):")
print(T_camera_to_torso)

import os
os.makedirs('../transforms', exist_ok=True)
np.save('../transforms/camera_to_robot.npy', T_camera_to_torso)
print("\n✓ 保存到: ../transforms/camera_to_robot.npy")
