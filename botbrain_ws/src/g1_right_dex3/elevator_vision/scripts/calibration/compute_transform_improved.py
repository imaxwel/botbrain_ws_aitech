#!/usr/bin/env python3
"""
改进版手眼标定：利用四元数修正 tag-指尖偏移 + 迭代离群点剔除后做 SVD 求解
tag 相对 right_tcp_link（指尖）的偏移（TCP坐标系，单位m）：
  x: -0.02（tag在指尖后方2cm）
  z: +0.01（tag在指尖上方1cm）
"""
import json
import shutil
import numpy as np
from pathlib import Path

TAG_OFFSET_TCP = np.array([-0.022, 0.0, 0.006])  # 实测：x=-2.2cm(tag在指尖后), z=+0.6cm(tag在指尖上)


def quat_to_rot(q_xyzw):
    """[qx,qy,qz,qw] → 3×3 旋转矩阵"""
    qx, qy, qz, qw = q_xyzw
    return np.array([
        [1-2*(qy**2+qz**2), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx**2+qz**2), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx**2+qy**2)],
    ])


def rot_to_quat(R):
    """3×3 旋转矩阵 → [qx,qy,qz,qw]"""
    tr = R[0,0]+R[1,1]+R[2,2]
    if tr > 0:
        s = 0.5/np.sqrt(tr+1)
        return np.array([(R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, 0.25/s])
    elif R[0,0] > R[1,1] and R[0,0] > R[2,2]:
        s = 2*np.sqrt(1+R[0,0]-R[1,1]-R[2,2])
        return np.array([0.25*s, (R[0,1]+R[1,0])/s, (R[0,2]+R[2,0])/s, (R[2,1]-R[1,2])/s])
    elif R[1,1] > R[2,2]:
        s = 2*np.sqrt(1+R[1,1]-R[0,0]-R[2,2])
        return np.array([(R[0,1]+R[1,0])/s, 0.25*s, (R[1,2]+R[2,1])/s, (R[0,2]-R[2,0])/s])
    else:
        s = 2*np.sqrt(1+R[2,2]-R[0,0]-R[1,1])
        return np.array([(R[0,2]+R[2,0])/s, (R[1,2]+R[2,1])/s, 0.25*s, (R[1,0]-R[0,1])/s])


def kabsch_svd(src, dst):
    """SVD求从src到dst的刚体变换：dst≈ R @ src + t"""
    sm, dm = src.mean(0), dst.mean(0)
    H = (src - sm).T @ (dst - dm)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = dm - R @ sm
    T = np.eye(4); T[:3,:3] = R; T[:3,3] = t
    return T


def main():
    here = Path(__file__).parent
    transforms_dir = here.parent.parent / 'transforms'
    transforms_dir.mkdir(parents=True, exist_ok=True)

    with open(here / 'calibration_data.json') as f:
        data = json.load(f)

    cam_all  = np.array(data['camera_points'])
    tcp_all  = np.array(data['robot_points'])
    quat_all = np.array(data['robot_quats'])

    # 将tag偏移从TCP坐标系旋转到躯干坐标系
    tag_all = np.array([
        tcp_all[i] + quat_to_rot(quat_all[i]) @ TAG_OFFSET_TCP
        for i in range(len(tcp_all))
    ])

    # 迭代剔除残差 > mean+2σ 的离群点
    mask = np.ones(len(cam_all), dtype=bool)
    for _ in range(3):
        T = kabsch_svd(cam_all[mask], tag_all[mask])
        pred = (T[:3,:3] @ cam_all[mask].T).T + T[:3,3]
        res = np.linalg.norm(pred - tag_all[mask], axis=1)
        inliers = res < (res.mean() + 2 * res.std())
        if inliers.all():
            break
        mask[np.where(mask)[0][~inliers]] = False

    T = kabsch_svd(cam_all[mask], tag_all[mask])

    # 残差报告
    pred_all = (T[:3,:3] @ cam_all.T).T + T[:3,3]
    res_all  = np.linalg.norm(pred_all - tag_all, axis=1)
    print("各点残差 (mm):")
    for i, r in enumerate(res_all):
        status = " 【剔除】" if not mask[i] else ""
        print(f"  点{i+1:2d}: {r*1000:5.1f}mm{status}")
    inlier_res = res_all[mask]
    print(f"  使用 {mask.sum()}/{len(mask)} 点 | 均值: {inlier_res.mean()*1000:.1f}mm | 最大: {inlier_res.max()*1000:.1f}mm")

    # 备份旧参数
    old_file = transforms_dir / 'camera_to_robot.npy'
    if old_file.exists():
        shutil.copy(old_file, transforms_dir / 'camera_to_robot_old.npy')
        print(f"\n旧参数已备份 → camera_to_robot_old.npy")

    np.save(old_file, T)

    q = rot_to_quat(T[:3,:3])
    t = T[:3,3]
    print(f"\n新标定参数 (torso_link → camera_color_optical_frame):")
    print(f"  Translation : [{t[0]:.6f}, {t[1]:.6f}, {t[2]:.6f}]")
    print(f"  Quaternion  : [{q[0]:.6f}, {q[1]:.6f}, {q[2]:.6f}, {q[3]:.6f}]")
    print(f"\nstatic_transform_publisher 参数:")
    print(f"  {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} {q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}")


if __name__ == '__main__':
    main()
