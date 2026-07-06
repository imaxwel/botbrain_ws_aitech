#!/usr/bin/env python3
"""
手眼标定计算脚本（眼在手外，Eye-to-Hand）
使用 cv2.calibrateHandEye (AX=XB) 求解 torso_link → camera_color_optical_frame

输入: handeye-flange-calibration-data.md 或 .json
  每行格式（md）:
    tag=(cx,cy,cz) tag_quat=(cqx,cqy,cqz,cqw) Translation:[rx,ry,rz] Rotation:[rqx,rqy,rqz,rqw]
"""
import json, re, sys, os
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R

DATA_PATH = os.path.join(os.path.dirname(__file__), 'handeye-flange-calibration-data.md')
OUT_PATH = '/botbrain_ws/src/g1_right_dex3/elevator_vision/transforms/camera_to_robot.npy'

_NUM = r'[-+]?\d+\.?\d*'
_RE = re.compile(
    r'tag=\((' + _NUM + r'),\s*(' + _NUM + r'),\s*(' + _NUM + r')\)'
    r'.*?tag_quat=\((' + _NUM + r'),(' + _NUM + r'),(' + _NUM + r'),(' + _NUM + r')\)'
    r'.*?Translation:\s*\[(' + _NUM + r'),\s*(' + _NUM + r'),\s*(' + _NUM + r')\]'
    r'.*?Rotation.*?\[(' + _NUM + r'),\s*(' + _NUM + r'),\s*(' + _NUM + r'),\s*(' + _NUM + r')\]'
)


def load(path):
    if path.endswith('.json'):
        with open(path) as f:
            d = json.load(f)
        return (np.array(d['camera_points']), np.array(d['camera_quats']),
                np.array(d['robot_points']),  np.array(d['robot_quats']))
    # markdown 格式
    cp, cq, rp, rq = [], [], [], []
    with open(path) as f:
        for line in f:
            m = _RE.search(line)
            if not m:
                continue
            g = [float(x) for x in m.groups()]
            cp.append(g[0:3]); cq.append(g[3:7])
            rp.append(g[7:10]); rq.append(g[10:14])
    return np.array(cp), np.array(cq), np.array(rp), np.array(rq)


def main():
    path = sys.argv[1] if len(sys.argv) > 1 else DATA_PATH
    cp, cq, rp, rq = load(path)
    n = len(cp)
    print(f'加载 {n} 组点对')
    assert n >= 6, f'至少需要 6 组，当前只有 {n} 组'

    R_t2c = [R.from_quat(q).as_matrix() for q in cq]   # tag in camera
    t_t2c = [p.reshape(3, 1) for p in cp]
    R_g2b = [R.from_quat(q).as_matrix() for q in rq]   # EE in base
    t_g2b = [p.reshape(3, 1) for p in rp]

    R_cam, t_cam = cv2.calibrateHandEye(
        R_g2b, t_g2b, R_t2c, t_t2c,
        method=cv2.CALIB_HAND_EYE_TSAI
    )

    T = np.eye(4)
    T[:3, :3] = R_cam
    T[:3,  3] = t_cam.flatten()

    np.set_printoptions(precision=6, suppress=True)
    print('\nT (torso_link -> camera_color_optical_frame):')
    print(T)

    tx = T[0, 3]
    print(f'\nT[0,3] = {tx:.6f}  {"OK > 0.04" if tx > 0.04 else "FAIL < 0.04, 请检查数据"}')

    np.save(OUT_PATH, T)
    print(f'\n已保存至 {OUT_PATH}')
    print('下一步：运行 README-handeye-flange-calibration.md 步骤四脚本提取参数')


if __name__ == '__main__':
    main()
