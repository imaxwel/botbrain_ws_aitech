#!/usr/bin/env python3
"""
D405 深度点云采集脚本
输出：
  - pointcloud_<timestamp>.ply   彩色点云（CloudCompare / MeshLab 可打开）
  - depth_vis_<timestamp>.png    深度伪彩色图（快速目视检查）
  - color_<timestamp>.png        对应彩色图
"""
import pyrealsense2 as rs
import numpy as np
import cv2
import struct
import os
from datetime import datetime

SAVE_DIR = os.path.dirname(os.path.abspath(__file__))
WIDTH, HEIGHT, FPS = 640, 480, 30
# D405 有效深度范围（米）
DEPTH_MIN, DEPTH_MAX = 0.07, 1.5


def save_ply(path, points, colors):
    """保存彩色点云为 ASCII PLY 文件。"""
    valid = (
        np.isfinite(points).all(axis=1) &
        (points[:, 2] > DEPTH_MIN) &
        (points[:, 2] < DEPTH_MAX)
    )
    pts = points[valid]
    col = colors[valid]
    n = len(pts)
    with open(path, 'w') as f:
        f.write("ply\nformat ascii 1.0\n")
        f.write(f"element vertex {n}\n")
        f.write("property float x\nproperty float y\nproperty float z\n")
        f.write("property uchar red\nproperty uchar green\nproperty uchar blue\n")
        f.write("end_header\n")
        for i in range(n):
            x, y, z = pts[i]
            r, g, b = col[i]
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {r} {g} {b}\n")
    return n


def main():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    config.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)

    profile = pipeline.start(config)

    # 深度单位（米/计数）
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print(f"深度比例尺: {depth_scale:.6f} m/count")

    # 对齐深度到彩色帧
    align = rs.align(rs.stream.color)

    # 点云对象
    pc = rs.pointcloud()

    try:
        # 跳过前几帧，等待自动曝光稳定
        print("预热中（10帧）...")
        for _ in range(10):
            pipeline.wait_for_frames()

        frames = pipeline.wait_for_frames()
        aligned = align.process(frames)

        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            raise RuntimeError("未能获取帧")

        color_img = np.asanyarray(color_frame.get_data())
        depth_img = np.asanyarray(depth_frame.get_data())  # uint16, 单位 count

        # 深度伪彩色可视化
        depth_m = depth_img * depth_scale
        depth_clipped = np.clip(depth_m, DEPTH_MIN, DEPTH_MAX)
        depth_norm = ((depth_clipped - DEPTH_MIN) / (DEPTH_MAX - DEPTH_MIN) * 255).astype(np.uint8)
        depth_vis = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        # 无效点（z=0）显示为黑色
        depth_vis[depth_img == 0] = 0

        # 生成点云
        pc.map_to(color_frame)
        points_rs = pc.calculate(depth_frame)
        vtx = np.asanyarray(points_rs.get_vertices()).view(np.float32).reshape(-1, 3)
        col_flat = color_img.reshape(-1, 3)[:, ::-1]  # BGR→RGB

        # 统计有效点
        valid_mask = (
            np.isfinite(vtx).all(axis=1) &
            (vtx[:, 2] > DEPTH_MIN) &
            (vtx[:, 2] < DEPTH_MAX)
        )
        valid_pts = vtx[valid_mask]
        print(f"\n点云统计：")
        print(f"  总点数: {len(vtx):,}")
        print(f"  有效点: {len(valid_pts):,}  ({100*len(valid_pts)/len(vtx):.1f}%)")
        if len(valid_pts):
            print(f"  X 范围: [{valid_pts[:,0].min():.3f}, {valid_pts[:,0].max():.3f}] m")
            print(f"  Y 范围: [{valid_pts[:,1].min():.3f}, {valid_pts[:,1].max():.3f}] m")
            print(f"  Z 范围: [{valid_pts[:,2].min():.3f}, {valid_pts[:,2].max():.3f}] m")
            print(f"  中心距离（Z 均值）: {valid_pts[:,2].mean():.3f} m")

        # 保存文件
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        ply_path   = os.path.join(SAVE_DIR, f"pointcloud_{ts}.ply")
        depth_path = os.path.join(SAVE_DIR, f"depth_vis_{ts}.png")
        color_path = os.path.join(SAVE_DIR, f"color_{ts}.png")

        n_saved = save_ply(ply_path, vtx, col_flat)
        cv2.imwrite(depth_path, depth_vis)
        cv2.imwrite(color_path, color_img)

        print(f"\n已保存:")
        print(f"  PLY  点云 ({n_saved:,} 点): {ply_path}")
        print(f"  深度伪彩色图:              {depth_path}")
        print(f"  彩色图:                    {color_path}")

    finally:
        pipeline.stop()


if __name__ == '__main__':
    main()
