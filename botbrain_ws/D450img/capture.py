#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

SAVE_DIR = os.path.dirname(os.path.abspath(__file__))

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipeline.start(config)
try:
    # 跳过前几帧，等待自动曝光稳定
    for _ in range(5):
        pipeline.wait_for_frames()

    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        raise RuntimeError("未获取到彩色帧")

    img = np.asanyarray(color_frame.get_data())
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    path = os.path.join(SAVE_DIR, f"frame_{ts}.png")
    cv2.imwrite(path, img)
    print(f"已保存: {path}")
finally:
    pipeline.stop()
