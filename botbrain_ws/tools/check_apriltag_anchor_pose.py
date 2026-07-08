#!/usr/bin/env python3
"""Independent AprilTag anchor pose checker.

Subscribes to an RGB image + CameraInfo, detects one AprilTag, averages a few
camera-frame pose samples, and compares them against a stored anchor baseline.
It does not publish any robot control messages.
"""

import argparse
import math
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from pupil_apriltags import Detector
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def parse_vec3(text):
    values = [float(v.strip()) for v in text.split(',')]
    if len(values) != 3:
        raise argparse.ArgumentTypeError('expected x,y,z')
    return np.asarray(values, dtype=np.float64)


class AnchorChecker(Node):
    def __init__(self, args):
        super().__init__('apriltag_anchor_pose_checker')
        self.args = args
        self.bridge = CvBridge()
        self.camera_info = None
        self.samples = []
        self.last_stamp = None
        self.detector = Detector(
            families=args.family,
            nthreads=1,
            quad_decimate=args.quad_decimate,
            refine_edges=1,
        )
        self.create_subscription(CameraInfo, args.camera_info_topic, self.info_cb, 10)
        self.create_subscription(Image, args.image_topic, self.image_cb, 10)

    def info_cb(self, msg):
        self.camera_info = msg

    def image_cb(self, msg):
        if self.camera_info is None or len(self.samples) >= self.args.samples:
            return
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])

        scale = self.args.detect_scale
        if scale != 1.0:
            gray_detect = cv2.resize(
                gray,
                (int(gray.shape[1] * scale), int(gray.shape[0] * scale)),
                interpolation=cv2.INTER_AREA,
            )
        else:
            gray_detect = gray

        detections = self.detector.detect(
            gray_detect,
            estimate_tag_pose=True,
            camera_params=(fx * scale, fy * scale, cx * scale, cy * scale),
            tag_size=self.args.tag_size,
        )
        usable = []
        for d in detections:
            if d.tag_id != self.args.tag_id:
                continue
            if d.hamming != 0 or float(d.decision_margin) < self.args.min_margin:
                continue
            usable.append(d)
        if not usable:
            return

        best = max(usable, key=lambda d: float(d.decision_margin))
        t = np.asarray(best.pose_t, dtype=np.float64).reshape(3)
        self.samples.append((t, float(best.decision_margin)))
        print(
            f'sample {len(self.samples)}/{self.args.samples}: '
            f'id={best.tag_id} margin={float(best.decision_margin):.2f} '
            f'camera_xyz=({t[0]:.6f},{t[1]:.6f},{t[2]:.6f})',
            flush=True,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image-topic', default='/g1_robot/front_camera/color/image_raw')
    parser.add_argument('--camera-info-topic', default='/g1_robot/front_camera/color/camera_info')
    parser.add_argument('--family', default='tag36h11')
    parser.add_argument('--tag-id', type=int, default=7)
    parser.add_argument('--tag-size', type=float, default=0.08)
    parser.add_argument('--baseline-camera-xyz', type=parse_vec3, default=parse_vec3('0.0541,-0.1319,0.4130'))
    parser.add_argument('--tolerance-m', type=float, default=0.03)
    parser.add_argument('--samples', type=int, default=8)
    parser.add_argument('--timeout-s', type=float, default=20.0)
    parser.add_argument('--min-margin', type=float, default=25.0)
    parser.add_argument('--detect-scale', type=float, default=1.0)
    parser.add_argument('--quad-decimate', type=float, default=1.0)
    args = parser.parse_args()

    rclpy.init()
    node = AnchorChecker(args)
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline and len(node.samples) < args.samples:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if not node.samples:
        print('FAIL: no accepted AprilTag samples')
        raise SystemExit(2)

    positions = np.asarray([item[0] for item in node.samples], dtype=np.float64)
    margins = np.asarray([item[1] for item in node.samples], dtype=np.float64)
    avg = positions.mean(axis=0)
    std = positions.std(axis=0)
    err = avg - args.baseline_camera_xyz
    err_norm = float(np.linalg.norm(err))
    passed = err_norm <= args.tolerance_m

    print('--- result ---')
    print(f'tag_id: {args.tag_id}')
    print(f'samples: {len(node.samples)}')
    print(f'margin_avg: {float(margins.mean()):.2f}')
    print(f'baseline_camera_xyz: ({args.baseline_camera_xyz[0]:.6f},{args.baseline_camera_xyz[1]:.6f},{args.baseline_camera_xyz[2]:.6f})')
    print(f'current_camera_xyz:  ({avg[0]:.6f},{avg[1]:.6f},{avg[2]:.6f})')
    print(f'std_camera_xyz:      ({std[0]:.6f},{std[1]:.6f},{std[2]:.6f})')
    print(f'error_xyz:           ({err[0]:.6f},{err[1]:.6f},{err[2]:.6f})')
    print(f'error_norm_m: {err_norm:.6f}')
    print(f'tolerance_m: {args.tolerance_m:.6f}')
    print('PASS' if passed else 'FAIL')
    raise SystemExit(0 if passed else 1)


if __name__ == '__main__':
    main()