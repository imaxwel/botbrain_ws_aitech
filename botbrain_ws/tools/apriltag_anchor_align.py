#!/usr/bin/env python3
"""Independent AprilTag anchor alignment controller.

This tool is intentionally outside any ROS package launch flow. It subscribes
to the front RGB camera, detects one AprilTag, compares it with a recorded
anchor observation, and optionally publishes a small Twist command so the robot
body moves until the tag returns to the recorded image position.

Default mode is dry-run: no motion is published unless --enable-motion is set.
"""

import argparse
import math
import time
from dataclasses import dataclass

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from pupil_apriltags import Detector
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


def parse_vec2(text):
    values = [float(v.strip()) for v in text.split(",")]
    if len(values) != 2:
        raise argparse.ArgumentTypeError("expected u,v")
    return np.asarray(values, dtype=np.float64)


def parse_vec3(text):
    values = [float(v.strip()) for v in text.split(",")]
    if len(values) != 3:
        raise argparse.ArgumentTypeError("expected x,y,z")
    return np.asarray(values, dtype=np.float64)


def clamp(value, limit):
    return max(-limit, min(limit, value))


@dataclass
class DetectionSample:
    center: np.ndarray
    camera_xyz: np.ndarray
    margin: float


class AprilTagAnchorAlign(Node):
    def __init__(self, args):
        super().__init__("apriltag_anchor_align")
        self.args = args
        self.bridge = CvBridge()
        self.camera_info = None
        self.last_stamp = None
        self.latest = None
        self.last_detection_time = 0.0
        self.active_motion_s = 0.0
        self.step_count = 0
        self.motion_until = 0.0
        self.settle_until = 0.0
        self.active_cmd = Twist()
        self.stable_count = 0
        self.done = False
        self.succeeded = False
        self.failure_reason = "timed out before anchor alignment reached"

        self.detector = Detector(
            families=args.family,
            nthreads=1,
            quad_decimate=args.quad_decimate,
            refine_edges=1,
        )
        self.create_subscription(CameraInfo, args.camera_info_topic, self.info_cb, 10)
        self.create_subscription(Image, args.image_topic, self.image_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, args.cmd_topic, 10)
        self.timer = self.create_timer(1.0 / args.control_hz, self.control_cb)

        if args.enable_motion:
            self.get_logger().warn(
                f"motion enabled: publishing Twist to {args.cmd_topic}; "
                "keep an emergency stop ready"
            )
        else:
            self.get_logger().info("dry-run mode: no Twist will be published")

    def info_cb(self, msg):
        self.camera_info = msg

    def image_cb(self, msg):
        if self.camera_info is None or self.done:
            return

        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self.last_stamp:
            return
        self.last_stamp = stamp

        bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
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

        candidates = []
        for detection in detections:
            if detection.tag_id != self.args.tag_id:
                continue
            if detection.hamming != 0:
                continue
            if float(detection.decision_margin) < self.args.min_margin:
                continue
            candidates.append(detection)

        if not candidates:
            return

        best = max(candidates, key=lambda item: float(item.decision_margin))
        center = np.asarray(best.center, dtype=np.float64) / scale
        camera_xyz = np.asarray(best.pose_t, dtype=np.float64).reshape(3)
        self.latest = DetectionSample(
            center=center,
            camera_xyz=camera_xyz,
            margin=float(best.decision_margin),
        )
        self.last_detection_time = time.monotonic()

    def control_cb(self):
        if self.done:
            return

        now = time.monotonic()
        if self.args.enable_motion and now < self.motion_until:
            self.cmd_pub.publish(self.active_cmd)
            return
        if self.args.enable_motion and now < self.settle_until:
            self.publish_stop()
            return

        if self.latest is None or now - self.last_detection_time > self.args.detection_stale_s:
            self.stable_count = 0
            if self.args.enable_motion:
                self.publish_stop()
            self.get_logger().warn("waiting for accepted AprilTag detection")
            return

        sample = self.latest
        pixel_error = sample.center - self.args.baseline_center
        pose_error = sample.camera_xyz - self.args.baseline_camera_xyz
        u_error = float(pixel_error[0])
        v_error = float(pixel_error[1])
        x_error = float(pose_error[0])
        z_error = float(pose_error[2])

        cmd = self.compute_command(u_error, x_error, z_error)

        aligned = (
            abs(u_error) <= self.args.pixel_tolerance
            and abs(z_error) <= self.args.z_tolerance
            and (not self.args.enable_lateral or abs(x_error) <= self.args.x_tolerance)
        )
        if aligned:
            self.stable_count += 1
        else:
            self.stable_count = 0

        self.get_logger().info(
            "tag=%d margin=%.1f center=(%.1f,%.1f) "
            "pixel_err=(%.1f,%.1f) camera_xyz=(%.4f,%.4f,%.4f) "
            "pose_err=(%.4f,%.4f,%.4f) cmd=(%.3f,%.3f,%.3f) "
            "step=%d/%d active=%.1f/%.1f stable=%d/%d"
            % (
                self.args.tag_id,
                sample.margin,
                sample.center[0],
                sample.center[1],
                u_error,
                v_error,
                sample.camera_xyz[0],
                sample.camera_xyz[1],
                sample.camera_xyz[2],
                pose_error[0],
                pose_error[1],
                pose_error[2],
                cmd.linear.x,
                cmd.linear.y,
                cmd.angular.z,
                self.step_count,
                self.args.max_steps,
                self.active_motion_s,
                self.args.max_active_motion_s,
                self.stable_count,
                self.args.required_stable_count,
            )
        )

        if self.stable_count >= self.args.required_stable_count:
            self.publish_stop()
            self.done = True
            self.succeeded = True
            self.get_logger().info("anchor alignment reached")
            return

        if self.args.enable_motion:
            if self.step_count >= self.args.max_steps:
                self.publish_stop()
                self.done = True
                self.failure_reason = (
                    "step limit reached before anchor alignment; "
                    "check the latest logged error before rerunning"
                )
                self.get_logger().error(
                    "stopped after %d movement steps without reaching anchor"
                    % self.args.max_steps
                )
                return
            if self.active_motion_s + self.args.move_burst_s > self.args.max_active_motion_s:
                self.publish_stop()
                self.done = True
                self.failure_reason = (
                    "active motion limit reached before anchor alignment; "
                    "check clearance before rerunning"
                )
                self.get_logger().error(
                    "stopped after %.1fs active motion limit; rerun only after checking clearance"
                    % self.args.max_active_motion_s
                )
                return
            self.step_count += 1
            self.active_motion_s += self.args.move_burst_s
            self.active_cmd = cmd
            self.motion_until = now + self.args.move_burst_s
            self.settle_until = self.motion_until + self.args.settle_s
            self.cmd_pub.publish(cmd)

    def compute_command(self, u_error, x_error, z_error):
        # Choose one dominant axis per burst so each move is easy to observe and
        # can be stopped before compounding a wrong sign near obstacles.
        candidates = []
        if abs(u_error) > self.args.pixel_deadband and self.args.max_angular_z > 0.0:
            candidates.append(("yaw", abs(u_error) / max(self.args.pixel_tolerance, 1e-6)))
        if (
            self.args.enable_lateral
            and abs(x_error) > self.args.x_deadband
            and self.args.max_linear_y > 0.0
        ):
            candidates.append(("lateral", abs(x_error) / max(self.args.x_tolerance, 1e-6)))
        if abs(z_error) > self.args.z_deadband and self.args.max_linear_x > 0.0:
            candidates.append(("forward", abs(z_error) / max(self.args.z_tolerance, 1e-6)))

        cmd = Twist()
        if not candidates:
            return cmd

        axis = max(candidates, key=lambda item: item[1])[0]
        if axis == "yaw":
            sign = -1.0 if not self.args.invert_yaw else 1.0
            cmd.angular.z = self.limit_with_min(
                sign * self.args.kp_u * u_error,
                self.args.max_angular_z,
                self.args.min_angular_z,
            )
        elif axis == "lateral":
            sign = -1.0 if not self.args.invert_lateral else 1.0
            cmd.linear.y = self.limit_with_min(
                sign * self.args.kp_x * x_error,
                self.args.max_linear_y,
                self.args.min_linear_y,
            )
        elif axis == "forward":
            sign = 1.0 if not self.args.invert_forward else -1.0
            cmd.linear.x = self.limit_with_min(
                sign * self.args.kp_z * z_error,
                self.args.max_linear_x,
                self.args.min_linear_x,
            )
        return cmd

    @staticmethod
    def limit_with_min(value, max_abs, min_abs):
        if max_abs <= 0.0:
            return 0.0
        limited = clamp(value, max_abs)
        if min_abs <= 0.0 or abs(limited) >= min_abs:
            return limited
        return math.copysign(min(min_abs, max_abs), value)

    def publish_stop(self):
        if self.args.enable_motion:
            self.cmd_pub.publish(Twist())


def build_arg_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image-topic", default="/g1_robot/front_camera/color/image_raw")
    parser.add_argument("--camera-info-topic", default="/g1_robot/front_camera/color/camera_info")
    parser.add_argument("--cmd-topic", default="/g1_robot/cmd_vel_rosa")
    parser.add_argument("--family", default="tag36h11")
    parser.add_argument("--tag-id", type=int, default=7)
    parser.add_argument("--tag-size", type=float, default=0.08)
    parser.add_argument("--baseline-center", type=parse_vec2, default=parse_vec2("401.5,63.3"))
    parser.add_argument(
        "--baseline-camera-xyz",
        type=parse_vec3,
        default=parse_vec3("0.0541,-0.1319,0.4130"),
    )
    parser.add_argument("--pixel-tolerance", type=float, default=10.0)
    parser.add_argument("--z-tolerance", type=float, default=0.015)
    parser.add_argument("--x-tolerance", type=float, default=0.015)
    parser.add_argument("--required-stable-count", type=int, default=8)
    parser.add_argument("--timeout-s", type=float, default=20.0)
    parser.add_argument("--control-hz", type=float, default=5.0)
    parser.add_argument("--detection-stale-s", type=float, default=0.7)
    parser.add_argument("--min-margin", type=float, default=25.0)
    parser.add_argument("--detect-scale", type=float, default=1.0)
    parser.add_argument("--quad-decimate", type=float, default=1.0)

    parser.add_argument("--kp-u", type=float, default=0.0010)
    parser.add_argument("--kp-z", type=float, default=0.25)
    parser.add_argument("--kp-x", type=float, default=0.15)
    parser.add_argument("--pixel-deadband", type=float, default=4.0)
    parser.add_argument("--z-deadband", type=float, default=0.006)
    parser.add_argument("--x-deadband", type=float, default=0.006)
    parser.add_argument("--max-linear-x", type=float, default=0.01)
    parser.add_argument("--max-linear-y", type=float, default=0.008)
    parser.add_argument("--max-angular-z", type=float, default=0.06)
    parser.add_argument("--min-linear-x", type=float, default=0.0)
    parser.add_argument("--min-linear-y", type=float, default=0.0)
    parser.add_argument("--min-angular-z", type=float, default=0.0)
    parser.add_argument("--max-active-motion-s", type=float, default=3.0)
    parser.add_argument("--move-burst-s", type=float, default=0.6)
    parser.add_argument("--settle-s", type=float, default=0.6)
    parser.add_argument("--max-steps", type=int, default=8)

    parser.add_argument("--enable-motion", action="store_true")
    parser.add_argument("--enable-lateral", action="store_true")
    parser.add_argument("--invert-yaw", action="store_true")
    parser.add_argument("--invert-lateral", action="store_true")
    parser.add_argument("--invert-forward", action="store_true")
    return parser


def main():
    args = build_arg_parser().parse_args()
    rclpy.init()
    node = AprilTagAnchorAlign(args)
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and time.monotonic() < deadline and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()

    if node.succeeded:
        raise SystemExit(0)
    print("FAIL: " + node.failure_reason)
    raise SystemExit(1)


if __name__ == "__main__":
    main()
