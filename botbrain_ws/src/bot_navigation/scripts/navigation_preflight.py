#!/usr/bin/env python3
"""Wait for fresh obstacle data and a verified map localization before Nav2."""

import math
import sys
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Float32
from tf2_ros import Buffer, TransformException, TransformListener


def _roll_pitch(transform: TransformStamped):
    q = transform.transform.rotation
    norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    x, y, z, w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(sin_pitch)


class NavigationPreflight(Node):
    def __init__(self):
        super().__init__('navigation_preflight')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('ready_topic', '/localization_ready')
        self.declare_parameter(
            'confidence_topic', '/localization_3d_confidence')
        self.declare_parameter('twist_odom_topic', '/g1_robot/odom')
        self.declare_parameter('pose_odom_topic', '/Odometry_loc')
        self.declare_parameter('allow_pose_derived_twist', True)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'g1_robot/base_footprint')
        self.declare_parameter('timeout_sec', 300.0)
        self.declare_parameter('max_scan_age_sec', 1.0)
        self.declare_parameter('max_confidence_age_sec', 1.0)
        self.declare_parameter('max_twist_odom_age_sec', 0.5)
        self.declare_parameter('max_pose_odom_age_sec', 0.5)
        self.declare_parameter('derived_twist_min_dt_sec', 0.02)
        self.declare_parameter('max_derived_linear_speed', 2.0)
        self.declare_parameter('max_derived_angular_speed', 3.0)
        self.declare_parameter('min_confidence', 0.55)
        self.declare_parameter('max_base_height_error', 0.20)
        self.declare_parameter('max_base_tilt_deg', 5.0)

        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.ready_topic = str(self.get_parameter('ready_topic').value)
        self.confidence_topic = str(
            self.get_parameter('confidence_topic').value)
        self.twist_odom_topic = str(
            self.get_parameter('twist_odom_topic').value)
        self.pose_odom_topic = str(
            self.get_parameter('pose_odom_topic').value)
        self.allow_pose_derived_twist = bool(
            self.get_parameter('allow_pose_derived_twist').value)
        self.map_frame = str(
            self.get_parameter('map_frame').value).lstrip('/')
        self.base_frame = str(
            self.get_parameter('base_frame').value).lstrip('/')
        self.timeout_sec = max(
            0.0, float(self.get_parameter('timeout_sec').value))
        self.max_scan_age = max(
            0.1, float(self.get_parameter('max_scan_age_sec').value))
        self.max_confidence_age = max(
            0.1, float(self.get_parameter('max_confidence_age_sec').value))
        self.max_twist_odom_age = max(
            0.1, float(self.get_parameter('max_twist_odom_age_sec').value))
        self.max_pose_odom_age = max(
            0.1, float(self.get_parameter('max_pose_odom_age_sec').value))
        self.derived_twist_min_dt = max(
            0.001, float(self.get_parameter('derived_twist_min_dt_sec').value))
        self.max_derived_linear_speed = max(
            0.1, float(self.get_parameter('max_derived_linear_speed').value))
        self.max_derived_angular_speed = max(
            0.1, float(self.get_parameter('max_derived_angular_speed').value))
        self.min_confidence = max(
            0.0, float(self.get_parameter('min_confidence').value))
        self.max_base_height_error = max(
            0.01, float(self.get_parameter('max_base_height_error').value))
        self.max_base_tilt = math.radians(max(
            0.1, float(self.get_parameter('max_base_tilt_deg').value)))

        self._last_scan_receive = None
        self._last_scan_stamp = None
        self._last_confidence_receive = None
        self._last_twist_odom_receive = None
        self._last_twist_odom_stamp = None
        self._twist_odom_valid = False
        self._last_pose_odom_receive = None
        self._last_pose_odom_stamp = None
        self._last_pose_odom_sample = None
        self._pose_odom_valid = False
        self._confidence = 0.0
        self._localization_ready = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        ready_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_callback,
            qos_profile_sensor_data)
        self.create_subscription(
            Bool, self.ready_topic, self._ready_callback, ready_qos)
        self.create_subscription(
            Float32, self.confidence_topic, self._confidence_callback, 10)
        self.create_subscription(
            Odometry, self.twist_odom_topic, self._twist_odom_callback, 10)
        self.create_subscription(
            Odometry, self.pose_odom_topic, self._pose_odom_callback, 20)

    def _ros_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _scan_callback(self, msg):
        self._last_scan_receive = time.monotonic()
        self._last_scan_stamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _ready_callback(self, msg):
        self._localization_ready = bool(msg.data)

    def _confidence_callback(self, msg):
        self._confidence = float(msg.data)
        self._last_confidence_receive = time.monotonic()

    def _twist_odom_callback(self, msg):
        twist = msg.twist.twist
        self._twist_odom_valid = (
            msg.child_frame_id.lstrip('/') == self.base_frame and
            all(math.isfinite(value) for value in (
                twist.linear.x, twist.linear.y, twist.angular.z
            ))
        )
        self._last_twist_odom_receive = time.monotonic()
        self._last_twist_odom_stamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _pose_odom_callback(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = msg.pose.pose
        quaternion_norm = math.sqrt(
            pose.orientation.x * pose.orientation.x +
            pose.orientation.y * pose.orientation.y +
            pose.orientation.z * pose.orientation.z +
            pose.orientation.w * pose.orientation.w)
        valid_pose = (
            msg.header.frame_id.lstrip('/') == 'camera_init' and
            msg.child_frame_id.lstrip('/') == 'body' and
            all(math.isfinite(value) for value in (
                stamp, pose.position.x, pose.position.y,
                pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w,
                quaternion_norm,
            )) and
            quaternion_norm >= 1e-9
        )
        yaw = None
        if valid_pose:
            qx = pose.orientation.x / quaternion_norm
            qy = pose.orientation.y / quaternion_norm
            qz = pose.orientation.z / quaternion_norm
            qw = pose.orientation.w / quaternion_norm
            yaw = math.atan2(
                2.0 * (qw * qz + qx * qy),
                1.0 - 2.0 * (qy * qy + qz * qz),
            )

        previous = self._last_pose_odom_sample
        dt = stamp - previous[3] if previous is not None else math.inf
        valid_interval = (
            previous is not None and
            self.derived_twist_min_dt <= dt <= self.max_pose_odom_age)
        valid_motion = False
        if valid_pose and valid_interval:
            linear_speed = math.hypot(
                pose.position.x - previous[0],
                pose.position.y - previous[1]) / dt
            yaw_rate = abs(math.atan2(
                math.sin(yaw - previous[2]),
                math.cos(yaw - previous[2]))) / dt
            valid_motion = (
                math.isfinite(linear_speed) and
                math.isfinite(yaw_rate) and
                linear_speed <= self.max_derived_linear_speed and
                yaw_rate <= self.max_derived_angular_speed
            )
        if valid_pose:
            self._last_pose_odom_sample = (
                float(pose.position.x), float(pose.position.y), yaw, stamp)
        self._last_pose_odom_stamp = stamp
        self._last_pose_odom_receive = time.monotonic()
        self._pose_odom_valid = valid_pose and valid_interval and valid_motion

    @staticmethod
    def _message_is_fresh(
            receive_time, stamp, max_age, monotonic_now, ros_now):
        if receive_time is None or stamp is None:
            return False
        receive_age = monotonic_now - receive_time
        stamp_age = ros_now - stamp
        return receive_age <= max_age and -0.25 <= stamp_age <= max_age

    def _scan_is_fresh(self, monotonic_now, ros_now):
        if self._last_scan_receive is None or self._last_scan_stamp is None:
            return False
        return self._message_is_fresh(
            self._last_scan_receive, self._last_scan_stamp, self.max_scan_age,
            monotonic_now, ros_now,
        )

    def _confidence_is_fresh(self, monotonic_now):
        return (
            self._last_confidence_receive is not None and
            monotonic_now - self._last_confidence_receive <=
            self.max_confidence_age and
            self._confidence >= self.min_confidence
        )

    def _twist_odom_is_fresh(self, monotonic_now, ros_now):
        return (
            self._twist_odom_valid and
            self._message_is_fresh(
                self._last_twist_odom_receive,
                self._last_twist_odom_stamp,
                self.max_twist_odom_age,
                monotonic_now,
                ros_now,
            )
        )

    def _pose_odom_is_fresh(self, monotonic_now, ros_now):
        return (
            self.allow_pose_derived_twist and
            self._pose_odom_valid and
            self._message_is_fresh(
                self._last_pose_odom_receive,
                self._last_pose_odom_stamp,
                self.max_pose_odom_age,
                monotonic_now,
                ros_now,
            )
        )

    def _planar_map_tf_is_valid(self):
        try:
            transform = self._tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
            )
        except TransformException:
            return False

        translation = transform.transform.translation
        angles = _roll_pitch(transform)
        if angles is None:
            return False
        roll, pitch = angles
        return (
            all(math.isfinite(v) for v in (
                translation.x, translation.y, translation.z, roll, pitch)) and
            abs(translation.z) <= self.max_base_height_error and
            abs(roll) <= self.max_base_tilt and
            abs(pitch) <= self.max_base_tilt
        )

    def wait_until_ready(self):
        start = time.monotonic()
        next_log = start
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            ros_now = self._ros_now()
            scan_ok = self._scan_is_fresh(now, ros_now)
            unitree_twist_ok = self._twist_odom_is_fresh(now, ros_now)
            pose_twist_ok = self._pose_odom_is_fresh(now, ros_now)
            twist_odom_ok = unitree_twist_ok or pose_twist_ok
            twist_source = (
                'unitree' if unitree_twist_ok else
                'fast_lio_pose' if pose_twist_ok else 'none')
            confidence_ok = self._confidence_is_fresh(now)
            tf_ok = (
                self._planar_map_tf_is_valid()
                if self._localization_ready else False)
            if (scan_ok and twist_odom_ok and self._localization_ready and
                    confidence_ok and tf_ok):
                self.get_logger().info(
                    'Navigation preflight passed: fresh scan and coherent '
                    f'odometry twist_source={twist_source}, '
                    'localization ready, '
                    f'confidence={self._confidence:.3f}, planar map TF valid')
                return True

            if now >= next_log:
                self.get_logger().info(
                    'Waiting for navigation inputs: '
                    f'scan={scan_ok} twist_odom={twist_odom_ok} '
                    f'twist_source={twist_source} '
                    f'ready={self._localization_ready} '
                    f'confidence={self._confidence:.3f}/'
                    f'{self.min_confidence:.3f} '
                    f'tf={tf_ok}')
                next_log = now + 2.0

            if self.timeout_sec > 0.0 and now - start >= self.timeout_sec:
                self.get_logger().error(
                    'Navigation preflight timed out after '
                    f'{self.timeout_sec:.1f} s')
                return False
        return False


def main():
    rclpy.init()
    node = NavigationPreflight()
    success = False
    try:
        success = node.wait_until_ready()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
