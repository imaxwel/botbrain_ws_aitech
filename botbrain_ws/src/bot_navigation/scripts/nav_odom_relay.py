#!/usr/bin/env python3
"""Publish a Nav2 odometry message whose pose and TF share FAST-LIO state."""

import copy
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _yaw_from_quaternion(quaternion):
    norm = math.sqrt(
        quaternion.x * quaternion.x + quaternion.y * quaternion.y +
        quaternion.z * quaternion.z + quaternion.w * quaternion.w)
    if not math.isfinite(norm) or norm < 1e-9:
        return None
    x = quaternion.x / norm
    y = quaternion.y / norm
    z = quaternion.z / norm
    w = quaternion.w / norm
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return yaw


def _planar_quaternion(quaternion):
    yaw = _yaw_from_quaternion(quaternion)
    if yaw is None:
        return None
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _shortest_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class NavOdomRelay(Node):
    def __init__(self):
        super().__init__('nav_odom_relay')
        self.declare_parameter('pose_topic', '/Odometry_loc')
        self.declare_parameter('twist_topic', '/g1_robot/odom')
        self.declare_parameter('output_topic', '/g1_robot/nav_odom')
        self.declare_parameter('output_frame', 'g1_robot/odom')
        self.declare_parameter('child_frame', 'g1_robot/base_footprint')
        self.declare_parameter('twist_timeout_sec', 0.5)
        self.declare_parameter('derive_twist_from_pose', True)
        self.declare_parameter('derived_twist_min_dt_sec', 0.02)
        self.declare_parameter('derived_twist_max_dt_sec', 0.5)
        self.declare_parameter('max_derived_linear_speed', 2.0)
        self.declare_parameter('max_derived_angular_speed', 3.0)

        pose_topic = str(self.get_parameter('pose_topic').value)
        twist_topic = str(self.get_parameter('twist_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.output_frame = str(
            self.get_parameter('output_frame').value).lstrip('/')
        self.child_frame = str(
            self.get_parameter('child_frame').value).lstrip('/')
        self.twist_timeout = max(
            0.05, float(self.get_parameter('twist_timeout_sec').value))
        self.derive_twist_from_pose = bool(
            self.get_parameter('derive_twist_from_pose').value)
        self.derived_twist_min_dt = max(
            0.001, float(self.get_parameter('derived_twist_min_dt_sec').value))
        self.derived_twist_max_dt = max(
            self.derived_twist_min_dt,
            float(self.get_parameter('derived_twist_max_dt_sec').value))
        self.max_derived_linear_speed = max(
            0.1, float(self.get_parameter('max_derived_linear_speed').value))
        self.max_derived_angular_speed = max(
            0.1, float(self.get_parameter('max_derived_angular_speed').value))

        self._last_twist = None
        self._last_twist_receive = None
        self._last_twist_stamp = None
        self._last_pose_sample = None
        self._last_stale_warning = -math.inf
        self._twist_source = None
        self._publisher = self.create_publisher(Odometry, output_topic, 20)
        self.create_subscription(
            Odometry, twist_topic, self._twist_callback, 20)
        self.create_subscription(Odometry, pose_topic, self._pose_callback, 20)
        self.get_logger().info(
            f'Nav odom relay: pose={pose_topic} twist={twist_topic} '
            f'output={output_topic}')

    def _announce_twist_source(self, source):
        if source == self._twist_source:
            return
        if source == 'unitree':
            self.get_logger().info(
                'Nav odometry is using fresh Unitree planar twist')
        elif source == 'fast_lio_pose':
            self.get_logger().warning(
                'Unitree twist is unavailable; deriving planar twist from '
                'successive FAST-LIO poses')
        else:
            self.get_logger().warning(
                'No valid odometry twist source; publishing zero twist with '
                'high covariance')
        self._twist_source = source

    def _derive_pose_twist(self, x, y, yaw, stamp):
        current = (x, y, yaw, stamp)
        previous = self._last_pose_sample
        self._last_pose_sample = current
        if not self.derive_twist_from_pose or previous is None:
            return None

        prev_x, prev_y, prev_yaw, prev_stamp = previous
        dt = stamp - prev_stamp
        if not self.derived_twist_min_dt <= dt <= self.derived_twist_max_dt:
            return None

        velocity_world_x = (x - prev_x) / dt
        velocity_world_y = (y - prev_y) / dt
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        velocity_body_x = (
            cos_yaw * velocity_world_x + sin_yaw * velocity_world_y)
        velocity_body_y = (
            -sin_yaw * velocity_world_x + cos_yaw * velocity_world_y)
        yaw_rate = _shortest_angle(yaw - prev_yaw) / dt
        if not all(math.isfinite(value) for value in (
                velocity_body_x, velocity_body_y, yaw_rate)):
            return None
        if (math.hypot(velocity_body_x, velocity_body_y) >
                self.max_derived_linear_speed or
                abs(yaw_rate) > self.max_derived_angular_speed):
            return None
        return velocity_body_x, velocity_body_y, yaw_rate

    def _twist_callback(self, msg):
        twist = msg.twist.twist
        if (msg.child_frame_id.lstrip('/') != self.child_frame or
                not all(math.isfinite(value) for value in (
                    twist.linear.x, twist.linear.y, twist.angular.z
                ))):
            now = time.monotonic()
            if now - self._last_stale_warning >= 5.0:
                self.get_logger().warning(
                    'Ignoring Unitree odometry with a mismatched child frame '
                    'or non-finite planar twist')
                self._last_stale_warning = now
            return
        self._last_twist = copy.deepcopy(msg.twist)
        self._last_twist_receive = time.monotonic()
        self._last_twist_stamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _pose_callback(self, msg):
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        if yaw is None:
            self.get_logger().warning(
                'Ignoring FAST-LIO odometry with invalid quaternion')
            return
        quaternion = (
            0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))
        if not all(math.isfinite(value) for value in (
                msg.pose.pose.position.x, msg.pose.pose.position.y)):
            self.get_logger().warning(
                'Ignoring FAST-LIO odometry with non-finite position')
            return

        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        twist_stamp_age = (
            ros_now - self._last_twist_stamp
            if self._last_twist_stamp is not None else math.inf)
        pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        derived_twist = self._derive_pose_twist(
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y), yaw, pose_stamp)
        output = Odometry()
        output.header.stamp = msg.header.stamp
        output.header.frame_id = self.output_frame
        output.child_frame_id = self.child_frame
        output.pose.pose.position.x = float(msg.pose.pose.position.x)
        output.pose.pose.position.y = float(msg.pose.pose.position.y)
        output.pose.pose.position.z = 0.0
        output.pose.pose.orientation.x = quaternion[0]
        output.pose.pose.orientation.y = quaternion[1]
        output.pose.pose.orientation.z = quaternion[2]
        output.pose.pose.orientation.w = quaternion[3]
        output.pose.covariance = list(msg.pose.covariance)

        twist_is_fresh = (
            self._last_twist is not None and
            self._last_twist_receive is not None and
            now - self._last_twist_receive <= self.twist_timeout and
            -0.25 <= twist_stamp_age <= self.twist_timeout
        )
        if twist_is_fresh:
            output.twist = copy.deepcopy(self._last_twist)
            output.twist.twist.linear.z = 0.0
            output.twist.twist.angular.x = 0.0
            output.twist.twist.angular.y = 0.0
            self._announce_twist_source('unitree')
        elif derived_twist is not None:
            output.twist.twist.linear.x = derived_twist[0]
            output.twist.twist.linear.y = derived_twist[1]
            output.twist.twist.angular.z = derived_twist[2]
            output.twist.covariance[0] = 0.10
            output.twist.covariance[7] = 0.10
            output.twist.covariance[35] = 0.20
            self._announce_twist_source('fast_lio_pose')
        else:
            output.twist.covariance[0] = 1e3
            output.twist.covariance[7] = 1e3
            output.twist.covariance[35] = 1e3
            self._announce_twist_source('none')
            if now - self._last_stale_warning >= 5.0:
                self.get_logger().warning(
                    'Unitree odometry twist is missing or stale; publishing '
                    'zero nav twist')
                self._last_stale_warning = now

        self._publisher.publish(output)


def main():
    rclpy.init()
    node = NavOdomRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
