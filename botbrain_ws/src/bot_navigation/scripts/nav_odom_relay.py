#!/usr/bin/env python3
"""Publish a Nav2 odometry message whose pose and TF share FAST-LIO state."""

import copy
import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def _planar_quaternion(quaternion):
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
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


class NavOdomRelay(Node):
    def __init__(self):
        super().__init__('nav_odom_relay')
        self.declare_parameter('pose_topic', '/Odometry_loc')
        self.declare_parameter('twist_topic', '/g1_robot/odom')
        self.declare_parameter('output_topic', '/g1_robot/nav_odom')
        self.declare_parameter('output_frame', 'g1_robot/odom')
        self.declare_parameter('child_frame', 'g1_robot/base_footprint')
        self.declare_parameter('twist_timeout_sec', 0.5)

        pose_topic = str(self.get_parameter('pose_topic').value)
        twist_topic = str(self.get_parameter('twist_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.output_frame = str(
            self.get_parameter('output_frame').value).lstrip('/')
        self.child_frame = str(
            self.get_parameter('child_frame').value).lstrip('/')
        self.twist_timeout = max(
            0.05, float(self.get_parameter('twist_timeout_sec').value))

        self._last_twist = None
        self._last_twist_receive = None
        self._last_twist_stamp = None
        self._last_stale_warning = -math.inf
        self._announced_ready = False
        self._publisher = self.create_publisher(Odometry, output_topic, 20)
        self.create_subscription(
            Odometry, twist_topic, self._twist_callback, 20)
        self.create_subscription(Odometry, pose_topic, self._pose_callback, 20)
        self.get_logger().info(
            f'Nav odom relay: pose={pose_topic} twist={twist_topic} '
            f'output={output_topic}')

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
        quaternion = _planar_quaternion(msg.pose.pose.orientation)
        if quaternion is None:
            self.get_logger().warning(
                'Ignoring FAST-LIO odometry with invalid quaternion')
            return
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
            if not self._announced_ready:
                self.get_logger().info(
                    'Publishing coherent planar nav odometry with fresh '
                    'Unitree twist')
                self._announced_ready = True
        else:
            output.twist.covariance[0] = 1e3
            output.twist.covariance[7] = 1e3
            output.twist.covariance[35] = 1e3
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
