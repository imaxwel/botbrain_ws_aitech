#!/usr/bin/env python3
"""Verify that a rolling OccupancyGrid is centered on the robot TF."""

import math
import sys
import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class CostmapCenterCheck(Node):
    def __init__(self):
        super().__init__('costmap_center_check')
        self.declare_parameter(
            'costmap_topic', '/g1_robot/local_costmap/costmap')
        self.declare_parameter('base_frame', 'g1_robot/base_footprint')
        self.declare_parameter('timeout_sec', 8.0)
        self.declare_parameter('tolerance_cells', 2.0)

        self.costmap_topic = str(self.get_parameter('costmap_topic').value)
        self.base_frame = str(
            self.get_parameter('base_frame').value).lstrip('/')
        self.timeout = max(0.5, float(self.get_parameter('timeout_sec').value))
        self.tolerance_cells = max(
            0.5, float(self.get_parameter('tolerance_cells').value))
        self.message = None
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, self.costmap_topic, self._callback, qos)

    def _callback(self, msg):
        self.message = msg

    def run(self):
        deadline = time.monotonic() + self.timeout
        last_tf_error = None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            msg = self.message
            if msg is not None:
                global_frame = msg.header.frame_id.lstrip('/')
                stamp_is_zero = (
                    msg.header.stamp.sec == 0 and
                    msg.header.stamp.nanosec == 0)
                if not global_frame or stamp_is_zero:
                    self.get_logger().error(
                        'OccupancyGrid requires a frame and non-zero stamp')
                    return False
                try:
                    transform = self.buffer.lookup_transform(
                        global_frame,
                        self.base_frame,
                        Time.from_msg(msg.header.stamp),
                    )
                    break
                except TransformException as error:
                    # The costmap often arrives just ahead of its matching TF.
                    # Keep spinning so the listener can fill the buffer.
                    last_tf_error = error

            if time.monotonic() >= deadline:
                if msg is None:
                    self.get_logger().error(
                        f'No OccupancyGrid received from {self.costmap_topic}')
                else:
                    self.get_logger().error(
                        f'No same-stamp TF {global_frame} <- '
                        f'{self.base_frame}: '
                        f'{last_tf_error}')
                return False
        else:
            return False

        resolution = float(msg.info.resolution)
        if (not math.isfinite(resolution) or resolution <= 0.0 or
                msg.info.width <= 0 or msg.info.height <= 0):
            self.get_logger().error('OccupancyGrid has invalid geometry')
            return False

        origin = msg.info.origin
        quaternion = origin.orientation
        norm = math.sqrt(
            quaternion.x * quaternion.x + quaternion.y * quaternion.y +
            quaternion.z * quaternion.z + quaternion.w * quaternion.w)
        if not math.isfinite(norm) or norm < 1e-9:
            self.get_logger().error(
                'OccupancyGrid origin has an invalid quaternion')
            return False
        x = quaternion.x / norm
        y = quaternion.y / norm
        z = quaternion.z / norm
        w = quaternion.w / norm
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        half_width = msg.info.width * resolution * 0.5
        half_height = msg.info.height * resolution * 0.5
        center_x = (
            origin.position.x + math.cos(yaw) * half_width -
            math.sin(yaw) * half_height)
        center_y = (
            origin.position.y + math.sin(yaw) * half_width +
            math.cos(yaw) * half_height)
        base_x = transform.transform.translation.x
        base_y = transform.transform.translation.y
        error = math.hypot(center_x - base_x, center_y - base_y)
        tolerance = self.tolerance_cells * resolution
        self.get_logger().info(
            f'costmap_frame={global_frame} '
            f'origin=({msg.info.origin.position.x:.3f}, '
            f'{msg.info.origin.position.y:.3f}) '
            f'center=({center_x:.3f}, {center_y:.3f}) '
            f'base=({base_x:.3f}, {base_y:.3f}) error={error:.3f}m '
            f'tolerance={tolerance:.3f}m')
        if error > tolerance:
            self.get_logger().error(
                'Rolling local costmap is not centered on the robot')
            return False
        return True


def main():
    rclpy.init()
    node = CostmapCenterCheck()
    success = False
    try:
        success = node.run()
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
