#!/usr/bin/env python3
"""Relay /livox/imu → /livox/imu_corrected with Y and Z axes negated.

MID360 is mounted roll-180 (upside-down). The Livox SDK rotates the
pointcloud via MID360_config.json extrinsic, but publishes IMU in the raw
sensor frame (Z-down). Negating Y and Z brings the IMU into the same
Z-up frame as the corrected pointcloud, so FAST_LIO can use
extrinsic_R = identity and produce a correctly-oriented map.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuFlip(Node):
    def __init__(self):
        super().__init__('imu_flip')
        self.pub = self.create_publisher(Imu, '/livox/imu_corrected', 10)
        self.create_subscription(Imu, '/livox/imu', self._cb, 10)

    def _cb(self, msg: Imu):
        msg.angular_velocity.y    *= -1.0
        msg.angular_velocity.z    *= -1.0
        msg.linear_acceleration.y *= -1.0
        msg.linear_acceleration.z *= -1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ImuFlip())


if __name__ == '__main__':
    main()
