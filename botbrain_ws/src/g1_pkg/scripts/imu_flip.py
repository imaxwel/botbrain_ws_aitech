#!/usr/bin/env python3
"""Rotate the upside-down MID360 IMU into the corrected LiDAR axes.

MID360 is mounted roll-180 (upside-down). The Livox SDK rotates the
pointcloud via MID360_config.json extrinsic, but publishes IMU in the raw
sensor frame. Negating Y and Z applies R_x(pi), bringing the IMU vectors
into the same axes as the corrected point cloud. FAST-LIO can then keep
extrinsic_R = identity.

The incoming timestamp and frame id are preserved. FAST-LIO only consumes
angular_velocity and linear_acceleration from this message.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


INPUT_TOPIC = '/livox/imu'
OUTPUT_TOPIC = '/livox/imu_corrected'
DIAGNOSTIC_SAMPLE_COUNT = 200


class ImuFlip(Node):
    def __init__(self):
        super().__init__('imu_flip')

        # Keep RELIABLE QoS compatible with FAST-LIO's default IMU subscriber.
        imu_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.pub = self.create_publisher(Imu, OUTPUT_TOPIC, imu_qos)
        self.sub = self.create_subscription(Imu, INPUT_TOPIC, self._cb, imu_qos)

        self.sample_count = 0
        self.acc_sum = [0.0, 0.0, 0.0]
        self.get_logger().info(
            f'IMU axis correction active: {INPUT_TOPIC} -> {OUTPUT_TOPIC}; '
            'applying R_x(pi): x unchanged, y/z negated')

    def _cb(self, msg: Imu):
        msg.angular_velocity.y *= -1.0
        msg.angular_velocity.z *= -1.0
        msg.linear_acceleration.y *= -1.0
        msg.linear_acceleration.z *= -1.0

        if self.sample_count < DIAGNOSTIC_SAMPLE_COUNT:
            self.sample_count += 1
            self.acc_sum[0] += msg.linear_acceleration.x
            self.acc_sum[1] += msg.linear_acceleration.y
            self.acc_sum[2] += msg.linear_acceleration.z
            if self.sample_count == DIAGNOSTIC_SAMPLE_COUNT:
                mean = [value / self.sample_count for value in self.acc_sum]
                norm = sum(value * value for value in mean) ** 0.5
                self.get_logger().info(
                    'first 200 corrected IMU samples: '
                    f'mean_acc=[{mean[0]:+.3f}, {mean[1]:+.3f}, {mean[2]:+.3f}] '
                    f'|a|={norm:.3f} m/s^2; keep the robot stationary until '
                    'FAST-LIO prints "IMU Initial Done"')

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuFlip()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
