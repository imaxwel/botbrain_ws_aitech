#!/usr/bin/env python3
"""
Relay: /initialpose -> z correction -> /initialpose_corrected
Foxglove/RViz 2D tools send z=0; this node replaces z with ref_z (default 1.247).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPoseZFix(Node):
    def __init__(self):
        super().__init__('initialpose_z_fix')
        self.declare_parameter('ref_z', 1.247)
        self._ref_z = self.get_parameter('ref_z').value
        self._pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose_corrected', 10)
        self.create_subscription(PoseWithCovarianceStamped, 'initialpose', self._cb, 10)
        self.get_logger().info(f'initialpose_z_fix ready — ref_z={self._ref_z}')

    def _cb(self, msg: PoseWithCovarianceStamped) -> None:
        frame_id = msg.header.frame_id.lstrip('/')
        if frame_id != 'map':
            self.get_logger().warning(
                f"ignoring /initialpose in frame '{msg.header.frame_id}'; "
                "set Foxglove Fixed Frame to 'map'")
            return
        if abs(msg.pose.pose.position.z) < 0.5:   # 2D tool sent z≈0
            msg.pose.pose.position.z = self._ref_z
        self._pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(InitialPoseZFix())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
