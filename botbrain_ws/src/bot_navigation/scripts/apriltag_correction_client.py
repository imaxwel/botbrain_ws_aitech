#!/usr/bin/env python3
"""
Simple client to trigger AprilTag navigation correction
Usage: ros2 run bot_navigation apriltag_correction_client.py
"""

import rclpy
from rclpy.node import Node
import sys


class AprilTagCorrectionClient(Node):
    def __init__(self):
        super().__init__('apriltag_correction_client')

    def trigger_navigation(self, x, y, yaw):
        """Trigger navigation to goal with AprilTag correction"""
        # Import here to access the node
        from apriltag_nav_correction import AprilTagNavCorrection

        correction_node = AprilTagNavCorrection()
        success = correction_node.navigate_to_goal(x, y, yaw)

        if success:
            self.get_logger().info(f'Navigation triggered to ({x}, {y}, {yaw})')
        else:
            self.get_logger().error('Failed to trigger navigation')


def main(args=None):
    rclpy.init(args=args)

    # Example usage: navigate to (2.0, 1.0) with 0 yaw
    client = AprilTagCorrectionClient()
    client.trigger_navigation(2.0, 1.0, 0.0)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
