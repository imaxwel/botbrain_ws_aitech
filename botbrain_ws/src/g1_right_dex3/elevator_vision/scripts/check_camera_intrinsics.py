#!/usr/bin/env python3
"""查看D435i相机内参"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class IntrinsicsChecker(Node):
    def __init__(self):
        super().__init__('intrinsics_checker')
        self.create_subscription(
            CameraInfo,
            '/camera/realsense2_camera/color/camera_info',
            self._callback,
            10
        )
        self.get_logger().info('等待相机内参...')

    def _callback(self, msg):
        k = msg.k
        print(f"\n相机内参:")
        print(f"  fx = {k[0]:.2f}")
        print(f"  fy = {k[4]:.2f}")
        print(f"  cx = {k[2]:.2f}")
        print(f"  cy = {k[5]:.2f}")
        print(f"  分辨率: {msg.width}x{msg.height}\n")
        raise SystemExit(0)

def main():
    rclpy.init()
    node = IntrinsicsChecker()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
