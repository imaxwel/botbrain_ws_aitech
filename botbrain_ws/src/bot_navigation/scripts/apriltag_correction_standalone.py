#!/usr/bin/env python3
"""
独立的AprilTag位置修正节点 - 用于测试
不依赖导航，可直接启动进行位置修正测试
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from std_srvs.srv import SetBool
import math


class AprilTagCorrectionStandalone(Node):
    def __init__(self):
        super().__init__('apriltag_correction_standalone')

        # 参数
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('kp_linear', 0.3)
        self.declare_parameter('kp_angular', 0.5)
        self.declare_parameter('position_tolerance', 0.05)
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 0.3)

        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.kp_linear = self.get_parameter('kp_linear').value
        self.kp_angular = self.get_parameter('kp_angular').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.max_linear = self.get_parameter('max_linear_vel').value
        self.max_angular = self.get_parameter('max_angular_vel').value

        # 状态
        self.enabled = False
        self.tag_detected = False
        self.tag_pose = None

        # 订阅AprilTag检测
        self._tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.tag_callback,
            10
        )

        # 发布速度命令
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 启动/停止服务
        self._enable_srv = self.create_service(
            SetBool,
            'enable_correction',
            self.enable_callback
        )

        # 控制循环定时器
        self._timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('AprilTag独立修正节点已启动')
        self.get_logger().info('使用服务启动: ros2 service call /enable_correction std_srvs/srv/SetBool "{data: true}"')

    def tag_callback(self, msg: AprilTagDetectionArray):
        """处理AprilTag检测"""
        self.tag_detected = False
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                self.tag_detected = True
                self.tag_pose = detection.pose.pose.pose
                break

    def enable_callback(self, request, response):
        """启动/停止修正服务"""
        self.enabled = request.data
        if self.enabled:
            self.get_logger().info('AprilTag位置修正已启动')
            response.message = '修正已启动'
        else:
            self.get_logger().info('AprilTag位置修正已停止')
            self.stop_robot()
            response.message = '修正已停止'
        response.success = True
        return response

    def control_loop(self):
        """视觉伺服控制循环"""
        if not self.enabled:
            return

        if not self.tag_detected or self.tag_pose is None:
            self.get_logger().warn('未检测到AprilTag', throttle_duration_sec=2.0)
            self.stop_robot()
            return

        # 计算误差（tag_pose在相机坐标系）
        x_error = self.tag_pose.position.z  # 前向距离
        y_error = -self.tag_pose.position.x  # 横向偏移

        # 检查是否达到精度
        if abs(x_error) < self.pos_tol and abs(y_error) < self.pos_tol:
            self.get_logger().info('已达到目标位置精度')
            self.stop_robot()
            self.enabled = False
            return

        # 视觉伺服控制
        cmd = Twist()
        cmd.linear.x = self.kp_linear * x_error
        cmd.angular.z = self.kp_angular * y_error

        # 速度限制
        cmd.linear.x = max(-self.max_linear, min(self.max_linear, cmd.linear.x))
        cmd.angular.z = max(-self.max_angular, min(self.max_angular, cmd.angular.z))

        self._cmd_vel_pub.publish(cmd)
        self.get_logger().info(f'误差: x={x_error:.3f}m, y={y_error:.3f}m', throttle_duration_sec=1.0)

    def stop_robot(self):
        """停止机器人"""
        self._cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagCorrectionStandalone()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
