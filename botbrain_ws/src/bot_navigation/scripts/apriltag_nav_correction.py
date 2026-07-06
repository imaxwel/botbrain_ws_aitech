#!/usr/bin/env python3
"""
AprilTag Navigation Correction Node
Navigates to target, then uses AprilTag detection for precise position correction.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from enum import Enum
import math

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from apriltag_msgs.msg import AprilTagDetectionArray
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose


class CorrectionState(Enum):
    IDLE = 0
    NAVIGATING = 1
    CORRECTING = 2
    COMPLETED = 3


class AprilTagNavCorrection(Node):
    def __init__(self):
        super().__init__('apriltag_nav_correction')

        self.cb_group = ReentrantCallbackGroup()

        # Parameters
        self.declare_parameter('target_tag_id', 0)
        self.declare_parameter('correction_linear_gain', 0.3)
        self.declare_parameter('correction_angular_gain', 0.5)
        self.declare_parameter('position_tolerance', 0.05)  # meters
        self.declare_parameter('angle_tolerance', 0.1)  # radians
        self.declare_parameter('max_correction_time', 30.0)  # seconds

        self.target_tag_id = self.get_parameter('target_tag_id').value
        self.kp_linear = self.get_parameter('correction_linear_gain').value
        self.kp_angular = self.get_parameter('correction_angular_gain').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.ang_tol = self.get_parameter('angle_tolerance').value
        self.max_correction_time = self.get_parameter('max_correction_time').value

        # State
        self.state = CorrectionState.IDLE
        self.tag_detected = False
        self.tag_pose = None
        self.correction_start_time = None

        # Nav2 action client
        self._nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.cb_group
        )

        # AprilTag subscriber
        self._tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            '/apriltag/detections',
            self.tag_callback,
            10,
            callback_group=self.cb_group
        )

        # Velocity publisher
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer for correction loop
        self._correction_timer = self.create_timer(
            0.1,
            self.correction_loop,
            callback_group=self.cb_group
        )

        self.get_logger().info('AprilTag Navigation Correction node initialized')

    def tag_callback(self, msg: AprilTagDetectionArray):
        """Process AprilTag detections"""
        self.tag_detected = False
        for detection in msg.detections:
            if detection.id == self.target_tag_id:
                self.tag_detected = True
                self.tag_pose = detection.pose.pose.pose
                break

    def navigate_to_goal(self, x: float, y: float, yaw: float, frame_id: str = 'map'):
        """Send navigation goal to Nav2"""
        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 action server not available')
            return False

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f'Sending navigation goal: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')
        self.state = CorrectionState.NAVIGATING

        send_goal_future = self._nav_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        """Handle navigation goal acceptance"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self.state = CorrectionState.IDLE
            return

        self.get_logger().info('Navigation goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        """Handle navigation completion"""
        result = future.result().result
        self.get_logger().info('Navigation completed, switching to AprilTag correction mode')
        self.state = CorrectionState.CORRECTING
        self.correction_start_time = self.get_clock().now()

    def correction_loop(self):
        """Visual servoing loop for AprilTag-based position correction"""
        if self.state != CorrectionState.CORRECTING:
            return

        # Check timeout
        if self.correction_start_time:
            elapsed = (self.get_clock().now() - self.correction_start_time).nanoseconds / 1e9
            if elapsed > self.max_correction_time:
                self.get_logger().warn('Correction timeout, stopping')
                self.stop_robot()
                self.state = CorrectionState.COMPLETED
                return

        if not self.tag_detected or self.tag_pose is None:
            self.get_logger().warn('AprilTag not detected, stopping', throttle_duration_sec=2.0)
            self.stop_robot()
            return

        # Calculate error (tag pose is in camera frame)
        x_error = self.tag_pose.position.z  # Forward distance
        y_error = -self.tag_pose.position.x  # Lateral offset

        # Check if within tolerance
        if abs(x_error) < self.pos_tol and abs(y_error) < self.pos_tol:
            self.get_logger().info('Position correction completed successfully')
            self.stop_robot()
            self.state = CorrectionState.COMPLETED
            return

        # Visual servoing control
        cmd = Twist()
        cmd.linear.x = self.kp_linear * x_error
        cmd.angular.z = self.kp_angular * y_error

        # Velocity limits
        cmd.linear.x = max(-0.2, min(0.2, cmd.linear.x))
        cmd.angular.z = max(-0.3, min(0.3, cmd.angular.z))

        self._cmd_vel_pub.publish(cmd)

    def stop_robot(self):
        """Send zero velocity command"""
        self._cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagNavCorrection()

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
