#!/usr/bin/env python3
"""
Localization drift monitor.

Monitors /localization_3d_confidence. When ICP fitness drops below
threshold for consecutive readings, warns the user and optionally
cancels active navigation to prevent unrecoverable drift.

Usage:
  ros2 run bot_navigation localization_monitor.py
  ros2 run bot_navigation localization_monitor.py --ros-args -p auto_cancel:=true
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')

        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('consecutive_count', 5)
        self.declare_parameter('auto_cancel', False)
        self.declare_parameter('robot', 'g1_robot')

        self.threshold = self.get_parameter('confidence_threshold').value
        self.count_limit = self.get_parameter('consecutive_count').value
        self.auto_cancel = self.get_parameter('auto_cancel').value
        self.robot = self.get_parameter('robot').value

        self.low_count = 0
        self._last_alert_time = 0.0
        self._alert_cooldown_sec = 30.0

        self.create_subscription(Float32, '/localization_3d_confidence', self._cb, 10)

        if self.auto_cancel:
            from action_msgs.srv import CancelGoal
            self._cancel_cli = self.create_client(
                CancelGoal,
                f'/{self.robot}/navigate_to_pose/_action/cancel_goal'
            )

        self.get_logger().info(
            f'Localization monitor started: '
            f'warn_threshold={self.threshold}, warn_count={self.count_limit}, '
            f'auto_cancel={self.auto_cancel}'
        )

    def _cb(self, msg: Float32):
        confidence = msg.data

        if confidence < self.threshold:
            self.low_count += 1
            self.get_logger().warn(
                f'Low ICP confidence: {confidence:.3f}  '
                f'({self.low_count}/{self.count_limit})',
                throttle_duration_sec=1.0
            )
            if self.low_count >= self.count_limit:
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - self._last_alert_time >= self._alert_cooldown_sec:
                    self.get_logger().error(
                        f'ICP DRIFT DETECTED! confidence={confidence:.3f} < {self.threshold} '
                        f'for {self.count_limit} consecutive readings.\n'
                        f'  Action required: re-publish /initialpose in Foxglove to re-localize.'
                    )
                    if self.auto_cancel:
                        self._cancel_navigation()
                    self._last_alert_time = now
                self.low_count = 0
        else:
            if self.low_count > 0:
                self.get_logger().info(f'ICP confidence recovered: {confidence:.3f}')
            self.low_count = 0

    def _cancel_navigation(self):
        from action_msgs.srv import CancelGoal
        if not self._cancel_cli.wait_for_service(timeout_sec=0.5):
            self.get_logger().warn('Cancel service unavailable')
            return
        req = CancelGoal.Request()  # empty = cancel all goals
        self._cancel_cli.call_async(req)
        self.get_logger().warn('Navigation cancelled due to ICP drift.')


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
