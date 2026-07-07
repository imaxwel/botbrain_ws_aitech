#!/usr/bin/env python3
"""
Localization drift monitor (Strategy D + auto re-anchor).

Monitors /localization_3d_confidence. When ICP fitness drops below
threshold for consecutive readings, warns the user and optionally
cancels active navigation to prevent unrecoverable drift.

Auto re-anchor (new): when confidence drops below anchor_threshold for
anchor_count consecutive readings, look up current map->base_footprint TF
and publish to /initialpose — helps FAST-LIO re-converge without manual
intervention. Fires at most once per anchor_cooldown_sec.

Usage:
  ros2 run bot_navigation localization_monitor.py
  ros2 run bot_navigation localization_monitor.py --ros-args -p auto_cancel:=true
  ros2 run bot_navigation localization_monitor.py --ros-args -p auto_anchor:=false
"""
import rclpy
import rclpy.duration
import rclpy.time
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseWithCovarianceStamped

import tf2_ros
from tf2_ros import TransformException


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')

        # --- warn / cancel parameters (existing) ---
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('consecutive_count', 5)
        self.declare_parameter('auto_cancel', False)
        self.declare_parameter('robot', 'g1_robot')

        # --- auto re-anchor parameters (new) ---
        self.declare_parameter('auto_anchor', True)
        self.declare_parameter('anchor_threshold', 0.25)     # lower than warn threshold
        self.declare_parameter('anchor_count', 8)            # consecutive bad readings
        self.declare_parameter('anchor_cooldown_sec', 60.0)  # min seconds between anchors

        self.threshold        = self.get_parameter('confidence_threshold').value
        self.count_limit      = self.get_parameter('consecutive_count').value
        self.auto_cancel      = self.get_parameter('auto_cancel').value
        self.robot            = self.get_parameter('robot').value
        self.auto_anchor      = self.get_parameter('auto_anchor').value
        self.anchor_threshold = self.get_parameter('anchor_threshold').value
        self.anchor_count_lim = self.get_parameter('anchor_count').value
        self.anchor_cooldown  = self.get_parameter('anchor_cooldown_sec').value

        self.low_count         = 0
        self.anchor_low_count  = 0
        self._last_alert_time  = 0.0
        self._last_anchor_time = 0.0
        self._alert_cooldown_sec = 30.0

        self.create_subscription(Float32, '/localization_3d_confidence', self._cb, 10)

        if self.auto_cancel:
            from action_msgs.srv import CancelGoal
            self._cancel_cli = self.create_client(
                CancelGoal,
                f'/{self.robot}/navigate_to_pose/_action/cancel_goal'
            )

        # TF listener + /initialpose publisher for auto re-anchor
        if self.auto_anchor:
            self._tf_buffer       = tf2_ros.Buffer()
            self._tf_listener     = tf2_ros.TransformListener(self._tf_buffer, self)
            self._initialpose_pub = self.create_publisher(
                PoseWithCovarianceStamped, '/initialpose', 10
            )

        self.get_logger().info(
            f'Localization monitor started — '
            f'warn_threshold={self.threshold}, warn_count={self.count_limit}, '
            f'auto_cancel={self.auto_cancel}, auto_anchor={self.auto_anchor} '
            f'(anchor_thr={self.anchor_threshold}, '
            f'anchor_cnt={self.anchor_count_lim}, '
            f'cooldown={self.anchor_cooldown}s)'
        )

    def _cb(self, msg: Float32):
        confidence = msg.data

        # ---- warn / cancel (unchanged) ----
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

        # ---- auto re-anchor (new) ----
        if self.auto_anchor:
            if confidence < self.anchor_threshold:
                self.anchor_low_count += 1
                if self.anchor_low_count >= self.anchor_count_lim:
                    now = self.get_clock().now().nanoseconds * 1e-9
                    if now - self._last_anchor_time >= self.anchor_cooldown:
                        self._auto_anchor()
                        self._last_anchor_time = now
                    self.anchor_low_count = 0
            else:
                self.anchor_low_count = 0

    def _auto_anchor(self):
        """Look up current map→base_footprint TF and republish to /initialpose.

        Gives FAST-LIO a soft hint to re-converge around the current
        (possibly slightly drifted) estimate. Loose covariance lets the
        estimator adjust rather than hard-resetting.
        """
        robot_frame = f'{self.robot}/base_footprint'
        try:
            t = self._tf_buffer.lookup_transform(
                'map',
                robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
        except TransformException as e:
            self.get_logger().warn(f'Auto-anchor: TF lookup failed ({e}), skipping.')
            return

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id    = 'map'
        msg.header.stamp       = self.get_clock().now().to_msg()
        msg.pose.pose.position.x    = t.transform.translation.x
        msg.pose.pose.position.y    = t.transform.translation.y
        msg.pose.pose.position.z    = t.transform.translation.z
        msg.pose.pose.orientation.x = t.transform.rotation.x
        msg.pose.pose.orientation.y = t.transform.rotation.y
        msg.pose.pose.orientation.z = t.transform.rotation.z
        msg.pose.pose.orientation.w = t.transform.rotation.w
        # Loose covariance: let FAST-LIO treat this as a soft hint
        msg.pose.covariance[0]  = 0.30   # x  variance  (σ ≈ 0.55 m)
        msg.pose.covariance[7]  = 0.30   # y  variance
        msg.pose.covariance[35] = 0.10   # yaw variance (σ ≈ 0.32 rad ≈ 18°)

        self._initialpose_pub.publish(msg)
        self.get_logger().warn(
            f'Auto re-anchored ICP at '
            f'({t.transform.translation.x:.3f}, {t.transform.translation.y:.3f}) '
            f'— confidence was below {self.anchor_threshold}'
        )

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
