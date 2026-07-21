#!/usr/bin/env python3
"""
Localization drift monitor.

Monitors /localization_3d_confidence and the measured twist carried by
/g1_robot/nav_odom. It can cancel active navigation when localization or
velocity feedback becomes unsafe.

Usage:
  ros2 run bot_navigation localization_monitor.py
  ros2 run bot_navigation localization_monitor.py --ros-args \
    -p auto_cancel:=true
"""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


class LocalizationMonitor(Node):
    def __init__(self):
        super().__init__('localization_monitor')

        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('consecutive_count', 5)
        self.declare_parameter('auto_cancel', False)
        self.declare_parameter('robot', 'g1_robot')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('scan_timeout_sec', 1.5)
        self.declare_parameter('confidence_timeout_sec', 2.0)
        self.declare_parameter('startup_grace_sec', 2.0)
        self.declare_parameter('nav_odom_topic', '/g1_robot/nav_odom')
        self.declare_parameter('nav_odom_timeout_sec', 0.75)
        self.declare_parameter('max_twist_variance', 100.0)
        self.declare_parameter('low_confidence_duration_sec', 5.0)
        # Stop immediately on unhealthy inputs, but only cancel a goal after
        # the fault persists. A short scheduling hiccup can then recover and
        # continue the same goal instead of becoming ABORTED.
        self.declare_parameter('cancel_after_sec', 3.0)
        self.declare_parameter('cancel_retry_sec', 1.0)
        self.declare_parameter('cancel_request_timeout_sec', 2.0)
        self.declare_parameter('publish_safety_stop', False)
        self.declare_parameter(
            'safety_stop_topic', '/g1_robot/cmd_vel_nav_safety')

        self.threshold = self.get_parameter('confidence_threshold').value
        self.count_limit = max(
            1, int(self.get_parameter('consecutive_count').value))
        self.auto_cancel = self.get_parameter('auto_cancel').value
        self.robot = self.get_parameter('robot').value
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.scan_timeout = max(
            0.2, float(self.get_parameter('scan_timeout_sec').value))
        self.confidence_timeout = max(
            0.5, float(self.get_parameter('confidence_timeout_sec').value))
        self.startup_grace = max(
            0.0, float(self.get_parameter('startup_grace_sec').value))
        self.nav_odom_topic = str(
            self.get_parameter('nav_odom_topic').value)
        self.nav_odom_timeout = max(
            0.1, float(self.get_parameter('nav_odom_timeout_sec').value))
        self.max_twist_variance = max(
            0.0, float(self.get_parameter('max_twist_variance').value))
        self.low_confidence_duration = max(
            0.5,
            float(self.get_parameter('low_confidence_duration_sec').value),
        )
        self.cancel_after = max(
            0.5, float(self.get_parameter('cancel_after_sec').value))
        self.cancel_retry = max(
            0.2, float(self.get_parameter('cancel_retry_sec').value))
        self.cancel_request_timeout = max(
            0.5,
            float(self.get_parameter('cancel_request_timeout_sec').value),
        )
        self.publish_safety_stop = bool(
            self.get_parameter('publish_safety_stop').value)
        self.safety_stop_topic = str(
            self.get_parameter('safety_stop_topic').value)

        self.low_count = 0
        self._low_confidence_since = None
        self._last_alert_time = 0.0
        self._alert_cooldown_sec = 30.0
        self._localization_unhealthy = False
        self._confidence = 0.0
        self._last_confidence_receive = None
        self._confidence_stream_unhealthy = False
        self._last_confidence_alert = -math.inf
        self._started_at = time.monotonic()
        self._last_scan_receive = None
        self._last_scan_stamp = None
        self._scan_unhealthy = False
        self._scan_unhealthy_since = None
        self._last_scan_alert = -math.inf
        self._last_nav_odom_receive = None
        self._last_nav_odom_stamp = None
        self._nav_odom_valid = False
        self._nav_odom_unhealthy = False
        self._nav_odom_unhealthy_since = None
        self._last_nav_odom_alert = -math.inf
        self._navigation_unhealthy_since = None
        self._last_cancel_attempt = -math.inf
        self._cancel_future = None
        self._cancel_request_started = None
        self._cancel_requested_for_episode = False
        self._safety_stop_active = False

        self.create_subscription(
            Float32, '/localization_3d_confidence', self._cb, 10)
        self.create_subscription(
            LaserScan, self.scan_topic, self._scan_cb,
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, self.nav_odom_topic, self._nav_odom_cb, 20)
        self.create_timer(0.2, self._health_check)
        self._safety_stop_publisher = (
            self.create_publisher(Twist, self.safety_stop_topic, 10)
            if self.publish_safety_stop else None
        )

        if self.auto_cancel:
            self._cancel_cli = self.create_client(
                Trigger,
                (
                    f'/{self.robot}/cancel_nav2_goal'
                    if self.robot else '/cancel_nav2_goal'
                ),
            )

        self.get_logger().info(
            f'Localization monitor started: '
            f'warn_threshold={self.threshold}, warn_count={self.count_limit}, '
            f'low_duration={self.low_confidence_duration:.1f}s, '
            f'confidence_timeout={self.confidence_timeout:.1f}s, '
            f'cancel_after={self.cancel_after:.1f}s, '
            f'scan={self.scan_topic}, nav_odom={self.nav_odom_topic}, '
            f'auto_cancel={self.auto_cancel}, '
            f'safety_stop={self.publish_safety_stop}'
        )

    def _cb(self, msg: Float32):
        confidence = float(msg.data)
        now_monotonic = time.monotonic()
        self._confidence = confidence
        self._last_confidence_receive = now_monotonic
        confidence_low = (
            not math.isfinite(confidence) or confidence <= self.threshold)

        if confidence_low:
            if self._low_confidence_since is None:
                self._low_confidence_since = now_monotonic
            self.low_count = min(self.low_count + 1, self.count_limit)
            low_duration = now_monotonic - self._low_confidence_since
            self.get_logger().warn(
                f'Low ICP confidence: {confidence:.3f}  '
                f'({self.low_count}/{self.count_limit}, '
                f'{low_duration:.1f}/{self.low_confidence_duration:.1f}s)',
                throttle_duration_sec=1.0
            )
            if (
                self.low_count >= self.count_limit and
                low_duration >= self.low_confidence_duration
            ):
                self._localization_unhealthy = True
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - self._last_alert_time >= self._alert_cooldown_sec:
                    self.get_logger().error(
                        f'ICP DRIFT DETECTED! confidence={confidence:.3f} < '
                        f'{self.threshold} '
                        f'for {low_duration:.1f} continuous seconds.\n'
                        '  Action required: re-publish /initialpose in '
                        'Foxglove to re-localize.'
                    )
                    self._last_alert_time = now
        else:
            if self.low_count > 0 or self._localization_unhealthy:
                self.get_logger().info(
                    f'ICP confidence recovered: {confidence:.3f}')
            self.low_count = 0
            self._low_confidence_since = None
            self._localization_unhealthy = False

    def _scan_cb(self, _msg: LaserScan):
        self._last_scan_receive = time.monotonic()
        self._last_scan_stamp = (
            _msg.header.stamp.sec + _msg.header.stamp.nanosec * 1e-9)

    def _nav_odom_cb(self, msg: Odometry):
        twist = msg.twist.twist
        covariance = msg.twist.covariance
        expected_child = (
            f'{self.robot}/base_footprint'
            if self.robot else 'base_footprint')
        planar_variances = (covariance[0], covariance[7], covariance[35])
        self._nav_odom_valid = (
            msg.child_frame_id.lstrip('/') == expected_child and
            all(math.isfinite(value) for value in (
                twist.linear.x, twist.linear.y, twist.angular.z)) and
            all(
                math.isfinite(value) and
                0.0 <= value < self.max_twist_variance
                for value in planar_variances
            )
        )
        self._last_nav_odom_receive = time.monotonic()
        self._last_nav_odom_stamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    @staticmethod
    def _stamp_is_fresh(stamp, ros_now, max_age):
        return (
            stamp is not None and
            math.isfinite(stamp) and
            -0.25 <= ros_now - stamp <= max_age
        )

    def _health_check(self):
        now = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        self._poll_cancel_result(now)
        waiting_for_initial_inputs = (
            now - self._started_at < self.startup_grace and
            (
                self._last_scan_receive is None or
                self._last_nav_odom_receive is None or
                self._last_confidence_receive is None
            )
        )
        if waiting_for_initial_inputs:
            if self._safety_stop_publisher is not None:
                self._safety_stop_publisher.publish(Twist())
            return

        confidence_healthy = (
            self._last_confidence_receive is not None and
            now - self._last_confidence_receive <= self.confidence_timeout and
            math.isfinite(self._confidence)
        )
        if confidence_healthy:
            if self._confidence_stream_unhealthy:
                self.get_logger().info(
                    'Localization confidence stream recovered')
            self._confidence_stream_unhealthy = False
        else:
            if not self._confidence_stream_unhealthy:
                self._confidence_stream_unhealthy = True
                self._last_confidence_alert = -math.inf
            if now - self._last_confidence_alert >= 5.0:
                self.get_logger().error(
                    'Localization confidence is missing, stale, or '
                    'non-finite; navigation cannot continue safely.')
                self._last_confidence_alert = now

        scan_healthy = (
            self._last_scan_receive is not None and
            now - self._last_scan_receive <= self.scan_timeout and
            self._stamp_is_fresh(
                self._last_scan_stamp, ros_now, self.scan_timeout)
        )
        if scan_healthy:
            if self._scan_unhealthy:
                self.get_logger().info(
                    f'Scan stream recovered on {self.scan_topic}')
            self._scan_unhealthy = False
            self._scan_unhealthy_since = None
        else:
            self._scan_unhealthy = True
            if self._scan_unhealthy_since is None:
                self._scan_unhealthy_since = now
            if now - self._last_scan_alert >= 5.0:
                self.get_logger().error(
                    f'Scan stream is missing or stale on {self.scan_topic}; '
                    'navigation cannot continue safely.')
                self._last_scan_alert = now
        nav_odom_healthy = (
            self._nav_odom_valid and
            self._last_nav_odom_receive is not None and
            now - self._last_nav_odom_receive <= self.nav_odom_timeout and
            self._stamp_is_fresh(
                self._last_nav_odom_stamp, ros_now, self.nav_odom_timeout)
        )
        if nav_odom_healthy:
            if self._nav_odom_unhealthy:
                self.get_logger().info(
                    f'Navigation odometry recovered on {self.nav_odom_topic}')
            self._nav_odom_unhealthy = False
            self._nav_odom_unhealthy_since = None
        else:
            self._nav_odom_unhealthy = True
            if self._nav_odom_unhealthy_since is None:
                self._nav_odom_unhealthy_since = now
            if now - self._last_nav_odom_alert >= 5.0:
                self.get_logger().error(
                    f'Navigation odometry is missing, stale, or has invalid '
                    f'twist covariance on {self.nav_odom_topic}; navigation '
                    'cannot continue safely.')
                self._last_nav_odom_alert = now

        navigation_unhealthy = (
            self._scan_unhealthy or self._nav_odom_unhealthy or
            self._localization_unhealthy or
            self._confidence_stream_unhealthy)
        if navigation_unhealthy:
            if self._navigation_unhealthy_since is None:
                self._navigation_unhealthy_since = now
        else:
            self._navigation_unhealthy_since = None
            self._cancel_requested_for_episode = False
        if navigation_unhealthy and self._safety_stop_publisher is not None:
            self._safety_stop_publisher.publish(Twist())
            if not self._safety_stop_active:
                self.get_logger().error(
                    f'Navigation safety stop engaged on '
                    f'{self.safety_stop_topic}')
            self._safety_stop_active = True
        elif self._safety_stop_active:
            self.get_logger().info(
                'Navigation safety stop released after health recovery')
            self._safety_stop_active = False

        fault_duration = (
            now - self._navigation_unhealthy_since
            if self._navigation_unhealthy_since is not None else 0.0
        )
        if (
            self.auto_cancel and navigation_unhealthy and
            fault_duration >= self.cancel_after
        ):
            reason = (
                'stale scan'
                if self._scan_unhealthy else
                'invalid navigation odometry'
                if self._nav_odom_unhealthy else
                'missing localization confidence'
                if self._confidence_stream_unhealthy else
                'low ICP confidence')
            self._cancel_navigation(reason)

    def _poll_cancel_result(self, now):
        if self._cancel_future is None:
            return
        if self._cancel_future.done():
            try:
                response = self._cancel_future.result()
            except Exception as error:
                self.get_logger().error(
                    f'Navigation cancel service failed: {error}')
            else:
                if response is None or not response.success:
                    message = response.message if response is not None else ''
                    self.get_logger().error(
                        f'Navigation cancel was not confirmed: {message}')
                else:
                    self.get_logger().warn(
                        f'Navigation cancel confirmed: {response.message}',
                        throttle_duration_sec=5.0)
            self._cancel_future = None
            self._cancel_request_started = None
            return
        if (
            self._cancel_request_started is not None and
            now - self._cancel_request_started >= self.cancel_request_timeout
        ):
            self.get_logger().error(
                'Navigation cancel service timed out; safety zero remains '
                'active')
            try:
                self._cancel_cli.remove_pending_request(self._cancel_future)
            except Exception:
                pass
            self._cancel_future = None
            self._cancel_request_started = None

    def _cancel_navigation(self, reason):
        now = time.monotonic()
        if self._cancel_requested_for_episode:
            return
        if now - self._last_cancel_attempt < self.cancel_retry:
            return
        if self._cancel_future is not None and not self._cancel_future.done():
            return
        self._last_cancel_attempt = now
        if not self._cancel_cli.service_is_ready():
            self.get_logger().warn('Cancel service unavailable')
            return
        self._cancel_future = self._cancel_cli.call_async(Trigger.Request())
        self._cancel_request_started = now
        self._cancel_requested_for_episode = True
        self.get_logger().warn(
            f'Navigation cancel requested due to {reason}.',
            throttle_duration_sec=5.0)


def main():
    rclpy.init()
    node = LocalizationMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
