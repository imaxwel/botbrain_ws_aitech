#!/usr/bin/env python3

"""Build a ground-relative LaserScan from the accepted FAST-LIO world cloud."""

import math
import time

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener

from g1_pkg.grid_mapping_core import (
    accumulate_consistent_horizontal_plane,
    confirm_temporal_scan_obstacles,
    fit_ground_plane_ransac,
    navigation_scan_ranges,
    quaternion_to_matrix,
)


class NavigationScanProjector(Node):
    """Project only verified navigation-height surfaces into ``/scan``."""

    def __init__(self):
        super().__init__('navigation_scan_projector')
        self._declare_parameters()
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.angle_min = float(self.get_parameter('angle_min').value)
        self.angle_max = float(self.get_parameter('angle_max').value)
        self.angle_increment = float(
            self.get_parameter('angle_increment').value)
        self.scan_time = float(self.get_parameter('scan_time').value)
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.min_height = float(self.get_parameter('min_height').value)
        self.max_height = float(self.get_parameter('max_height').value)
        self.transform_timeout = max(
            0.0, float(self.get_parameter('transform_timeout').value))
        self.cloud_max_age = max(
            0.02, float(self.get_parameter('cloud_max_age').value))
        self.ground_filter_enabled = bool(
            self.get_parameter('ground_filter_enabled').value)
        self.ground_candidate_half_band = float(
            self.get_parameter('ground_candidate_half_band').value)
        self.ground_max_tilt_deg = float(
            self.get_parameter('ground_max_tilt_deg').value)
        self.ground_distance_threshold = float(
            self.get_parameter('ground_distance_threshold').value)
        self.ground_min_inliers = int(
            self.get_parameter('ground_min_inliers').value)
        self.ground_min_inlier_ratio = float(
            self.get_parameter('ground_min_inlier_ratio').value)
        self.ground_max_origin_error = float(
            self.get_parameter('ground_max_origin_error').value)
        self.ground_max_median_residual = float(
            self.get_parameter('ground_max_median_residual').value)
        self.ground_ransac_iterations = int(
            self.get_parameter('ground_ransac_iterations').value)
        self.ground_max_candidates = int(
            self.get_parameter('ground_max_candidates').value)
        self.ground_confirmation_frames = max(
            1, int(self.get_parameter('ground_confirmation_frames').value))
        self.ground_consistency_height = float(
            self.get_parameter('ground_consistency_height').value)
        self.ground_consistency_tilt_deg = float(
            self.get_parameter('ground_consistency_tilt_deg').value)
        self.ground_plane_max_age = float(
            self.get_parameter('ground_plane_max_age').value)
        self.max_bridge_angle = float(
            self.get_parameter('max_bridge_angle').value)
        self.max_bridge_distance = float(
            self.get_parameter('max_bridge_distance').value)
        self.temporal_confirmation_frames = max(
            1, int(self.get_parameter(
                'temporal_confirmation_frames').value))
        self.temporal_angle_window_bins = max(
            0, int(self.get_parameter('temporal_angle_window_bins').value))
        self.temporal_range_tolerance = max(
            0.0, float(self.get_parameter(
                'temporal_range_tolerance').value))
        self.immediate_obstacle_range = max(
            self.range_min, float(self.get_parameter(
                'immediate_obstacle_range').value))

        if not self.target_frame:
            raise ValueError('target_frame must not be empty')
        if self.angle_increment <= 0.0 or self.angle_max <= self.angle_min:
            raise ValueError('invalid LaserScan angle parameters')
        if self.range_max <= self.range_min:
            raise ValueError('range_max must be greater than range_min')
        if self.max_height <= self.min_height:
            raise ValueError('max_height must be greater than min_height')

        self._tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._publisher = self.create_publisher(
            LaserScan, 'scan', qos_profile_sensor_data)
        self._cloud_subscription = self.create_subscription(
            PointCloud2, 'cloud_in', self._cloud_callback,
            qos_profile_sensor_data)
        self._process_timer = self.create_timer(
            0.01, self._process_pending_cloud)
        self._pending_cloud = None
        self._pending_received = None
        self._last_processed_stamp = None
        self._last_diagnostic = -math.inf
        self._dropped_transform_clouds = 0
        self._ground_fit_failures = 0
        self._ground_candidate = None
        self._ground_candidate_count = 0
        self._active_ground = None
        self._active_ground_time = None
        self._previous_raw_ranges = None
        self._previous_confirmations = None
        self.get_logger().info(
            'Navigation scan projection active: cloud -> %s, height '
            '%.2f..%.2f m, range %.2f..%.2f m, adaptive_ground=%s'
            % (
                self.target_frame, self.min_height, self.max_height,
                self.range_min, self.range_max,
                self.ground_filter_enabled,
            )
        )

    def _declare_parameters(self):
        defaults = {
            'target_frame': 'g1_robot/base_footprint',
            'transform_timeout': 0.0,
            'cloud_max_age': 0.20,
            'min_height': 0.20,
            'max_height': 1.35,
            'angle_min': -math.pi,
            'angle_max': math.pi,
            'angle_increment': 0.007,
            'scan_time': 0.1,
            'range_min': 0.45,
            'range_max': 5.0,
            'ground_filter_enabled': True,
            'ground_candidate_half_band': 0.22,
            'ground_max_tilt_deg': 8.0,
            'ground_distance_threshold': 0.045,
            'ground_min_inliers': 40,
            'ground_min_inlier_ratio': 0.15,
            'ground_max_origin_error': 0.10,
            'ground_max_median_residual': 0.035,
            'ground_ransac_iterations': 32,
            'ground_max_candidates': 3000,
            'ground_confirmation_frames': 2,
            'ground_consistency_height': 0.05,
            'ground_consistency_tilt_deg': 2.0,
            'ground_plane_max_age': 0.50,
            'max_bridge_angle': 0.18,
            'max_bridge_distance': 0.45,
            'temporal_confirmation_frames': 2,
            'temporal_angle_window_bins': 16,
            'temporal_range_tolerance': 0.25,
            'immediate_obstacle_range': 0.90,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _cloud_callback(self, msg: PointCloud2):
        # Keep only the latest accepted cloud. If its exact-stamp transform is
        # not available yet, the timer retries briefly; no historical cloud is
        # replayed into the costmap after a localization gap.
        self._pending_cloud = msg
        self._pending_received = time.monotonic()

    @staticmethod
    def _transform_points(points, transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = quaternion_to_matrix([
            rotation.x, rotation.y, rotation.z, rotation.w])
        offset = np.array([
            translation.x, translation.y, translation.z], dtype=np.float64)
        return points @ matrix.T + offset

    def _process_pending_cloud(self):
        msg = self._pending_cloud
        received = self._pending_received
        if msg is None or received is None:
            return
        if time.monotonic() - received > self.cloud_max_age:
            self._pending_cloud = None
            self._pending_received = None
            self._dropped_transform_clouds += 1
            self.get_logger().warn(
                'Dropped accepted cloud: exact timestamp transform to %s '
                'was unavailable for %.0f ms'
                % (self.target_frame, self.cloud_max_age * 1000.0),
                throttle_duration_sec=2.0,
            )
            return
        try:
            transform = self._tf_buffer.lookup_transform(
                self.target_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=self.transform_timeout),
            )
        except TransformException:
            return

        self._pending_cloud = None
        self._pending_received = None
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        if stamp == self._last_processed_stamp:
            return
        self._last_processed_stamp = stamp
        try:
            structured = point_cloud2.read_points(
                msg, field_names=['x', 'y', 'z'], skip_nans=False)
            points = np.column_stack((
                structured['x'], structured['y'], structured['z'],
            )).astype(np.float64, copy=False)
        except (AssertionError, KeyError, TypeError, ValueError) as error:
            self.get_logger().error(
                f'Cannot decode accepted PointCloud2: {error}',
                throttle_duration_sec=2.0,
            )
            return
        points = self._transform_points(points, transform)
        finite_points = points[np.isfinite(points).all(axis=1)]

        now_monotonic = time.monotonic()
        fitted_ground = None
        ground_coefficients = None
        ground_metrics = {'reason': 'disabled', 'tilt_deg': math.nan}
        if self.ground_filter_enabled and len(finite_points):
            fitted_ground, ground_metrics = fit_ground_plane_ransac(
                finite_points,
                np.zeros(3, dtype=np.float64),
                0.0,
                min_range=max(self.range_min, 0.55),
                max_range=self.range_max,
                candidate_below=self.ground_candidate_half_band,
                candidate_above=self.ground_candidate_half_band,
                max_tilt_deg=self.ground_max_tilt_deg,
                distance_threshold=self.ground_distance_threshold,
                min_inliers=self.ground_min_inliers,
                min_inlier_ratio=self.ground_min_inlier_ratio,
                max_expected_error=self.ground_max_origin_error,
                max_median_residual=self.ground_max_median_residual,
                iterations=self.ground_ransac_iterations,
                max_candidates=self.ground_max_candidates,
            )
            if fitted_ground is None:
                self._ground_fit_failures += 1
            else:
                (
                    self._ground_candidate,
                    self._ground_candidate_count,
                    confirmed,
                ) = accumulate_consistent_horizontal_plane(
                    self._ground_candidate,
                    self._ground_candidate_count,
                    fitted_ground,
                    max_height_delta=self.ground_consistency_height,
                    max_tilt_delta_deg=self.ground_consistency_tilt_deg,
                    required_count=self.ground_confirmation_frames,
                )
                if confirmed:
                    self._active_ground = self._ground_candidate.copy()
                    self._active_ground_time = now_monotonic

        if (
            self._active_ground is not None and
            self._active_ground_time is not None and
            now_monotonic - self._active_ground_time <=
            self.ground_plane_max_age
        ):
            ground_coefficients = self._active_ground

        ranges, metrics = navigation_scan_ranges(
            finite_points,
            angle_min=self.angle_min,
            angle_max=self.angle_max,
            angle_increment=self.angle_increment,
            range_min=self.range_min,
            range_max=self.range_max,
            min_obstacle_height=self.min_height,
            max_obstacle_height=self.max_height,
            ground_coefficients=ground_coefficients,
            max_bridge_angle=self.max_bridge_angle,
            max_bridge_distance=self.max_bridge_distance,
        )
        raw_ranges = ranges
        ranges, confirmations, confirmed_bins = (
            confirm_temporal_scan_obstacles(
                raw_ranges,
                self._previous_raw_ranges,
                self._previous_confirmations,
                required_frames=self.temporal_confirmation_frames,
                angle_window_bins=self.temporal_angle_window_bins,
                range_tolerance=self.temporal_range_tolerance,
                immediate_range=self.immediate_obstacle_range,
            )
        )
        self._previous_raw_ranges = raw_ranges
        self._previous_confirmations = confirmations
        scan = LaserScan()
        scan.header.stamp = msg.header.stamp
        scan.header.frame_id = self.target_frame
        scan.angle_min = self.angle_min
        scan.angle_max = self.angle_min + (
            len(ranges) - 1) * self.angle_increment
        scan.angle_increment = self.angle_increment
        scan.time_increment = 0.0
        scan.scan_time = self.scan_time
        scan.range_min = self.range_min
        scan.range_max = self.range_max
        scan.ranges = ranges.astype(np.float32).tolist()
        self._publisher.publish(scan)

        now = now_monotonic
        if now - self._last_diagnostic >= 5.0:
            self._last_diagnostic = now
            tilt = ground_metrics.get('tilt_deg', math.nan)
            plane_text = (
                f'ground_tilt={tilt:.2f}deg confirmed'
                if ground_coefficients is not None else
                'ground=fallback('
                f'{ground_metrics.get("reason", "awaiting_confirmation")})'
            )
            self.get_logger().info(
                'Navigation scan: points=%d obstacles=%d measured_bins=%d '
                'surface_fill=%d confirmed_bins=%d %s tf_drops=%d '
                'ground_failures=%d'
                % (
                    metrics['finite_points'], metrics['obstacle_points'],
                    metrics['measured_bins'], metrics['bridged_bins'],
                    confirmed_bins, plane_text, self._dropped_transform_clouds,
                    self._ground_fit_failures,
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationScanProjector()
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
