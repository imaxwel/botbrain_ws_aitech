#!/usr/bin/env python3
"""Build a persistent 2D OccupancyGrid from accepted FAST-LIO scans.

The default input is /cloud_registered_1 in camera_init. Each world-frame
cloud is paired exactly by timestamp with /Odometry_loc so the scan can be
self-filtered in the FAST-LIO body frame and ray-traced from the LiDAR origin.

Only a validated floor plane is used. Floor, obstacle, and free-space evidence
is accumulated once per cell per scan with bounded log odds, allowing later
free observations to clear a false obstacle.
"""

import argparse
import math
import os
import sys
from collections import OrderedDict
from threading import Lock

os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

import numpy as np
import rclpy
import rclpy.duration
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from std_msgs.msg import Header
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from g1_pkg.grid_mapping_core import (
    classify_points,
    expand_cell_ids,
    fit_ground_plane_ransac,
    plane_height,
    quaternion_to_matrix,
    raytrace_cell_ids,
    select_ray_endpoint_indices,
    unique_cell_ids,
    update_log_odds_grid,
    world_to_body,
)


FREE = 0
OCCUPIED = 100
UNKNOWN = -1


def _frame_name(frame_id):
    return frame_id.lstrip("/")


def _stamp_key(stamp):
    return int(stamp.sec), int(stamp.nanosec)


class GridAccumulator(Node):
    def __init__(self, args):
        super().__init__("grid_accumulator")

        self.res = args.resolution
        self.rate = max(0.1, args.rate)
        self.map_frame = _frame_name(args.map_frame)
        self.body_frame = _frame_name(args.body_frame)
        self.cloud_topic = args.cloud_topic
        self.odom_topic = args.odom_topic
        self.grid_topic = args.grid_topic
        self.skip_frames = args.skip_frames
        self.pre_transformed = args.pre_transformed
        self.pose_cache_size = max(1, args.pose_cache_size)
        self.map_z = args.map_z
        self.grid_margin = args.grid_margin

        # Fixed-z fallback parameters are retained for explicit legacy mode.
        self.ground_z = args.ground_z
        self.ground_z_min = args.ground_z_min
        self.obstacle_z = args.obstacle_z
        self.obstacle_z_max = args.obstacle_z_max
        self.invert_z = args.invert_z
        self.z_offset = args.z_offset

        self.use_ground_plane = args.use_ground_plane
        self.sensor_height = args.sensor_height
        self.below_ground_tolerance = args.below_ground_tolerance
        self.ground_margin = args.ground_margin
        self.obstacle_margin = args.obstacle_margin
        self.max_obstacle_height = args.max_obstacle_height
        self.plane_smooth = args.plane_smooth
        self.plane_update_interval = max(1, args.plane_update_interval)
        self.plane_init_frames = max(1, args.plane_init_frames)
        self.plane_fit_min_range = args.plane_fit_min_range
        self.plane_fit_max_range = args.plane_fit_max_range
        self.plane_candidate_band = args.plane_candidate_band
        self.plane_max_tilt_deg = args.plane_max_tilt_deg
        self.plane_distance_threshold = args.plane_distance_threshold
        self.plane_min_inliers = args.plane_min_inliers
        self.plane_min_inlier_ratio = args.plane_min_inlier_ratio
        self.plane_max_expected_error = args.plane_max_expected_error
        self.plane_max_median_residual = args.plane_max_median_residual
        self.plane_max_jump = args.plane_max_jump
        self.plane_ransac_iterations = args.plane_ransac_iterations

        self.max_point_range = args.max_point_range
        self.self_filter = args.self_filter
        self.self_x_min = args.self_x_min
        self.self_x_max = args.self_x_max
        self.self_y_abs = args.self_y_abs
        self.self_z_min = args.self_z_min
        self.self_z_max = args.self_z_max
        self.lidar_offset = np.array([
            args.lidar_offset_x,
            args.lidar_offset_y,
            args.lidar_offset_z,
        ], dtype=np.float64)

        self.raytrace = args.raytrace
        self.raytrace_range = args.raytrace_range
        self.raytrace_bins = max(1, args.raytrace_bins)
        self.obstacle_spread_radius = max(0.0, args.obstacle_spread_radius)
        self.free_spread_radius = max(0.0, args.free_spread_radius)
        self.free_update = args.free_update
        self.obstacle_update = args.obstacle_update
        self.log_odds_min = args.log_odds_min
        self.log_odds_max = args.log_odds_max
        self.free_threshold = args.free_threshold
        self.occupied_threshold = args.occupied_threshold
        if self.occupied_threshold is None:
            # Half an update above N-1 hits makes --min-obs-hits count frames.
            self.occupied_threshold = max(
                self.obstacle_update,
                (args.min_obs_hits - 0.5) * self.obstacle_update,
            )

        self.debug_clouds = args.debug_clouds
        self.debug_every = max(1, args.debug_every)
        self.max_debug_points = max(1, args.max_debug_points)

        self.grid = None
        self.log_odds = None
        self.observed = None
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.lock = Lock()

        self.plane_coeffs = None
        self.plane_candidate = None
        self.plane_candidate_count = 0
        self.plane_metrics = {"reason": "not_attempted"}
        self.rng = np.random.default_rng(0xB07B2A1)

        self.odom_cache = OrderedDict()
        self.pending_clouds = OrderedDict()
        self.warn_times = {}

        self.frames = 0
        self.processed_frames = 0
        self.classified_frames = 0
        self.ground_pts = 0
        self.obs_pts = 0
        self.self_filtered_pts = 0
        self.range_filtered_pts = 0
        self.plane_rejections = 0
        self.unmatched_clouds = 0
        self.invalid_frames = 0
        self.free_cell_updates = 0
        self.obstacle_cell_updates = 0
        self.last_class_counts = {}

        if not self.pre_transformed:
            self.tf_buf = Buffer()
            self.tf_listener = TransformListener(self.tf_buf, self)

        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=max(20, self.pose_cache_size),
        )
        self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_cb, cloud_qos)
        self.create_subscription(
            Odometry, self.odom_topic, self.odom_cb, odom_qos)

        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(
            OccupancyGrid, self.grid_topic, latched_qos)

        debug_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.ground_debug_pub = self.create_publisher(
            PointCloud2, args.ground_debug_topic, debug_qos)
        self.obstacle_debug_pub = self.create_publisher(
            PointCloud2, args.obstacle_debug_topic, debug_qos)
        self.self_debug_pub = self.create_publisher(
            PointCloud2, args.self_debug_topic, debug_qos)

        self.create_timer(1.0 / self.rate, self.publish_grid)
        self.create_timer(5.0, self.log_stats)

        self.get_logger().info(
            "grid_accumulator: "
            f"res={self.res:.3f}m cloud={self.cloud_topic} "
            f"odom={self.odom_topic} frame={self.map_frame} "
            f"floor=[-{self.below_ground_tolerance:.2f},"
            f"{self.ground_margin:.2f})m "
            f"obstacle=[{self.obstacle_margin:.2f},"
            f"{self.max_obstacle_height:.2f})m "
            f"occ_threshold={self.occupied_threshold:.2f} "
            f"obs_spread={self.obstacle_spread_radius:.3f}m "
            f"free_spread={self.free_spread_radius:.3f}m "
            f"self_filter={'on' if self.self_filter else 'off'} "
            f"raytrace={'on' if self.raytrace else 'off'}")

    def _warn_throttled(self, key, message, period=5.0):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.warn_times.get(key, -math.inf) >= period:
            self.warn_times[key] = now
            self.get_logger().warning(message)

    def _put_bounded(self, cache, key, value, *, cloud_cache=False):
        cache[key] = value
        cache.move_to_end(key)
        while len(cache) > self.pose_cache_size:
            cache.popitem(last=False)
            if cloud_cache:
                self.unmatched_clouds += 1

    def odom_cb(self, msg: Odometry):
        if (_frame_name(msg.header.frame_id) != self.map_frame or
                _frame_name(msg.child_frame_id) != self.body_frame):
            self.invalid_frames += 1
            self._warn_throttled(
                "odom_frame",
                "Ignoring Odometry_loc with frame pair "
                f"'{msg.header.frame_id}' -> '{msg.child_frame_id}'; "
                f"expected '{self.map_frame}' -> '{self.body_frame}'")
            return

        key = _stamp_key(msg.header.stamp)
        pending = self.pending_clouds.pop(key, None)
        if pending is not None:
            self._process_pretransformed_cloud(pending, msg)
            return
        self._put_bounded(self.odom_cache, key, msg)

    def cloud_cb(self, msg: PointCloud2):
        self.frames += 1
        if self.frames <= self.skip_frames:
            return

        if self.pre_transformed:
            if _frame_name(msg.header.frame_id) != self.map_frame:
                self.invalid_frames += 1
                self._warn_throttled(
                    "cloud_frame",
                    f"Ignoring world cloud frame '{msg.header.frame_id}'; "
                    f"expected '{self.map_frame}'")
                return
            key = _stamp_key(msg.header.stamp)
            odom = self.odom_cache.pop(key, None)
            if odom is None:
                self._put_bounded(
                    self.pending_clouds, key, msg, cloud_cache=True)
                return
            self._process_pretransformed_cloud(msg, odom)
            return

        self._process_legacy_body_cloud(msg)

    @staticmethod
    def _read_xyz(msg):
        raw = pc2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        raw = np.asarray(list(raw))
        if raw.size == 0:
            return np.empty((0, 3), dtype=np.float64)
        if raw.dtype.names is not None:
            return np.column_stack([
                raw["x"], raw["y"], raw["z"]]).astype(np.float64)
        return raw.reshape(-1, 3).astype(np.float64)

    @staticmethod
    def _pose_components(odom):
        position_msg = odom.pose.pose.position
        rotation_msg = odom.pose.pose.orientation
        position = np.array([
            position_msg.x, position_msg.y, position_msg.z],
            dtype=np.float64)
        rotation = quaternion_to_matrix([
            rotation_msg.x,
            rotation_msg.y,
            rotation_msg.z,
            rotation_msg.w,
        ])
        if not np.isfinite(position).all():
            raise ValueError("non-finite odometry position")
        return position, rotation

    def _process_pretransformed_cloud(self, msg, odom):
        try:
            body_position, body_to_world = self._pose_components(odom)
        except ValueError as exc:
            self.invalid_frames += 1
            self._warn_throttled("odom_pose", f"Ignoring invalid odometry: {exc}")
            return

        points_world = self._read_xyz(msg)
        if len(points_world) == 0:
            return
        points_body = world_to_body(
            points_world, body_position, body_to_world)
        self._process_points(
            msg, points_world, points_body, body_position, body_to_world)

    def _process_legacy_body_cloud(self, msg):
        points_body = self._read_xyz(msg)
        if len(points_body) == 0:
            return
        try:
            transform = self.tf_buf.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                Time.from_msg(msg.header.stamp),
                rclpy.duration.Duration(seconds=0.0))
        except Exception:
            self.unmatched_clouds += 1
            self._warn_throttled(
                "legacy_tf",
                "Legacy body cloud has no exact non-blocking map transform")
            return

        translation = transform.transform.translation
        rotation_msg = transform.transform.rotation
        body_position = np.array([
            translation.x, translation.y, translation.z], dtype=np.float64)
        try:
            body_to_world = quaternion_to_matrix([
                rotation_msg.x,
                rotation_msg.y,
                rotation_msg.z,
                rotation_msg.w,
            ])
        except ValueError:
            self.invalid_frames += 1
            return
        points_world = points_body @ body_to_world.T + body_position
        self._process_points(
            msg, points_world, points_body, body_position, body_to_world)

    def _process_points(
            self, msg, points_world, points_body,
            body_position, body_to_world):
        finite = (
            np.isfinite(points_world).all(axis=1) &
            np.isfinite(points_body).all(axis=1)
        )
        self_mask = np.zeros(len(points_world), dtype=bool)
        if self.self_filter:
            self_mask = (
                finite &
                (points_body[:, 0] >= self.self_x_min) &
                (points_body[:, 0] <= self.self_x_max) &
                (np.abs(points_body[:, 1]) <= self.self_y_abs) &
                (points_body[:, 2] >= self.self_z_min) &
                (points_body[:, 2] <= self.self_z_max)
            )

        lidar_origin = body_position + body_to_world @ self.lidar_offset
        planar_range = np.linalg.norm(
            points_world[:, :2] - lidar_origin[:2], axis=1)
        range_mask = np.isfinite(planar_range) & (
            planar_range <= self.max_point_range)
        keep = finite & range_mask & ~self_mask
        filtered_points = points_world[keep]
        self_points = points_world[self_mask]
        self.self_filtered_pts += int(self_mask.sum())
        self.range_filtered_pts += int((finite & ~range_mask).sum())
        if len(filtered_points) < self.plane_min_inliers:
            return

        self.processed_frames += 1
        if self.use_ground_plane:
            self._update_ground_plane(filtered_points, body_position)
            if self.plane_coeffs is None:
                self._publish_debug(msg, None, None, self_points)
                return
            ground_mask, obstacle_mask, heights = classify_points(
                filtered_points,
                self.plane_coeffs,
                below_ground_tolerance=self.below_ground_tolerance,
                ground_margin=self.ground_margin,
                obstacle_margin=self.obstacle_margin,
                max_obstacle_height=self.max_obstacle_height,
            )
        else:
            fixed_z = filtered_points[:, 2]
            ground_mask = (
                (fixed_z >= self.ground_z_min) &
                (fixed_z < self.ground_z)
            )
            obstacle_mask = (
                (fixed_z >= self.obstacle_z) &
                (fixed_z < self.obstacle_z_max)
            )
            heights = fixed_z - self.map_z

        self.classified_frames += 1
        self.ground_pts += int(ground_mask.sum())
        self.obs_pts += int(obstacle_mask.sum())
        self.last_class_counts = {
            "ground": int(ground_mask.sum()),
            "obstacle": int(obstacle_mask.sum()),
            "below": int((heights < -self.below_ground_tolerance).sum()),
            "transition": int((
                (heights >= self.ground_margin) &
                (heights < self.obstacle_margin)).sum()),
            "high": int((heights >= self.max_obstacle_height).sum()),
            "self": int(self_mask.sum()),
        }

        with self.lock:
            free_count, obstacle_count = self._ingest(
                filtered_points[:, 0],
                filtered_points[:, 1],
                ground_mask,
                obstacle_mask,
                lidar_origin[:2],
            )
        self.free_cell_updates += free_count
        self.obstacle_cell_updates += obstacle_count
        self._publish_debug(
            msg,
            filtered_points[ground_mask],
            filtered_points[obstacle_mask],
            self_points,
        )

    def _update_ground_plane(self, points_world, body_position):
        should_fit = (
            self.plane_coeffs is None or
            self.processed_frames % self.plane_update_interval == 0
        )
        if not should_fit:
            return

        expected_floor_z = body_position[2] - self.sensor_height
        candidate, metrics = fit_ground_plane_ransac(
            points_world,
            body_position,
            expected_floor_z,
            min_range=self.plane_fit_min_range,
            max_range=self.plane_fit_max_range,
            candidate_below=self.plane_candidate_band,
            candidate_above=self.plane_candidate_band,
            max_tilt_deg=self.plane_max_tilt_deg,
            distance_threshold=self.plane_distance_threshold,
            min_inliers=self.plane_min_inliers,
            min_inlier_ratio=self.plane_min_inlier_ratio,
            max_expected_error=self.plane_max_expected_error,
            max_median_residual=self.plane_max_median_residual,
            iterations=self.plane_ransac_iterations,
            rng=self.rng,
        )
        self.plane_metrics = metrics
        if candidate is None:
            self.plane_rejections += 1
            return

        if self.plane_coeffs is None:
            if self.plane_candidate is None:
                self.plane_candidate = candidate
                self.plane_candidate_count = 1
            else:
                previous_height = float(plane_height(
                    self.plane_candidate,
                    body_position[0], body_position[1]))
                candidate_height = float(plane_height(
                    candidate, body_position[0], body_position[1]))
                slope_delta = float(np.linalg.norm(
                    candidate[:2] - self.plane_candidate[:2]))
                if (abs(candidate_height - previous_height) <= self.plane_max_jump and
                        slope_delta <= math.tan(math.radians(2.0))):
                    self.plane_candidate = (
                        0.5 * self.plane_candidate + 0.5 * candidate)
                    self.plane_candidate_count += 1
                else:
                    self.plane_candidate = candidate
                    self.plane_candidate_count = 1

            if self.plane_candidate_count >= self.plane_init_frames:
                self.plane_coeffs = self.plane_candidate.copy()
                a, b, c = self.plane_coeffs
                self.get_logger().info(
                    "Validated ground plane initialized: "
                    f"z={a:+.5f}*x{b:+.5f}*y{c:+.4f} "
                    f"tilt={metrics['tilt_deg']:.2f}deg "
                    f"inliers={metrics['inlier_count']}/"
                    f"{metrics['candidate_count']} "
                    f"median={metrics['median_residual']:.3f}m")
            return

        old_height = float(plane_height(
            self.plane_coeffs, body_position[0], body_position[1]))
        new_height = float(plane_height(
            candidate, body_position[0], body_position[1]))
        if abs(new_height - old_height) > self.plane_max_jump:
            self.plane_rejections += 1
            self.plane_metrics = dict(metrics)
            self.plane_metrics["reason"] = "plane_jump"
            return
        self.plane_coeffs = (
            self.plane_smooth * self.plane_coeffs +
            (1.0 - self.plane_smooth) * candidate)

    def _ingest(self, xs, ys, ground_mask, obstacle_mask, sensor_xy):
        relevant = ground_mask | obstacle_mask
        if not relevant.any():
            return 0, 0

        relevant_x = xs[relevant]
        relevant_y = ys[relevant]
        x_min = min(float(relevant_x.min()), float(sensor_xy[0]))
        x_max = max(float(relevant_x.max()), float(sensor_xy[0]))
        y_min = min(float(relevant_y.min()), float(sensor_xy[1]))
        y_max = max(float(relevant_y.max()), float(sensor_xy[1]))

        if self.grid is None:
            self.origin_x = x_min - self.grid_margin
            self.origin_y = y_min - self.grid_margin
            width = max(1, int(math.ceil(
                (x_max - x_min + 2.0 * self.grid_margin) / self.res)))
            height = max(1, int(math.ceil(
                (y_max - y_min + 2.0 * self.grid_margin) / self.res)))
            self.grid = np.full((height, width), UNKNOWN, dtype=np.int8)
            self.log_odds = np.zeros((height, width), dtype=np.float32)
            self.observed = np.zeros((height, width), dtype=bool)
        else:
            self._grow_to_include(
                x_min, y_min, x_max, y_max, margin=self.grid_margin)

        ix = ((xs - self.origin_x) / self.res).astype(np.int64)
        iy = ((ys - self.origin_y) / self.res).astype(np.int64)
        height, width = self.grid.shape
        valid = (
            (ix >= 0) & (ix < width) &
            (iy >= 0) & (iy < height)
        )
        ix, iy = ix[valid], iy[valid]
        valid_xs, valid_ys = xs[valid], ys[valid]
        ground_mask = ground_mask[valid]
        obstacle_mask = obstacle_mask[valid]

        ground_cells = unique_cell_ids(
            ix[ground_mask], iy[ground_mask], width)
        ground_cells = expand_cell_ids(
            ground_cells,
            width,
            height,
            self.free_spread_radius,
            self.res,
            assume_unique=True,
        )
        obstacle_cells = unique_cell_ids(
            ix[obstacle_mask], iy[obstacle_mask], width)
        obstacle_cells = expand_cell_ids(
            obstacle_cells,
            width,
            height,
            self.obstacle_spread_radius,
            self.res,
            assume_unique=True,
        )
        free_cells = ground_cells

        if self.raytrace:
            endpoint_indices, include_endpoint = select_ray_endpoint_indices(
                valid_xs,
                valid_ys,
                ground_mask,
                obstacle_mask,
                sensor_xy,
                max_range=self.raytrace_range,
                angle_bins=self.raytrace_bins,
            )
            if len(endpoint_indices):
                endpoint_ix = ((
                    valid_xs[endpoint_indices] - self.origin_x
                ) / self.res).astype(np.int64)
                endpoint_iy = ((
                    valid_ys[endpoint_indices] - self.origin_y
                ) / self.res).astype(np.int64)
                origin_ix = int((sensor_xy[0] - self.origin_x) / self.res)
                origin_iy = int((sensor_xy[1] - self.origin_y) / self.res)
                ray_cells = raytrace_cell_ids(
                    origin_ix,
                    origin_iy,
                    endpoint_ix,
                    endpoint_iy,
                    include_endpoint,
                    width,
                    height,
                )
                free_cells = np.union1d(free_cells, ray_cells)

        free_cells = np.setdiff1d(
            free_cells, obstacle_cells, assume_unique=True)
        update_log_odds_grid(
            self.grid,
            self.log_odds,
            self.observed,
            free_cells,
            obstacle_cells,
            free_update=self.free_update,
            obstacle_update=self.obstacle_update,
            minimum=self.log_odds_min,
            maximum=self.log_odds_max,
            free_threshold=self.free_threshold,
            occupied_threshold=self.occupied_threshold,
            free_value=FREE,
            occupied_value=OCCUPIED,
            unknown_value=UNKNOWN,
            # unique_cell_ids/expand_cell_ids return unique ids, and the
            # obstacle-wins setdiff above makes the two arrays disjoint.
            assume_unique_disjoint=True,
        )
        return int(len(free_cells)), int(len(obstacle_cells))

    def _grow_to_include(self, x_min, y_min, x_max, y_max, margin):
        height, width = self.grid.shape
        current_x_max = self.origin_x + width * self.res
        current_y_max = self.origin_y + height * self.res
        need_left = max(0.0, self.origin_x - (x_min - margin))
        need_right = max(0.0, (x_max + margin) - current_x_max)
        need_bottom = max(0.0, self.origin_y - (y_min - margin))
        need_top = max(0.0, (y_max + margin) - current_y_max)
        if need_left == need_right == need_bottom == need_top == 0.0:
            return

        pad_left = int(math.ceil(need_left / self.res))
        pad_right = int(math.ceil(need_right / self.res))
        pad_bottom = int(math.ceil(need_bottom / self.res))
        pad_top = int(math.ceil(need_top / self.res))
        new_width = width + pad_left + pad_right
        new_height = height + pad_bottom + pad_top

        new_grid = np.full(
            (new_height, new_width), UNKNOWN, dtype=np.int8)
        new_grid[
            pad_bottom:pad_bottom + height,
            pad_left:pad_left + width,
        ] = self.grid
        self.grid = new_grid

        new_log_odds = np.zeros(
            (new_height, new_width), dtype=np.float32)
        new_log_odds[
            pad_bottom:pad_bottom + height,
            pad_left:pad_left + width,
        ] = self.log_odds
        self.log_odds = new_log_odds

        new_observed = np.zeros((new_height, new_width), dtype=bool)
        new_observed[
            pad_bottom:pad_bottom + height,
            pad_left:pad_left + width,
        ] = self.observed
        self.observed = new_observed

        self.origin_x -= pad_left * self.res
        self.origin_y -= pad_bottom * self.res

    def _sample_debug_points(self, points):
        if points is None or len(points) == 0:
            return []
        step = max(1, int(math.ceil(len(points) / self.max_debug_points)))
        return points[::step].astype(np.float32).tolist()

    def _publish_debug(self, msg, ground_points, obstacle_points, self_points):
        if not self.debug_clouds:
            return
        if self.processed_frames % self.debug_every != 0:
            return
        header = Header()
        header.stamp = msg.header.stamp
        header.frame_id = self.map_frame
        self.ground_debug_pub.publish(pc2.create_cloud_xyz32(
            header, self._sample_debug_points(ground_points)))
        self.obstacle_debug_pub.publish(pc2.create_cloud_xyz32(
            header, self._sample_debug_points(obstacle_points)))
        self.self_debug_pub.publish(pc2.create_cloud_xyz32(
            header, self._sample_debug_points(self_points)))

    def publish_grid(self):
        with self.lock:
            if self.grid is None:
                return
            height, width = self.grid.shape
            data = self.grid.ravel().tolist()
            origin_x, origin_y = self.origin_x, self.origin_y
            if self.use_ground_plane and self.plane_coeffs is not None:
                display_z = float(plane_height(
                    self.plane_coeffs, origin_x, origin_y))
            else:
                display_z = self.map_z

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = width
        msg.info.height = height
        pose = Pose()
        pose.position.x = origin_x
        pose.position.y = origin_y
        pose.position.z = display_z
        pose.orientation.w = 1.0
        msg.info.origin = pose
        msg.data = data
        self.pub.publish(msg)

    def log_stats(self):
        plane_reason = self.plane_metrics.get("reason", "unknown")
        plane_info = f" plane={plane_reason}"
        candidate_count = self.plane_metrics.get("candidate_count")
        inlier_count = self.plane_metrics.get("inlier_count")
        median_residual = self.plane_metrics.get("median_residual")
        if candidate_count is not None and inlier_count is not None:
            plane_info += f" inliers={inlier_count}/{candidate_count}"
        if median_residual is not None and math.isfinite(median_residual):
            plane_info += f" residual={median_residual:.3f}m"
        if self.plane_coeffs is not None:
            tilt = math.degrees(math.atan(math.hypot(
                self.plane_coeffs[0], self.plane_coeffs[1])))
            plane_info += (
                f" floor_z={self.plane_coeffs[2]:.3f}m"
                f" tilt={tilt:.2f}deg")
        sync_info = (
            f" sync_pending={len(self.pending_clouds)}"
            f" sync_dropped={self.unmatched_clouds}"
            f" invalid={self.invalid_frames}"
            f" plane_rejected={self.plane_rejections}")

        if self.grid is None:
            self.get_logger().info(
                f"frames={self.frames} processed={self.processed_frames} "
                f"waiting_for_valid_grid{plane_info}{sync_info}")
            return

        with self.lock:
            height, width = self.grid.shape
            free_cells = int((self.grid == FREE).sum())
            occupied_cells = int((self.grid == OCCUPIED).sum())
            unknown_cells = int((self.grid == UNKNOWN).sum())
        counts = self.last_class_counts
        self.get_logger().info(
            f"frames={self.frames} processed={self.processed_frames} "
            f"classified={self.classified_frames} "
            f"last_ground={counts.get('ground', 0)} "
            f"last_obs={counts.get('obstacle', 0)} "
            f"last_below={counts.get('below', 0)} "
            f"last_high={counts.get('high', 0)} "
            f"last_self={counts.get('self', 0)} "
            f"grid={width}x{height} free={free_cells} "
            f"occ={occupied_cells} unknown={unknown_cells}"
            f"{plane_info}{sync_info}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--resolution", type=float, default=0.05)
    parser.add_argument("--rate", type=float, default=2.0)
    parser.add_argument("--map-frame", default="camera_init")
    parser.add_argument("--body-frame", default="body")
    parser.add_argument("--cloud-topic", default="/cloud_registered_1")
    parser.add_argument("--odom-topic", default="/Odometry_loc")
    parser.add_argument("--grid-topic", default="/accumulated_grid")
    parser.add_argument("--pose-cache-size", type=int, default=30)
    parser.add_argument("--skip-frames", type=int, default=20)
    parser.add_argument("--grid-margin", type=float, default=5.0)
    parser.add_argument("--map-z", type=float, default=-1.247)

    parser.add_argument(
        "--pre-transformed", action="store_true", default=True)
    parser.add_argument(
        "--no-pre-transformed", dest="pre_transformed", action="store_false")
    parser.add_argument("--invert-z", action="store_true")
    parser.add_argument("--z-offset", type=float, default=0.0)
    parser.add_argument("--ground-z-min", type=float, default=-1.35)
    parser.add_argument("--ground-z", type=float, default=-1.15)
    parser.add_argument("--obstacle-z", type=float, default=-1.14)
    parser.add_argument("--obstacle-z-max", type=float, default=0.35)

    parser.add_argument(
        "--use-ground-plane", action="store_true", default=True)
    parser.add_argument(
        "--no-ground-plane", dest="use_ground_plane", action="store_false")
    parser.add_argument("--sensor-height", type=float, default=1.247)
    parser.add_argument(
        "--below-ground-tolerance", type=float, default=0.10)
    parser.add_argument("--ground-margin", type=float, default=0.08)
    parser.add_argument("--obstacle-margin", type=float, default=0.10)
    parser.add_argument(
        "--max-obstacle-height", type=float, default=1.60)
    parser.add_argument("--plane-smooth", type=float, default=0.90)
    parser.add_argument("--plane-update-interval", type=int, default=10)
    parser.add_argument("--plane-init-frames", type=int, default=3)
    parser.add_argument("--plane-fit-min-range", type=float, default=0.80)
    parser.add_argument("--plane-fit-max-range", type=float, default=12.0)
    parser.add_argument("--plane-candidate-band", type=float, default=0.25)
    parser.add_argument("--plane-max-tilt-deg", type=float, default=5.0)
    parser.add_argument(
        "--plane-distance-threshold", type=float, default=0.05)
    parser.add_argument("--plane-min-inliers", type=int, default=80)
    parser.add_argument(
        "--plane-min-inlier-ratio", type=float, default=0.20)
    parser.add_argument(
        "--plane-max-expected-error", type=float, default=0.18)
    parser.add_argument(
        "--plane-max-median-residual", type=float, default=0.035)
    parser.add_argument("--plane-max-jump", type=float, default=0.08)
    parser.add_argument("--plane-ransac-iterations", type=int, default=64)

    parser.add_argument("--max-point-range", type=float, default=30.0)
    parser.add_argument(
        "--no-self-filter", dest="self_filter", action="store_false")
    parser.set_defaults(self_filter=True)
    parser.add_argument("--self-x-min", type=float, default=-0.40)
    parser.add_argument("--self-x-max", type=float, default=0.40)
    parser.add_argument("--self-y-abs", type=float, default=0.40)
    parser.add_argument("--self-z-min", type=float, default=-1.35)
    parser.add_argument("--self-z-max", type=float, default=0.15)
    parser.add_argument("--lidar-offset-x", type=float, default=-0.011)
    parser.add_argument("--lidar-offset-y", type=float, default=-0.02329)
    parser.add_argument("--lidar-offset-z", type=float, default=0.04412)

    parser.add_argument("--raytrace", dest="raytrace", action="store_true")
    parser.add_argument(
        "--no-raytrace", dest="raytrace", action="store_false")
    parser.set_defaults(raytrace=False)
    parser.add_argument("--raytrace-range", type=float, default=15.0)
    parser.add_argument("--raytrace-bins", type=int, default=360)
    parser.add_argument("--obstacle-spread-radius", type=float, default=0.05)
    parser.add_argument("--free-spread-radius", type=float, default=0.05)
    parser.add_argument("--min-obs-hits", type=int, default=3)
    parser.add_argument("--free-update", type=float, default=0.30)
    parser.add_argument("--obstacle-update", type=float, default=0.85)
    parser.add_argument("--log-odds-min", type=float, default=-2.0)
    parser.add_argument("--log-odds-max", type=float, default=3.5)
    parser.add_argument("--free-threshold", type=float, default=-0.40)
    parser.add_argument("--occupied-threshold", type=float, default=None)

    parser.add_argument("--debug-clouds", action="store_true")
    parser.add_argument("--debug-every", type=int, default=10)
    parser.add_argument("--max-debug-points", type=int, default=5000)
    parser.add_argument("--ground-debug-topic", default="/grid_ground_debug")
    parser.add_argument(
        "--obstacle-debug-topic", default="/grid_obstacle_debug")
    parser.add_argument("--self-debug-topic", default="/grid_self_debug")
    args = parser.parse_args()

    rclpy.init()
    node = GridAccumulator(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"shutting down: frames={node.frames} "
            f"processed={node.processed_frames} "
            f"classified={node.classified_frames}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
