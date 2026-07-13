#!/usr/bin/env python3
"""Build a 2D OccupancyGrid from FAST-LIO registered point clouds.

Preferred input: /cloud_registered_1 (PointCloud2, camera_init/world frame).
Legacy input: /cloud_registered_body_1 (PointCloud2, body frame plus TF).
Publishes: /accumulated_grid (OccupancyGrid, camera_init/map frame).

The preferred world-frame mode avoids applying a second, time-sensitive
body-to-map transform and keeps ground-plane fitting in one coordinate frame.
Body-frame mode remains available for legacy callers with
``--no-pre-transformed``.

FAST-LIO saves the 3D PCD through /map_save (or on shutdown). This node
accumulates the separate 2D OccupancyGrid that nav2_map_server saves as
PGM plus YAML.
"""
import argparse
import math
import os
import sys
import time
from threading import Lock, Thread

os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")

import numpy as np
import rclpy
import rclpy.duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


FREE = 0
OCCUPIED = 100
UNKNOWN = -1


class GridAccumulator(Node):
    def __init__(self, args):
        super().__init__("grid_accumulator")
        self.res = args.resolution
        self.ground_z = args.ground_z
        self.ground_z_min = args.ground_z_min
        self.obstacle_z = args.obstacle_z
        self.obstacle_z_max = args.obstacle_z_max
        self.invert_z = args.invert_z
        self.z_offset = args.z_offset
        self.rate = args.rate
        self.body_frame = args.body_frame
        self.map_frame = args.map_frame
        self.cloud_topic = args.cloud_topic
        self.grid_topic = args.grid_topic
        self.skip_frames = args.skip_frames
        self.pre_transformed = args.pre_transformed  # cloud already in map frame
        self.min_obs_hits = args.min_obs_hits        # hits needed before marking OCCUPIED
        self.map_z = args.map_z  # z-offset for grid display in 3D view

        # Ground-plane estimation (RANSAC-style least-squares) — robust to
        # tilted camera_init frames. When enabled, classification uses
        # height-above-fitted-plane instead of absolute z thresholds.
        # This is critical for bipedal robots where the torso/IMU may be
        # slightly tilted at startup or during walking.
        self.use_ground_plane = getattr(args, 'use_ground_plane', True)
        self.ground_margin = getattr(args, 'ground_margin', 0.08)     # 8 cm above plane = FREE
        self.obstacle_margin = getattr(args, 'obstacle_margin', 0.15) # 15 cm above plane = OCCUPIED start
        self.max_obstacle_height = getattr(args, 'max_obstacle_height', 2.5) # 2.5m above local floor = ceiling
        self.plane_coeffs = None  # [a, b, c] for z = a*x + b*y + c
        self.plane_smooth = getattr(args, 'plane_smooth', 0.9)  # EMA smoothing factor
        self.plane_initialized = False

        if not self.pre_transformed:
            self.tf_buf = Buffer()
            self.tf_listener = TransformListener(self.tf_buf, self)

        self.grid = None             # int8 numpy array, shape (h, w)
        self.hit_count = None        # uint16 numpy array, counts obstacle hits per cell
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.lock = Lock()

        self.frames = 0
        self.ground_pts = 0
        self.obs_pts = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            PointCloud2, self.cloud_topic, self.cloud_cb, sensor_qos)
        latched_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(OccupancyGrid, self.grid_topic, latched_qos)
        self.create_timer(1.0 / self.rate, self.publish_grid)
        self.create_timer(5.0, self.log_stats)

        self.get_logger().info(
            f"grid_accumulator: res={self.res}m, "
            f"ground_plane={'ON' if self.use_ground_plane else 'OFF'}, "
            f"ground=[{self.ground_z_min},{self.ground_z}), "
            f"obs=[{self.obstacle_z},{self.obstacle_z_max}), "
            f"margin_free={self.ground_margin}m margin_obs={self.obstacle_margin}m, "
            f"min_obs_hits={self.min_obs_hits}, "
            f"sub={self.cloud_topic}, pub={self.grid_topic}")

    def log_stats(self):
        if self.grid is None:
            self.get_logger().info(
                f"frames={self.frames} (no grid yet; waiting for first cloud + TF)")
        else:
            plane_info = ""
            if self.use_ground_plane and self.plane_coeffs is not None:
                _, _, c = self.plane_coeffs
                plane_info = f" floor_z={c:.3f}m"
            self.get_logger().info(
                f"frames={self.frames} ground={self.ground_pts} obs={self.obs_pts} "
                f"grid={self.grid.shape[1]}x{self.grid.shape[0]} "
                f"origin=({self.origin_x:.2f},{self.origin_y:.2f}){plane_info}")

    def cloud_cb(self, msg: PointCloud2):
        self.frames += 1
        if self.frames <= self.skip_frames:
            return  # skip fast_lio initialization frames (unstable pose)

        raw = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        raw = np.asarray(list(raw))
        if raw.size == 0:
            return
        if raw.dtype.names is not None:
            pts = np.column_stack([raw["x"], raw["y"], raw["z"]]).astype(np.float64)
        else:
            pts = raw.reshape(-1, 3).astype(np.float64)

        if self.pre_transformed:
            # /cloud_registered_1: points already in map frame — no TF needed.
            pts_map = pts
            fixed_z = pts[:, 2]
        else:
            # /cloud_registered_body_1: body frame — TF to map required.
            try:
                tf = self.tf_buf.lookup_transform(
                    self.map_frame, msg.header.frame_id,
                    msg.header.stamp,
                    rclpy.duration.Duration(seconds=0.1))
            except Exception:
                return
            fixed_z = pts[:, 2]
            if self.invert_z:
                fixed_z = -fixed_z
            fixed_z = fixed_z + self.z_offset
            q = tf.transform.rotation
            t = tf.transform.translation
            xx, yy, zz, ww = q.x, q.y, q.z, q.w
            rot = np.array([
                [1-2*(yy*yy+zz*zz), 2*(xx*yy-zz*ww), 2*(xx*zz+yy*ww)],
                [2*(xx*yy+zz*ww), 1-2*(xx*xx+zz*zz), 2*(yy*zz-xx*ww)],
                [2*(xx*zz-yy*ww), 2*(yy*zz+xx*ww), 1-2*(xx*xx+yy*yy)],
            ])
            pts_map = pts @ rot.T + np.array([t.x, t.y, t.z])

        if self.use_ground_plane:
            # ---- plane-relative classification (tilt-robust) ----
            # Plane x/y/z must all be in the same map frame. The previous
            # body-cloud path mixed map-frame x/y with body-frame z.
            plane_z = pts_map[:, 2]
            coeffs = self._estimate_ground_plane(
                pts_map[:, 0], pts_map[:, 1], plane_z)
            if coeffs is not None:
                ground_mask, obs_mask = self._classify_by_plane(
                    pts_map[:, 0], pts_map[:, 1], plane_z, coeffs)
            else:
                # Fallback to map-frame fixed-z while the plane converges.
                ground_mask = ((plane_z >= self.ground_z_min) &
                               (plane_z < self.ground_z))
                obs_mask = ((plane_z < self.ground_z_min) |
                            ((plane_z > self.obstacle_z) &
                             (plane_z < self.obstacle_z_max)))
        else:
            # ---- fixed-z classification (legacy) ----
            ground_mask = ((fixed_z >= self.ground_z_min) &
                           (fixed_z < self.ground_z))
            obs_mask = ((fixed_z < self.ground_z_min) |
                        ((fixed_z > self.obstacle_z) &
                         (fixed_z < self.obstacle_z_max)))

        with self.lock:
            self._ingest(pts_map[:, 0], pts_map[:, 1], ground_mask, obs_mask)
            self.ground_pts += int(ground_mask.sum())
            self.obs_pts += int(obs_mask.sum())

    def _estimate_ground_plane(self, xs, ys, zs):
        """Fit a ground plane z = a*x + b*y + c via stratified sampling.

        Divides the x-y space into a grid (~8×8 cells) and takes the
        lowest few z values from EACH cell as floor candidates.  This
        ensures floor samples are spatially distributed across the entire
        mapped area — critical when the camera_init frame is tilted and
        a global "lowest 30% z" would concentrate all samples on the
        downhill side, biasing the plane fit.

        Applies MAD-based outlier rejection within each cell, then a
        least-squares plane fit across all candidate points.  Results
        are EMA-smoothed across frames for stability.

        Returns [a, b, c] coefficients, or None if insufficient points.
        """
        n = len(zs)
        if n < 100:
            return self.plane_coeffs  # keep last known plane

        # ---- stratified sampling: grid cells × lowest-per-cell ----
        x_min, x_max = float(xs.min()), float(xs.max())
        y_min, y_max = float(ys.min()), float(ys.max())
        span_x = x_max - x_min
        span_y = y_max - y_min

        # 8×8 grid = 64 cells; at least 1 m per cell to avoid tiny cells
        cell_size = max(span_x / 8.0, span_y / 8.0, 1.0)
        n_cells_x = max(1, int(np.ceil(span_x / cell_size)))
        n_cells_y = max(1, int(np.ceil(span_y / cell_size)))

        cell_ix = np.clip(
            ((xs - x_min) / cell_size).astype(np.int32), 0, n_cells_x - 1)
        cell_iy = np.clip(
            ((ys - y_min) / cell_size).astype(np.int32), 0, n_cells_y - 1)

        # Collect up to 8 lowest-z points per cell as floor candidates
        PER_CELL = 8
        cell_ground_idx = []
        for cx in range(n_cells_x):
            for cy in range(n_cells_y):
                in_cell = np.where((cell_ix == cx) & (cell_iy == cy))[0]
                n_cell = len(in_cell)
                if n_cell < 4:
                    continue
                n_take = min(PER_CELL, n_cell // 2)
                # local argpartition within this cell
                z_cell = zs[in_cell]
                kth = np.argpartition(z_cell, n_take)[:n_take]
                cell_ground_idx.append(in_cell[kth])

        if not cell_ground_idx:
            return self.plane_coeffs  # not enough cells

        ground_idx = np.concatenate(cell_ground_idx)
        gx = xs[ground_idx]
        gy = ys[ground_idx]
        gz = zs[ground_idx]

        # ---- MAD outlier rejection on the pooled candidates ----
        med_z = np.median(gz)
        mad_z = np.median(np.abs(gz - med_z))
        if mad_z < 0.005:
            mad_z = 0.005
        inlier = np.abs(gz - med_z) < 3.0 * mad_z
        gx, gy, gz = gx[inlier], gy[inlier], gz[inlier]

        if len(gz) < 50:
            return self.plane_coeffs

        # ---- least-squares plane fit ----
        try:
            coeffs, _r, _rank, _s = np.linalg.lstsq(
                np.column_stack([gx, gy, np.ones_like(gx)]), gz, rcond=None)
            new_plane = np.asarray(coeffs, dtype=np.float64)
        except np.linalg.LinAlgError:
            return self.plane_coeffs

        # ---- EMA smoothing ----
        if self.plane_coeffs is None:
            self.plane_coeffs = new_plane
            self.plane_initialized = True
        else:
            alpha = 1.0 - self.plane_smooth
            self.plane_coeffs = (self.plane_smooth * self.plane_coeffs +
                                 alpha * new_plane)

        # ---- log (first 5 + every 500 frames) ----
        if self.frames <= self.skip_frames + 5 or self.frames % 500 == 0:
            a, b, c = self.plane_coeffs
            nz = 1.0 / np.sqrt(a*a + b*b + 1.0)
            tilt_deg = np.degrees(np.arccos(nz))
            # floor height range across the mapped area
            z_min_floor = a * x_min + b * y_min + c
            z_max_floor = a * x_max + b * y_max + c
            self.get_logger().info(
                f"ground plane: z={a:+.4f}*x{b:+.4f}*y{c:+.4f}  "
                f"tilt={tilt_deg:.2f}°  cells={n_cells_x}×{n_cells_y}  "
                f"floor_z_range=[{z_min_floor:.3f}, {z_max_floor:.3f}]m")

        return self.plane_coeffs

    def _classify_by_plane(self, xs, ys, zs, coeffs):
        """Classify points by height-above-fitted-ground-plane.

        - height ∈ [0, ground_margin)          → FREE  (floor surface)
        - height ∈ [obstacle_margin, max_obs_h)→ OCCUPIED (walls, furniture)
        - height < 0                            → OCCUPIED (drop-off / step-down)
        - height ∈ [ground_margin, obstacle_margin) → UNKNOWN (transition zone)
        - height ≥ max_obstacle_height          → IGNORED (ceiling)

        max_obstacle_height is measured from the LOCAL ground plane at each
        (x,y), so the ceiling cutoff stays correct even with tilted maps.
        """
        a, b, c = coeffs
        z_plane = a * xs + b * ys + c   # expected z on the ground plane at each point
        height = zs - z_plane           # height above LOCAL ground plane

        # Per-point ceiling cutoff: points > max_obstacle_height above
        # the local ground are ceiling/overhang → ignored.
        ground_mask = (height >= -0.02) & (height < self.ground_margin)
        obs_mask = ((height < -0.02) |
                    ((height >= self.obstacle_margin) & (height < self.max_obstacle_height)))

        return ground_mask, obs_mask

    def _ingest(self, xs, ys, ground_mask, obs_mask):
        # Determine bbox of new points
        x_min, x_max = float(xs.min()), float(xs.max())
        y_min, y_max = float(ys.min()), float(ys.max())

        if self.grid is None:
            # Initialise grid centred near these points with small margin
            margin = 5.0
            self.origin_x = x_min - margin
            self.origin_y = y_min - margin
            w = int(math.ceil((x_max - x_min + 2 * margin) / self.res))
            h = int(math.ceil((y_max - y_min + 2 * margin) / self.res))
            self.grid = np.full((h, w), UNKNOWN, dtype=np.int8)
            self.hit_count = np.zeros((h, w), dtype=np.uint16)
        else:
            # Grow grid if needed (auto_resize)
            self._grow_to_include(x_min, y_min, x_max, y_max, margin=5.0)

        # Convert to indices
        ix = ((xs - self.origin_x) / self.res).astype(np.int32)
        iy = ((ys - self.origin_y) / self.res).astype(np.int32)
        h, w = self.grid.shape
        valid = (ix >= 0) & (ix < w) & (iy >= 0) & (iy < h)
        ix, iy = ix[valid], iy[valid]
        ground_mask = ground_mask[valid]
        obs_mask = obs_mask[valid]

        # FREE first (so OCCUPIED can win on overlapping cells in same frame)
        free_idx = (iy[ground_mask], ix[ground_mask])
        # Only mark as FREE cells that are still UNKNOWN — don't undo OCCUPIED
        free_undecided = self.grid[free_idx] == UNKNOWN
        self.grid[free_idx[0][free_undecided], free_idx[1][free_undecided]] = FREE
        # OCCUPIED: accumulate hit count, only mark after min_obs_hits threshold
        obs_iy, obs_ix = iy[obs_mask], ix[obs_mask]
        np.add.at(self.hit_count, (obs_iy, obs_ix), 1)
        confirmed = self.hit_count[obs_iy, obs_ix] >= self.min_obs_hits
        self.grid[obs_iy[confirmed], obs_ix[confirmed]] = OCCUPIED

    def _grow_to_include(self, x_min, y_min, x_max, y_max, margin):
        h, w = self.grid.shape
        cur_x_max = self.origin_x + w * self.res
        cur_y_max = self.origin_y + h * self.res
        need_left = max(0.0, self.origin_x - (x_min - margin))
        need_right = max(0.0, (x_max + margin) - cur_x_max)
        need_bottom = max(0.0, self.origin_y - (y_min - margin))
        need_top = max(0.0, (y_max + margin) - cur_y_max)
        if need_left == need_right == need_bottom == need_top == 0:
            return
        pad_l = int(math.ceil(need_left / self.res))
        pad_r = int(math.ceil(need_right / self.res))
        pad_b = int(math.ceil(need_bottom / self.res))
        pad_t = int(math.ceil(need_top / self.res))
        new_w = w + pad_l + pad_r
        new_h = h + pad_b + pad_t
        new_grid = np.full((new_h, new_w), UNKNOWN, dtype=np.int8)
        new_grid[pad_b:pad_b + h, pad_l:pad_l + w] = self.grid
        self.grid = new_grid
        new_hit = np.zeros((new_h, new_w), dtype=np.uint16)
        new_hit[pad_b:pad_b + h, pad_l:pad_l + w] = self.hit_count
        self.hit_count = new_hit
        self.origin_x -= pad_l * self.res
        self.origin_y -= pad_b * self.res

    def publish_grid(self):
        with self.lock:
            if self.grid is None:
                return
            h, w = self.grid.shape
            data = self.grid.flatten().tolist()
            origin_x, origin_y = self.origin_x, self.origin_y
            # Auto-align grid display z to estimated floor height when
            # ground-plane estimation is active. Falls back to fixed --map-z.
            if self.use_ground_plane and self.plane_coeffs is not None:
                _, _, floor_z = self.plane_coeffs
                display_z = float(floor_z)
            else:
                display_z = self.map_z
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = w
        msg.info.height = h
        pose = Pose()
        pose.position.x = origin_x
        pose.position.y = origin_y
        pose.position.z = display_z  # auto or fixed, aligns grid with floor in 3D view
        pose.orientation.w = 1.0
        msg.info.origin = pose
        msg.data = data
        self.pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--resolution", type=float, default=0.05)
    ap.add_argument("--ground-z", type=float, default=0.15)
    ap.add_argument("--ground-z-min", type=float, default=-0.5,
                    help="lower bound for ground z; points below this are step-downs/drops and marked OCCUPIED")
    ap.add_argument("--obstacle-z", type=float, default=0.25)
    ap.add_argument("--obstacle-z-max", type=float, default=1.8,
                    help="upper obstacle-z bound in the fixed-z classification frame; higher points are ignored")
    ap.add_argument("--min-obs-hits", type=int, default=3,
                    help="Minimum scan hits before a cell is marked OCCUPIED (default 3, increase to reduce moving-person false obstacles)")
    ap.add_argument("--rate", type=float, default=2.0)
    ap.add_argument("--body-frame", default="body")
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--cloud-topic", default="/cloud_registered_1")
    ap.add_argument("--grid-topic", default="/accumulated_grid")
    ap.add_argument("--invert-z", action="store_true",
                    help="(body-frame mode only) invert z before classification")
    ap.add_argument("--z-offset", type=float, default=0.0,
                    help="(body-frame mode only) shift z after invert")
    ap.add_argument("--pre-transformed", action="store_true", default=True,
                    help="cloud is already in map frame (/cloud_registered_1); skip TF lookup")
    ap.add_argument("--no-pre-transformed", dest="pre_transformed", action="store_false",
                    help="use body-frame cloud + TF (/cloud_registered_body_1 mode)")
    ap.add_argument("--skip-frames", type=int, default=20,
                    help="skip first N point cloud frames (fast_lio convergence warmup)")
    ap.add_argument("--map-z", type=float, default=0.0,
                    help="z-offset for 2D grid display in 3D view (set to sensor-to-floor distance, e.g., -1.27 for 1.27m sensor height)")
    ap.add_argument("--use-ground-plane", action="store_true", default=True,
                    help="Use RANSAC ground-plane estimation for tilt-robust classification (default: on)")
    ap.add_argument("--no-ground-plane", dest="use_ground_plane", action="store_false",
                    help="Disable ground-plane estimation; use fixed z thresholds instead")
    ap.add_argument("--ground-margin", type=float, default=0.08,
                    help="Height above ground plane considered FREE (m, default 0.08)")
    ap.add_argument("--obstacle-margin", type=float, default=0.15,
                    help="Height above ground plane where obstacles start (m, default 0.15)")
    ap.add_argument("--plane-smooth", type=float, default=0.9,
                    help="EMA smoothing factor for ground plane coefficients (0-1, default 0.9)")
    ap.add_argument("--max-obstacle-height", type=float, default=2.5,
                    help="Max height above local ground plane for obstacles (m, default 2.5); taller = ceiling → ignored")
    args = ap.parse_args()

    rclpy.init()
    node = GridAccumulator(args)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    try:
        while rclpy.ok():
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(
            f"shutting down. final stats: frames={node.frames} "
            f"ground={node.ground_pts} obs={node.obs_pts}")
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
