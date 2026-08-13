"""ROS-independent helpers for building a 2D grid from registered LiDAR scans."""

import math

import numpy as np


def quaternion_to_matrix(quaternion):
    """Return the body-to-world rotation matrix for [x, y, z, w]."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("invalid zero/non-finite quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def world_to_body(points_world, body_position, body_to_world_rotation):
    """Transform row-vector world points into the FAST-LIO body frame."""
    points_world = np.asarray(points_world, dtype=np.float64)
    body_position = np.asarray(body_position, dtype=np.float64)
    rotation = np.asarray(body_to_world_rotation, dtype=np.float64)
    return (points_world - body_position) @ rotation


def plane_height(coefficients, xs, ys):
    coefficients = np.asarray(coefficients, dtype=np.float64)
    return coefficients[0] * xs + coefficients[1] * ys + coefficients[2]


def accumulate_consistent_horizontal_plane(
        previous, count, candidate, *, max_height_delta, required_count,
        max_tilt_delta_deg=None):
    """Accumulate consecutive horizontal floor candidates safely.

    A candidate outside the current cluster starts a new sequence.  The
    caller may replace its active floor model only when ``confirmed`` is true.
    """
    candidate = np.asarray(candidate, dtype=np.float64)
    if candidate.shape != (3,) or not np.isfinite(candidate).all():
        raise ValueError("invalid plane candidate")
    if previous is None:
        return candidate.copy(), 1, required_count <= 1
    previous = np.asarray(previous, dtype=np.float64)
    if previous.shape != (3,) or not np.isfinite(previous).all():
        raise ValueError("invalid previous plane candidate")
    height_inconsistent = (
        abs(float(candidate[2] - previous[2])) > max_height_delta)
    tilt_inconsistent = False
    if max_tilt_delta_deg is not None:
        previous_tilt = np.array(previous[:2], dtype=np.float64)
        candidate_tilt = np.array(candidate[:2], dtype=np.float64)
        tilt_inconsistent = (
            float(np.linalg.norm(candidate_tilt - previous_tilt)) >
            math.tan(math.radians(max_tilt_delta_deg))
        )
    if height_inconsistent or tilt_inconsistent:
        return candidate.copy(), 1, required_count <= 1
    previous_count = max(0, int(count))
    next_count = min(previous_count + 1, max(1, int(required_count)))
    if previous_count >= required_count:
        # Once confirmed, keep a bounded two-sample smoother. An all-history
        # average becomes effectively frozen after long runs and can let a
        # slowly changing floor tilt leak back into the obstacle band.
        averaged = 0.5 * previous + 0.5 * candidate
    else:
        averaged = (previous_count * previous + candidate) / next_count
    return averaged, next_count, next_count >= required_count


def fit_ground_plane_ransac(
        points,
        body_position,
        expected_floor_z,
        *,
        min_range=0.8,
        max_range=12.0,
        candidate_below=0.25,
        candidate_above=0.25,
        max_tilt_deg=5.0,
        distance_threshold=0.05,
        min_inliers=80,
        min_inlier_ratio=0.20,
        max_expected_error=0.18,
        max_median_residual=0.035,
        iterations=64,
        max_candidates=5000,
        rng=None):
    """Fit z=a*x+b*y+c to floor candidates with a constrained RANSAC.

    Candidate selection is anchored to the known G1 sensor height. This keeps
    walls, the robot body, and the ceiling out of the plane hypothesis set.
    """
    points = np.asarray(points, dtype=np.float64)
    body_position = np.asarray(body_position, dtype=np.float64)
    metrics = {
        "reason": "unknown",
        "candidate_count": 0,
        "inlier_count": 0,
        "inlier_ratio": 0.0,
        "tilt_deg": math.inf,
        "median_residual": math.inf,
        "floor_at_body": math.nan,
    }
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < min_inliers:
        metrics["reason"] = "too_few_points"
        return None, metrics

    delta_xy = points[:, :2] - body_position[:2]
    ranges = np.linalg.norm(delta_xy, axis=1)
    candidates_mask = (
        np.isfinite(points).all(axis=1) &
        (ranges >= min_range) &
        (ranges <= max_range) &
        (points[:, 2] >= expected_floor_z - candidate_below) &
        (points[:, 2] <= expected_floor_z + candidate_above)
    )
    candidates = points[candidates_mask]
    metrics["candidate_count"] = int(len(candidates))
    if len(candidates) < min_inliers:
        metrics["reason"] = "too_few_floor_candidates"
        return None, metrics

    if rng is None:
        rng = np.random.default_rng(0)
    if len(candidates) > max_candidates:
        candidates = candidates[
            rng.choice(len(candidates), size=max_candidates, replace=False)]

    best_inliers = None
    best_count = 0
    best_median = math.inf
    max_tilt_cos = math.cos(math.radians(max_tilt_deg))

    for _ in range(iterations):
        sample = candidates[rng.choice(len(candidates), size=3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        normal_norm = np.linalg.norm(normal)
        if normal_norm < 1e-9:
            continue
        normal /= normal_norm
        if normal[2] < 0.0:
            normal = -normal
        if normal[2] < max_tilt_cos:
            continue
        offset = -float(np.dot(normal, sample[0]))
        residuals = np.abs(candidates @ normal + offset)
        inliers = residuals <= distance_threshold
        count = int(inliers.sum())
        if count == 0:
            continue
        median = float(np.median(residuals[inliers]))
        if count > best_count or (count == best_count and median < best_median):
            best_inliers = inliers
            best_count = count
            best_median = median

    required_count = max(
        int(min_inliers),
        int(math.ceil(min_inlier_ratio * len(candidates))),
    )
    if best_inliers is None or best_count < required_count:
        metrics["reason"] = "insufficient_ransac_inliers"
        metrics["inlier_count"] = best_count
        metrics["inlier_ratio"] = best_count / max(1, len(candidates))
        return None, metrics

    inlier_points = candidates[best_inliers]
    centroid = inlier_points.mean(axis=0)
    try:
        _u, _s, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
    except np.linalg.LinAlgError:
        metrics["reason"] = "svd_failed"
        return None, metrics
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    if normal[2] < 0.0:
        normal = -normal
    offset = -float(np.dot(normal, centroid))
    tilt_deg = math.degrees(math.acos(float(np.clip(normal[2], -1.0, 1.0))))
    if normal[2] < max_tilt_cos:
        metrics["reason"] = "tilt_too_large"
        metrics["tilt_deg"] = tilt_deg
        return None, metrics

    residuals = np.abs(candidates @ normal + offset)
    refined_inliers = residuals <= distance_threshold
    inlier_count = int(refined_inliers.sum())
    inlier_ratio = inlier_count / max(1, len(candidates))
    median_residual = (
        float(np.median(residuals[refined_inliers]))
        if inlier_count else math.inf
    )
    if inlier_count < required_count or median_residual > max_median_residual:
        metrics.update({
            "reason": "refined_fit_quality",
            "inlier_count": inlier_count,
            "inlier_ratio": inlier_ratio,
            "tilt_deg": tilt_deg,
            "median_residual": median_residual,
        })
        return None, metrics

    a = -normal[0] / normal[2]
    b = -normal[1] / normal[2]
    c = -offset / normal[2]
    coefficients = np.array([a, b, c], dtype=np.float64)
    floor_at_body = float(plane_height(
        coefficients, body_position[0], body_position[1]))
    expected_error = abs(floor_at_body - expected_floor_z)
    metrics.update({
        "reason": "ok",
        "inlier_count": inlier_count,
        "inlier_ratio": inlier_ratio,
        "tilt_deg": tilt_deg,
        "median_residual": median_residual,
        "floor_at_body": floor_at_body,
        "expected_error": expected_error,
    })
    if expected_error > max_expected_error:
        metrics["reason"] = "unexpected_floor_height"
        return None, metrics
    return coefficients, metrics


def classify_points(
        points,
        coefficients,
        *,
        below_ground_tolerance=0.10,
        ground_margin=0.08,
        obstacle_margin=0.10,
        max_obstacle_height=1.60):
    """Classify floor and navigation-height obstacle points.

    Points below the floor tolerance, in the transition band, or above the
    navigation obstacle height are intentionally ignored.
    """
    points = np.asarray(points, dtype=np.float64)
    local_floor = plane_height(coefficients, points[:, 0], points[:, 1])
    heights = points[:, 2] - local_floor
    ground_mask = (
        (heights >= -below_ground_tolerance) &
        (heights < ground_margin)
    )
    obstacle_mask = (
        (heights >= obstacle_margin) &
        (heights < max_obstacle_height)
    )
    return ground_mask, obstacle_mask, heights


def bridge_scan_surface_gaps(
        ranges,
        *,
        angle_min,
        angle_increment,
        max_angle_gap,
        max_surface_gap):
    """Fill only the angular gaps bounded by one short physical surface.

    A voxel-filtered cloud can contain real wall returns that are several
    LaserScan bins apart.  Passing those sparse bins to a costmap denoiser can
    erase the wall.  This function reconstructs the ray/line-segment
    intersections between two measured endpoints, but only when the endpoints
    are close in both angle and Euclidean distance.  It therefore does not
    bridge doorways or invent an obstacle beyond the measured segment.
    """
    output = np.asarray(ranges, dtype=np.float64).copy()
    if output.ndim != 1:
        raise ValueError("ranges must be one-dimensional")
    if (
        len(output) < 3 or angle_increment <= 0.0 or
        max_angle_gap <= angle_increment or max_surface_gap <= 0.0
    ):
        return output, 0

    measured = np.flatnonzero(np.isfinite(output))
    if len(measured) < 2:
        return output, 0

    filled = 0
    max_bin_gap = int(math.floor(max_angle_gap / angle_increment))
    for first, second in zip(measured[:-1], measured[1:]):
        bin_gap = int(second - first)
        if bin_gap <= 1 or bin_gap > max_bin_gap:
            continue

        first_angle = angle_min + float(first) * angle_increment
        second_angle = angle_min + float(second) * angle_increment
        first_point = np.array([
            output[first] * math.cos(first_angle),
            output[first] * math.sin(first_angle),
        ])
        second_point = np.array([
            output[second] * math.cos(second_angle),
            output[second] * math.sin(second_angle),
        ])
        segment = second_point - first_point
        if float(np.linalg.norm(segment)) > max_surface_gap:
            continue

        for index in range(int(first) + 1, int(second)):
            angle = angle_min + float(index) * angle_increment
            direction = np.array([math.cos(angle), math.sin(angle)])
            denominator = (
                segment[0] * direction[1] -
                segment[1] * direction[0]
            )
            if abs(float(denominator)) < 1e-9:
                continue
            numerator = -(
                first_point[0] * direction[1] -
                first_point[1] * direction[0]
            )
            fraction = numerator / denominator
            if fraction < -1e-6 or fraction > 1.0 + 1e-6:
                continue
            intersection = first_point + fraction * segment
            ray_range = float(np.dot(intersection, direction))
            if ray_range > 0.0 and math.isfinite(ray_range):
                output[index] = min(output[index], ray_range)
                filled += 1
    return output, filled


def navigation_scan_ranges(
        points,
        *,
        angle_min=-math.pi,
        angle_max=math.pi,
        angle_increment=0.007,
        range_min=0.45,
        range_max=5.0,
        min_obstacle_height=0.20,
        max_obstacle_height=1.35,
        ground_coefficients=None,
        structure_cell_size=0.30,
        structure_min_vertical_span=0.16,
        structure_neighbor_cells=1,
        max_bridge_angle=0.18,
        max_bridge_distance=0.45):
    """Project geometrically verified obstacles into a navigation scan.

    A point being above one fitted floor plane is not sufficient evidence of
    an obstacle.  FAST-LIO floor warping can leave a thin, persistent sheet
    above that plane, which also defeats a temporal persistence filter.  We
    therefore require each obstacle endpoint to be next to a local XY column
    with real vertical extent.  Walls, people and furniture sides satisfy this
    condition; a lifted but locally thin floor sheet does not.

    The small neighbor radius lets a box top inherit evidence from its visible
    side without flooding a broad horizontal floor patch.  This geometric gate
    deliberately runs before the later near-range/temporal safety policy.
    """
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if angle_increment <= 0.0 or angle_max <= angle_min:
        raise ValueError("invalid scan angle limits")
    if range_min < 0.0 or range_max <= range_min:
        raise ValueError("invalid scan range limits")
    if max_obstacle_height <= min_obstacle_height:
        raise ValueError("invalid obstacle height limits")
    if structure_cell_size <= 0.0:
        raise ValueError("structure_cell_size must be positive")
    if structure_min_vertical_span <= 0.0:
        raise ValueError("structure_min_vertical_span must be positive")

    bin_count = int(math.floor(
        (angle_max - angle_min) / angle_increment)) + 1
    scan = np.full(bin_count, np.inf, dtype=np.float64)
    if len(points) == 0:
        return scan, {
            "finite_points": 0,
            "obstacle_points": 0,
            "height_candidate_points": 0,
            "rejected_thin_points": 0,
            "structure_cells": 0,
            "measured_bins": 0,
            "bridged_bins": 0,
        }

    finite = np.isfinite(points).all(axis=1)
    filtered = points[finite]
    if ground_coefficients is None:
        heights = filtered[:, 2]
    else:
        heights = filtered[:, 2] - plane_height(
            ground_coefficients, filtered[:, 0], filtered[:, 1])
    planar_ranges = np.hypot(filtered[:, 0], filtered[:, 1])
    angles = np.arctan2(filtered[:, 1], filtered[:, 0])
    height_candidate = (
        (heights >= min_obstacle_height) &
        (heights <= max_obstacle_height) &
        (planar_ranges >= range_min) &
        (planar_ranges <= range_max) &
        (angles >= angle_min) &
        (angles <= angle_max)
    )

    # Aggregate vertical evidence in fixed base-frame XY columns. Only points
    # around the navigable floor-to-obstacle band participate; deep floor
    # outliers and ceiling returns must not manufacture a large span.
    structure_domain = (
        (heights >= -0.10) &
        (heights <= max_obstacle_height) &
        (planar_ranges >= range_min) &
        (planar_ranges <= range_max)
    )
    cell_x = np.floor(filtered[:, 0] / structure_cell_size).astype(np.int64)
    cell_y = np.floor(filtered[:, 1] / structure_cell_size).astype(np.int64)
    domain_indices = np.flatnonzero(structure_domain)
    structural_cells = set()
    if len(domain_indices):
        domain_cells = np.column_stack((
            cell_x[domain_indices], cell_y[domain_indices]))
        unique_cells, inverse = np.unique(
            domain_cells, axis=0, return_inverse=True)
        cell_min = np.full(len(unique_cells), np.inf, dtype=np.float64)
        cell_max = np.full(len(unique_cells), -np.inf, dtype=np.float64)
        cell_count = np.zeros(len(unique_cells), dtype=np.int32)
        np.minimum.at(cell_min, inverse, heights[domain_indices])
        np.maximum.at(cell_max, inverse, heights[domain_indices])
        np.add.at(cell_count, inverse, 1)
        structural = (
            (cell_count >= 2) &
            ((cell_max - cell_min) >= structure_min_vertical_span) &
            (cell_max >= min_obstacle_height)
        )
        structural_cells = {
            (int(x), int(y)) for x, y in unique_cells[structural]
        }

    obstacle = np.zeros(len(filtered), dtype=bool)
    candidate_indices = np.flatnonzero(height_candidate)
    neighbor_radius = max(0, int(structure_neighbor_cells))
    for point_index in candidate_indices:
        cx = int(cell_x[point_index])
        cy = int(cell_y[point_index])
        supported = any(
            (cx + dx, cy + dy) in structural_cells
            for dx in range(-neighbor_radius, neighbor_radius + 1)
            for dy in range(-neighbor_radius, neighbor_radius + 1)
        )
        if supported:
            obstacle[point_index] = True

    obstacle_ranges = planar_ranges[obstacle]
    obstacle_angles = angles[obstacle]
    if len(obstacle_ranges):
        bins = np.floor(
            (obstacle_angles - angle_min) / angle_increment
        ).astype(np.int64)
        bins = np.clip(bins, 0, bin_count - 1)
        np.minimum.at(scan, bins, obstacle_ranges)

    measured_bins = int(np.isfinite(scan).sum())
    scan, bridged_bins = bridge_scan_surface_gaps(
        scan,
        angle_min=angle_min,
        angle_increment=angle_increment,
        max_angle_gap=max_bridge_angle,
        max_surface_gap=max_bridge_distance,
    )
    return scan, {
        "finite_points": int(len(filtered)),
        "obstacle_points": int(len(obstacle_ranges)),
        "height_candidate_points": int(height_candidate.sum()),
        "rejected_thin_points": int(height_candidate.sum() - obstacle.sum()),
        "structure_cells": int(len(structural_cells)),
        "measured_bins": measured_bins,
        "bridged_bins": int(bridged_bins),
    }


def confirm_temporal_scan_obstacles(
        current_ranges,
        previous_ranges,
        previous_confirmations,
        *,
        required_frames=2,
        angle_window_bins=8,
        range_tolerance=0.25,
        immediate_range=0.90):
    """Accumulate obstacle evidence and decay it across short dropouts.

    This is an occupancy-evidence filter rather than a Kalman position filter:
    hits add two units, misses remove one, and publication uses hysteresis.
    Thus one- and two-frame returns never become obstacles when three frames
    are required, while an already confirmed wall/person can survive one lost
    scan without making the costmap blink. Far endpoints are associated within
    an angular/range window to tolerate normal robot and obstacle motion.

    Returns within ``immediate_range`` bypass only the temporal confirmation.
    The caller must apply geometric ground rejection before this function.

    Returns ``(published, tracked_ranges, evidence, published_count)``. The
    tracked state, not the published output, must be supplied on the next call
    so unconfirmed candidates and one-frame held obstacles retain evidence.
    """
    current = np.asarray(current_ranges, dtype=np.float64)
    if current.ndim != 1:
        raise ValueError("current_ranges must be one-dimensional")
    if required_frames <= 1:
        confirmations = np.where(np.isfinite(current), 1, 0).astype(np.uint8)
        return (
            current.copy(), current.copy(), confirmations,
            int(np.isfinite(current).sum()),
        )

    previous = None
    previous_counts = None
    if previous_ranges is not None:
        previous = np.asarray(previous_ranges, dtype=np.float64)
        if previous.shape != current.shape:
            previous = None
    if previous_confirmations is not None:
        previous_counts = np.asarray(previous_confirmations, dtype=np.uint8)
        if previous_counts.shape != current.shape:
            previous_counts = None

    output = np.full(current.shape, np.inf, dtype=np.float64)
    tracked = np.full(current.shape, np.inf, dtype=np.float64)
    confirmations = np.zeros(current.shape, dtype=np.uint8)
    finite_indices = np.flatnonzero(np.isfinite(current))
    confirmed_count = 0
    window = max(0, int(angle_window_bins))
    tolerance = max(0.0, float(range_tolerance))
    hit_increment = 2
    publish_threshold = max(1, 2 * int(required_frames) - 1)
    matched_previous = set()

    for index in finite_indices:
        distance = float(current[index])
        previous_count = 0
        if previous is not None:
            start = max(0, int(index) - window)
            stop = min(len(current), int(index) + window + 1)
            candidates = np.flatnonzero(
                np.isfinite(previous[start:stop]) &
                (np.abs(previous[start:stop] - distance) <= tolerance)
            )
            if len(candidates):
                candidate_indices = candidates + start
                best = int(candidate_indices[np.argmin(
                    np.abs(previous[candidate_indices] - distance)
                )])
                matched_previous.add(best)
                previous_count = (
                    int(previous_counts[best])
                    if previous_counts is not None else hit_increment
                )

        confirmations[index] = (
            min(255, publish_threshold + 1)
            if distance <= immediate_range else
            min(255, previous_count + hit_increment)
        )
        tracked[index] = distance
        if confirmations[index] >= publish_threshold:
            output[index] = distance

    # Decay unmatched tracks instead of clearing them abruptly. One missed
    # scan is held after confirmation; a second consecutive miss falls below
    # the publication threshold with the default three-frame policy.
    if previous is not None:
        for previous_index in np.flatnonzero(np.isfinite(previous)):
            previous_index = int(previous_index)
            if previous_index in matched_previous:
                continue
            if np.isfinite(tracked[previous_index]):
                continue
            old_count = (
                int(previous_counts[previous_index])
                if previous_counts is not None else hit_increment
            )
            decayed = max(0, old_count - 1)
            if decayed <= 0:
                continue
            tracked[previous_index] = previous[previous_index]
            confirmations[previous_index] = decayed
            if decayed >= publish_threshold:
                output[previous_index] = previous[previous_index]

    confirmed_count = int(np.isfinite(output).sum())
    return output, tracked, confirmations, confirmed_count


def unique_cell_ids(ix, iy, width):
    if len(ix) == 0:
        return np.empty(0, dtype=np.int64)
    return np.unique(
        np.asarray(iy, dtype=np.int64) * int(width) +
        np.asarray(ix, dtype=np.int64))


def expand_cell_ids(
        cell_ids, width, height, radius, resolution, *, assume_unique=False):
    """Expand occupied evidence to a small metric neighborhood."""
    cell_ids = np.asarray(cell_ids, dtype=np.int64)
    if not assume_unique:
        cell_ids = np.unique(cell_ids)
    if len(cell_ids) == 0 or radius <= 0.0:
        return cell_ids

    radius_cells = int(math.ceil(radius / resolution))
    offsets = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if math.hypot(dx * resolution, dy * resolution) <= radius + 1e-9:
                offsets.append((dx, dy))
    offsets = np.asarray(offsets, dtype=np.int64)

    base_x = cell_ids % int(width)
    base_y = cell_ids // int(width)
    expanded_x = base_x[:, None] + offsets[None, :, 0]
    expanded_y = base_y[:, None] + offsets[None, :, 1]
    valid = (
        (expanded_x >= 0) & (expanded_x < int(width)) &
        (expanded_y >= 0) & (expanded_y < int(height))
    )
    return np.unique(
        expanded_y[valid] * int(width) + expanded_x[valid])


def select_ray_endpoint_indices(
        xs,
        ys,
        ground_mask,
        obstacle_mask,
        sensor_xy,
        *,
        max_range=15.0,
        angle_bins=360):
    """Select at most one safe ray endpoint per azimuth bin.

    The nearest obstacle wins a bin. If no obstacle is present, the farthest
    ground return is used to clear observed floor space.
    """
    xs = np.asarray(xs, dtype=np.float64)
    ys = np.asarray(ys, dtype=np.float64)
    ground_mask = np.asarray(ground_mask, dtype=bool)
    obstacle_mask = np.asarray(obstacle_mask, dtype=bool)
    dx = xs - float(sensor_xy[0])
    dy = ys - float(sensor_xy[1])
    ranges = np.hypot(dx, dy)
    valid = np.isfinite(ranges) & (ranges > 1e-6) & (ranges <= max_range)
    angles = np.arctan2(dy, dx)
    bins = np.floor((angles + math.pi) * angle_bins / (2.0 * math.pi)).astype(np.int64)
    bins = np.clip(bins, 0, angle_bins - 1)

    obstacle_indices = np.flatnonzero(obstacle_mask & valid)
    ground_indices = np.flatnonzero(ground_mask & valid)

    selected_obstacles = np.empty(0, dtype=np.int64)
    obstacle_bins = np.empty(0, dtype=np.int64)
    if len(obstacle_indices):
        order = np.lexsort((ranges[obstacle_indices], bins[obstacle_indices]))
        ordered = obstacle_indices[order]
        ordered_bins = bins[ordered]
        first = np.r_[True, ordered_bins[1:] != ordered_bins[:-1]]
        selected_obstacles = ordered[first]
        obstacle_bins = bins[selected_obstacles]

    selected_ground = np.empty(0, dtype=np.int64)
    if len(ground_indices):
        if len(obstacle_bins):
            ground_indices = ground_indices[
                ~np.isin(bins[ground_indices], obstacle_bins)]
        if len(ground_indices):
            order = np.lexsort((-ranges[ground_indices], bins[ground_indices]))
            ordered = ground_indices[order]
            ordered_bins = bins[ordered]
            first = np.r_[True, ordered_bins[1:] != ordered_bins[:-1]]
            selected_ground = ordered[first]

    endpoint_indices = np.concatenate([selected_obstacles, selected_ground])
    include_endpoint = np.concatenate([
        np.zeros(len(selected_obstacles), dtype=bool),
        np.ones(len(selected_ground), dtype=bool),
    ])
    return endpoint_indices, include_endpoint


def raytrace_cell_ids(
        origin_ix,
        origin_iy,
        endpoint_ix,
        endpoint_iy,
        include_endpoint,
        width,
        height):
    """Vectorized grid ray traversal returning unique free cell ids."""
    endpoint_ix = np.asarray(endpoint_ix, dtype=np.int64)
    endpoint_iy = np.asarray(endpoint_iy, dtype=np.int64)
    include_endpoint = np.asarray(include_endpoint, dtype=bool)
    if len(endpoint_ix) == 0:
        return np.empty(0, dtype=np.int64)

    dx = endpoint_ix - int(origin_ix)
    dy = endpoint_iy - int(origin_iy)
    steps = np.maximum(np.abs(dx), np.abs(dy))
    counts = steps + include_endpoint.astype(np.int64)
    max_count = int(counts.max())
    if max_count <= 0:
        return np.empty(0, dtype=np.int64)

    k = np.arange(max_count, dtype=np.float64)[None, :]
    denominators = np.maximum(steps, 1).astype(np.float64)[:, None]
    ray_x = np.rint(int(origin_ix) + dx[:, None] * k / denominators).astype(np.int64)
    ray_y = np.rint(int(origin_iy) + dy[:, None] * k / denominators).astype(np.int64)
    valid = (
        (k < counts[:, None]) &
        (ray_x >= 0) & (ray_x < int(width)) &
        (ray_y >= 0) & (ray_y < int(height))
    )
    return np.unique(ray_y[valid] * int(width) + ray_x[valid])


def update_log_odds_grid(
        grid,
        log_odds,
        observed,
        free_cells,
        obstacle_cells,
        *,
        free_update=0.30,
        obstacle_update=0.85,
        minimum=-2.0,
        maximum=3.5,
        free_threshold=-0.40,
        occupied_threshold=2.0,
        free_value=0,
        occupied_value=100,
        unknown_value=-1,
        assume_unique_disjoint=False):
    """Apply one scan of unique free/occupied evidence to an occupancy grid."""
    free_cells = np.asarray(free_cells, dtype=np.int64)
    obstacle_cells = np.asarray(obstacle_cells, dtype=np.int64)
    if assume_unique_disjoint:
        affected = np.concatenate((free_cells, obstacle_cells))
    else:
        free_cells = np.unique(free_cells)
        obstacle_cells = np.unique(obstacle_cells)
        if len(obstacle_cells):
            free_cells = np.setdiff1d(
                free_cells, obstacle_cells, assume_unique=True)
        affected = np.union1d(free_cells, obstacle_cells)
    if len(affected) == 0:
        return affected

    flat_log_odds = log_odds.ravel()
    flat_observed = observed.ravel()
    flat_grid = grid.ravel()
    if len(free_cells):
        flat_log_odds[free_cells] -= free_update
    if len(obstacle_cells):
        flat_log_odds[obstacle_cells] += obstacle_update
    flat_log_odds[affected] = np.clip(
        flat_log_odds[affected], minimum, maximum)
    flat_observed[affected] = True

    flat_grid[affected] = unknown_value
    free = affected[flat_log_odds[affected] <= free_threshold]
    occupied = affected[flat_log_odds[affected] >= occupied_threshold]
    flat_grid[free] = free_value
    flat_grid[occupied] = occupied_value
    return affected
