import math

import numpy as np

from g1_pkg.grid_mapping_core import (
    classify_points,
    expand_cell_ids,
    fit_ground_plane_ransac,
    quaternion_to_matrix,
    raytrace_cell_ids,
    select_ray_endpoint_indices,
    unique_cell_ids,
    update_log_odds_grid,
    world_to_body,
)


def test_world_to_body_round_trip_for_yaw_rotation():
    half = math.sqrt(0.5)
    rotation = quaternion_to_matrix([0.0, 0.0, half, half])
    body_position = np.array([2.0, -1.0, 0.5])
    points_body = np.array([[1.0, 0.0, -0.5], [0.0, 2.0, 0.2]])
    points_world = points_body @ rotation.T + body_position
    recovered = world_to_body(points_world, body_position, rotation)
    np.testing.assert_allclose(recovered, points_body, atol=1e-9)


def test_ransac_rejects_ceiling_and_fits_floor():
    rng = np.random.default_rng(1234)
    floor_xy = rng.uniform(-6.0, 6.0, size=(1500, 2))
    floor_z = -1.247 + rng.normal(0.0, 0.008, size=(1500, 1))
    floor = np.hstack([floor_xy, floor_z])

    ceiling_xy = rng.uniform(-6.0, 6.0, size=(800, 2))
    ceiling = np.column_stack([
        ceiling_xy,
        np.full(800, 1.15),
    ])
    wall_y = rng.uniform(-5.0, 5.0, size=500)
    wall_z = rng.uniform(-1.1, 1.0, size=500)
    wall = np.column_stack([
        np.full(500, 4.0),
        wall_y,
        wall_z,
    ])
    below_noise = np.column_stack([
        rng.uniform(-5.0, 5.0, size=(100, 2)),
        rng.uniform(-1.8, -1.5, size=100),
    ])
    points = np.vstack([floor, ceiling, wall, below_noise])

    coefficients, metrics = fit_ground_plane_ransac(
        points,
        np.array([0.0, 0.0, 0.0]),
        -1.247,
        rng=np.random.default_rng(9),
    )
    assert metrics["reason"] == "ok"
    assert coefficients is not None
    assert abs(coefficients[0]) < 0.002
    assert abs(coefficients[1]) < 0.002
    assert abs(coefficients[2] + 1.247) < 0.015


def test_classification_ignores_below_floor_and_ceiling():
    points = np.array([
        [0.0, 0.0, -1.25],  # floor
        [1.0, 0.0, -0.75],  # 0.497 m obstacle
        [2.0, 0.0, -1.50],  # below floor: ignored
        [3.0, 0.0, 0.50],   # 1.747 m high: ignored
        [4.0, 0.0, -1.157],  # transition band: ignored
    ])
    ground, obstacle, _heights = classify_points(
        points, np.array([0.0, 0.0, -1.247]))
    np.testing.assert_array_equal(
        ground, [True, False, False, False, False])
    np.testing.assert_array_equal(
        obstacle, [False, True, False, False, False])


def test_log_odds_requires_three_distinct_updates_and_can_clear():
    grid = np.full((1, 4), -1, dtype=np.int8)
    log_odds = np.zeros((1, 4), dtype=np.float32)
    observed = np.zeros((1, 4), dtype=bool)

    for _ in range(2):
        update_log_odds_grid(
            grid, log_odds, observed, [], [2], occupied_threshold=2.0)
        assert grid[0, 2] == -1
    update_log_odds_grid(
        grid, log_odds, observed, [], [2], occupied_threshold=2.0)
    assert grid[0, 2] == 100

    # A bounded false obstacle is eventually cleared by repeated free scans.
    for _ in range(11):
        update_log_odds_grid(
            grid, log_odds, observed, [2], [], occupied_threshold=2.0)
    assert grid[0, 2] == 0


def test_obstacle_wins_when_cell_is_also_reported_free():
    grid = np.full((1, 2), -1, dtype=np.int8)
    log_odds = np.zeros((1, 2), dtype=np.float32)
    observed = np.zeros((1, 2), dtype=bool)
    update_log_odds_grid(
        grid,
        log_odds,
        observed,
        free_cells=[1],
        obstacle_cells=[1],
        occupied_threshold=0.8,
    )
    assert log_odds[0, 1] > 0.0
    assert grid[0, 1] == 100


def test_raytrace_excludes_obstacle_endpoint_and_includes_ground_endpoint():
    obstacle_ray = raytrace_cell_ids(
        0, 0, [3], [0], [False], width=5, height=1)
    ground_ray = raytrace_cell_ids(
        0, 0, [3], [0], [True], width=5, height=1)
    np.testing.assert_array_equal(obstacle_ray, [0, 1, 2])
    np.testing.assert_array_equal(ground_ray, [0, 1, 2, 3])


def test_cell_evidence_is_unique_per_scan():
    cells = unique_cell_ids(
        ix=[2, 2, 2, 4],
        iy=[1, 1, 1, 1],
        width=10,
    )
    np.testing.assert_array_equal(cells, [12, 14])


def test_nearest_obstacle_wins_ray_bin_over_ground():
    xs = np.array([2.0, 4.0, 6.0])
    ys = np.zeros(3)
    ground = np.array([True, False, True])
    obstacle = np.array([False, True, False])
    indices, include_endpoint = select_ray_endpoint_indices(
        xs,
        ys,
        ground,
        obstacle,
        sensor_xy=[0.0, 0.0],
        max_range=10.0,
        angle_bins=360,
    )
    np.testing.assert_array_equal(indices, [1])
    np.testing.assert_array_equal(include_endpoint, [False])


def test_obstacle_spread_is_metric_and_clipped_at_grid_edge():
    center = expand_cell_ids(
        [4], width=3, height=3, radius=0.05, resolution=0.05)
    corner = expand_cell_ids(
        [0], width=3, height=3, radius=0.05, resolution=0.05)
    np.testing.assert_array_equal(center, [1, 3, 4, 5, 7])
    np.testing.assert_array_equal(corner, [0, 1, 3])


def test_spread_confirms_obstacle_that_jitters_by_one_cell():
    grid = np.full((5, 5), -1, dtype=np.int8)
    log_odds = np.zeros((5, 5), dtype=np.float32)
    observed = np.zeros((5, 5), dtype=bool)
    first = expand_cell_ids(
        [12], width=5, height=5, radius=0.05, resolution=0.05)
    second = expand_cell_ids(
        [13], width=5, height=5, radius=0.05, resolution=0.05)
    update_log_odds_grid(
        grid, log_odds, observed, [], first, occupied_threshold=2.125)
    update_log_odds_grid(
        grid, log_odds, observed, [], second, occupied_threshold=2.125)
    update_log_odds_grid(
        grid, log_odds, observed, [], first, occupied_threshold=2.125)
    assert grid[2, 2] == 100
    assert grid[2, 3] == 100


def test_repeated_ground_support_clears_transient_person_footprint():
    grid = np.full((7, 7), -1, dtype=np.int8)
    log_odds = np.zeros((7, 7), dtype=np.float32)
    observed = np.zeros((7, 7), dtype=bool)
    person = expand_cell_ids(
        [24], width=7, height=7, radius=0.05, resolution=0.05)
    for _ in range(3):
        update_log_odds_grid(
            grid, log_odds, observed, [], person,
            occupied_threshold=2.125)
    assert grid[3, 3] == 100

    floor = expand_cell_ids(
        [24], width=7, height=7, radius=0.05, resolution=0.05)
    for _ in range(11):
        update_log_odds_grid(
            grid, log_odds, observed, floor, [],
            free_update=0.30, occupied_threshold=2.125)
    assert np.all(grid.ravel()[person] == 0)
