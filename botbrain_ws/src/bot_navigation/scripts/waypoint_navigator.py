#!/usr/bin/env python3
"""
Navigate to saved waypoints by name.

Usage:
  waypoint_navigator.py <name> [name2 ...]  # navigate to waypoints in order
  waypoint_navigator.py --list              # list saved waypoints

Options:
  --file PATH     waypoints YAML file
  --scene NAME    override the scene selected by select_map_scene.sh
  --robot NAME    robot namespace      (default: g1_robot)
  --loop          repeat the sequence indefinitely
  --scan-timeout SEC   wait this long for a fresh /scan before sending a goal
"""
import sys
import math
import time
import argparse
from pathlib import Path

from bot_navigation.waypoint_store import (
    DEFAULT_SCENE_FILE,
    current_scene,
    goal_grid_occupancy,
    live_map_scene,
    load_database,
    save_database,
    scene_waypoints,
)


def _default_file() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('bot_navigation'))
        return (
            share.parents[3] / 'src' / 'bot_navigation' / 'nav_waypoints.yaml'
        )
    except Exception:
        return Path.home() / '.ros' / 'nav_waypoints.yaml'


DEFAULT_FILE = _default_file()


def _quaternion_yaw(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def _angle_error(target: float, current: float) -> float:
    delta = target - current
    return math.atan2(math.sin(delta), math.cos(delta))


def _planar_quaternion(wp: dict):
    x = float(wp.get('qx', 0.0))
    y = float(wp.get('qy', 0.0))
    z = float(wp.get('qz', 0.0))
    w = float(wp.get('qw', 1.0))
    yaw = _quaternion_yaw(x, y, z, w)
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def _load(path: Path, scene: str):
    if not path.exists():
        print(f'Waypoints file not found: {path}')
        print(
            'Record waypoints first with: '
            'waypoint_recorder.py record <name>'
        )
        sys.exit(1)
    try:
        database = load_database(path, legacy_scene=scene)
    except ValueError as error:
        print(f'Invalid waypoints file: {error}')
        sys.exit(1)
    return (
        scene_waypoints(database, scene),
        database,
        bool(database.get('_legacy_format', False)),
    )


def navigate(
    names: list,
    db: dict,
    robot: str,
    loop: bool,
    scan_topic: str,
    scan_timeout: float,
    max_scan_age: float,
    success_distance_limit: float,
    scene: str,
    map_timeout: float,
    goal_grid_check_radius: float,
    occupied_threshold: int,
    waypoint_file: Path,
    waypoint_database: dict,
    legacy_format: bool,
) -> bool:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.qos import qos_profile_sensor_data
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import OccupancyGrid
    from action_msgs.msg import GoalStatus
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('waypoint_navigator')
    client = ActionClient(node, NavigateToPose, f'/{robot}/navigate_to_pose')
    active_goal_handle = [None]

    last_scan_received = [None]
    last_scan_stamp = [None]
    last_map = [None]

    def scan_cb(msg: LaserScan):
        last_scan_received[0] = node.get_clock().now().nanoseconds * 1e-9
        last_scan_stamp[0] = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

    def map_cb(msg: OccupancyGrid):
        last_map[0] = msg

    scan_subscription = node.create_subscription(
        LaserScan,
        scan_topic,
        scan_cb,
        qos_profile_sensor_data,
    )
    map_qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    map_subscription = node.create_subscription(
        OccupancyGrid,
        '/map',
        map_cb,
        map_qos,
    )

    def wait_for_fresh_scan() -> bool:
        if scan_timeout <= 0.0:
            return True

        deadline = node.get_clock().now().nanoseconds * 1e-9 + scan_timeout
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            now = node.get_clock().now().nanoseconds * 1e-9
            if (
                last_scan_received[0] is not None
                and last_scan_stamp[0] is not None
            ):
                receive_age = now - last_scan_received[0]
                stamp_age = now - last_scan_stamp[0]
                if (
                    receive_age <= max_scan_age
                    and -0.25 <= stamp_age <= max_scan_age
                ):
                    return True
            if now >= deadline:
                node.get_logger().error(
                    f'No fresh {scan_topic} received within '
                    f'{scan_timeout:.1f} s. '
                    'FAST-LIO/TF/costmap is unhealthy; goal was not sent.'
                )
                return False

    def wait_for_map() -> bool:
        deadline = node.get_clock().now().nanoseconds * 1e-9 + map_timeout
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if last_map[0] is not None:
                return True
            if node.get_clock().now().nanoseconds * 1e-9 >= deadline:
                node.get_logger().error(
                    f'No /map received within {map_timeout:.1f} s; '
                    'goal was not sent.'
                )
                return False

    def goal_is_clear(name: str, wp: dict) -> bool:
        grid = last_map[0]
        origin = grid.info.origin
        orientation = origin.orientation
        origin_yaw = _quaternion_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )
        try:
            result = goal_grid_occupancy(
                data=grid.data,
                width=int(grid.info.width),
                height=int(grid.info.height),
                resolution=float(grid.info.resolution),
                origin_x=float(origin.position.x),
                origin_y=float(origin.position.y),
                origin_yaw=origin_yaw,
                goal_x=float(wp['x']),
                goal_y=float(wp['y']),
                check_radius=goal_grid_check_radius,
                occupied_threshold=occupied_threshold,
            )
        except (TypeError, ValueError) as error:
            node.get_logger().error(
                f'Cannot validate waypoint {name!r} against /map: {error}'
            )
            return False
        if not result['in_bounds']:
            print(
                f'  ✗ Refusing "{name}": target is outside the active map grid'
            )
            return False
        if result['occupied_count']:
            print(
                f'  ✗ Refusing "{name}": target overlaps '
                f'{result["occupied_count"]} occupied grid cell(s) within '
                f'{goal_grid_check_radius:.2f} m '
                f'(max occupancy={result["max_occupancy"]})'
            )
            return False
        return True

    try:
        actual_scene = live_map_scene(node, timeout_sec=map_timeout)
    except (RuntimeError, ValueError) as error:
        node.get_logger().error(
            f'Cannot verify the active map scene: {error}. No goal was sent.'
        )
        node.destroy_subscription(map_subscription)
        node.destroy_subscription(scan_subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return False
    if actual_scene != scene:
        node.get_logger().error(
            f'Active map scene is {actual_scene!r}, but waypoint storage '
            'selected '
            f'{scene!r}. Run select_map_scene.sh before navigating.'
        )
        node.destroy_subscription(map_subscription)
        node.destroy_subscription(scan_subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return False

    if legacy_format:
        save_database(waypoint_file, waypoint_database)
        print(f"Migrated legacy waypoints into scene '{scene}'.")

    if not client.wait_for_server(timeout_sec=60.0):
        node.get_logger().error(
            'NavigateToPose server not available (60 s timeout)'
        )
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        return False

    def go_to(name: str) -> bool:
        wp = db[name]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = wp.get('frame', f'{robot}/map')
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        goal.pose.pose.position.z = 0.0
        qx, qy, qz, qw = _planar_quaternion(wp)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        target_yaw = _quaternion_yaw(qx, qy, qz, qw)

        last_distance = [None]
        last_path_remaining = [None]
        last_yaw_error = [None]
        last_recoveries = [0]
        last_navigation_time = [0.0]
        reported_recoveries = [0]
        last_feedback_log = [0.0]

        def feedback_cb(fb):
            feedback = fb.feedback
            current = feedback.current_pose.pose.position
            dx = current.x - float(wp['x'])
            dy = current.y - float(wp['y'])
            last_distance[0] = math.hypot(dx, dy)
            orientation = feedback.current_pose.pose.orientation
            current_yaw = _quaternion_yaw(
                orientation.x,
                orientation.y,
                orientation.z,
                orientation.w,
            )
            last_yaw_error[0] = abs(_angle_error(target_yaw, current_yaw))
            last_path_remaining[0] = float(feedback.distance_remaining)
            last_recoveries[0] = int(feedback.number_of_recoveries)
            duration = feedback.navigation_time
            last_navigation_time[0] = duration.sec + duration.nanosec * 1e-9

            if last_recoveries[0] > reported_recoveries[0]:
                reported_recoveries[0] = last_recoveries[0]
                print(
                    f'  ! Nav2 recovery triggered '
                    f'(count={last_recoveries[0]}, '
                    f'distance={last_distance[0]:.2f} m)'
                )

            now = node.get_clock().now().nanoseconds * 1e-9
            if now - last_feedback_log[0] >= 1.0:
                last_feedback_log[0] = now
                print(
                    f'  distance={last_distance[0]:.2f} m, '
                    f'path_remaining={last_path_remaining[0]:.2f} m, '
                    f'yaw_error={math.degrees(last_yaw_error[0]):.1f} deg, '
                    f'recoveries={last_recoveries[0]}, '
                    f'elapsed={last_navigation_time[0]:.1f} s'
                )

        if not wait_for_fresh_scan():
            return False
        if not wait_for_map() or not goal_is_clear(name, wp):
            return False

        print(
            f'→ Navigating to "{name}" in scene "{scene}" '
            f'(x={wp["x"]:.3f}, y={wp["y"]:.3f})'
        )
        gh_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, gh_future)
        gh = gh_future.result()

        if not gh.accepted:
            print('  ✗ Goal rejected')
            return False

        active_goal_handle[0] = gh
        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(node, res_future)

        result = res_future.result()
        active_goal_handle[0] = None
        if result is None:
            print(f'  ✗ Failed to get NavigateToPose result for "{name}"')
            return False

        status = result.status
        status_names = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        final_distance = last_distance[0]
        final_details = []
        if final_distance is not None:
            final_details.append(f'distance={final_distance:.2f} m')
        if last_path_remaining[0] is not None:
            final_details.append(
                f'path_remaining={last_path_remaining[0]:.2f} m'
            )
        if last_yaw_error[0] is not None:
            final_details.append(
                f'yaw_error={math.degrees(last_yaw_error[0]):.1f} deg')
        final_details.append(f'recoveries={last_recoveries[0]}')
        final_details.append(f'elapsed={last_navigation_time[0]:.1f} s')
        final_text = f', {", ".join(final_details)}'
        if status != GoalStatus.STATUS_SUCCEEDED:
            status_name = status_names.get(status, str(status))
            print(f'  ✗ Failed to reach "{name}" ({status_name}{final_text})')
            return False

        too_far_from_goal = False
        if final_distance is not None:
            too_far_from_goal = final_distance > success_distance_limit
        if too_far_from_goal:
            print(
                f'  ✗ Nav2 reported success but "{name}" is still '
                f'{final_distance:.2f} m away '
                f'(limit={success_distance_limit:.2f} m)'
            )
            return False

        print(f'  ✓ Reached "{name}"{final_text}')
        return True

    completed = True
    try:
        while True:
            for name in names:
                if not go_to(name):
                    print(
                        'Stopping waypoint sequence after navigation failure.'
                    )
                    completed = False
                    return completed
            if not loop:
                break
    except KeyboardInterrupt:
        print('\nNavigation interrupted.')
        goal_handle = active_goal_handle[0]
        if goal_handle is not None and rclpy.ok():
            print('Cancelling the active Nav2 goal...')
            cancel_future = goal_handle.cancel_goal_async()
            cancel_deadline = time.monotonic() + 2.0
            while (rclpy.ok() and not cancel_future.done() and
                   time.monotonic() < cancel_deadline):
                rclpy.spin_once(node, timeout_sec=0.05)
        completed = False
    finally:
        node.destroy_subscription(map_subscription)
        node.destroy_subscription(scan_subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return completed


def main():
    p = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('waypoints', nargs='*', help='Waypoint names')
    p.add_argument('--file', type=Path, default=DEFAULT_FILE)
    p.add_argument('--scene', help='Override the currently selected map scene')
    p.add_argument('--scene-file', type=Path, default=DEFAULT_SCENE_FILE,
                   help=argparse.SUPPRESS)
    p.add_argument('--robot', default='g1_robot')
    p.add_argument(
        '--loop',
        action='store_true',
        help='Repeat sequence indefinitely',
    )
    p.add_argument('--list', action='store_true', help='List saved waypoints')
    p.add_argument('--scan-topic', default='/scan')
    p.add_argument('--scan-timeout', type=float, default=5.0)
    p.add_argument('--max-scan-age', type=float, default=1.0)
    p.add_argument('--success-distance-limit', type=float, default=0.35)
    p.add_argument('--map-timeout', type=float, default=5.0)
    p.add_argument('--goal-grid-check-radius', type=float, default=0.10)
    p.add_argument('--occupied-threshold', type=int, default=65)
    args = p.parse_args()
    args.file = args.file.expanduser().resolve()
    print(f'Waypoints file: {args.file}')
    try:
        scene, scene_source = current_scene(args.scene, args.scene_file)
    except ValueError as error:
        print(f'Invalid map scene: {error}')
        sys.exit(1)
    print(f"Map scene: {scene} ({scene_source})")

    db, waypoint_database, legacy_format = _load(args.file, scene)

    if args.list:
        if legacy_format:
            print(
                f"Legacy waypoint format: treating existing points as scene "
                f"'{scene}'. Navigation will migrate the file after verifying "
                "the active map."
            )
        if not db:
            print(f"No waypoints saved for scene '{scene}'.")
        else:
            print(f"Scene: {scene}")
            print(f"{'NAME':<20} {'X':>8} {'Y':>8}  FRAME")
            print('-' * 55)
            for name, wp in sorted(db.items()):
                frame = wp.get('frame', 'map')
                print(
                    f"{name:<20} {wp['x']:>8.3f} "
                    f"{wp['y']:>8.3f}  {frame}"
                )
        return

    if not args.waypoints:
        p.error('Provide at least one waypoint name, or use --list')

    missing = [n for n in args.waypoints if n not in db]
    if missing:
        print(f'Unknown waypoints in scene {scene!r}: {missing}')
        print(f'Available: {sorted(db.keys())}')
        sys.exit(1)

    success = navigate(
        args.waypoints,
        db,
        args.robot,
        args.loop,
        args.scan_topic,
        max(0.0, args.scan_timeout),
        max(0.1, args.max_scan_age),
        max(0.05, args.success_distance_limit),
        scene,
        max(0.1, args.map_timeout),
        max(0.0, args.goal_grid_check_radius),
        min(100, max(1, args.occupied_threshold)),
        args.file,
        waypoint_database,
        legacy_format,
    )
    if not success:
        sys.exit(1)


if __name__ == '__main__':
    main()
