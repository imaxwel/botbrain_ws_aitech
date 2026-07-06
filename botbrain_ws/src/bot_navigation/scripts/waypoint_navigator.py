#!/usr/bin/env python3
"""
Navigate to saved waypoints by name.

Usage:
  waypoint_navigator.py <name> [name2 ...]  # navigate to waypoints in order
  waypoint_navigator.py --list              # list saved waypoints

Options:
  --file PATH     waypoints YAML file (default: ~/.ros/nav_waypoints.yaml)
  --robot NAME    robot namespace      (default: g1_robot)
  --loop          repeat the sequence indefinitely
"""
import sys
import math
import yaml
import argparse
from pathlib import Path

# --- Proximity override: bypass nav2 goal checker for micro-correction deadlocks ---
NEAR_GOAL_M   = 0.20   # declare success when distance_remaining drops below this (m)
NEAR_GOAL_SEC = 2.0    # must stay within NEAR_GOAL_M for this many seconds


def _default_file() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('bot_navigation'))
        return share.parents[3] / 'src' / 'bot_navigation' / 'nav_waypoints.yaml'
    except Exception:
        return Path.home() / '.ros' / 'nav_waypoints.yaml'

DEFAULT_FILE = _default_file()


def _load(path: Path) -> dict:
    if not path.exists():
        print(f'Waypoints file not found: {path}')
        print('Record waypoints first with: waypoint_recorder.py record <name>')
        sys.exit(1)
    return (yaml.safe_load(path.read_text()) or {}).get('waypoints', {})


def navigate(names: list, db: dict, robot: str, loop: bool) -> None:
    import rclpy
    import time
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
    from action_msgs.msg import GoalStatus

    rclpy.init()
    node = Node('waypoint_navigator')
    client = ActionClient(node, NavigateToPose, f'/{robot}/navigate_to_pose')

    # Strategy E: re-anchor ICP to waypoint exact coordinates after each arrival
    initialpose_pub = node.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)

    def publish_anchor(wp: dict) -> None:
        """Publish waypoint exact pose to /initialpose to re-lock ICP drift."""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.pose.pose.position.x = float(wp['x'])
        msg.pose.pose.position.y = float(wp['y'])
        msg.pose.pose.position.z = 0.0  # z_fix relay will correct to IMU height
        msg.pose.pose.orientation.x = float(wp.get('qx', 0.0))
        msg.pose.pose.orientation.y = float(wp.get('qy', 0.0))
        msg.pose.pose.orientation.z = float(wp.get('qz', 0.0))
        msg.pose.pose.orientation.w = float(wp.get('qw', 1.0))
        msg.pose.covariance[0]  = 0.10  # x variance
        msg.pose.covariance[7]  = 0.10  # y variance
        msg.pose.covariance[35] = 0.05  # yaw variance
        initialpose_pub.publish(msg)
        print(f'  ↻ Re-anchored ICP to waypoint ({wp["x"]:.3f}, {wp["y"]:.3f})')

    if not client.wait_for_server(timeout_sec=10.0):
        node.get_logger().error('NavigateToPose server not available (10 s timeout)')
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    def go_to(name: str) -> bool:
        wp = db[name]
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = wp.get('frame', f'{robot}/map')
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(wp['x'])
        goal.pose.pose.position.y = float(wp['y'])
        goal.pose.pose.position.z = float(wp.get('z', 0.0))
        goal.pose.pose.orientation.x = float(wp.get('qx', 0.0))
        goal.pose.pose.orientation.y = float(wp.get('qy', 0.0))
        goal.pose.pose.orientation.z = float(wp.get('qz', 0.0))
        goal.pose.pose.orientation.w = float(wp.get('qw', 1.0))

        # Proximity override: if nav2 goal checker gets stuck in micro-correction loop
        # (robot physically can't execute tiny lateral moves after rotation drift),
        # we declare success independently once distance_remaining stays below
        # NEAR_GOAL_M for NEAR_GOAL_SEC seconds.
        near_since    = [None]   # timestamp when robot first entered proximity zone
        override_done = [False]  # True once proximity override fires
        gh_ref        = [None]   # goal handle reference for cancellation

        def feedback_cb(fb):
            # Use direct Euclidean distance, NOT distance_remaining (which is path length)
            cp   = fb.feedback.current_pose.pose.position
            dx   = cp.x - float(wp['x'])
            dy   = cp.y - float(wp['y'])
            dist = math.sqrt(dx * dx + dy * dy)
            now  = time.time()
            if dist <= NEAR_GOAL_M:
                if near_since[0] is None:
                    near_since[0] = now
                    print(f'  ≈ Within {dist:.2f} m of goal, proximity hold started...')
                elif (now - near_since[0] >= NEAR_GOAL_SEC) and not override_done[0]:
                    override_done[0] = True
                    print(f'  ✓ Proximity hold complete ({dist:.2f} m), overriding nav2 goal')
                    gh_ref[0].cancel_goal_async()
            else:
                if near_since[0] is not None:
                    near_since[0] = None  # drifted away, reset hold timer

        print(f'→ Navigating to "{name}" (x={wp["x"]:.3f}, y={wp["y"]:.3f})')
        gh_future = client.send_goal_async(goal, feedback_callback=feedback_cb)
        rclpy.spin_until_future_complete(node, gh_future)
        gh = gh_future.result()

        if not gh.accepted:
            print(f'  ✗ Goal rejected')
            return False

        gh_ref[0] = gh
        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(node, res_future)

        status = res_future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED or override_done[0]:
            print(f'  ✓ Reached "{name}"')
            # Strategy E: re-anchor ICP to exact waypoint coordinates to reset drift
            publish_anchor(db[name])
            time.sleep(1.5)  # wait for ICP to re-lock before next waypoint
            return True
        else:
            print(f'  ✗ Failed to reach "{name}" (status={status})')
            return False

    try:
        while True:
            for name in names:
                go_to(name)
            if not loop:
                break
    except KeyboardInterrupt:
        print('\nNavigation interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('waypoints', nargs='*', help='Waypoint names')
    p.add_argument('--file', type=Path, default=DEFAULT_FILE)
    p.add_argument('--robot', default='g1_robot')
    p.add_argument('--loop', action='store_true', help='Repeat sequence indefinitely')
    p.add_argument('--list', action='store_true', help='List saved waypoints')
    args = p.parse_args()

    db = _load(args.file)

    if args.list:
        if not db:
            print('No waypoints saved.')
        else:
            print(f"{'NAME':<20} {'X':>8} {'Y':>8}  FRAME")
            print('-' * 55)
            for name, wp in sorted(db.items()):
                print(f"{name:<20} {wp['x']:>8.3f} {wp['y']:>8.3f}  {wp.get('frame','map')}")
        return

    if not args.waypoints:
        p.error('Provide at least one waypoint name, or use --list')

    missing = [n for n in args.waypoints if n not in db]
    if missing:
        print(f'Unknown waypoints: {missing}')
        print(f'Available: {sorted(db.keys())}')
        sys.exit(1)

    navigate(args.waypoints, db, args.robot, args.loop)


if __name__ == '__main__':
    main()
