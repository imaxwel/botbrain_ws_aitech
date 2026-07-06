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
import yaml
import argparse
from pathlib import Path

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
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from nav2_msgs.action import NavigateToPose
    from geometry_msgs.msg import PoseStamped
    from action_msgs.msg import GoalStatus

    rclpy.init()
    node = Node('waypoint_navigator')
    client = ActionClient(node, NavigateToPose, f'/{robot}/navigate_to_pose')

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

        print(f'→ Navigating to "{name}" (x={wp["x"]:.3f}, y={wp["y"]:.3f})')
        gh_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, gh_future)
        gh = gh_future.result()

        if not gh.accepted:
            print(f'  ✗ Goal rejected')
            return False

        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(node, res_future)

        if res_future.result().status == GoalStatus.STATUS_SUCCEEDED:
            print(f'  ✓ Reached "{name}"')
            return True
        else:
            print(f'  ✗ Failed to reach "{name}" (status={res_future.result().status})')
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
