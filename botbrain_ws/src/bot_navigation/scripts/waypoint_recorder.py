#!/usr/bin/env python3
"""
Record named navigation waypoints from the robot's current pose.

Usage:
  waypoint_recorder.py record <name>   # save current robot pose
  waypoint_recorder.py list            # show all saved waypoints
  waypoint_recorder.py delete <name>   # remove a waypoint

Options:
  --file PATH     waypoints YAML file (default: ~/.ros/nav_waypoints.yaml)
  --robot NAME    robot namespace      (default: g1_robot)
"""
import sys
import yaml
import argparse
from pathlib import Path

def _default_file() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('bot_navigation'))
        # install layout: <ws>/install/bot_navigation/share/bot_navigation/
        # parents[3] = <ws>/
        return share.parents[3] / 'src' / 'bot_navigation' / 'nav_waypoints.yaml'
    except Exception:
        return Path.home() / '.ros' / 'nav_waypoints.yaml'

DEFAULT_FILE = _default_file()


def _load(path: Path) -> dict:
    if path.exists():
        return (yaml.safe_load(path.read_text()) or {}).get('waypoints', {})
    return {}


def _save(path: Path, waypoints: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(yaml.dump({'waypoints': waypoints}, default_flow_style=False))


def cmd_record(name: str, file: Path, robot: str) -> None:
    import rclpy
    from rclpy.node import Node
    from tf2_ros import Buffer, TransformListener

    map_frame = f'{robot}/map'
    base_frame = f'{robot}/base_footprint'

    rclpy.init()
    node = Node('waypoint_recorder')
    buf = Buffer()
    TransformListener(buf, node)

    deadline = node.get_clock().now().nanoseconds + 5_000_000_000
    tf = None
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        try:
            tf = buf.lookup_transform(map_frame, base_frame, rclpy.time.Time())
            break
        except Exception:
            if node.get_clock().now().nanoseconds > deadline:
                node.get_logger().error('TF lookup timed out (5 s). Is FAST-LIO running?')
                node.destroy_node(); rclpy.shutdown(); sys.exit(1)

    t, r = tf.transform.translation, tf.transform.rotation
    waypoints = _load(file)
    waypoints[name] = dict(
        x=round(t.x, 4), y=round(t.y, 4), z=round(t.z, 4),
        qx=round(r.x, 6), qy=round(r.y, 6), qz=round(r.z, 6), qw=round(r.w, 6),
        frame=map_frame,
    )
    _save(file, waypoints)
    print(f"✓ Saved '{name}': x={t.x:.3f}, y={t.y:.3f}, frame={map_frame}")
    node.destroy_node(); rclpy.shutdown()


def cmd_list(file: Path) -> None:
    waypoints = _load(file)
    if not waypoints:
        print('No waypoints saved.')
        return
    print(f"{'NAME':<20} {'X':>8} {'Y':>8}  FRAME")
    print('-' * 55)
    for name, wp in sorted(waypoints.items()):
        print(f"{name:<20} {wp['x']:>8.3f} {wp['y']:>8.3f}  {wp.get('frame', 'map')}")


def cmd_delete(name: str, file: Path) -> None:
    waypoints = _load(file)
    if name not in waypoints:
        print(f"Waypoint '{name}' not found. Available: {list(waypoints)}")
        sys.exit(1)
    del waypoints[name]
    _save(file, waypoints)
    print(f"Deleted '{name}'")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('command', choices=['record', 'list', 'delete'])
    p.add_argument('name', nargs='?', help='Waypoint name')
    p.add_argument('--file', type=Path, default=DEFAULT_FILE)
    p.add_argument('--robot', default='g1_robot')
    args = p.parse_args()

    if args.command == 'record':
        if not args.name:
            p.error('record requires a waypoint name')
        cmd_record(args.name, args.file, args.robot)
    elif args.command == 'list':
        cmd_list(args.file)
    elif args.command == 'delete':
        if not args.name:
            p.error('delete requires a waypoint name')
        cmd_delete(args.name, args.file)


if __name__ == '__main__':
    main()
