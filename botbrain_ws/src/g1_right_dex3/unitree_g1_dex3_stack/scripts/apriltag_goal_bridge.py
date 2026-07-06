#!/usr/bin/python3
"""ROS 2 node: cache /apriltag/target_pose, trigger on G to /goal_pose."""

import os
import select
import termios
import tty
import collections
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory

import tf2_ros
import tf2_geometry_msgs  # noqa: F401


class AprilTagGoalBridge(Node):
    def __init__(self):
        super().__init__('apriltag_goal_bridge')

        # ------------------------------------------------------------------
        # 3a. Parameter declarations
        # ------------------------------------------------------------------
        self.declare_parameter('reach_max_distance', 0.55)
        self.declare_parameter('stale_threshold_s', 1.0)
        self.declare_parameter('smoothing_window', 5)
        self.declare_parameter('trigger_key', 'g')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('target_pose_topic', '/apriltag/target_pose')
        self.declare_parameter('fixed_orientation_enabled', False)
        self.declare_parameter('fixed_rpy', [0.0, 0.0, 0.0])

        self.reach_max = float(self.get_parameter('reach_max_distance').value)
        self.stale_threshold = float(
            self.get_parameter('stale_threshold_s').value)
        self.smoothing_window = int(
            self.get_parameter('smoothing_window').value)
        self.trigger_char = str(
            self.get_parameter('trigger_key').value).lower()
        goal_topic = str(self.get_parameter('goal_pose_topic').value)
        target_topic = str(self.get_parameter('target_pose_topic').value)
        self.fixed_orientation_enabled = bool(
            self.get_parameter('fixed_orientation_enabled').value)
        fixed_rpy = list(self.get_parameter('fixed_rpy').value)
        self.fixed_rpy = [float(v) for v in fixed_rpy[:3]]

        # ------------------------------------------------------------------
        # 3b. State variables
        # ------------------------------------------------------------------
        self.position_cache = collections.deque(
            maxlen=self.smoothing_window)           # D-07 sliding window
        self._last_stamp = None                       # latest msg.header.stamp
        self._last_orientation = None                 # latest orientation (copy, D-08)
        self._last_target = None                      # full PoseStamped ref (debug)
        self._waiting_for_completion = False           # D-03 in-flight guard
        self._shoulder_origin = None                   # (x, y, z) tuple, D-13
        self._completion_timer = None                  # one-shot timer handle
        self._shoulder_retry_count = 0                 # retry counter, max 10

        # ------------------------------------------------------------------
        # 3c. TF2 setup (D-13)
        # ------------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ------------------------------------------------------------------
        # 3d. Subscription to target_pose_topic (D-06)
        # ------------------------------------------------------------------
        self.create_subscription(
            PoseStamped, target_topic, self._target_cb, 10)

        # ------------------------------------------------------------------
        # 3e. Subscription to /joint_trajectory_targets (D-03 completion)
        # ------------------------------------------------------------------
        self.create_subscription(
            JointTrajectory, '/joint_trajectory_targets',
            self._traj_cb, 10)

        # ------------------------------------------------------------------
        # 3f. Publisher to goal_pose_topic (D-01)
        # ------------------------------------------------------------------
        self.goal_pub = self.create_publisher(
            PoseStamped, goal_topic, 10)

        # ------------------------------------------------------------------
        # 3g. Keyboard raw-terminal setup (D-01/D-02)
        # ------------------------------------------------------------------
        self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)

        # ------------------------------------------------------------------
        # 3h. Timers
        # ------------------------------------------------------------------
        self.create_timer(0.1, self._tick)                        # keyboard poll
        self.create_timer(0.5, self._retry_shoulder_lookup)       # shoulder TF retry

        # ------------------------------------------------------------------
        # 3i. Ready log
        # ------------------------------------------------------------------
        self.get_logger().info(
            f'[apriltag_goal_bridge] Ready — press '
            f'{self.trigger_char.upper()} to trigger '
            f'(reach_max={self.reach_max}m, '
            f'smoothing={self.smoothing_window}, '
            f'stale={self.stale_threshold}s)')

    def _fixed_orientation_quaternion(self):
        roll, pitch, yaw = self.fixed_rpy
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        return (
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy,
        )

    # ------------------------------------------------------------------
    # 4. _target_cb — cache incoming /apriltag/target_pose
    # ------------------------------------------------------------------
    def _target_cb(self, msg: PoseStamped):
        """Store position in sliding window, cache stamp and orientation."""
        self.position_cache.append((
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ))
        self._last_stamp = msg.header.stamp
        self._last_orientation = msg.pose.orientation
        self._last_target = msg

    # ------------------------------------------------------------------
    # 5. _traj_cb — trajectory completion signal (D-03)
    # ------------------------------------------------------------------
    def _traj_cb(self, msg: JointTrajectory):
        """Set one-shot timer to clear waiting_for_completion after trajectory end."""
        if not msg.points:
            return

        # Compute duration from last point's time_from_start + 1.0s safety margin
        t = msg.points[-1].time_from_start
        duration_s = float(t.sec) + float(t.nanosec) * 1e-9 + 1.0

        # Cancel existing completion timer if one exists
        if self._completion_timer is not None:
            self.destroy_timer(self._completion_timer)
            self._completion_timer = None

        # Create one-shot timer — destroy itself after firing
        def _on_completion():
            self._waiting_for_completion = False
            if self._completion_timer is not None:
                self.destroy_timer(self._completion_timer)
                self._completion_timer = None

        self._completion_timer = self.create_timer(duration_s, _on_completion)

    # ------------------------------------------------------------------
    # 6. _retry_shoulder_lookup — cache shoulder origin via TF (D-13)
    # ------------------------------------------------------------------
    def _retry_shoulder_lookup(self):
        """Lookup right_shoulder_pitch_link origin in torso_link, retry up to 10x."""
        if self._shoulder_origin is not None:
            return

        self._shoulder_retry_count += 1
        try:
            transform = self.tf_buffer.lookup_transform(
                'torso_link', 'right_shoulder_pitch_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5))
            self._shoulder_origin = (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            )
            self.get_logger().info(
                f'[apriltag_goal_bridge] Cached shoulder origin: '
                f'({self._shoulder_origin[0]:.3f}, '
                f'{self._shoulder_origin[1]:.3f}, '
                f'{self._shoulder_origin[2]:.3f})')
        except Exception as ex:
            if self._shoulder_retry_count == 1 or self._shoulder_retry_count % 10 == 0:
                self.get_logger().warn(
                    f'[apriltag_goal_bridge] Shoulder TF lookup failed '
                    f'({self._shoulder_retry_count}): {ex}')

    # ------------------------------------------------------------------
    # 7. _tick — periodic keyboard poll (0.1s)
    # ------------------------------------------------------------------
    def _tick(self):
        """Poll keyboard fd, trigger on configured key."""
        if not select.select([self.fd], [], [], 0.0)[0]:
            return
        try:
            ch = os.read(self.fd, 1).decode('utf-8', errors='ignore')
        except Exception:
            return
        if not ch:
            return
        if ch.lower() != self.trigger_char:
            return
        self._on_trigger()

    # ------------------------------------------------------------------
    # 8. _on_trigger — central guard + publish (D-01..D-14)
    # ------------------------------------------------------------------
    def _on_trigger(self):
        """Evaluate all guards; publish /goal_pose only if all pass."""

        # Guard 1 — empty cache (D-10)
        if not self.position_cache:
            self.get_logger().warn(
                '[apriltag_goal_bridge] no AprilTag detected yet')
            return

        # Guard 2 — shoulder origin not available
        if self._shoulder_origin is None:
            self.get_logger().warn(
                '[apriltag_goal_bridge] shoulder origin not yet available')
            return

        # Guard 3 — in-flight (D-03)
        if self._waiting_for_completion:
            self.get_logger().warn(
                '[apriltag_goal_bridge] previous goal still in flight, '
                'ignoring G')
            return

        # Guard 4 — stale (D-09)
        if self._last_stamp is not None:
            now = self.get_clock().now()
            stamp_time = Time.from_msg(self._last_stamp)
            age_s = (now - stamp_time).nanoseconds * 1e-9
            if age_s > self.stale_threshold:
                self.get_logger().warn(
                    f'[apriltag_goal_bridge] no fresh AprilTag pose '
                    f'(last seen {age_s:.1f} s ago)')
                return

        # Guard 5 — reachability (D-11, D-14)
        # Compute sliding average position (pure Python, no numpy)
        xs = [p[0] for p in self.position_cache]
        ys = [p[1] for p in self.position_cache]
        zs = [p[2] for p in self.position_cache]
        avg_x = sum(xs) / len(xs)
        avg_y = sum(ys) / len(ys)
        avg_z = sum(zs) / len(zs)

        sx, sy, sz = self._shoulder_origin
        dist = math.sqrt(
            (avg_x - sx) ** 2 + (avg_y - sy) ** 2 + (avg_z - sz) ** 2)

        goal_x = avg_x
        goal_y = avg_y
        goal_z = avg_z
        if dist >= self.reach_max:
            if dist <= 1e-9 or self.reach_max <= 0.0:
                self.get_logger().warn(
                    f'[apriltag_goal_bridge] invalid reach projection '
                    f'(dist={dist:.6f}, reach_max={self.reach_max}), not publishing')
                return
            scale = self.reach_max / dist
            goal_x = sx + (avg_x - sx) * scale
            goal_y = sy + (avg_y - sy) * scale
            goal_z = sz + (avg_z - sz) * scale
            self.get_logger().warn(
                f'[apriltag_goal_bridge] reach exceeds {dist:.3f} m '
                f'> {self.reach_max} m; publishing nearest reach-limit target '
                f'({goal_x:.3f}, {goal_y:.3f}, {goal_z:.3f}) for planner fallback')

        # --- All guards pass (D-01) ---
        self._waiting_for_completion = True

        goal = PoseStamped()
        goal.header.frame_id = 'torso_link'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = goal_z
        if self.fixed_orientation_enabled:
            qx, qy, qz, qw = self._fixed_orientation_quaternion()
            goal.pose.orientation.x = qx
            goal.pose.orientation.y = qy
            goal.pose.orientation.z = qz
            goal.pose.orientation.w = qw
        else:
            goal.pose.orientation = self._last_orientation  # raw copy per D-08

        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'[apriltag_goal_bridge] G pressed — '
            f'target=({goal_x:.3f}, {goal_y:.3f}, {goal_z:.3f}) @ torso_link, '
            f'|target-shoulder|={dist:.3f} m, publishing /goal_pose')

    # ------------------------------------------------------------------
    # 9. destroy_node override — restore terminal settings
    # ------------------------------------------------------------------
    def destroy_node(self):
        """Restore terminal and close fd before shutdown."""
        try:
            termios.tcsetattr(
                self.fd, termios.TCSADRAIN, self.old_settings)
        except Exception:
            pass
        try:
            os.close(self.fd)
        except Exception:
            pass
        super().destroy_node()

    # ------------------------------------------------------------------
    # 10. main entry point
    # ------------------------------------------------------------------


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagGoalBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
