#!/usr/bin/python3

import math
import os
import select
import signal
import subprocess
import threading
import time
import termios
import tty

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory

_ACTIVE_NODE = None


def _quaternion_from_rpy(rpy):
    roll, pitch, yaw = rpy
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


def _copy_pose_stamped(msg):
    out = PoseStamped()
    out.header = msg.header
    out.pose.position.x = msg.pose.position.x
    out.pose.position.y = msg.pose.position.y
    out.pose.position.z = msg.pose.position.z
    out.pose.orientation.x = msg.pose.orientation.x
    out.pose.orientation.y = msg.pose.orientation.y
    out.pose.orientation.z = msg.pose.orientation.z
    out.pose.orientation.w = msg.pose.orientation.w
    return out


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ('1', 'true', 'yes', 'on')
    return bool(value)


class AprilTagButtonPressNode(Node):
    def __init__(self):
        super().__init__('apriltag_button_press_node')

        self.declare_parameter('trigger_key', 'g')
        self.declare_parameter('target_pose_topic', '/apriltag/target_pose')
        self.declare_parameter('goal_pose_topic', '/goal_pose')
        self.declare_parameter('joint_trajectory_topic', '/joint_trajectory_targets')
        self.declare_parameter('return_to_standing_topic', '/executor/return_to_standing')
        self.declare_parameter('capture_trigger_topic', '/apriltag/capture_trigger')
        self.declare_parameter('base_rpy', [0.0, -1.5708, 0.0])
        self.declare_parameter('alt_rpy_y_threshold', float('inf'))
        self.declare_parameter('alt_rpy', [0.0, -1.5708, 0.0])
        self.declare_parameter('pre_contact_offset_x', 0.05)
        self.declare_parameter('press_x_offset', 0.0)
        self.declare_parameter('press_y_offset', 0.0)
        self.declare_parameter('press_z_offset', 0.0)
        self.declare_parameter('dex3_setpoint_script', '/botbrain_ws/src/g1_right_dex3/unitree_dex3_cpp/example/control_dex3_right_setpoint.py')
        self.declare_parameter('dex3_net_if', 'enP8p1s0')
        self.declare_parameter('pre_extend_pose', [0.0, -1.05, -1.7, 1.7, 1.8, 0.0, 0.0])
        self.declare_parameter('close_pose', [0.0, -1.05, -1.7, 1.7, 1.8, 1.7, 1.8])
        self.declare_parameter('pre_extend_wait_s', 2.5)
        self.declare_parameter('close_wait_s', 2.5)
        self.declare_parameter('capture_wait_timeout_s', 6.0)
        self.declare_parameter('target_pose_stale_s', 1.5)
        self.declare_parameter('traj_wait_timeout_s', 5.0)
        self.declare_parameter('traj_completion_buffer_s', 0.8)
        self.declare_parameter('ramp_wait_s', 3.5)
        self.declare_parameter('dry_run', False)

        self.trigger_char = str(self.get_parameter('trigger_key').value).lower()
        target_topic = str(self.get_parameter('target_pose_topic').value)
        goal_topic = str(self.get_parameter('goal_pose_topic').value)
        traj_topic = str(self.get_parameter('joint_trajectory_topic').value)
        return_topic = str(self.get_parameter('return_to_standing_topic').value)
        capture_topic = str(self.get_parameter('capture_trigger_topic').value)

        self.base_rpy = [float(v) for v in list(self.get_parameter('base_rpy').value)[:3]]
        self.base_quat = _quaternion_from_rpy(self.base_rpy)
        self.alt_rpy_y_threshold = float(self.get_parameter('alt_rpy_y_threshold').value)
        alt_rpy = [float(v) for v in list(self.get_parameter('alt_rpy').value)[:3]]
        self.alt_quat = _quaternion_from_rpy(alt_rpy)
        self.pre_contact_offset_x = float(self.get_parameter('pre_contact_offset_x').value)
        self.press_x_offset = float(self.get_parameter('press_x_offset').value)
        self.press_y_offset = float(self.get_parameter('press_y_offset').value)
        self.press_z_offset = float(self.get_parameter('press_z_offset').value)
        self.dex3_setpoint_script = str(self.get_parameter('dex3_setpoint_script').value)
        self.dex3_net_if = str(self.get_parameter('dex3_net_if').value)
        self.pre_extend_pose = [float(v) for v in list(self.get_parameter('pre_extend_pose').value)[:7]]
        self.close_pose = [float(v) for v in list(self.get_parameter('close_pose').value)[:7]]
        self.pre_extend_wait_s = float(self.get_parameter('pre_extend_wait_s').value)
        self.close_wait_s = float(self.get_parameter('close_wait_s').value)
        self.capture_wait_timeout_s = float(self.get_parameter('capture_wait_timeout_s').value)
        self.target_pose_stale_s = float(self.get_parameter('target_pose_stale_s').value)
        self.traj_wait_timeout_s = float(self.get_parameter('traj_wait_timeout_s').value)
        self.traj_completion_buffer_s = float(self.get_parameter('traj_completion_buffer_s').value)
        self.ramp_wait_s = float(self.get_parameter('ramp_wait_s').value)
        self.dry_run = _as_bool(self.get_parameter('dry_run').value)

        self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
        self.return_pub = self.create_publisher(Empty, return_topic, 10)
        self.capture_pub = self.create_publisher(Empty, capture_topic, 10) if capture_topic else None
        self.create_subscription(PoseStamped, target_topic, self._target_cb, 10)
        self.create_subscription(JointTrajectory, traj_topic, self._traj_cb, 10)

        self._lock = threading.Lock()
        self._target_event = threading.Event()
        self._traj_event = threading.Event()
        self._last_target = None
        self._last_target_time = 0.0
        self._last_traj_duration_s = 0.0
        self._busy = False
        self._shutdown = False
        self._return_sent = False
        self._sequence_thread = None

        self.fd = None
        self.old_settings = None
        if self.trigger_char:
            self.fd = os.open('/dev/tty', os.O_RDONLY | os.O_NONBLOCK)
            self.old_settings = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
            self.create_timer(0.05, self._tick)
        self.create_timer(0.1, self._shutdown_tick)

        self.get_logger().info(
            f'[apriltag_button_press_node] Ready — press {self.trigger_char.upper()} '
            f'to run pre→extend→press→pre→close→return '
            f'(dry_run={self.dry_run})')

    def _target_cb(self, msg):
        with self._lock:
            self._last_target = _copy_pose_stamped(msg)
            self._last_target_time = time.monotonic()
        self._target_event.set()

    def _traj_cb(self, msg):
        if not msg.points:
            return
        t = msg.points[-1].time_from_start
        duration_s = float(t.sec) + float(t.nanosec) * 1e-9
        with self._lock:
            self._last_traj_duration_s = duration_s
        self._traj_event.set()

    def _tick(self):
        if self._shutdown or self.fd is None:
            return
        if not select.select([self.fd], [], [], 0.0)[0]:
            return
        try:
            ch = os.read(self.fd, 1).decode('utf-8', errors='ignore')
        except Exception:
            return
        if ch and ch.lower() == self.trigger_char:
            self._start_sequence()

    def _shutdown_tick(self):
        if self._shutdown:
            rclpy.try_shutdown()

    def _start_sequence(self):
        with self._lock:
            if self._busy:
                self.get_logger().warn('[apriltag_button_press_node] sequence already running, ignoring trigger')
                return
            self._busy = True
            self._return_sent = False
        self._sequence_thread = threading.Thread(target=self._run_sequence, daemon=True)
        self._sequence_thread.start()

    def _wait_event(self, event, timeout_s):
        deadline = time.monotonic() + max(0.0, timeout_s)
        while not self._shutdown and time.monotonic() < deadline:
            if event.wait(timeout=0.05):
                return True
        return False

    def _sleep_interruptible(self, duration_s):
        deadline = time.monotonic() + max(0.0, duration_s)
        while not self._shutdown and time.monotonic() < deadline:
            time.sleep(min(0.05, deadline - time.monotonic()))
        return not self._shutdown

    def _make_goal(self, source, x=None, y=None, z=None):
        goal = PoseStamped()
        goal.header.frame_id = source.header.frame_id or 'torso_link'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = source.pose.position.x if x is None else x
        goal.pose.position.y = source.pose.position.y if y is None else y
        goal.pose.position.z = source.pose.position.z if z is None else z
        quat = self.alt_quat if source.pose.position.y > self.alt_rpy_y_threshold else self.base_quat
        goal.pose.orientation.x = quat[0]
        goal.pose.orientation.y = quat[1]
        goal.pose.orientation.z = quat[2]
        goal.pose.orientation.w = quat[3]
        return goal

    def _send_goal_and_wait(self, name, goal):
        self._traj_event.clear()
        self.goal_pub.publish(goal)
        self.get_logger().info(
            f'[apriltag_button_press_node] sent {name} goal: '
            f'({goal.pose.position.x:.3f}, {goal.pose.position.y:.3f}, {goal.pose.position.z:.3f})')
        if not self._wait_event(self._traj_event, self.traj_wait_timeout_s):
            self.get_logger().error(f'[apriltag_button_press_node] timed out waiting for trajectory for {name}')
            return False
        with self._lock:
            duration_s = self._last_traj_duration_s
        return self._sleep_interruptible(duration_s + self.traj_completion_buffer_s)

    def _run_dex3(self, label, pose, wait_s):
        if self.dry_run:
            self.get_logger().info(f'[apriltag_button_press_node] dry_run: skipping Dex-3 {label}')
            return True
        if not os.path.exists(self.dex3_setpoint_script):
            self.get_logger().error(
                f'[apriltag_button_press_node] Dex-3 setpoint script not found: {self.dex3_setpoint_script}')
            return False
        cmd = ['/usr/bin/python3', self.dex3_setpoint_script, self.dex3_net_if]
        cmd.extend(f'{v:.6f}' for v in pose)
        self.get_logger().info(f'[apriltag_button_press_node] running Dex-3 {label}')
        try:
            completed = subprocess.run(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                timeout=max(5.0, wait_s + 5.0),
                env=os.environ.copy(),
            )
        except subprocess.TimeoutExpired as ex:
            self.get_logger().error(f'[apriltag_button_press_node] Dex-3 {label} timed out: {ex}')
            return False
        except Exception as ex:
            self.get_logger().error(f'[apriltag_button_press_node] Dex-3 {label} failed: {ex}')
            return False
        if completed.stdout:
            for line in completed.stdout.splitlines():
                self.get_logger().info(f'[Dex3 {label}] {line}')
        if completed.returncode != 0:
            # CycloneDDS C++ assertion on exit (SIGABRT/-6) is a known cleanup
            # crash; the script completed successfully if it printed "Done."
            if completed.returncode == -6 and completed.stdout and 'Done.' in completed.stdout:
                self.get_logger().warn(
                    f'[apriltag_button_press_node] Dex-3 {label} exited with -6 '
                    f'(CycloneDDS cleanup crash after "Done." — treating as success)')
            else:
                self.get_logger().error(
                    f'[apriltag_button_press_node] Dex-3 {label} exited with {completed.returncode}')
                return False
        return not self._shutdown

    def _publish_return_to_standing(self):
        if self._return_sent:
            return
        self.return_pub.publish(Empty())
        self._return_sent = True
        self.get_logger().info('[apriltag_button_press_node] published /executor/return_to_standing')

    def _run_sequence(self):
        _CACHE_TTL = 30.0  # 电梯面板位置不变，允许30s内缓存复用（覆盖手臂按压动作期间的相机断流）
        try:
            with self._lock:
                cached_age = (time.monotonic() - self._last_target_time) if self._last_target else float('inf')
            if cached_age >= _CACHE_TTL:
                self._target_event.clear()
                if self.capture_pub is not None:
                    self.capture_pub.publish(Empty())
                self.get_logger().info(
                    f'[apriltag_button_press_node] waiting for YOLO detection (up to {self.capture_wait_timeout_s:.0f}s)...')
                if not self._wait_event(self._target_event, self.capture_wait_timeout_s):
                    self.get_logger().warn('[apriltag_button_press_node] no detection within timeout')
                    return
            else:
                if self.capture_pub is not None:
                    self.capture_pub.publish(Empty())
            with self._lock:
                target = _copy_pose_stamped(self._last_target)
                age_s = time.monotonic() - self._last_target_time
            if age_s > _CACHE_TTL:
                self.get_logger().warn(f'[apriltag_button_press_node] target stale ({age_s:.2f}s)')
                return
            self.get_logger().info(
                f'[apriltag_button_press_node] tag accepted: '
                f'({target.pose.position.x:.3f}, {target.pose.position.y:.3f}, {target.pose.position.z:.3f}) '
                f'@ {target.header.frame_id or "torso_link"}')

            pre = self._make_goal(
                target,
                x=target.pose.position.x - self.pre_contact_offset_x + self.press_x_offset,
                y=target.pose.position.y + self.press_y_offset,
                z=target.pose.position.z + self.press_z_offset,
            )
            press = self._make_goal(
                target,
                x=target.pose.position.x + self.press_x_offset,
                y=target.pose.position.y + self.press_y_offset,
                z=target.pose.position.z + self.press_z_offset,
            )

            if not self._send_goal_and_wait('pre-contact', pre):
                return
            if not self._run_dex3('extend-middle-finger', self.pre_extend_pose, self.pre_extend_wait_s):
                return
            if not self._send_goal_and_wait('press-target', press):
                return
            if not self._send_goal_and_wait('retreat-pre-contact', pre):
                return
            if not self._run_dex3('close-hand', self.close_pose, self.close_wait_s):
                return
            self._publish_return_to_standing()
            self._sleep_interruptible(self.ramp_wait_s)
            self.get_logger().info('[apriltag_button_press_node] press complete')
        finally:
            if self._shutdown:
                self._publish_return_to_standing()
            with self._lock:
                self._busy = False

    def request_shutdown(self):
        self._shutdown = True
        if not self._busy:
            self._publish_return_to_standing()

    def destroy_node(self):
        if self.fd is not None and self.old_settings is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass
        if self.fd is not None:
            try:
                os.close(self.fd)
            except Exception:
                pass
        super().destroy_node()


def _signal_handler(signum, frame):
    if _ACTIVE_NODE is not None:
        _ACTIVE_NODE.request_shutdown()
    rclpy.try_shutdown()


def main(args=None):
    global _ACTIVE_NODE
    rclpy.init(args=args)
    node = AprilTagButtonPressNode()
    _ACTIVE_NODE = node
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.request_shutdown()
        node.destroy_node()
        _ACTIVE_NODE = None
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
