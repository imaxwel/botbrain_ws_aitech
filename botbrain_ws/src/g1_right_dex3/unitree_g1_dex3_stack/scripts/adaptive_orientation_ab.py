#!/usr/bin/python3
"""Phase 8 A/B verification harness for adaptive end-effector orientation.

Publishes a fixed 8-target tabletop test set in `torso_link` frame to
`/goal_pose` one target at a time, subscribes to `/joint_trajectory_targets`,
and records per-target PASS/FAIL based on whether at least one trajectory
message is received within `timeout_sec` (default 3.0 s) of each goal publish.

The planner reads its OWN `adaptive_orientation_enabled` ROS parameter at
launch time. This harness is a black-box client — flip the planner toggle
via `ros2 launch unitree_g1_dex3_stack planner.launch.py
adaptive_orientation_enabled:={true|false}` before running this harness.

Parameters
----------
adaptive : bool, default `True`
    Informational label only — appears in startup INFO and the final
    summary line so operators can correlate the run to which planner
    toggle they launched against. Does NOT pass through to the planner.
timeout_sec : float, default `3.0`
    Per-target wait time before declaring failure.

Exit codes
----------
0   — all 8 targets PASSed (D-15 acceptance criterion).
1   — at least one target FAILed.
130 — interrupted via SIGINT (Ctrl+C).

Scope (D-16)
------------
Planner-only verification. This harness MUST NOT subscribe to or import
any executor-related symbol. It does not start the executor; it neither
commands nor moves the robot. The only outputs are `/goal_pose` publishes;
the only inputs are `/joint_trajectory_targets` messages.
"""
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory


# Tabletop test set in torso_link frame (D-13, D-14).
# 8 points spanning the AprilTag-common tabletop region (0.30..0.55 m
# forward in +X, -0.40..-0.05 m lateral in Y, -0.10..+0.15 m vertical
# in Z) — all well outside the 0.05 m shoulder-rejection radius (D-08).
# Order is deterministic and MUST NOT be changed without updating
# 08-VALIDATION.md.
TARGETS = [
    # (label,            x,    y,     z) in torso_link, meters
    ('center',         0.40, -0.20,  0.00),
    ('center-near',    0.30, -0.20,  0.00),
    ('center-far',     0.55, -0.20,  0.00),
    ('right-side',     0.40, -0.40,  0.00),
    ('left-of-mid',    0.40, -0.05,  0.00),
    ('low',            0.40, -0.20, -0.10),
    ('high',           0.40, -0.20,  0.15),
    ('diag',           0.45, -0.30,  0.05),
]

# Fixed baseline quaternion used in BOTH adaptive-on and adaptive-off
# runs so the only variable is the planner's adaptive_orientation_enabled
# parameter. Copied verbatim from keyboard_trigger_node.py — this is
# the historical pre-Phase-8 baseline.
BASELINE_QUAT = (-0.68194788, 0.06844694, -0.07816853, 0.72398328)

_SETTLE_SEC = 1.0  # publisher↔subscriber connection settle delay


class AdaptiveOrientationAB(Node):
    def __init__(self):
        super().__init__('adaptive_orientation_ab')

        # Parameters — informational only; planner reads its own toggle.
        self.declare_parameter('adaptive', True)
        self.declare_parameter('timeout_sec', 3.0)
        self.adaptive_label = self.get_parameter('adaptive').value
        self.timeout_sec = float(self.get_parameter('timeout_sec').value)

        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.traj_sub = self.create_subscription(
            JointTrajectory, '/joint_trajectory_targets', self._traj_cb, 10)

        # Driver state
        self._phase = 'settle'
        self._settle_remaining = _SETTLE_SEC
        self._current_index = 0
        self._publish_time = 0.0
        self._traj_received_for_current = False
        self.results = []  # list of (label, x, y, z, passed)
        self._finished = False
        self._exit_status = 0

        self.get_logger().info(
            f'[AdaptiveAB] Starting harness — adaptive_label={self.adaptive_label}, '
            f'timeout_sec={self.timeout_sec:.2f}, targets={len(TARGETS)}')

        self.create_timer(0.1, self._tick)

    def _traj_cb(self, _msg):
        # Presence-only — the planner already validates trajectory contents
        # via its existing pre-publish auto-fix block (see ik_fcl_ompl_planner.cpp).
        self._traj_received_for_current = True

    def _publish_goal(self, label, x, y, z):
        self._traj_received_for_current = False
        goal = PoseStamped()
        goal.header.frame_id = 'torso_link'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = float(z)
        goal.pose.orientation.x = BASELINE_QUAT[0]
        goal.pose.orientation.y = BASELINE_QUAT[1]
        goal.pose.orientation.z = BASELINE_QUAT[2]
        goal.pose.orientation.w = BASELINE_QUAT[3]
        self.goal_pub.publish(goal)
        self._publish_time = self._now_sec()
        self.get_logger().info(
            f'[AdaptiveAB] Published target {self._current_index + 1}/{len(TARGETS)} '
            f'"{label}" at ({x:.3f}, {y:.3f}, {z:.3f})')

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _tick(self):
        if self._phase == 'settle':
            self._settle_remaining -= 0.1
            if self._settle_remaining <= 0.0:
                self._phase = 'publishing'
            return

        if self._phase == 'publishing':
            label, x, y, z = TARGETS[self._current_index]
            self._publish_goal(label, x, y, z)
            self._phase = 'waiting'
            return

        if self._phase == 'waiting':
            label, x, y, z = TARGETS[self._current_index]
            elapsed = self._now_sec() - self._publish_time
            if self._traj_received_for_current:
                self.results.append((label, x, y, z, True))
                self.get_logger().info(
                    f'[AdaptiveAB]   "{label}" PASS (trajectory received in {elapsed:.2f} s)')
                self._advance()
            elif elapsed > self.timeout_sec:
                self.results.append((label, x, y, z, False))
                self.get_logger().warn(
                    f'[AdaptiveAB]   "{label}" FAIL (no trajectory within {self.timeout_sec:.2f} s)')
                self._advance()
            return

    def _advance(self):
        self._current_index += 1
        if self._current_index >= len(TARGETS):
            self._summarize()
            self._phase = 'done'
            self._finished = True
        else:
            self._phase = 'publishing'

    def _summarize(self):
        passed = sum(1 for r in self.results if r[4])
        self.get_logger().info('[AdaptiveAB] === Per-target results ===')
        for label, x, y, z, ok in self.results:
            status = 'PASS' if ok else 'FAIL'
            self.get_logger().info(
                f'[AdaptiveAB]   {status:<4}  "{label:<13}"  ({x:+.3f}, {y:+.3f}, {z:+.3f})')
        self.get_logger().info(
            f'[AdaptiveAB] === PASS_COUNT {passed}/{len(TARGETS)} — '
            f'adaptive={self.adaptive_label} ===')
        self._exit_status = 0 if passed == len(TARGETS) else 1


def main(args=None):
    rclpy.init(args=args)
    node = AdaptiveOrientationAB()
    exit_status = 1
    try:
        while rclpy.ok() and not node._finished:
            rclpy.spin_once(node, timeout_sec=0.1)
        exit_status = node._exit_status
    except KeyboardInterrupt:
        exit_status = 130
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    sys.exit(exit_status)


if __name__ == '__main__':
    main()
