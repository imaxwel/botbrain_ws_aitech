#!/usr/bin/python3
"""Phase 9 end-to-end UAT harness.

Measures TCP position error via KDL FK for 4 tabletop targets.
Operator places tag, presses G, harness records expected position
from /apriltag/target_pose and actual TCP from /joint_states FK.

Usage:
  ros2 run unitree_g1_dex3_stack apriltag_reach_uat.py
"""
import sys
import time
import math
import os

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
import numpy as np
import pinocchio as pin

from ament_index_python.packages import get_package_share_directory


# 4-point tabletop subset filtered from Phase 8 8-point set (D-21).
# All satisfy: distance to right_shoulder_pitch_link <= 0.55 m AND
# right-side workspace (+Y_torso half-space, all Y values negative).
TARGETS = [
    # (label,            x,    y,     z) in torso_link, meters
    ('center',         0.40, -0.20,  0.00),
    ('right-side',     0.40, -0.40,  0.00),
    ('low',            0.40, -0.20, -0.10),
    ('diag',           0.45, -0.30,  0.05),
]

# Right arm URDF joint names — same ordering as g1_dex3_joint_defs.hpp
# and read_tcp_pose.py RIGHT_ARM_URDF_JOINTS.
RIGHT_ARM_URDF_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

TCP_OFFSET_X = 0.175  # m, same as URDF right_tcp_link and planner tcp_offset_x


def _build_reduced_model(urdf_path):
    """Build Pinocchio reduced model with right-arm 7 DOF + TCP frame."""
    model = pin.buildModelFromUrdf(urdf_path)
    lock_ids = []
    for i, name in enumerate(model.names):
        if name not in RIGHT_ARM_URDF_JOINTS and name != "universe":
            lock_ids.append(model.getJointId(name))
    reduced = pin.buildReducedModel(model, lock_ids, np.zeros(model.nq))
    reduced.addFrame(
        pin.Frame(
            "right_tcp",
            reduced.getJointId("right_wrist_yaw_joint"),
            pin.SE3(np.eye(3), np.array([TCP_OFFSET_X, 0.0, 0.0]).T),
            pin.FrameType.OP_FRAME,
        )
    )
    return reduced


class AprilTagReachUAT(Node):
    """End-to-end UAT harness with FK-based TCP error measurement.

    FSM phases:
      init -> waiting_tag -> waiting_traj -> measuring -> next_point -> done
    """

    def __init__(self):
        super().__init__('apriltag_reach_uat')

        # --- Parameters ---
        self.declare_parameter('error_threshold', 0.03)   # 3 cm (D-23)
        self.declare_parameter('settle_time', 1.0)         # seconds after traj
        self.declare_parameter('timeout_sec', 10.0)        # max wait per point
        self.error_threshold = float(
            self.get_parameter('error_threshold').value)
        self.settle_time = float(
            self.get_parameter('settle_time').value)
        self.timeout_sec = float(
            self.get_parameter('timeout_sec').value)

        # --- FSM state ---
        self._phase = 'init'
        self._current_index = 0
        self._joint_state = None
        self._traj_received = False
        self._expected_pos = None     # (x, y, z) from /apriltag/target_pose
        self._traj_arrival_time = None
        self.results = []              # (label, expected, actual, error, ok)
        self._exit_status = 0
        self._publish_time = None
        self._tag_seen = False
        self._finished = False

        # --- KDL FK model (built once from URDF) ---
        urdf_path = self._find_urdf()
        self._reduced = _build_reduced_model(urdf_path)
        self._rmodel = self._reduced
        self._rdata = self._reduced.createData()
        self._torso_id = self._reduced.getFrameId("torso_link")
        self._tcp_id = self._reduced.getFrameId("right_tcp")

        # --- Subscriptions ---
        self.create_subscription(
            JointState, '/joint_states', self._js_cb, 10)
        self.create_subscription(
            JointTrajectory, '/joint_trajectory_targets', self._traj_cb, 10)
        self.create_subscription(
            PoseStamped, '/apriltag/target_pose', self._target_cb, 10)

        # --- FSM tick timer ---
        self.create_timer(0.1, self._tick)

        # --- Startup log ---
        self.get_logger().info(
            '[UAT] Phase 9 End-to-End Verification Harness')
        self.get_logger().info(
            '[UAT] Targets: 4 (center, right-side, low, diag)')

    def _find_urdf(self):
        """Locate collision-primitives URDF via ament_index."""
        share_dir = get_package_share_directory('unitree_g1_dex3_stack')
        return os.path.join(
            share_dir, 'robots', 'g1_description',
            'g1_29dof_lock_waist_with_hand_rev_1_0_collision_primitives.urdf')

    def _js_cb(self, msg):
        """Cache latest joint state for FK computation."""
        self._joint_state = msg

    def _traj_cb(self, msg):
        """Signal trajectory arrival (presence-only, content not inspected)."""
        self._traj_received = True
        self._traj_arrival_time = self.get_clock().now()

    def _target_cb(self, msg):
        """Cache expected target position from AprilTag detection."""
        self._expected_pos = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        )
        self._tag_seen = True

    # ----- FSM tick -----

    def _tick(self):
        """FSM dispatch — called at 0.1 s interval."""
        if self._phase == 'init':
            self._phase = 'waiting_tag'
            label, x, y, z = TARGETS[self._current_index]
            self.get_logger().info(
                f'[UAT] Place tag at point 1 '
                f'({label}: {x:.2f}, {y:.2f}, {z:.2f}), press G')
            return

        if self._phase == 'waiting_tag':
            if self._tag_seen and self._expected_pos is not None:
                self.get_logger().info(
                    f'[UAT] Tag detected at '
                    f'({self._expected_pos[0]:.3f}, '
                    f'{self._expected_pos[1]:.3f}, '
                    f'{self._expected_pos[2]:.3f})')
                self.get_logger().info('[UAT] Press G to trigger, or wait...')
                self._phase = 'waiting_traj'
                self._traj_received = False
                self._publish_time = self.get_clock().now()
            return

        if self._phase == 'waiting_traj':
            if self._traj_received:
                self.get_logger().info(
                    f'[UAT] Trajectory received, waiting '
                    f'{self.settle_time}s for joint states to settle')
                self._phase = 'measuring'
                self._traj_arrival_time = self.get_clock().now()
            else:
                elapsed = (self.get_clock().now() -
                           self._publish_time).nanoseconds * 1e-9
                if elapsed > self.timeout_sec:
                    label = TARGETS[self._current_index][0]
                    self.get_logger().warn(
                        f'[UAT] Timeout waiting for trajectory '
                        f'at point {label}')
                    self.results.append(
                        (label, self._expected_pos,
                         (0.0, 0.0, 0.0), 999.0, False))
                    self._phase = 'next_point'
            return

        if self._phase == 'measuring':
            elapsed = (self.get_clock().now() -
                       self._traj_arrival_time).nanoseconds * 1e-9
            if elapsed >= self.settle_time:
                actual = self._compute_tcp_position()
                expected = self._expected_pos
                if expected is None:
                    expected = (0.0, 0.0, 0.0)
                error = math.sqrt(
                    (actual[0] - expected[0]) ** 2 +
                    (actual[1] - expected[1]) ** 2 +
                    (actual[2] - expected[2]) ** 2
                )
                ok = error <= self.error_threshold
                label = TARGETS[self._current_index][0]
                self.results.append(
                    (label, expected, actual, error, ok))
                status = 'PASS' if ok else 'FAIL'
                self.get_logger().info(
                    f'[UAT] Point {self._current_index + 1}: '
                    f'expected=({expected[0]:.3f}, '
                    f'{expected[1]:.3f}, {expected[2]:.3f}) '
                    f'actual=({actual[0]:.4f}, '
                    f'{actual[1]:.4f}, {actual[2]:.4f}) '
                    f'error_m={error:.4f} {status}')
                self._phase = 'next_point'
            return

        if self._phase == 'next_point':
            self._current_index += 1
            if self._current_index >= len(TARGETS):
                self.get_logger().info('[UAT] All targets complete')
                self._phase = 'done'
                self._summarize()
            else:
                self._tag_seen = False
                self._expected_pos = None
                self._traj_received = False
                self._phase = 'waiting_tag'
                label, x, y, z = TARGETS[self._current_index]
                self.get_logger().info(
                    f'[UAT] Place tag at point '
                    f'{self._current_index + 1} '
                    f'({label}: {x:.2f}, {y:.2f}, {z:.2f}), '
                    f'press G')
            return

        if self._phase == 'done':
            return

    # ----- FK computation -----

    def _compute_tcp_position(self):
        """Compute right TCP (x, y, z) in torso_link frame via Pinocchio FK.

        Uses the reduced model built in __init__ with only right arm 7 DOF.
        Returns (0, 0, 0) if no joint state received yet.
        """
        if self._joint_state is None:
            return (0.0, 0.0, 0.0)

        js = dict(zip(self._joint_state.name,
                      self._joint_state.position))
        q = np.zeros(self._rmodel.nq)
        for name in RIGHT_ARM_URDF_JOINTS:
            if name in js:
                jid = self._rmodel.getJointId(name)
                q[jid - 1] = js[name]

        pin.framesForwardKinematics(self._rmodel, self._rdata, q)
        tcp_in_pelvis = self._rdata.oMf[self._tcp_id]
        torso_pose = self._rdata.oMf[self._torso_id]
        tcp_pose = torso_pose.actInv(tcp_in_pelvis)
        pos = tcp_pose.translation
        return (pos[0], pos[1], pos[2])

    # ----- Summary -----

    def _summarize(self):
        """Print per-target results table and set exit status (D-24)."""
        passed = sum(1 for r in self.results if r[4])
        self.get_logger().info('[UAT] === Per-target results ===')
        header = (
            f'  {"Point":<12} {"Expected(xyz)":<30} '
            f'{"Actual(xyz)":<30} {"Error(m)":<10} Status'
        )
        self.get_logger().info(header)
        self.get_logger().info('  ' + '-' * 96)
        for idx, (label, expected, actual, error, ok) in enumerate(
                self.results):
            status = 'PASS' if ok else 'FAIL'
            exp_str = (
                f'({expected[0]:.3f}, {expected[1]:.3f}, '
                f'{expected[2]:.3f})')
            act_str = (
                f'({actual[0]:.4f}, {actual[1]:.4f}, '
                f'{actual[2]:.4f})')
            self.get_logger().info(
                f'  {label:<12} {exp_str:<30} {act_str:<30} '
                f'{error:<10.4f} {status}')
        self.get_logger().info('  ' + '-' * 96)
        self.get_logger().info(
            f'[UAT] PASS_COUNT {passed}/{len(TARGETS)}')
        # D-24: 4/4 required for exit 0
        self._exit_status = 0 if passed == len(TARGETS) else 1
        self._finished = True


def main(args=None):
    """Entry point with spin_once loop for exit-on-completion."""
    rclpy.init(args=args)
    node = AprilTagReachUAT()
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
