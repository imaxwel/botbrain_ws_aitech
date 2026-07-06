#!/usr/bin/python3
"""Publish right-arm gravity compensation torques using Pinocchio RNEA.

Subscribes to /lf/lowstate, computes gravity torques for 7 right-arm joints,
and publishes them as Float32MultiArray on /right_arm_gravity_torques at ~250 Hz.
"""
import os
import numpy as np
import pinocchio as pin

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32MultiArray
from unitree_hg.msg import LowState

# Right arm motor indices in LowState.motor_state[]
RIGHT_ARM_MOTOR_INDICES = [22, 23, 24, 25, 26, 27, 28]

RIGHT_ARM_URDF_JOINTS = [
    'right_shoulder_pitch_joint',
    'right_shoulder_roll_joint',
    'right_shoulder_yaw_joint',
    'right_elbow_joint',
    'right_wrist_roll_joint',
    'right_wrist_pitch_joint',
    'right_wrist_yaw_joint',
]

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def find_urdf():
    """Locate the G1 URDF."""
    try:
        from ament_index_python import get_package_share_directory
        share_dir = get_package_share_directory('unitree_g1_dex3_stack')
        candidate = os.path.join(
            share_dir, 'robots', 'g1_description',
            'g1_29dof_lock_waist_with_hand_rev_1_0.urdf')
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass
    # Walk up from script dir
    cur = SCRIPT_DIR
    for _ in range(6):
        candidate = os.path.join(
            cur, 'robots', 'g1_description',
            'g1_29dof_lock_waist_with_hand_rev_1_0.urdf')
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent
    raise FileNotFoundError('Could not locate URDF')


def build_reduced_model(urdf_path):
    """Build a Pinocchio reduced model with only right arm joints."""
    model = pin.buildModelFromUrdf(urdf_path)
    lock_ids = []
    for name in model.names:
        if name not in RIGHT_ARM_URDF_JOINTS and name != 'universe':
            lock_ids.append(model.getJointId(name))
    reduced = pin.buildReducedModel(model, lock_ids, np.zeros(model.nq))
    return reduced


class GravityTorquePublisher(Node):
    def __init__(self):
        super().__init__('gravity_torque_publisher')

        urdf_path = find_urdf()
        self.get_logger().info(f'Loading URDF: {urdf_path}')
        self.model = build_reduced_model(urdf_path)
        self.data = self.model.createData()
        self.get_logger().info(
            f'Pinocchio reduced model: {self.model.nq} joints')

        # Verify joint order matches
        for i, name in enumerate(self.model.names):
            if name != 'universe':
                self.get_logger().info(f'  Joint {i}: {name}')

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            LowState, '/lf/lowstate', self.lowstate_cb, qos)

        self.pub = self.create_publisher(
            Float32MultiArray, '/right_arm_gravity_torques', 10)

        self.get_logger().info('Gravity torque publisher ready')

    def lowstate_cb(self, msg: LowState):
        # Extract right arm joint positions
        q = np.zeros(self.model.nq)
        for i, motor_idx in enumerate(RIGHT_ARM_MOTOR_INDICES):
            if i < self.model.nq and motor_idx < len(msg.motor_state):
                q[i] = msg.motor_state[motor_idx].q

        # RNEA with zero velocity and acceleration = pure gravity torques
        v = np.zeros(self.model.nv)
        a = np.zeros(self.model.nv)
        tau_gravity = pin.rnea(self.model, self.data, q, v, a)

        out = Float32MultiArray()
        out.data = [float(t) for t in tau_gravity]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = GravityTorquePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
