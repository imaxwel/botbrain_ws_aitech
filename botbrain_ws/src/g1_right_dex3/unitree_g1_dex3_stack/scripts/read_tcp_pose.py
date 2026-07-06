#!/usr/bin/env python3
"""读取 G1 右臂关节角度并计算 TCP 位姿（与 dex3_stack planner/executor 一致）。

TCP 定义: right_wrist_yaw_link 末端 + X 轴偏移 0.175 m（即 right_tcp_link）。
坐标系: torso_link（与 planner KDL chain 的 base_link 一致）。

用法:
    python3 read_tcp_pose.py          # 真机 (domain_id=0)
    python3 read_tcp_pose.py --sim    # 仿真 (domain_id=1)
"""
import argparse
import numpy as np
import os
import sys

from scipy.spatial.transform import Rotation as R
import pinocchio as pin
from unitree_sdk2py.core.channel import ChannelFactoryInitialize

# xr_teleoperate 包含 G1_29_ArmController
XR_TELEOP_DIR = "/botbrain_ws/src/g1_right_dex3/xr_teleoperate"
sys.path.insert(0, XR_TELEOP_DIR)
from teleop.robot_control.robot_arm import G1_29_ArmController

# 右臂 7 个关节名（与 g1_dex3_joint_defs.hpp kRightArmJointIndices 对应）
RIGHT_ARM_JOINT_NAMES = [
    "RightShoulderPitch",  # 22
    "RightShoulderRoll",   # 23
    "RightShoulderYaw",    # 24
    "RightElbow",          # 25
    "RightWristRoll",      # 26
    "RightWristPitch",     # 27
    "RightWristYaw",       # 28
]

# TCP 偏移: 与 URDF right_tcp_joint 及 planner tcp_offset_x 一致
TCP_OFFSET_X = 0.175  # m, 沿 wrist_yaw X 轴

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_FILENAME = "g1_29dof_lock_waist_with_hand_rev_1_0.urdf"


def find_urdf():
    """Locate the G1 URDF without requiring a hand-built scripts/robots/ symlink.

    Lookup order:
      1. ament_index_python.get_package_share_directory('unitree_g1_dex3_stack')/robots/g1_description/<file>
         — works after `colcon build` + `source install/setup.bash`.
      2. Walk up from this script's directory, checking ``<dir>/robots/g1_description/<file>``
         — works on a fresh checkout without colcon, since the URDF lives at
         ``<repo>/src/unitree_g1_dex3_stack-main/robots/g1_description/`` next
         to ``scripts/``.
    """
    # 1. ament_index colcon-installed share directory
    try:
        from ament_index_python import get_package_share_directory  # type: ignore
        share_dir = get_package_share_directory("unitree_g1_dex3_stack")
        candidate = os.path.join(share_dir, "robots", "g1_description", URDF_FILENAME)
        if os.path.isfile(candidate):
            return candidate
    except Exception:
        pass

    # 2. Walk up from the script directory until robots/g1_description/<file> is found
    cur = SCRIPT_DIR
    for _ in range(6):  # bounded climb; repo layout has it within 1-2 levels
        candidate = os.path.join(cur, "robots", "g1_description", URDF_FILENAME)
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(cur)
        if parent == cur:  # reached filesystem root
            break
        cur = parent

    raise FileNotFoundError(
        f"Could not locate {URDF_FILENAME!r}. Tried ament_index "
        f"share_dir/robots/g1_description/ and ancestor robots/g1_description/ "
        f"directories starting at {SCRIPT_DIR!r}. Ensure the package is "
        f"either colcon-installed or run from inside the repo checkout."
    )


URDF_PATH = find_urdf()

# URDF 中右臂关节名（用于 Pinocchio buildReducedModel）
RIGHT_ARM_URDF_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]


def build_reduced_model(urdf_path):
    """构建仅含右臂 7 自由度的 Pinocchio 模型，并添加 TCP frame。"""
    model = pin.buildModelFromUrdf(urdf_path)

    # 锁定除右臂以外的所有关节（用 joint ID 列表）
    lock_ids = []
    for i, name in enumerate(model.names):
        if name not in RIGHT_ARM_URDF_JOINTS and name != "universe":
            lock_ids.append(model.getJointId(name))

    reduced = pin.buildReducedModel(model, lock_ids, np.zeros(model.nq))

    # 添加 TCP frame: right_wrist_yaw_joint + X 轴偏移 0.175 m
    reduced.addFrame(
        pin.Frame(
            "right_tcp",
            reduced.getJointId("right_wrist_yaw_joint"),
            pin.SE3(np.eye(3), np.array([TCP_OFFSET_X, 0.0, 0.0]).T),
            pin.FrameType.OP_FRAME,
        )
    )
    return reduced


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="读取右臂关节角度并计算 TCP 位姿")
    parser.add_argument("--sim", action="store_true", help="仿真模式 (domain_id=1)")
    parser.add_argument("--network-interface", type=str, default=None)
    args = parser.parse_args()

    domain_id = 1 if args.sim else 0
    ChannelFactoryInitialize(domain_id, networkInterface=args.network_interface)

    arm_ctrl = G1_29_ArmController(motion_mode=False, simulation_mode=args.sim)
    reduced = build_reduced_model(URDF_PATH)
    data = reduced.createData()

    torso_id = reduced.getFrameId("torso_link")
    tcp_id = reduced.getFrameId("right_tcp")

    print("\n" + "=" * 60)
    print("  右臂关节角度")
    print("-" * 60)

    dual_q = arm_ctrl.get_current_dual_arm_q()  # 14维: 前7左臂, 后7右臂
    right_q_rad = dual_q[7:]
    right_q_deg = np.degrees(right_q_rad)

    for name, deg, rad in zip(RIGHT_ARM_JOINT_NAMES, right_q_deg, right_q_rad):
        print(f"  {name:>22s}:  {deg:+8.2f}°  ({rad:+7.4f} rad)")

    # FK: 所有 frame 位姿相对于模型根 (pelvis)
    pin.framesForwardKinematics(reduced, data, right_q_rad)

    # TCP 相对于 torso_link: torso_inv * tcp
    torso_pose = data.oMf[torso_id]
    tcp_in_pelvis = data.oMf[tcp_id]
    tcp_pose = torso_pose.actInv(tcp_in_pelvis)

    pos = tcp_pose.translation
    rpy_deg = np.degrees(R.from_matrix(tcp_pose.rotation).as_euler("xyz"))

    print("\n" + "-" * 60)
    print(f"  右手 TCP 位姿 (相对于 torso_link, offset={TCP_OFFSET_X} m)")
    print("-" * 60)
    print(f"  位置 X: {pos[0]:+.5f} m")
    print(f"  位置 Y: {pos[1]:+.5f} m")
    print(f"  位置 Z: {pos[2]:+.5f} m")
    print(f"  Roll  : {rpy_deg[0]:+.2f}°")
    print(f"  Pitch : {rpy_deg[1]:+.2f}°")
    print(f"  Yaw   : {rpy_deg[2]:+.2f}°")
    print("\n  4x4 齐次变换矩阵:")
    print(np.round(tcp_pose.homogeneous, 6))
    print("=" * 60)