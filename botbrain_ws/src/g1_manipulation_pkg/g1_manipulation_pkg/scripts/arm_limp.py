#!/usr/bin/env python3
"""
获取上身手臂控制权，调低手臂 PD 参数使其可被人力推动，腰部不受影响。
按 r 归还控制权，Ctrl+C 也会自动归还。

用法:
  ros2 run g1_manipulation_pkg arm_limp
"""
import os
import sys
import time
import threading
import termios
import tty
import yaml

from unitree_sdk2py.core.channel import (
    ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize,
)
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC

# 手臂关节索引 15-28（不含腰部 12/13/14）
ARM_JOINTS = list(range(15, 29))
WAIST_YAW  = 12   # 单独锁住，防止 arm_sdk 接管后变软
WEIGHT_JOINT = 29  # kNotUsedJoint0 — arm_sdk 混合权重

# 低 PD：手臂可被人力推动
KP_LIMP = 5.0
KD_LIMP = 0.5


def read_interface() -> str:
    cfg = "/botbrain_ws/robot_config.yaml"
    try:
        with open(cfg) as f:
            return yaml.safe_load(f).get("robot_configuration", {}).get("network_interface", "eth0")
    except Exception:
        return "eth0"


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    try:
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main():
    iface = read_interface()

    import unitree_sdk2py.core.channel as _ch
    class _NoDomain:
        def __init__(self, *a, **kw): pass
        def close(self): pass
        def __del__(self): pass
    _ch.Domain = _NoDomain

    ChannelFactoryInitialize(0, iface)

    state_buf = {"msg": None, "lock": threading.Lock()}

    def on_state(msg):
        with state_buf["lock"]:
            state_buf["msg"] = msg

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(on_state)

    pub = ChannelPublisher("rt/arm_sdk", LowCmd_)
    pub.Init()

    print("等待机器人状态...", flush=True)
    while True:
        with state_buf["lock"]:
            if state_buf["msg"] is not None:
                break
        time.sleep(0.01)

    crc = CRC()

    def get_q():
        with state_buf["lock"]:
            m = state_buf["msg"]
        return [m.motor_state[i].q for i in range(35)]

    def get_mode_machine():
        with state_buf["lock"]:
            m = state_buf["msg"]
        return getattr(m, "mode_machine", 0)

    def make_cmd(weight: float, q_vals: list) -> LowCmd_:
        cmd = unitree_hg_msg_dds__LowCmd_()
        cmd.mode_pr = 1
        cmd.mode_machine = get_mode_machine()
        # weight 关节
        cmd.motor_cmd[WEIGHT_JOINT].mode = 1
        cmd.motor_cmd[WEIGHT_JOINT].q    = weight
        cmd.motor_cmd[WEIGHT_JOINT].kp   = 0.0
        cmd.motor_cmd[WEIGHT_JOINT].kd   = 0.0
        # 腰部 yaw 锁住原位（高 PD，防止 arm_sdk 接管后变软）
        cmd.motor_cmd[WAIST_YAW].mode = 1
        cmd.motor_cmd[WAIST_YAW].q    = float(q_vals[WAIST_YAW])
        cmd.motor_cmd[WAIST_YAW].kp   = 300.0
        cmd.motor_cmd[WAIST_YAW].kd   = 3.0
        # 手臂关节
        for i in ARM_JOINTS:
            cmd.motor_cmd[i].mode = 1
            cmd.motor_cmd[i].q    = float(q_vals[i])
            cmd.motor_cmd[i].kp   = KP_LIMP
            cmd.motor_cmd[i].kd   = KD_LIMP
        cmd.crc = crc.Crc(cmd)
        return cmd

    # ── 获取控制权：weight 从 0 渐升到 1（1 秒）──
    print("获取手臂控制权...", flush=True)
    steps = 50
    for i in range(steps + 1):
        w = float(i) / steps
        pub.Write(make_cmd(w, get_q()))
        time.sleep(0.02)

    print("控制权已获取，手臂 PD 已调低，可手动摆动手臂。")
    print("按 r 归还控制权并退出，Ctrl+C 同样归还后退出。\n")

    # ── 保持低 PD 命令（20 Hz）──
    running = True

    def hold_loop():
        while running:
            pub.Write(make_cmd(1.0, get_q()))
            time.sleep(0.05)

    t = threading.Thread(target=hold_loop, daemon=True)
    t.start()

    # ── 等待 'r' 键 ──
    settings = termios.tcgetattr(sys.stdin)
    try:
        while True:
            key = get_key(settings)
            if key in ("r", "R", "\x03"):
                break
    except Exception:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    running = False
    time.sleep(0.1)

    # ── 归还控制权：weight 从 1 渐降到 0（2 秒）──
    print("\n归还控制权...", flush=True)
    steps = 100
    for i in range(steps + 1):
        w = 1.0 - float(i) / steps
        pub.Write(make_cmd(w, get_q()))
        time.sleep(0.02)

    # mode_pr=0 彻底释放
    release = unitree_hg_msg_dds__LowCmd_()
    release.mode_pr = 0
    release.mode_machine = get_mode_machine()
    release.crc = crc.Crc(release)
    for _ in range(5):
        pub.Write(release)
        time.sleep(0.002)

    print("控制权已归还。")


if __name__ == "__main__":
    main()
