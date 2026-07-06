from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import List

NUM_HAND_DOFS = 7
DEFAULT_RATE_HZ = 50.0
DEFAULT_HOLD_S = 1.0
DEFAULT_RAMP_S = 1.0
MAX_ABS_Q = 2
DEFAULT_BASE_POSE = [0.0] * NUM_HAND_DOFS
DEFAULT_LAST_POSE_FILE = os.path.expanduser("~/.cache/dex3_right_last_pose.json")


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Move Dex-3 right-hand 7 DOFs to a target pose once (non-periodic), "
            "hold for a short duration, then exit."
        )
    )
    p.add_argument(
        "net_if",
        nargs="?",
        default="eth0",
        help="Network interface connected to robot (default: eth0)",
    )
    p.add_argument(
        "target",
        nargs=NUM_HAND_DOFS,
        type=float,
        metavar=("T0", "T1", "T2", "T3", "T4", "T5", "T6"),
        help="Target pose for right-hand 7 DOFs",
    )
    p.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE_HZ,
        help=f"Command publish rate in Hz (default: {DEFAULT_RATE_HZ})",
    )
    p.add_argument(
        "--hold",
        type=float,
        default=DEFAULT_HOLD_S,
        help=f"Hold duration in seconds (default: {DEFAULT_HOLD_S})",
    )
    p.add_argument(
        "--ramp",
        type=float,
        default=DEFAULT_RAMP_S,
        help=f"Linear ramp time from base to target in seconds (default: {DEFAULT_RAMP_S})",
    )
    p.add_argument(
        "--base",
        nargs=NUM_HAND_DOFS,
        type=float,
        default=DEFAULT_BASE_POSE,
        metavar=("B0", "B1", "B2", "B3", "B4", "B5", "B6"),
        help="Base pose for 7 DOFs before applying target pose",
    )
    p.add_argument(
        "--return-base",
        action="store_true",
        help="After hold, send base pose briefly before exit",
    )
    return p.parse_args()


def _clamp(v: float, limit: float) -> float:
    if v > limit:
        return limit
    if v < -limit:
        return -limit
    return v


def _wait_self_check(ctl, max_wait_s: float = 5.0) -> bool:
    deadline = time.time() + max_wait_s
    while time.time() < deadline:
        time.sleep(0.1)
        if ctl.self_check():
            return True
    return False


def _send_pose_for_duration(ctl, pose: List[float], rate_hz: float, duration_s: float) -> None:
    dt = 1.0 / rate_hz
    end_t = time.time() + duration_s
    while time.time() < end_t:
        ctl.step_hands([], pose)
        time.sleep(dt)


def _send_ramp(ctl, start_pose: List[float], end_pose: List[float], rate_hz: float, ramp_s: float) -> None:
    if ramp_s <= 0.0:
        ctl.step_hands([], end_pose)
        return

    dt = 1.0 / rate_hz
    steps = max(1, int(ramp_s * rate_hz))
    for k in range(1, steps + 1):
        alpha = k / steps
        pose = [start_pose[i] + (end_pose[i] - start_pose[i]) * alpha for i in range(NUM_HAND_DOFS)]
        ctl.step_hands([], pose)
        time.sleep(dt)


def _load_pose_file(path: str, max_abs_q: float) -> List[float] | None:
    try:
        with open(path, "r", encoding="utf-8") as f:
            obj = json.load(f)
    except FileNotFoundError:
        return None
    except Exception as e:
        print(f"Warning: failed to read last pose file '{path}': {e}")
        return None

    if not isinstance(obj, list) or len(obj) != NUM_HAND_DOFS:
        print(f"Warning: invalid last pose file format in '{path}', ignore it")
        return None

    try:
        return [_clamp(float(v), max_abs_q) for v in obj]
    except Exception:
        print(f"Warning: non-numeric values in last pose file '{path}', ignore it")
        return None


def _save_pose_file(path: str, pose: List[float]) -> None:
    try:
        folder = os.path.dirname(path)
        if folder:
            os.makedirs(folder, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump([float(v) for v in pose], f)
    except Exception as e:
        print(f"Warning: failed to save last pose file '{path}': {e}")


def main() -> int:
    args = _parse_args()

    if args.rate <= 0.0:
        print("Error: --rate must be > 0")
        return 2
    if args.hold <= 0.0:
        print("Error: --hold must be > 0")
        return 2
    if args.ramp < 0.0:
        print("Error: --ramp must be >= 0")
        return 2

    # Make local imports work when launched from different cwd.
    sys.path.insert(0, os.path.dirname(__file__))

    from config import RobotConfig  # noqa: E402
    from unitree_cpp import UnitreeController  # type: ignore # noqa: E402

    cfg = RobotConfig()
    u = cfg.unitree.to_dict()
    u["net_if"] = args.net_if
    u["hand_type"] = "Dex-3"
    u["num_dofs"] = cfg.num_dofs
    u["stiffness"] = cfg.stiffness
    u["damping"] = cfg.damping

    base = [_clamp(v, MAX_ABS_Q) for v in args.base]
    target_pose = [_clamp(v, MAX_ABS_Q) for v in args.target]

    last_pose = _load_pose_file(DEFAULT_LAST_POSE_FILE, MAX_ABS_Q)
    if last_pose is None:
        start_pose = base
        start_source = "base (auto fallback)"
    else:
        start_pose = last_pose
        start_source = "last (auto)"

    print(f"Starting Dex-3 right-hand setpoint control on net_if='{u['net_if']}'")
    print(
        f"ramp={args.ramp:.2f}s, return_ramp={args.ramp:.2f}s, "
        f"hold={args.hold:.2f}s, rate={args.rate:.1f}Hz"
    )
    print(f"max_abs_q fixed to {MAX_ABS_Q:.1f}")
    print(f"base={base}")
    print(f"start={start_pose}  source={start_source}")
    print(f"target={target_pose}")
    if os.environ.get("CYCLONEDDS_URI"):
        print(f"CYCLONEDDS_URI={os.environ['CYCLONEDDS_URI']}")

    ctl = UnitreeController(u)
    if not _wait_self_check(ctl):
        print("self_check failed. Check net_if/network/DDS setup.")
        return 3

    print("Ramping to setpoint...")
    _send_ramp(ctl, start_pose, target_pose, args.rate, args.ramp)
    print("Holding setpoint...")
    _send_pose_for_duration(ctl, target_pose, args.rate, args.hold)

    if args.return_base:
        print("Ramping back to base pose...")
        _send_ramp(ctl, target_pose, base, args.rate, args.ramp)
        _send_pose_for_duration(ctl, base, args.rate, 0.2)
        _save_pose_file(DEFAULT_LAST_POSE_FILE, base)
    else:
        _save_pose_file(DEFAULT_LAST_POSE_FILE, target_pose)

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
