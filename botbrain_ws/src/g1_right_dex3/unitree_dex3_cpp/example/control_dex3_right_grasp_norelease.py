from __future__ import annotations

import argparse
import os
import sys
import time
from typing import List, Sequence

NUM_HAND_DOFS = 7
MAX_ABS_Q = 2.0
DEFAULT_RATE_HZ = 80.0
DEFAULT_CLOSE_S = 1.8
DEFAULT_RELEASE_S = 1.2
DEFAULT_CONTACT_THRESHOLD = 0.06
DEFAULT_CONTACT_CONSEC = 4
DEFAULT_TACTILE_WAIT_S = 1.0
DEFAULT_STATUS_INTERVAL_S = 1.0
DEFAULT_OPEN_POSE = [0.0] * NUM_HAND_DOFS
DEFAULT_CLOSE_POSE = [0.0, -0.9, -1.5, 1.57, 1.7, 1.57, 1.7]


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Dex-3 right hand compliant grasp demo: close until contact, keep holding, "
            "and release back to the open pose when Ctrl+C is pressed."
        )
    )
    p.add_argument(
        "net_if",
        nargs="?",
        default="eth0",
        help="Network interface connected to robot (default: eth0)",
    )
    p.add_argument(
        "--open",
        nargs=NUM_HAND_DOFS,
        type=float,
        default=DEFAULT_OPEN_POSE,
        metavar=("O0", "O1", "O2", "O3", "O4", "O5", "O6"),
        help="Open pose (7 DOFs)",
    )
    p.add_argument(
        "--close",
        nargs=NUM_HAND_DOFS,
        type=float,
        default=DEFAULT_CLOSE_POSE,
        metavar=("C0", "C1", "C2", "C3", "C4", "C5", "C6"),
        help="Closed pose (7 DOFs)",
    )
    p.add_argument(
        "--rate",
        type=float,
        default=DEFAULT_RATE_HZ,
        help=f"Control loop rate in Hz (default: {DEFAULT_RATE_HZ})",
    )
    p.add_argument(
        "--close-time",
        type=float,
        default=DEFAULT_CLOSE_S,
        help=f"Close phase duration in seconds (default: {DEFAULT_CLOSE_S})",
    )
    p.add_argument(
        "--release-time",
        type=float,
        default=DEFAULT_RELEASE_S,
        help=f"Release-to-open duration in seconds after Ctrl+C (default: {DEFAULT_RELEASE_S})",
    )
    p.add_argument(
        "--contact-threshold",
        type=float,
        default=DEFAULT_CONTACT_THRESHOLD,
        help=(
            "Contact threshold on tactile aggregate finger_palm (default: "
            f"{DEFAULT_CONTACT_THRESHOLD})"
        ),
    )
    p.add_argument(
        "--contact-consec",
        type=int,
        default=DEFAULT_CONTACT_CONSEC,
        help=(
            "Required consecutive contact detections before latching (default: "
            f"{DEFAULT_CONTACT_CONSEC})"
        ),
    )
    p.add_argument(
        "--tactile-wait",
        type=float,
        default=DEFAULT_TACTILE_WAIT_S,
        help=(
            "How long to wait for first tactile frame in seconds before fallback "
            f"(default: {DEFAULT_TACTILE_WAIT_S})"
        ),
    )
    p.add_argument(
        "--status-interval",
        type=float,
        default=DEFAULT_STATUS_INTERVAL_S,
        help=(
            "How often to print hold status in seconds; set 0 to disable "
            f"(default: {DEFAULT_STATUS_INTERVAL_S})"
        ),
    )
    return p.parse_args()


def _clamp(v: float, limit: float = MAX_ABS_Q) -> float:
    if v > limit:
        return limit
    if v < -limit:
        return -limit
    return v


def _sanitize_pose(values: Sequence[float]) -> List[float]:
    if len(values) != NUM_HAND_DOFS:
        raise ValueError(f"Expected {NUM_HAND_DOFS} values, got {len(values)}")
    return [_clamp(float(v)) for v in values]


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


def _ramp_pose(alpha: float, start_pose: Sequence[float], end_pose: Sequence[float]) -> List[float]:
    return [start_pose[i] + (end_pose[i] - start_pose[i]) * alpha for i in range(NUM_HAND_DOFS)]


def _read_contact_level(ctl) -> float:
    tactile = ctl.get_dex3_right_tactile_agg()
    return float(tactile.finger_palm)


def _try_read_contact_level(ctl) -> float | None:
    try:
        return _read_contact_level(ctl)
    except RuntimeError:
        return None


def _wait_tactile_ready(ctl, wait_s: float, rate_hz: float) -> bool:
    if wait_s <= 0.0:
        return _try_read_contact_level(ctl) is not None

    dt = 1.0 / rate_hz
    deadline = time.time() + wait_s
    while time.time() < deadline:
        if _try_read_contact_level(ctl) is not None:
            return True
        time.sleep(dt)
    return False


def _hold_until_interrupt(
    ctl,
    pose: List[float],
    rate_hz: float,
    tactile_ready: bool,
    status_interval_s: float,
) -> None:
    dt = 1.0 / rate_hz
    next_status_t = time.time() + status_interval_s if status_interval_s > 0.0 else None
    while True:
        ctl.step_hands([], pose)

        now = time.time()
        if next_status_t is not None and now >= next_status_t:
            if tactile_ready:
                contact_level = _try_read_contact_level(ctl)
                if contact_level is None:
                    tactile_ready = False
                    print("Warning: tactile stream lost during hold, continue commanding hold pose.")
                else:
                    print(f"Holding grasp... finger_palm={contact_level:.4f}")
            #else:
             #   print("Holding grasp...")
            next_status_t = now + status_interval_s

        time.sleep(dt)


def _release_to_open(
    ctl,
    hold_pose: List[float],
    open_pose: List[float],
    rate_hz: float,
    release_time_s: float,
) -> None:
    if release_time_s <= 0.0:
        ctl.step_hands([], open_pose)
        _send_pose_for_duration(ctl, open_pose, rate_hz, 0.2)
        return

    dt = 1.0 / rate_hz
    release_steps = max(1, int(release_time_s * rate_hz))
    for step in range(1, release_steps + 1):
        alpha = step / release_steps
        cmd_pose = _ramp_pose(alpha, hold_pose, open_pose)
        ctl.step_hands([], cmd_pose)
        time.sleep(dt)
    _send_pose_for_duration(ctl, open_pose, rate_hz, 0.2)


def main() -> int:
    args = _parse_args()

    if args.rate <= 0.0:
        print("Error: --rate must be > 0")
        return 2
    if args.close_time <= 0.0:
        print("Error: --close-time must be > 0")
        return 2
    if args.release_time < 0.0:
        print("Error: --release-time must be >= 0")
        return 2
    if args.contact_threshold < 0.0:
        print("Error: --contact-threshold must be >= 0")
        return 2
    if args.contact_consec <= 0:
        print("Error: --contact-consec must be > 0")
        return 2
    if args.tactile_wait < 0.0:
        print("Error: --tactile-wait must be >= 0")
        return 2
    if args.status_interval < 0.0:
        print("Error: --status-interval must be >= 0")
        return 2

    sys.path.insert(0, os.path.dirname(__file__))

    from config import RobotConfig  # noqa: E402
    from unitree_cpp import UnitreeController  # type: ignore # noqa: E402

    open_pose = _sanitize_pose(args.open)
    close_pose = _sanitize_pose(args.close)

    cfg = RobotConfig()
    u = cfg.unitree.to_dict()
    u["net_if"] = args.net_if
    u["hand_type"] = "Dex-3"
    u["num_dofs"] = cfg.num_dofs
    u["stiffness"] = cfg.stiffness
    u["damping"] = cfg.damping

    print(f"Starting compliant grasp demo on net_if='{u['net_if']}'")
    print(f"open={open_pose}")
    print(f"close={close_pose}")
    print(
        f"rate={args.rate:.1f}Hz close_time={args.close_time:.2f}s "
        f"release_time={args.release_time:.2f}s "
        f"contact_threshold={args.contact_threshold:.4f} "
        f"contact_consec={args.contact_consec}"
    )
    if args.status_interval > 0.0:
        print(f"hold_status_interval={args.status_interval:.2f}s")

    ctl = UnitreeController(u)
    if not _wait_self_check(ctl):
        print("self_check failed. Check net_if/network/DDS setup.")
        return 3

    print("State: APPROACH_OPEN")
    _send_pose_for_duration(ctl, open_pose, args.rate, 0.3)

    tactile_ready = _wait_tactile_ready(ctl, args.tactile_wait, args.rate)
    if tactile_ready:
        print("Tactile stream ready.")
    else:
        print("Warning: tactile stream not ready, fallback to no-contact mode for this run.")

    print("State: CLOSE_UNTIL_CONTACT")
    dt = 1.0 / args.rate
    total_steps = max(1, int(args.close_time * args.rate))
    contact_count = 0
    contact_latched = False
    hold_pose = close_pose

    for step in range(1, total_steps + 1):
        alpha = step / total_steps
        cmd_pose = _ramp_pose(alpha, open_pose, close_pose)
        ctl.step_hands([], cmd_pose)

        if tactile_ready:
            contact_level = _try_read_contact_level(ctl)
            if contact_level is None:
                tactile_ready = False
                print("Warning: tactile stream lost, fallback to no-contact mode.")
            else:
                if contact_level >= args.contact_threshold:
                    contact_count += 1
                else:
                    contact_count = 0

                if contact_count >= args.contact_consec:
                    contact_latched = True
                    hold_pose = cmd_pose
                    print(
                        f"Contact latched at step={step}/{total_steps}, "
                        f"alpha={alpha:.3f}, finger_palm={contact_level:.4f}"
                    )
                    break

        time.sleep(dt)

    if not contact_latched:
        print("No contact detected before full close; holding at close pose.")

    print("State: HOLD_UNTIL_INTERRUPT")
    print("Press Ctrl+C to release and exit.")
    try:
        _hold_until_interrupt(ctl, hold_pose, args.rate, tactile_ready, args.status_interval)
    except KeyboardInterrupt:
        print("")
        print("State: RELEASE_ON_INTERRUPT")
        try:
            _release_to_open(ctl, hold_pose, open_pose, args.rate, args.release_time)
        except KeyboardInterrupt:
            print("Interrupted again during release; exiting immediately.")
            return 130

    print("Done.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
