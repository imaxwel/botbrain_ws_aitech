from __future__ import annotations

import argparse
import os
import sys
import time
import math

NUM_PACKETS = 9
NUM_TAXELS = 12
BASELINE_FRAMES = 5
PRINT_INTERVAL = 0.5  # seconds, i.e. 2 Hz
TACTILE_RETRY_INTERVAL = 0.1
TACTILE_READY_TIMEOUT_S = 5.0
TACTILE_TOPIC = "rt/lf/dex3/right/state"


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Dex-3 right-hand tactile debug (Python front-end, C++ backend)."
    )
    p.add_argument(
        "net_if",
        nargs="?",
        default="eth0",
        help="Network interface connected to robot (default: eth0)",
    )
    p.add_argument(
        "--id",
        type=int,
        default=None,
        help="Only print one tactile packet ID (0-8). Default: print all IDs.",
    )
    return p.parse_args()


def _capture_baseline(ctl) -> list[list[float]]:
    """Capture BASELINE_FRAMES frames and return their per-taxel average as baseline.

    Invalid (NaN) values are treated as 0 for both accumulation and averaging.
    """
    accum = [[0.0] * NUM_TAXELS for _ in range(NUM_PACKETS)]
    counts = [[0] * NUM_TAXELS for _ in range(NUM_PACKETS)]

    print(f"Capturing baseline ({BASELINE_FRAMES} frames)...")
    for frame in range(BASELINE_FRAMES):
        while True:
            try:
                raw = ctl.get_dex3_right_tactile_raw()
                break
            except RuntimeError as e:
                print(f"tactile not ready during baseline: {e}")
                time.sleep(TACTILE_RETRY_INTERVAL)

        for pid in range(NUM_PACKETS):
            for tid in range(NUM_TAXELS):
                v = raw.packets[pid].pressure[tid]
                if not math.isnan(v):
                    accum[pid][tid] += v
                    counts[pid][tid] += 1
        if frame < BASELINE_FRAMES - 1:
            time.sleep(0.05)  # short pause between frames

    baseline: list[list[float]] = []
    for pid in range(NUM_PACKETS):
        row: list[float] = []
        for tid in range(NUM_TAXELS):
            row.append(accum[pid][tid] / counts[pid][tid] if counts[pid][tid] > 0 else 0.0)
        baseline.append(row)

    print("Baseline captured:")
    for pid in range(NUM_PACKETS):
        vals = ", ".join(f"{baseline[pid][t]:.4f}" for t in range(NUM_TAXELS))
        print(f"  ID{pid}: [{vals}]")
    return baseline


def _print_tactile(ctl, baseline: list[list[float]], selected_id: int | None) -> None:
    """Print per-taxel (current - baseline) difference. NaN values map to 0."""
    raw = ctl.get_dex3_right_tactile_raw()

    print("\n[Dex3 Right Pressure Diff (current - baseline)]")
    packet_ids = [selected_id] if selected_id is not None else list(range(NUM_PACKETS))
    for pid in packet_ids:
        parts: list[str] = []
        for tid in range(NUM_TAXELS):
            v = raw.packets[pid].pressure[tid]
            cur = 0.0 if math.isnan(v) else v
            diff = cur - baseline[pid][tid]
            parts.append(f"{diff:.4f}")
        print(f"ID{pid}: [{', '.join(parts)}]")


def _wait_tactile_ready(ctl, timeout_s: float) -> None:
    deadline = time.time() + timeout_s
    last_error = ""
    while time.time() < deadline:
        try:
            ctl.get_dex3_right_tactile_raw()
            return
        except RuntimeError as e:
            last_error = str(e)
            time.sleep(TACTILE_RETRY_INTERVAL)
    raise RuntimeError(
        "Dex3 tactile topic not ready. "
        f"topic={TACTILE_TOPIC}, timeout={timeout_s:.1f}s, "
        f"last_error={last_error or 'unknown'}"
    )


def main() -> int:
    args = _parse_args()

    if args.id is not None and not (0 <= args.id < NUM_PACKETS):
        print(f"Error: --id must be in [0, {NUM_PACKETS - 1}]")
        return 2

    # Make `from config import RobotConfig` work whether launched from repo root
    # or from within the example directory.
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
    u["handstate_right_topic"] = TACTILE_TOPIC

    print(f"Starting Dex-3 right-hand debug on net_if='{u['net_if']}'...")
    print("Mode: tactile-only (read & print pressure at 2 Hz).")
    print(f"Using fixed tactile topic: {TACTILE_TOPIC}")
    if os.environ.get("CYCLONEDDS_URI"):
        print(f"CYCLONEDDS_URI={os.environ['CYCLONEDDS_URI']}")

    try:
        ctl = UnitreeController(u)
        _wait_tactile_ready(ctl, TACTILE_READY_TIMEOUT_S)
        print(f"Tactile topic ready: {TACTILE_TOPIC}")
    except RuntimeError as e:
        print(str(e))
        print("Hint: verify tactile publisher and net_if.")
        return 3

    ok = False
    for _ in range(50):
        time.sleep(0.1)
        if ctl.self_check():
            ok = True
            break
    if not ok:
        print("self_check failed. Check: net_if name, robot network link, DDS/SDK2 installation.")
        return 2

    baseline = _capture_baseline(ctl)
    while True:
        try:
            _print_tactile(ctl, baseline, args.id)
        except RuntimeError as e:
            print(f"tactile not ready: {e}")
        time.sleep(PRINT_INTERVAL)


if __name__ == "__main__":
    raise SystemExit(main())
