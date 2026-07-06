"""Tactile press guard for Dex-3 right hand ID5 sensor.

Monitors tactile packet ID5 and writes status lines to stdout so the
parent process (apriltag_button_press_node) can react:

  READY              – baseline captured, monitoring started
  THRESHOLD_EXCEEDED – max diff across all 12 taxels exceeded threshold

Usage:
    python tactile_press_guard.py [net_if] [--threshold T] [--hz HZ]

    net_if      Network interface (default: enP8p1s0)
    --threshold Contact threshold (baseline-subtracted diff, default: 0.05)
    --hz        Poll rate in Hz (default: 50)
"""
from __future__ import annotations

import argparse
import math
import os
import signal
import sys
import time

TARGET_ID = 5
NUM_TAXELS = 12
BASELINE_FRAMES = 5
TACTILE_TOPIC = "rt/lf/dex3/right/state"
_STOP = False


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Tactile press guard: ID5 threshold monitor.")
    p.add_argument("net_if", nargs="?", default="enP8p1s0",
                   help="Network interface (default: enP8p1s0)")
    p.add_argument("--threshold", type=float, default=0.05,
                   help="Contact threshold (diff units, default: 0.05)")
    p.add_argument("--hz", type=float, default=50.0,
                   help="Poll rate in Hz (default: 50)")
    return p.parse_args()


def _signal_handler(sig, frame):
    global _STOP
    _STOP = True


def _wait_ready(ctl, timeout_s: float = 8.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            ctl.get_dex3_right_tactile_raw()
            return
        except RuntimeError:
            time.sleep(0.05)
    raise RuntimeError(f"Tactile topic not ready after {timeout_s:.1f}s (topic={TACTILE_TOPIC})")


def _capture_baseline(ctl) -> list[float]:
    accum = [0.0] * NUM_TAXELS
    counts = [0] * NUM_TAXELS
    for _ in range(BASELINE_FRAMES):
        raw = ctl.get_dex3_right_tactile_raw()
        pkt = raw.packets[TARGET_ID]
        for t in range(NUM_TAXELS):
            v = pkt.pressure[t]
            if not math.isnan(v):
                accum[t] += v
                counts[t] += 1
        time.sleep(0.02)
    return [accum[t] / counts[t] if counts[t] > 0 else 0.0 for t in range(NUM_TAXELS)]


def _max_diff(ctl, baseline: list[float]) -> float:
    raw = ctl.get_dex3_right_tactile_raw()
    pkt = raw.packets[TARGET_ID]
    max_d = 0.0
    for t in range(NUM_TAXELS):
        v = pkt.pressure[t]
        if not math.isnan(v):
            d = v - baseline[t]
            if d > max_d:
                max_d = d
    return max_d


def main() -> int:
    global _STOP
    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    args = _parse_args()
    interval = 1.0 / max(1.0, args.hz)

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from config import RobotConfig          # noqa: E402
    from unitree_cpp import UnitreeController  # type: ignore  # noqa: E402

    cfg = RobotConfig()
    u = cfg.unitree.to_dict()
    u["net_if"] = args.net_if
    u["hand_type"] = "Dex-3"
    u["num_dofs"] = cfg.num_dofs
    u["stiffness"] = cfg.stiffness
    u["damping"] = cfg.damping
    u["handstate_right_topic"] = TACTILE_TOPIC

    try:
        ctl = UnitreeController(u)
        _wait_ready(ctl)
    except RuntimeError as e:
        sys.stdout.write(f"ERROR: {e}\n")
        sys.stdout.flush()
        return 1

    baseline = _capture_baseline(ctl)
    sys.stdout.write("READY\n")
    sys.stdout.flush()

    exceeded_reported = False
    while not _STOP:
        try:
            diff = _max_diff(ctl, baseline)
        except RuntimeError:
            time.sleep(interval)
            continue

        if not exceeded_reported and diff >= args.threshold:
            sys.stdout.write(f"THRESHOLD_EXCEEDED diff={diff:.4f}\n")
            sys.stdout.flush()
            exceeded_reported = True

        time.sleep(interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
