"""Real-time display of Dex3-1 right-hand tactile sensor packet ID5 (3x4 grid).

Usage:
    python show_tactile_id5.py [net_if] [--hz HZ] [--raw]

    net_if   Network interface connected to robot (default: eth0)
    --hz     Display update rate in Hz (default: 10)
    --raw    Show raw scaled values instead of baseline-subtracted diff
"""
from __future__ import annotations

import argparse
import math
import os
import sys
import time

TARGET_ID = 5
NUM_TAXELS = 12
NUM_PACKETS = 9
TACTILE_TOPIC = "rt/lf/dex3/right/state"
BASELINE_FRAMES = 5
INVALID_SENTINEL = 30000.0
SCALE = 10000.0

# 3-row x 4-col layout matching the official PressSensorState pressure[12] indexing
GRID_ROWS = 3
GRID_COLS = 4


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Real-time Dex3 right-hand tactile ID5 viewer.")
    p.add_argument("net_if", nargs="?", default="eth0",
                   help="Network interface (default: eth0)")
    p.add_argument("--hz", type=float, default=10.0,
                   help="Display refresh rate in Hz (default: 10)")
    p.add_argument("--raw", action="store_true",
                   help="Show raw scaled values, not baseline-subtracted diff")
    return p.parse_args()


def _wait_ready(ctl, timeout_s: float = 5.0) -> None:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            ctl.get_dex3_right_tactile_raw()
            return
        except RuntimeError:
            time.sleep(0.1)
    raise RuntimeError(
        f"Tactile topic not ready after {timeout_s}s. "
        f"topic={TACTILE_TOPIC}. Check net_if and robot connection."
    )


def _capture_baseline(ctl) -> list[float]:
    """Average BASELINE_FRAMES frames for ID5 taxels. Returns list[12]."""
    accum = [0.0] * NUM_TAXELS
    counts = [0] * NUM_TAXELS

    print(f"Capturing baseline for ID{TARGET_ID} ({BASELINE_FRAMES} frames)...")
    for _ in range(BASELINE_FRAMES):
        raw = ctl.get_dex3_right_tactile_raw()
        packet = raw.packets[TARGET_ID]
        for t in range(NUM_TAXELS):
            v = packet.pressure[t]
            if not math.isnan(v):
                accum[t] += v
                counts[t] += 1
        time.sleep(0.05)

    baseline = [accum[t] / counts[t] if counts[t] > 0 else 0.0 for t in range(NUM_TAXELS)]
    vals_str = ", ".join(f"{b:.4f}" for b in baseline)
    print(f"Baseline ID{TARGET_ID}: [{vals_str}]")
    return baseline


def _intensity_bar(value: float, width: int = 8, lo: float = 0.0, hi: float = 0.5) -> str:
    """ASCII intensity bar for a float value in [lo, hi]."""
    clamped = max(lo, min(hi, value))
    filled = int(round((clamped - lo) / (hi - lo) * width)) if hi > lo else 0
    return "#" * filled + "." * (width - filled)


def _render_grid(values: list[float], baseline: list[float], show_raw: bool) -> str:
    """Render 12 taxels as a 3x4 grid with bar indicators."""
    lines: list[str] = []
    lines.append(f"  Tactile ID{TARGET_ID}  {'(raw)' if show_raw else '(diff from baseline)'}")
    lines.append("  col:   0          1          2          3")
    lines.append("  " + "-" * 49)

    for row in range(GRID_ROWS):
        cells: list[str] = []
        for col in range(GRID_COLS):
            idx = row * GRID_COLS + col
            v = values[idx]
            base = baseline[idx]
            display_val = v if show_raw else (v - base)
            bar = _intensity_bar(display_val)
            cells.append(f"{display_val:+.3f}[{bar}]")
        lines.append(f"  r{row} | " + "  ".join(cells))

    lines.append("  " + "-" * 49)

    # column summary (mean across rows)
    col_means: list[float] = []
    for col in range(GRID_COLS):
        col_vals = [values[row * GRID_COLS + col] for row in range(GRID_ROWS)]
        base_col = [baseline[row * GRID_COLS + col] for row in range(GRID_ROWS)]
        display_col = [v if show_raw else v - b for v, b in zip(col_vals, base_col)]
        valid = [x for x in display_col if not math.isnan(x)]
        col_means.append(sum(valid) / len(valid) if valid else 0.0)
    mean_str = "  col avg: " + "  ".join(f"{m:+.3f}      " for m in col_means)
    lines.append(mean_str)

    return "\n".join(lines)


def _clear_screen() -> None:
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


def main() -> int:
    args = _parse_args()
    interval = 1.0 / max(0.5, args.hz)

    _example_dir = os.path.dirname(os.path.abspath(__file__))
    _build_dir = os.path.join(_example_dir, "..", "build")
    sys.path.insert(0, _example_dir)
    sys.path.insert(0, os.path.abspath(_build_dir))
    from config import RobotConfig       # noqa: E402
    from unitree_cpp import UnitreeController  # type: ignore  # noqa: E402

    cfg = RobotConfig()
    u = cfg.unitree.to_dict()
    u["net_if"] = args.net_if
    u["hand_type"] = "Dex-3"
    u["num_dofs"] = cfg.num_dofs
    u["stiffness"] = cfg.stiffness
    u["damping"] = cfg.damping
    u["handstate_right_topic"] = TACTILE_TOPIC

    print(f"Connecting on net_if='{args.net_if}', topic={TACTILE_TOPIC} ...")
    try:
        ctl = UnitreeController(u)
        _wait_ready(ctl)
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        return 1

    baseline = _capture_baseline(ctl)

    print(f"\nStarting real-time display at {args.hz:.1f} Hz. Press Ctrl-C to stop.\n")
    time.sleep(0.5)

    while True:
        try:
            raw = ctl.get_dex3_right_tactile_raw()
        except RuntimeError as e:
            _clear_screen()
            print(f"[WARN] tactile not ready: {e}")
            time.sleep(interval)
            continue

        packet = raw.packets[TARGET_ID]
        values = [packet.pressure[t] for t in range(NUM_TAXELS)]

        # Replace NaN with 0 for display
        values = [0.0 if math.isnan(v) else v for v in values]

        _clear_screen()
        print(_render_grid(values, baseline, args.raw))
        print(f"\n  lost={packet.lost}  ts={time.strftime('%H:%M:%S')}")
        print("  (Ctrl-C to quit)")

        time.sleep(interval)

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nStopped.")
