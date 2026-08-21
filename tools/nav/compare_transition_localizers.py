#!/usr/bin/env python3
"""Compare recorded Open3D and optional KISS-Matcher transition runs.

Each JSONL row must contain backend, case, success and latency_ms. Successful
rows may also contain fitness and rmse. This tool deliberately does not load a
matcher into the robot runtime; it is the offline A/B boundary used after a
KISS-Matcher build with its complete dependency set is available.
"""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Any, Iterable


REQUIRED_FIELDS = {"backend", "case", "success", "latency_ms"}


def load_records(path: Path) -> list[dict[str, Any]]:
    records: list[dict[str, Any]] = []
    for line_number, line in enumerate(
        path.read_text(encoding="utf-8").splitlines(), start=1
    ):
        if not line.strip():
            continue
        record = json.loads(line)
        if not isinstance(record, dict) or not REQUIRED_FIELDS <= record.keys():
            raise ValueError(f"invalid record at {path}:{line_number}")
        if not isinstance(record["success"], bool):
            raise ValueError(f"success must be boolean at {path}:{line_number}")
        latency = float(record["latency_ms"])
        if not math.isfinite(latency) or latency < 0.0:
            raise ValueError(f"invalid latency at {path}:{line_number}")
        normalized = dict(record)
        normalized["backend"] = str(record["backend"]).strip()
        normalized["case"] = str(record["case"]).strip()
        normalized["latency_ms"] = latency
        if not normalized["backend"] or not normalized["case"]:
            raise ValueError(f"empty backend or case at {path}:{line_number}")
        records.append(normalized)
    return records


def summarize(records: Iterable[dict[str, Any]]) -> dict[str, dict[str, float]]:
    grouped: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        grouped[record["backend"]].append(record)
    summary: dict[str, dict[str, float]] = {}
    for backend, items in sorted(grouped.items()):
        successful = [item for item in items if item["success"]]
        summary[backend] = {
            "runs": float(len(items)),
            "success_rate": len(successful) / len(items),
            "mean_latency_ms": sum(item["latency_ms"] for item in items)
            / len(items),
            "mean_fitness": (
                sum(float(item["fitness"]) for item in successful)
                / len(successful)
                if successful and all("fitness" in item for item in successful)
                else math.nan
            ),
            "mean_rmse": (
                sum(float(item["rmse"]) for item in successful)
                / len(successful)
                if successful and all("rmse" in item for item in successful)
                else math.nan
            ),
        }
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Compare offline floor-transition localization backends"
    )
    parser.add_argument("jsonl", type=Path)
    return parser.parse_args()


def main() -> int:
    records = load_records(parse_args().jsonl)
    if not records:
        raise SystemExit("no A/B records found")
    print(json.dumps(summarize(records), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
