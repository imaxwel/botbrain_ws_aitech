import importlib.util
import math
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[4]
SCRIPT = PROJECT_ROOT / "tools" / "nav" / "compare_transition_localizers.py"


def _load_module():
    spec = importlib.util.spec_from_file_location(
        "compare_transition_localizers", SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def test_ab_summary_keeps_backends_separate():
    module = _load_module()
    summary = module.summarize(
        [
            {
                "backend": "open3d",
                "case": "floor14_a",
                "success": True,
                "latency_ms": 100.0,
                "fitness": 0.99,
                "rmse": 0.10,
            },
            {
                "backend": "open3d",
                "case": "floor14_b",
                "success": False,
                "latency_ms": 120.0,
            },
            {
                "backend": "kiss_matcher",
                "case": "floor14_a",
                "success": True,
                "latency_ms": 80.0,
                "fitness": 0.98,
                "rmse": 0.12,
            },
        ]
    )

    assert summary["open3d"]["success_rate"] == 0.5
    assert summary["open3d"]["mean_latency_ms"] == 110.0
    assert summary["kiss_matcher"]["success_rate"] == 1.0
    assert not math.isnan(summary["kiss_matcher"]["mean_fitness"])
