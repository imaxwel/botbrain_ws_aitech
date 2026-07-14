import importlib.util
from pathlib import Path

import numpy as np


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT_PATH = PACKAGE_DIR / "scripts" / "shift_pcd_z.py"


def _load_shift_module():
    spec = importlib.util.spec_from_file_location("shift_pcd_z", SCRIPT_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_shift_binary_pcd_preserves_header_and_non_z_fields(tmp_path):
    dtype = np.dtype([
        ("x", "<f4"),
        ("y", "<f4"),
        ("z", "<f4"),
        ("intensity", "<f4"),
        ("ring", "<u2"),
    ])
    points = np.array([
        (1.0, -2.0, -1.247, 0.25, 3),
        (4.5, 5.0, 0.125, 0.75, 17),
        (-3.0, 8.0, 2.5, 1.0, 255),
    ], dtype=dtype)
    header = (
        b"# .PCD v0.7 - Point Cloud Data file format\n"
        b"VERSION 0.7\n"
        b"FIELDS x y z intensity ring\n"
        b"SIZE 4 4 4 4 2\n"
        b"TYPE F F F F U\n"
        b"COUNT 1 1 1 1 1\n"
        b"WIDTH 3\n"
        b"HEIGHT 1\n"
        b"VIEWPOINT 0 0 0 1 0 0 0\n"
        b"POINTS 3\n"
        b"DATA binary\n"
    )
    pcd_path = tmp_path / "synthetic.pcd"
    pcd_path.write_bytes(header + points.tobytes())

    _load_shift_module().shift_pcd_z(str(pcd_path), 1.247)

    shifted_raw = pcd_path.read_bytes()
    assert shifted_raw[: len(header)] == header
    shifted = np.frombuffer(shifted_raw[len(header):], dtype=dtype)
    np.testing.assert_allclose(
        shifted["z"], points["z"] + np.float32(1.247), rtol=0, atol=1e-6)
    for field in ("x", "y", "intensity", "ring"):
        np.testing.assert_array_equal(shifted[field], points[field])
    assert not pcd_path.with_suffix(".pcd.tmp").exists()


def test_shift_script_is_installed_with_g1_pkg():
    cmake = (PACKAGE_DIR / "CMakeLists.txt").read_text(encoding="utf-8")
    install_block = cmake.split("install(PROGRAMS", 1)[1].split(")", 1)[0]
    assert "scripts/shift_pcd_z.py" in install_block
