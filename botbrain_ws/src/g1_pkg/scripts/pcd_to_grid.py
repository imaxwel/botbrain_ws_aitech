#!/usr/bin/env python3
"""Offline: convert scans.pcd (fast_lio map) → nav2 OccupancyGrid PGM + YAML.

The PCD is already in map frame (camera_init), so projection is a direct z-slice.

Usage:
    python3 pcd_to_grid.py \
        --input  src/g1_pkg/maps/scans.pcd \
        --output src/g1_pkg/maps/accumulated_grid \
        --resolution 0.05 \
        --z-min 0.1 --z-max 2.0
"""
import argparse, math, struct, sys
from pathlib import Path
import numpy as np


def read_pcd_xyz(path: str) -> np.ndarray:
    """Parse PCD header then extract Nx3 XYZ as float64."""
    with open(path, 'rb') as f:
        fields, sizes, types, counts = [], [], [], []
        n_points, data_type = 0, 'ascii'
        header_bytes = 0
        for raw_line in f:
            header_bytes += len(raw_line)
            line = raw_line.decode('utf-8', errors='replace').strip()
            if line.startswith('FIELDS'):  fields = line.split()[1:]
            elif line.startswith('SIZE'):  sizes  = list(map(int, line.split()[1:]))
            elif line.startswith('TYPE'):  types  = line.split()[1:]
            elif line.startswith('COUNT'): counts = list(map(int, line.split()[1:]))
            elif line.startswith('POINTS'): n_points = int(line.split()[1])
            elif line.startswith('DATA'):
                data_type = line.split()[1]
                break  # binary data starts immediately after

        if data_type == 'binary':
            row_size = sum(s * c for s, c in zip(sizes, counts))
            raw = np.frombuffer(f.read(n_points * row_size), dtype=np.uint8)
            # build structured dtype
            dtype_fields = []
            for name, sz, tp, cnt in zip(fields, sizes, types, counts):
                np_type = {'F': f'f{sz}', 'I': f'i{sz}', 'U': f'u{sz}'}[tp]
                for i in range(cnt):
                    dtype_fields.append((f'{name}_{i}' if cnt > 1 else name, np_type))
            pts = raw.reshape(n_points, row_size).view(
                np.dtype(dtype_fields))
        else:
            pts = np.loadtxt(f, max_rows=n_points)
            # wrap as structured for uniform access below
            dt = [(name, 'f4') for name in fields]
            structured = np.zeros(len(pts), dtype=dt)
            for i, name in enumerate(fields):
                structured[name] = pts[:, i]
            pts = structured

    xyz = np.column_stack([
        pts['x'].astype(np.float64),
        pts['y'].astype(np.float64),
        pts['z'].astype(np.float64),
    ])
    return xyz


def pcd_to_grid(xyz, resolution, z_min, z_max, margin=1.0):
    mask = (xyz[:, 2] > z_min) & (xyz[:, 2] < z_max)
    obs = xyz[mask]
    if len(obs) == 0:
        raise ValueError(f"No points in z∈({z_min}, {z_max}). "
                         f"PCD z range: [{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}]")
    xs, ys = obs[:, 0], obs[:, 1]
    ox = xs.min() - margin
    oy = ys.min() - margin
    w = int(math.ceil((xs.max() - ox - margin + 2 * margin) / resolution))
    h = int(math.ceil((ys.max() - oy - margin + 2 * margin) / resolution))
    grid = np.zeros((h, w), dtype=np.uint8)
    ix = np.clip(((xs - ox) / resolution).astype(np.int32), 0, w - 1)
    iy = np.clip(((ys - oy) / resolution).astype(np.int32), 0, h - 1)
    grid[iy, ix] = 1
    return grid, ox, oy


def save(grid, ox, oy, resolution, stem):
    pgm = Path(stem).with_suffix('.pgm')
    yaml = Path(stem).with_suffix('.yaml')
    h, w = grid.shape
    # nav2: 0=occupied(black), 254=free(white)
    img = np.where(grid > 0, 0, 254).astype(np.uint8)
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())
    yaml.write_text(
        f'image: {pgm.name}\n'
        f'resolution: {resolution}\n'
        f'origin: [{ox:.6f}, {oy:.6f}, 0.0]\n'
        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'
    )
    print(f'Saved {pgm}  ({w}×{h} cells, {w*resolution:.1f}×{h*resolution:.1f} m)')
    print(f'Saved {yaml}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input',  required=True)
    ap.add_argument('--output', default='grid')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--z-min', type=float, default=0.1,
                    help='min obstacle height in map frame (m)')
    ap.add_argument('--z-max', type=float, default=2.0,
                    help='max obstacle height in map frame (m)')
    args = ap.parse_args()

    print(f'Reading {args.input} ...')
    xyz = read_pcd_xyz(args.input)
    print(f'  {len(xyz)} points  z∈[{xyz[:,2].min():.2f}, {xyz[:,2].max():.2f}]')

    grid, ox, oy = pcd_to_grid(xyz, args.resolution, args.z_min, args.z_max)
    save(grid, ox, oy, args.resolution, args.output)


if __name__ == '__main__':
    sys.exit(main())
