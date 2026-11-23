#!/usr/bin/env python3
"""Validate saved bag-run structure and draw a local-frame preview, not navigation QC."""
import argparse
import csv
import json
from pathlib import Path
import re
import sys

import numpy as np

sys.path.insert(0, str(next(p for p in Path(__file__).resolve().parents if (p / 'src/legged_bringup').is_dir()) / 'src/legged_bringup'))
from legged_bringup.map_io import read_snapshot


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run', type=Path)
    args = parser.parse_args()
    output = args.run / 'inspection'
    if output.exists():
        parser.error('Inspection already exists; refusing to overwrite it')
    report = json.loads((args.run / 'report.json').read_text())
    if not report.get('success') or len(report.get('saved_maps', [])) != 1:
        parser.error('Expected exactly one successfully saved map')
    snapshot = read_snapshot(report['saved_maps'][0])
    with (args.run / 'trajectory.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    stamps = np.array([int(row['stamp_ns']) for row in rows], dtype=np.int64)
    xyz = np.array([[float(row[k]) for k in ('x', 'y', 'z')] for row in rows])
    if len(rows) < 2 or not np.isfinite(xyz).all():
        parser.error('Missing or invalid odometry trajectory')
    dt = np.diff(stamps) / 1e9
    nx, ny = snapshot.dimensions
    ox, oy = snapshot.origin
    res = snapshot.resolution
    xmax, ymax = ox + nx * res, oy + ny * res
    inside = (xyz[:, 0] >= ox) & (xyz[:, 0] < xmax) & (xyz[:, 1] >= oy) & (xyz[:, 1] < ymax)
    height_log = (args.run / 'height.log').read_text()
    # Shutdown errors from launch/ROS signals are not mapping-time faults.
    active_log = height_log.split('user interrupted with ctrl-c')[0]
    diagnostic_lines = [line for line in active_log.splitlines() if 'input_stats ' in line]
    summary = {
        'scope': 'Structural validation and local odometry diagnostics; no ground-truth or map alignment approval',
        'frame': snapshot.frame, 'alignment_verified': False, 'navigation_ready': False,
        'snapshot_sha256': snapshot.digest, 'resolution_m': res,
        'geometry_cells_xy': [nx, ny], 'bounds_xy': [ox, oy, xmax, ymax],
        'stored_cells': len(snapshot.index), 'usable_cells': int(snapshot.usable.sum()),
        'observed_area_m2': float(snapshot.usable.sum() * res**2),
        'height_min_max_m': [float(snapshot.height.min()), float(snapshot.height.max())],
        'height_percentiles_1_50_99_m': np.percentile(snapshot.height, [1, 50, 99]).tolist(),
        'variance_min_max': [float(snapshot.variance.min()), float(snapshot.variance.max())],
        'odom_samples': len(rows), 'nonincreasing_odom_stamps': int((dt <= 0).sum()),
        'odom_stamp_rate_hz': float(1 / dt.mean()),
        'odom_stamp_gap_max_s': float(dt.max()),
        'trajectory_min_xyz': xyz.min(axis=0).tolist(), 'trajectory_max_xyz': xyz.max(axis=0).tolist(),
        'trajectory_outside_map_samples': int((~inside).sum()),
        'trajectory_length_3d_m': float(np.linalg.norm(np.diff(xyz, axis=0), axis=1).sum()),
        'endpoint_distance_3d_m': float(np.linalg.norm(xyz[-1] - xyz[0])),
        'outside_grid_warning_count': active_log.count('local cells outside fixed global bounds'),
        'rejected_patch_warning_count': active_log.count('Rejected local map:'),
        'clock_or_fusion_fault': bool(re.search(r'\[FATAL\]|epoch_faulted=True|fusion_faulted=True', active_log)),
        'last_mapper_diagnostics': diagnostic_lines[-1] if diagnostic_lines else None,
    }
    output.mkdir()
    with (output / 'metrics.json').open('x') as stream:
        json.dump(summary, stream, indent=2)
        stream.write('\n')

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, (ax, profile) = plt.subplots(1, 2, figsize=(14, 8), gridspec_kw={'width_ratios': [2, 1]})
    ix, iy = snapshot.offset
    gy, gx = snapshot.grid.shape
    extent = (ox + ix*res, ox + (ix+gx)*res, oy + iy*res, oy + (iy+gy)*res)
    lo, hi = np.nanpercentile(snapshot.grid, [1, 99])
    cmap = plt.get_cmap('terrain').copy()
    cmap.set_bad('#e0e0e0')
    im = ax.imshow(snapshot.grid, origin='lower', extent=extent, cmap=cmap,
                   interpolation='nearest', vmin=lo, vmax=hi if hi > lo else lo + .01)
    ax.plot(xyz[:, 0], xyz[:, 1], color='black', linewidth=.7, label='Re-estimated odometry')
    ax.scatter(xyz[0, 0], xyz[0, 1], color='lime', edgecolor='black', s=40, label='Start')
    ax.scatter(xyz[-1, 0], xyz[-1, 1], color='red', edgecolor='black', s=40, label='End')
    ax.set(xlabel='camera_init X [m]', ylabel='camera_init Y [m]', title='Height draft (gray = unknown)')
    ax.legend(fontsize=8)
    fig.colorbar(im, ax=ax, shrink=.7, label='Height [m]; color scale clipped to 1–99%')
    profile.plot((stamps-stamps[0])/1e9, xyz[:, 2])
    profile.set(xlabel='Sensor time from first odometry [s]', ylabel='Body Z [m]', title='Vertical trajectory (not ground truth)')
    profile.grid(alpha=.3)
    fig.suptitle('Recorded-bag height mapping — camera_init frame — NOT NAVIGATION APPROVED')
    fig.tight_layout()
    fig.savefig(output / 'preview.png', dpi=150)
    plt.close(fig)
    print(json.dumps(summary, indent=2))
    print(output / 'preview.png')


if __name__ == '__main__':
    main()
