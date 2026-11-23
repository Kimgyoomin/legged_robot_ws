#!/usr/bin/env python3
"""Repeatability and completed-callback timing; no ROS traffic or robot motion.

An XYZ capture is replayed at a SYNTHETIC fixed pose, not its original trajectory.
This isolates the estimator and is not a world-accuracy or localization test.
"""
import argparse
import hashlib
import importlib
import importlib.util
import json
from pathlib import Path
import platform
import sys
import time

import cupy as cp
import numpy as np

from rubi_height_mapping.core import ElevationMap
from rubi_height_mapping.parameter import Parameter

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'src/legged_bringup').is_dir())
WEIGHTS = str(ROOT / 'src/rubi_height_mapping/config/traversability_weights.npz')
ROTATION = np.eye(3, dtype=np.float32)
SENSOR = np.array([0, 0, .8], dtype=np.float32)


def load_baseline(package):
    folder = package / 'rubi_height_mapping'
    name = '_snapshot_validation_baseline'
    spec = importlib.util.spec_from_file_location(
        name, folder / '__init__.py', submodule_search_locations=[str(folder)])
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return (importlib.import_module(name + '.core').ElevationMap,
            importlib.import_module(name + '.parameter').Parameter, 'all')


def make_map(variant):
    cls, params, mode = variant
    p = params(weights_path=WEIGHTS, ray_mode=mode)
    p.update()
    return cls(p)


def feed(m, points):
    m.input_pointcloud(points, ['x', 'y', 'z'], ROTATION, SENSOR, 0, 0,
                       points_are_finite=True)


def compare(a, b):
    both = (a[2] > .5) & (b[2] > .5)
    return {
        'valid_mask_equal': bool(np.array_equal(a[2] > .5, b[2] > .5)),
        'max_common_height_difference_m': float(np.max(abs(a[0][both] - b[0][both]))) if both.any() else None,
        'max_all_layer_difference': float(np.max(abs(a - b))),
        'all_layers_bit_equal': bool(np.array_equal(a, b)),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path, required=True, help='Backed-up package directory')
    parser.add_argument('--cloud', type=Path, required=True, help='NPZ containing xyz, used at synthetic fixed pose')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=4)
    parser.add_argument('--scans', type=int, default=215)
    args = parser.parse_args()
    if args.repeats < 2 or args.scans < 30:
        parser.error('Use >=2 repeats and >=30 scans')
    with np.load(args.cloud, allow_pickle=False) as f:
        points = np.asarray(f['xyz'], dtype=np.float32)
    points = points[np.isfinite(points).all(axis=1)]
    range2 = np.einsum('ij,ij->i', points, points)
    points = points[(range2 >= .6**2) & (range2 <= 3**2)]
    if not len(points):
        parser.error('No finite XYZ inside the 0.6–3 m ROI')
    variants = {'before_all': load_baseline(args.baseline),
                'snapshot_all': (ElevationMap, Parameter, 'all'),
                'snapshot_voxel': (ElevationMap, Parameter, 'voxel')}
    repeatability = {}
    for name, variant in variants.items():
        states = []
        for _ in range(args.repeats):
            m = make_map(variant)
            for _ in range(args.scans):
                feed(m, points)
                cp.cuda.Stream.null.synchronize()
            states.append(m.elevation_map.get())
        repeatability[name] = [compare(states[0], a) for a in states[1:]]
    # All-ray point ordering must not influence the test beyond float tolerance.
    m = make_map(variants['snapshot_all'])
    n = make_map(variants['snapshot_all'])
    reversed_points = points[::-1].copy()
    for _ in range(args.scans):
        feed(m, points)
        feed(n, reversed_points)
    ordering = compare(m.elevation_map.get(), n.elevation_map.get())
    # Voxel representatives deliberately depend on original point index, so
    # reversing that mode is NOT an identical-ray-input equivalence test.

    axis = np.arange(-2, 2, .025, dtype=np.float32) + .0125
    x, y = np.meshgrid(axis, axis)
    z = np.where(x < -.5, 0, np.where(x < .5, .05, .15))
    dense = np.stack([x.ravel(), y.ravel(), z.ravel() - .8], axis=1).astype(np.float32)
    cases = {'recorded_roi': points, 'dense_102400': np.tile(dense, (4, 1))}
    timings = {}
    for case, cloud in cases.items():
        maps = {name: make_map(v) for name, v in variants.items()}
        for _ in range(15):
            for m in maps.values():
                feed(m, cloud)
        samples = {name: [] for name in maps}
        cpu = {name: [] for name in maps}
        for i in range(200):
            names = list(maps)
            if i % 2:
                names.reverse()
            for name in names:
                cp.cuda.Stream.null.synchronize()
                start = time.perf_counter()
                start_cpu = time.process_time()
                feed(maps[name], cloud)
                cp.cuda.Stream.null.synchronize()
                samples[name].append((time.perf_counter() - start) * 1000)
                cpu[name].append((time.process_time() - start_cpu) * 1000)
        timings[case] = {'points': len(cloud), 'milliseconds': {
            name: {'median': float(np.median(values)), 'p95': float(np.percentile(values, 95)),
                   'process_cpu_median': float(np.median(cpu[name]))}
            for name, values in samples.items()}}
    prop = cp.cuda.runtime.getDeviceProperties(0)
    m = make_map(variants['snapshot_all'])
    # Added residency relative to old FP32 new_map + two FP32 scalar buffers.
    extra_bytes = (m.map_snapshot.nbytes + m.visibility_map.nbytes
                   + m.new_map.nbytes // 2 + 8)
    result = {
        'scope': __doc__, 'gpu': prop['name'].decode(), 'cupy': cp.__version__,
        'platform': platform.platform(), 'map_m': 8, 'resolution_m': .05,
        'capture': {'path': str(args.cloud), 'sha256': hashlib.sha256(args.cloud.read_bytes()).hexdigest(),
                    'roi_points': len(points), 'pose': 'synthetic R=I, t=(0,0,0.8)'},
        'baseline': str(args.baseline), 'repeat_runs': args.repeats, 'scans_per_run': args.scans,
        'repeatability': repeatability, 'all_ray_reversed_input': ordering,
        'timing_scope': 'Completed core call including H2D, excludes decode, pose wait, moving grid, ROS/DDS, publication, wrapper, RViz',
        'warmups': 15, 'timed_iterations': 200, 'timing': timings,
        'additional_persistent_buffer_bytes': extra_bytes,
        'acceptance': {
            'snapshot_repeatability': all(
                d['valid_mask_equal'] and d['max_common_height_difference_m'] is not None
                and d['max_common_height_difference_m'] <= 2e-6
                for name in ('snapshot_all', 'snapshot_voxel') for d in repeatability[name]),
            'all_ray_ordering': ordering['valid_mask_equal']
                and ordering['max_common_height_difference_m'] is not None
                and ordering['max_common_height_difference_m'] <= 2e-6,
        },
        'limits': 'No Jetson benchmark; no physical-world/GT error claim; finite test coverage, not universal bitwise determinism. Voxel remains experimental.',
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2))
    if not all(result['acceptance'].values()):
        raise SystemExit('Numerical validation gate failed')


if __name__ == '__main__':
    main()
