#!/usr/bin/env python3
"""Isolated synthetic GPU/serialization benchmark, without ROS publishers.

Source the workspace first. Optional --baseline points to a copied
rubi_height_mapping package root. This is NOT an end-to-end ROS/Jetson benchmark.
"""
import argparse
import importlib
import importlib.util
import json
from pathlib import Path
import platform
import sys
import time

import cupy as cp
import numpy as np
from rclpy.serialization import serialize_message
from rubi_height_mapping.core import ElevationMap
from rubi_height_mapping.parameter import Parameter
from rubi_height_mapping.gridmap_utils import encode_layer_to_multiarray

ROOT = next(p for p in Path(__file__).resolve().parents if (p / 'src/legged_bringup').is_dir())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline', type=Path)
    parser.add_argument('--samples', type=int, default=150)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.samples < 10:
        parser.error('Use at least 10 samples')
    variants = {'optimized': (ElevationMap, Parameter, encode_layer_to_multiarray, True)}
    if args.baseline:
        pkg = args.baseline / 'rubi_height_mapping'
        spec = importlib.util.spec_from_file_location('baseline_height', pkg / '__init__.py', submodule_search_locations=[str(pkg)])
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        variants['baseline'] = (importlib.import_module('baseline_height.core').ElevationMap,
                                importlib.import_module('baseline_height.parameter').Parameter,
                                importlib.import_module('baseline_height.gridmap_utils').encode_layer_to_multiarray, False)
    # Static 0/5/15 cm terrain, points lie inside the deployed 0.6--3 m ROI.
    axis = np.arange(-2.0, 2.0, .025, dtype=np.float32) + .0125
    x, y = np.meshgrid(axis, axis)
    z = np.where(x < -.5, 0, np.where(x < .5, .05, .15)).astype(np.float32)
    points = np.stack([x.ravel(), y.ravel(), z.ravel()-.8], axis=1)
    sensor = np.array([0, 0, .8], np.float32)
    rotation = np.eye(3, dtype=np.float32)
    contexts = {}
    for name, (cls, param_cls, encoder, fast) in variants.items():
        param = param_cls(weights_path=str(ROOT/'src/rubi_height_mapping/config/traversability_weights.npz'))
        param.update()
        contexts[name] = (cls(param), encoder, fast, np.empty((2, param.cell_n-2, param.cell_n-2), np.float32))

    def fuse(ctx):
        m, _, fast, _ = ctx
        kwargs = {'points_are_finite': True} if fast else {}
        m.input_pointcloud(points, ['x','y','z'], rotation, sensor, 0, 0, **kwargs)

    def export(ctx):
        m, encoder, fast, out = ctx
        if fast:
            m.get_raw_layers_ref(out)
        else:
            for index, layer in enumerate(('elevation', 'variance')):
                m.get_map_with_name_ref(layer, out[index])
        for layer in out:
            serialize_message(encoder(layer))

    for _ in range(15):
        for ctx in contexts.values():
            fuse(ctx); export(ctx)
    cp.cuda.Stream.null.synchronize()
    records = {name: {stage: {'wall_ms': [], 'cpu_ms': []} for stage in ('fusion', 'export_and_serialize')} for name in contexts}
    # Alternate order to limit systematic warmup/order bias. CUDA sync includes
    # completion, avoiding the common error of timing only asynchronous launches.
    for iteration in range(args.samples):
        names = list(contexts)
        if iteration % 2:
            names.reverse()
        for name in names:
            for stage, fn in (('fusion', fuse), ('export_and_serialize', export)):
                cp.cuda.Stream.null.synchronize()
                wall = time.perf_counter(); cpu = time.process_time()
                fn(contexts[name])
                cp.cuda.Stream.null.synchronize()
                records[name][stage]['wall_ms'].append((time.perf_counter()-wall)*1000)
                records[name][stage]['cpu_ms'].append((time.process_time()-cpu)*1000)
    summary = {name: {stage: {clock: {'median': float(np.median(values)), 'p95': float(np.percentile(values,95))}
                            for clock, values in clocks.items()} for stage, clocks in stages.items()} for name, stages in records.items()}
    parity = None
    if 'baseline' in contexts:
        a = cp.asnumpy(contexts['baseline'][0].elevation_map)
        b = cp.asnumpy(contexts['optimized'][0].elevation_map)
        parity = {'all_7_layers_close': bool(np.allclose(a,b,atol=2e-6,rtol=2e-5,equal_nan=True)),
                  'max_absolute_difference': float(np.nanmax(abs(a-b)))}
        if not parity['all_7_layers_close']:
            raise AssertionError(parity)
    prop = cp.cuda.runtime.getDeviceProperties(0)
    result = {'scope': 'Synthetic kernel fusion and two-layer export only; excludes pose wait, DDS, wrapper, RViz, point decoding, and localization.',
              'hardware': platform.platform(), 'gpu': prop['name'].decode(), 'cupy': cp.__version__,
              'points_per_scan': len(points), 'map_length_m': 8, 'resolution_m': .05,
              'warmup_scans': 15, 'samples': args.samples, 'summary': summary, 'parity': parity, 'samples_ms': records}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k != 'samples_ms'}, indent=2))


if __name__ == '__main__':
    main()
