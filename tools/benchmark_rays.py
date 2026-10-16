#!/usr/bin/env python3
"""Compare all/voxel cleanup on identical inputs; no ROS publishers or actuation.

Includes host-to-device copy and completed fusion, excludes pose wait, map moves,
DDS, publication and RViz. A supplied XYZ capture uses a synthetic fixed pose for
cost comparison only, not a reconstruction/accuracy claim about that capture.
"""
import argparse
import hashlib
import importlib.util
import importlib
import json
import platform
from pathlib import Path
import sys
import time
import numpy as np
import cupy as cp
from rubi_height_mapping.core import ElevationMap
from rubi_height_mapping.parameter import Parameter

ROOT = Path(__file__).resolve().parents[1]
WEIGHTS = str(ROOT/'src/rubi_height_mapping/config/traversability_weights.npz')
R = np.eye(3,dtype=np.float32)
T = np.array([0,0,.8],np.float32)


def make_map(mode, cls=ElevationMap, params=Parameter):
    kwargs = {} if mode == 'original' else {'ray_mode':mode}
    p=params(weights_path=WEIGHTS,**kwargs);p.update()
    return cls(p)


def fuse(m, points):
    m.input_pointcloud(points,['x','y','z'],R,T,0,0,points_are_finite=True)


def ghost_test(mode):
    m=make_map(mode);row=m.cell_n//2;col=row+20
    for rr in (row,row+10):
        m.elevation_map[0,rr,col]=.7;m.elevation_map[1,rr,col]=.01
        m.elevation_map[2,rr,col]=1;m.elevation_map[4,rr,col]=2
        m.elevation_map[5,rr,col]=.7
    points=np.tile(np.array([[2.5,.011,-.8]],np.float32),(40,1))
    cleared=None
    for frame in range(1,21):
        m.normal_map[0:2]=0;m.normal_map[2]=1
        fuse(m,points)
        if float(m.elevation_map[2,row,col].get())<.5:
            cleared=frame;break
    return {'clear_after_scans':cleared,
            'occluded_cell_retained':bool(m.elevation_map[2,row+10,col].get()>.5),
            'unknown_cell_still_unknown':bool(m.elevation_map[2,row-10,col].get()<.5),
            'fixture':'Known horizontal normals; old 0.7m ghost, 40 identical rays per scan. Isolated cleanup behavior.'}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--baseline',type=Path)
    parser.add_argument('--cloud',type=Path)
    parser.add_argument('--samples',type=int,default=200)
    args=parser.parse_args()
    if args.samples<10:parser.error('At least 10 samples required')
    variants={'all':(ElevationMap,Parameter),'voxel':(ElevationMap,Parameter)}
    if args.baseline:
        folder=args.baseline/'rubi_height_mapping'
        spec=importlib.util.spec_from_file_location('original_ray_height',folder/'__init__.py',submodule_search_locations=[str(folder)])
        module=importlib.util.module_from_spec(spec);sys.modules[spec.name]=module;spec.loader.exec_module(module)
        variants['original']=(importlib.import_module('original_ray_height.core').ElevationMap,importlib.import_module('original_ray_height.parameter').Parameter)
    cases={}
    for name,spacing,repeats in [('sparse',.05,1),('dense',.025,1),('duplicates',.025,4)]:
        a=np.arange(-2,2,spacing,dtype=np.float32)+spacing/2;x,y=np.meshgrid(a,a)
        z=np.where(x<-.5,0,np.where(x<.5,.05,.15))
        cases[name]=np.tile(np.stack([x.ravel(),y.ravel(),z.ravel()-.8],axis=1).astype(np.float32),(repeats,1))
    capture=None
    if args.cloud:
        with np.load(args.cloud,allow_pickle=False) as f:points=np.asarray(f['xyz'],np.float32)
        points=points[np.isfinite(points).all(axis=1)]
        d=np.einsum('ij,ij->i',points,points)
        cases['recorded_roi']=points[(d>=.6**2)&(d<=3**2)]
        if not len(cases['recorded_roi']):parser.error('No points inside capture ROI')
        capture={'path':str(args.cloud),'sha256':hashlib.sha256(args.cloud.read_bytes()).hexdigest(),
                 'note':'Recorded XYZ distribution, synthetic R=I / t=(0,0,0.8); timing only.'}
    results={}
    for name,points in cases.items():
        maps={mode:make_map(mode,*classes) for mode,classes in variants.items()}
        for _ in range(15):
            for m in maps.values():fuse(m,points)
        cp.cuda.Stream.null.synchronize()
        timings={mode:[] for mode in maps};cpu={mode:[] for mode in maps}
        for i in range(args.samples):
            order=list(maps)
            if i%2:order.reverse()
            for mode in order:
                cp.cuda.Stream.null.synchronize()
                start=time.perf_counter();start_cpu=time.process_time()
                fuse(maps[mode],points)
                cp.cuda.Stream.null.synchronize()
                timings[mode].append(1000*(time.perf_counter()-start))
                cpu[mode].append(1000*(time.process_time()-start_cpu))
        result={'points':len(points),'timing_ms':{},'ray_selection':maps['voxel'].ray_selector.statistics()}
        for mode,values in timings.items():
            result['timing_ms'][mode]={'median':float(np.median(values)),'p95':float(np.percentile(values,95)),
                                      'process_cpu_median':float(np.median(cpu[mode]))}
        a=cp.asnumpy(maps['all'].get_elevation());b=cp.asnumpy(maps['voxel'].get_elevation())
        both=np.isfinite(a)&np.isfinite(b)
        result['height_comparison']={'valid_mask_equal':bool(np.array_equal(np.isfinite(a),np.isfinite(b))),
            'max_difference_m':float(np.max(abs(a[both]-b[both]))) if both.any() else None,
            'height_observations_all':float(cp.sum(maps['all'].new_map[2]).get()),
            'height_observations_voxel':float(cp.sum(maps['voxel'].new_map[2]).get())}
        if name!='recorded_roi':
            assert result['height_comparison']['valid_mask_equal']
            np.testing.assert_allclose(a,b,atol=2e-6,rtol=2e-5,equal_nan=True)
        if 'original' in maps:
            old=cp.asnumpy(maps['original'].elevation_map);new=cp.asnumpy(maps['all'].elevation_map)
            result['all_vs_original_close']=bool(np.allclose(old,new,atol=2e-6,rtol=2e-5,equal_nan=True))
            # Snapshot finalization intentionally changes conflict resolution;
            # baseline equality is a reported comparison, not an acceptance gate.
        results[name]=result
    prop=cp.cuda.runtime.getDeviceProperties(0)
    output={'scope':__doc__,'gpu':prop['name'].decode(),'platform':platform.platform(),'cupy':cp.__version__,
            'map_m':8,'resolution_m':.05,'warmup':15,'samples':args.samples,'capture':capture,
            'cases':results,'cleanup_fixture':{mode:ghost_test(mode) for mode in ('all','voxel')}}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(output,indent=2)+'\n')
    print(json.dumps(output,indent=2))


if __name__=='__main__':main()
