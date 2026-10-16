#!/usr/bin/env python3
"""Apply a measured rigid transform and regrid height centers. No automatic ICP.

Example: python3 tools/align_height.py --input map.rghm --transform T.yaml --output aligned
T.yaml contains T_map_from_height: [[...], [...], [...], [0,0,0,1]].
Unknown areas remain unfilled. Multi-height collisions become unknown; no edge inpainting.
"""
from pathlib import Path
import argparse
import hashlib
import json
import numpy as np
import yaml


def regrid(snapshot, transform, conflict=.025):
    t=np.asarray(transform,dtype=float)
    if (t.shape!=(4,4) or not np.isfinite(t).all() or
        not np.allclose(t[3],[0,0,0,1],atol=1e-8) or
        not np.allclose(t[:3,:3].T@t[:3,:3],np.eye(3),atol=1e-6) or
        abs(np.linalg.det(t[:3,:3])-1)>1e-6):raise ValueError('Expected a rigid T_map_from_height')
    nx=snapshot.dimensions[0];ind=snapshot.index[snapshot.usable];r=snapshot.resolution
    xyz=np.column_stack([snapshot.origin[0]+(ind%nx+.5)*r,
                         snapshot.origin[1]+(ind//nx+.5)*r,snapshot.height[snapshot.usable]])
    xyz=xyz@t[:3,:3].T+t[:3,3]
    cells=np.floor(xyz[:,:2]/r+1e-8).astype(np.int64)
    lower=cells.min(axis=0);upper=cells.max(axis=0);size=upper-lower+1
    if np.prod(size)>5_000_000:raise ValueError('Aligned grid exceeds five million cells')
    groups={}
    for i,(x,y) in enumerate(cells-lower):groups.setdefault(int(y*size[0]+x),[]).append(i)
    variance=snapshot.variance[snapshot.usable];stamps=snapshot.cell_stamp_ns[snapshot.usable]
    records=[];dropped=0
    for cell,indices in sorted(groups.items()):
        z=xyz[indices,2]
        if np.ptp(z)>conflict:dropped+=1;continue
        records.append((cell,float(np.median(z)),float(variance[indices].max()),int(stamps[indices].min())))
    return lower*r,size,records,dropped


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('input','transform','output'):parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    from legged_bringup.map_io import read_snapshot
    snapshot=read_snapshot(args.input);transform=yaml.safe_load(args.transform.read_text())['T_map_from_height']
    origin,size,records,dropped=regrid(snapshot,transform)
    args.output.mkdir(parents=True,exist_ok=False)
    cutoff=snapshot.text.splitlines()[1].split()[-3]
    with (args.output/'map.rghm').open('w') as f:
        f.write(f'RUBI_GLOBAL_HEIGHTMAP 1\n"map" {snapshot.resolution:.17g} {origin[0]:.17g} {origin[1]:.17g} {size[0]} {size[1]} {cutoff} {snapshot.stamp_ns} {len(records)}\n')
        for cell,z,v,stamp in records:f.write(f'{cell} {z:.17g} {v:.17g} {stamp} 1\n')
    qc={'verified':False,'method':'Apply measured rigid transform, regrid observed centers; QC still required',
        'T_map_from_height':transform,'source_sha256':snapshot.digest,'conflict_cells_removed':dropped,
        'height_resolution':snapshot.resolution,'interpolation':'none',
        'variance':'max of source variances; registration uncertainty is not modeled'}
    (args.output/'alignment.yaml').write_text(yaml.safe_dump(qc,sort_keys=False))
    print(json.dumps({'cells':len(records),'conflict_cells_removed':dropped,'output':str(args.output)}))


if __name__=='__main__':main()
