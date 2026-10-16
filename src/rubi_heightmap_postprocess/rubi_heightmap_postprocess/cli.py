import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time
import numpy as np
import scipy
import yaml
from . import __version__
from .filters import Settings, process
from .map_io import read_snapshot, write_snapshot, write_pcd
from .qc import compare_metrics
from .report import write_report


def main(argv=None):
    parser=argparse.ArgumentParser(description='Compare frozen RUBI maps without altering the input.')
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path, help='New directory; existing directories are refused')
    parser.add_argument('--config', type=Path)
    parser.add_argument('--qc-regions', type=Path, help='Optional YAML/JSON with a regions list, used only for evaluation')
    args=parser.parse_args(argv)
    if args.output.exists():
        parser.error('Output already exists. Use a new directory to retain earlier comparisons.')
    config=yaml.safe_load(args.config.read_text()) if args.config else {}
    settings=Settings(**(config or {})); settings.validate()
    rois=(yaml.safe_load(args.qc_regions.read_text()) or {}).get('regions',[]) if args.qc_regions else []
    snapshot=read_snapshot(args.input)
    start=time.perf_counter(); result=process(snapshot.grid,snapshot.resolution,settings)
    processing_seconds=time.perf_counter()-start
    methods=compare_metrics(snapshot,result,rois)
    module_dir=Path(__file__).parent
    code_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(module_dir.glob('*.py'))}
    metrics={'status':'experimental_preview_not_navigation_approval','version':__version__,
             'source':str(snapshot.source),'source_sha256':snapshot.digest,
             'frame':snapshot.frame,'resolution_m':snapshot.resolution,'snapshot_stamp_ns':snapshot.stamp_ns,
             'source_origin_xy_m':snapshot.origin,'source_dimensions_xy':snapshot.dimensions,
             'crop_offset_xy_cells':snapshot.offset,'parameters':asdict(settings),
             'processing_seconds_this_host':processing_seconds,
             'environment':{'python':sys.version,'numpy':np.__version__,'scipy':scipy.__version__},
             'implementation_sha256':code_hashes,'methods':methods,'regions':result['regions'],
             'candidate_regional_plane_cells':int((result['method']==1).sum()),
             'candidate_local_plane_cells':int((result['method']==2).sum()),
             'baseline':'Circular finite mean, observed mask restored. Does not run inpainting.',
             'variance_semantics':'Original variance retained as provenance, not recomputed posterior uncertainty.',
             'timestamps':'Original snapshot and cell timestamps preserved, never refreshed by postprocessing.',
             'world_truth_used':False}
    args.output.mkdir(parents=True,exist_ok=False)
    # metrics.json is written last; a failed run does not have a completion manifest.
    (args.output/'qc_regions.json').write_text(json.dumps({'regions':rois},indent=2)+'\n')
    for name in ('baseline','candidate'):
        dest=args.output/name;dest.mkdir()
        write_snapshot(snapshot,result[name],dest/'map.rghm')
        write_pcd(snapshot,result[name],dest/'surface.pcd')
    x,y=snapshot.xy()
    np.savez_compressed(args.output/'layers.npz',raw=snapshot.grid,x=x,y=y,
                        frame=np.array(snapshot.frame),stamp_ns=np.array(snapshot.stamp_ns,dtype=np.int64),
                        resolution=np.array(snapshot.resolution),
                        **{k:v for k,v in result.items() if isinstance(v,np.ndarray)})
    write_report(snapshot,result,metrics,args.output,rois)
    if hashlib.sha256(snapshot.source.read_bytes()).hexdigest()!=snapshot.digest:
        raise RuntimeError('Input changed during processing; comparison is incomplete')
    metrics['output_sha256']={str(p.relative_to(args.output)):hashlib.sha256(p.read_bytes()).hexdigest()
                              for p in args.output.rglob('*') if p.is_file()}
    (args.output/'metrics.json').write_text(json.dumps(metrics,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    print(json.dumps({'output':str(args.output.resolve()),'processing_seconds':processing_seconds,
                      'methods':methods},indent=2))


if __name__=='__main__':
    main()
