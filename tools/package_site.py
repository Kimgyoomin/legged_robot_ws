#!/usr/bin/env python3
"""Copy an already aligned PCD / occupancy / frozen height set with hashes.

This command does not perform registration. --qc is a reviewed YAML record
containing verified: true and the alignment method / measured residuals.
"""
from pathlib import Path
import argparse
import hashlib
import json
import shutil
import yaml


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('pcd','occupancy','height','qc','output'):
        parser.add_argument('--'+name,required=True,type=Path)
    parser.add_argument('--id',required=True)
    args=parser.parse_args()
    from legged_bringup.map_io import read_snapshot
    snap=read_snapshot(args.height)
    if snap.frame!='map': parser.error('Height frame must already be map; relabeling is not registration')
    qc=yaml.safe_load(args.qc.read_text())
    if qc.get('verified') is not True or not qc.get('method'):parser.error('QC record requires verified: true and method')
    if args.output.exists():parser.error('Output already exists; use a new version directory')
    occ=yaml.safe_load(args.occupancy.read_text());image=(args.occupancy.parent/occ['image']).resolve()
    # Read everything before creating the new bundle.
    files={'localization.pcd':args.pcd.read_bytes(),'height.rghm':args.height.read_bytes(),
           'floor'+image.suffix:image.read_bytes(),'qc.yaml':args.qc.read_bytes()}
    occ['image']='floor'+image.suffix
    files['occupancy.yaml']=yaml.safe_dump(occ,sort_keys=False).encode()
    args.output.mkdir(parents=True)
    for name,payload in files.items():(args.output/name).write_bytes(payload)
    assets={k:{'path':name,'sha256':hashlib.sha256(files[name]).hexdigest()}
        for k,name in {'pcd':'localization.pcd','height':'height.rghm','occupancy':'occupancy.yaml','image':occ['image']}.items()}
    site={'format':1,'id':args.id,'frame':'map','height_resolution':snap.resolution,
          'alignment':qc,'assets':assets}
    (args.output/'site.yaml').write_text(yaml.safe_dump(site,sort_keys=False))
    print(args.output.resolve()/'site.yaml')


if __name__=='__main__':main()
