#!/usr/bin/env python3
"""Create a new map manifest for a replacement height snapshot. Never overwrite."""
from pathlib import Path
import argparse, copy, yaml
from legged_bringup.site import load_site
from legged_bringup.map_io import read_snapshot

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for key in ('site','height','output'):p.add_argument('--'+key,required=True,type=Path)
    a=p.parse_args();site,assets=load_site(a.site)
    old=read_snapshot(assets['height']);new=read_snapshot(a.height)
    if (old.frame,old.resolution,old.origin,old.dimensions)!=(new.frame,new.resolution,new.origin,new.dimensions):
        p.error('Replacement frame/resolution/origin/dimensions changed; align and package a new site instead.')
    import os
    out=a.output.expanduser().resolve()
    result=copy.deepcopy(site);result['id']=site['id']+'-height-'+new.digest[:8]
    for key,file in assets.items():
        result['assets'][key]['path']=os.path.relpath(file,out.parent)
    result['assets']['height']={'path':os.path.relpath(new.source,out.parent),'sha256':new.digest}
    result['provenance']=dict(result.get('provenance',{}),replaces_site=str(a.site.resolve()),navigation_hardware_validated=False)
    out.parent.mkdir(parents=True,exist_ok=True)
    with out.open('x') as f:yaml.safe_dump(result,f,sort_keys=False)
    load_site(out)
    print(out)
if __name__=='__main__':main()
