from pathlib import Path
import os, subprocess, sys, yaml, numpy as np
import pytest
from legged_bringup.site import load_site
from legged_bringup.map_io import read_snapshot, write_snapshot
WS=Path(__file__).resolve().parents[3]

def test_height_replacement_keeps_old_site_and_resolves_paths(tmp_path):
    src=WS/'archive/20261109/maps/demo/site.yaml';original=src.read_bytes()
    _,assets=load_site(src);snap=read_snapshot(assets['height'])
    changed=tmp_path/'processed.rghm';write_snapshot(snap,snap.grid+.001,changed)
    output=tmp_path/'version2/site.yaml'
    r=subprocess.run([sys.executable,str(WS/'tools/replace_site_height.py'),'--site',str(src),'--height',str(changed),'--output',str(output)],capture_output=True,text=True)
    assert r.returncode==0,r.stderr
    _,actual=load_site(output)
    assert actual['height']==str(changed)
    assert actual['pcd']==assets['pcd']
    assert src.read_bytes()==original
    repeat=subprocess.run([sys.executable,str(WS/'tools/replace_site_height.py'),'--site',str(src),'--height',str(changed),'--output',str(output)],capture_output=True,text=True)
    assert repeat.returncode!=0

def test_replacement_rejects_frame_or_geometry_changes(tmp_path):
    src=WS/'archive/20261109/maps/demo/site.yaml';_,a=load_site(src)
    text=Path(a['height']).read_text().splitlines()
    for label, index, value in [('frame',0,'camera_init'),('origin',2,'1.0')]:
        rows=text[:];h=rows[1].split();h[index]=value;rows[1]=' '.join(h)
        changed=tmp_path/(label+'.rghm');changed.write_text('\n'.join(rows)+'\n')
        output=tmp_path/(label+'.yaml')
        r=subprocess.run([sys.executable,str(WS/'tools/replace_site_height.py'),'--site',str(src),'--height',str(changed),'--output',str(output)],capture_output=True,text=True)
        assert r.returncode!=0
        assert not output.exists()

def test_actual_site_points_to_untouched_saved_raw():
    if not (WS/'maps/raw/run_1794037249460292244/map.rghm').exists():
        pytest.skip('Local real-world map is not distributed with source')
    s,a=load_site(WS/'maps/raw/run_1794037249460292244/site.yaml')
    h=read_snapshot(a['height'])
    assert h.digest=='1ae7382f5ffcb09e6f418738dcfedaa7815a5198973929ab235987251f853cf8'
    assert h.frame=='map' and h.resolution==.05
    assert np.isfinite(h.grid).sum()==258096
