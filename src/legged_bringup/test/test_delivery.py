from pathlib import Path
import copy
import hashlib
import json
import yaml
import pytest
from legged_bringup.site import load_site

ROOT = Path(__file__).resolve().parents[3]


def test_modes_use_the_same_height_core_and_guarded_commands():
    config = ROOT/'src/legged_bringup/config'
    planner = yaml.safe_load((config/'planner.yaml').read_text())
    assert planner['replanning.enabled']
    assert planner['replanning.stop_before_hard_replan']
    assert planner['input_heightmap_topic'] == '/legged/height'
    pursuit = yaml.safe_load((config/'pursuit.yaml').read_text())['simple_pure_pursuit_controller']['ros__parameters']
    assert pursuit['cmd_vel_topic'] == '/legged/cmd_raw'
    collision = yaml.safe_load((config/'collision.yaml').read_text())['collision_monitor']['ros__parameters']
    assert collision['cmd_vel_in_topic'] == '/legged/cmd_raw'
    assert collision['cmd_vel_out_topic'] == '/legged/cmd_safe'
    costmap = yaml.safe_load((config/'costmap.yaml').read_text())
    for name in ('local_costmap','global_costmap'):
        p = costmap[name][name]['ros__parameters']
        assert 'obstacle_layer' in p['plugins']
        assert p['obstacle_layer']['livox_cloud']['data_type']=='PointCloud2'


def test_map_hash_and_alignment_are_checked(tmp_path):
    import shutil
    shutil.copytree(ROOT/'archive/20261109/maps/demo',tmp_path/'site')
    f=tmp_path/'site/site.yaml'
    site, assets=load_site(f)
    assert site['frame']=='map'
    Path(assets['height']).write_text('modified')
    with pytest.raises(ValueError,match='hash mismatch'): load_site(f)


@pytest.mark.parametrize('config_path', [
    'src/legged_bringup/config/sensor.json',
    'src/livox_ros_driver2/config/MID360s_config.json',
])
def test_sensor_rotation_has_no_translation(config_path):
    # Both the bringup and standalone RViz launch must publish upright axes.
    p=json.loads((ROOT/config_path).read_text())
    for lidar in p['lidar_configs']:
        extrinsic=lidar['extrinsic_parameter']
        assert extrinsic['roll']==180
        assert all(extrinsic[k]==0 for k in ['pitch','yaw','x','y','z'])


def test_frozen_map_keeps_acquisition_epoch():
    from legged_bringup.map_io import read_snapshot
    snap=read_snapshot(ROOT/'archive/20261109/maps/demo/height.rghm')
    assert snap.stamp_ns==1_000_000_000
    assert snap.frame=='map'


def test_alignment_applies_transform_before_regridding():
    import importlib.util
    import numpy as np
    from legged_bringup.map_io import read_snapshot
    spec=importlib.util.spec_from_file_location('align_height', ROOT/'tools/align_height.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    snap=read_snapshot(ROOT/'archive/20261109/maps/demo/height.rghm')
    transform=np.array([[0,-1,0,8],[1,0,0,2],[0,0,1,.1],[0,0,0,1]],float)
    origin,size,records,dropped=module.regrid(snap,transform)
    np.testing.assert_allclose(origin,[2.,2.])
    assert tuple(size)==(120,120) and len(records)==14400 and dropped==0
    assert all(abs(r[1]-.1)<1e-10 and r[3]==1_000_000_000 for r in records)
    transform[0,0]=2
    with pytest.raises(ValueError,match='rigid'):module.regrid(snap,transform)
