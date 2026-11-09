"""Construct the launch graph without starting any sensor or robot process."""
import importlib.util
from pathlib import Path

import pytest
import yaml
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription

PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    'standalone_localization', PACKAGE / 'launch/localization_nav_rubi.launch.py')
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


@pytest.fixture
def context(tmp_path, monkeypatch):
    monkeypatch.setattr(MODULE, 'get_package_share_directory', lambda _: str(PACKAGE))
    pcd = tmp_path / 'map.pcd'
    pcd.write_text('synthetic path-validation fixture')
    ctx = LaunchContext()
    ctx.launch_configurations['pcd_map'] = str(pcd)
    for action in MODULE.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    return ctx


def test_standalone_graph_and_pointcloud_profile(context):
    actions = MODULE.start(context)
    assert len(actions) == 1  # scoped DDS group
    children = actions[0].get_sub_entities()
    includes = [a for a in children if isinstance(a, IncludeLaunchDescription)]
    assert len(includes) == 1
    args = dict(includes[0].launch_arguments)
    assert args['config_file'] == 'mid360s_nav.yaml'
    assert args['rviz'] == 'false'  # wrapper is the only RViz owner
    assert context.launch_configurations['publish_2d_map'] == 'false'
    profile = yaml.safe_load((PACKAGE / 'config/mid360s_nav.yaml').read_text())
    bringup = yaml.safe_load((PACKAGE.parent / 'legged_bringup/config/fastlio.yaml').read_text())
    assert profile == bringup


def test_missing_map_fails_before_nodes_start(context):
    context.launch_configurations['pcd_map'] = '/nonexistent/rubi/map.pcd'
    with pytest.raises(ValueError, match='existing PCD'):
        MODULE.start(context)


def test_custommsg_config_is_rejected(context):
    context.launch_configurations['config'] = str(PACKAGE / 'config/mid360.yaml')
    with pytest.raises(ValueError, match='lidar_type: 4'):
        MODULE.start(context)


def test_optional_map_server_requires_map_yaml(context):
    context.launch_configurations['publish_2d_map'] = 'true'
    with pytest.raises(ValueError, match='map_yaml'):
        MODULE.start(context)


def test_default_real_map_and_clock_without_cli_arguments(monkeypatch):
    monkeypatch.setattr(MODULE, 'get_package_share_directory', lambda _: str(PACKAGE))
    ctx = LaunchContext()
    for action in MODULE.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(ctx)
    assert ctx.launch_configurations['use_sim_time'] == 'false'
    assert ctx.launch_configurations['use_rviz'] == 'true'
    assert Path(ctx.launch_configurations['pcd_map']) == (
        PACKAGE.parents[1] / 'maps/rubi_1f_legacy/data/1f_fast_lio_260731_segmented.pcd')


def test_default_map_survives_workspace_relocation(tmp_path):
    root = tmp_path / 'renamed_ws'
    manifest = root / 'maps/rubi_1f_legacy/site.yaml'
    manifest.parent.mkdir(parents=True)
    manifest.write_text('format: 1')
    for share in (root / 'src/FAST_LIO_LOCALIZATION2',
                  root / 'install/fast_lio_localization/share/fast_lio_localization',
                  root / 'install/share/fast_lio_localization'):
        assert MODULE.default_pcd_map(share) == str(
            manifest.parent / 'data/1f_fast_lio_260731_segmented.pcd')


def test_custommsg_recording_has_explicit_supported_profile(context):
    context.launch_configurations['input_format'] = 'custommsg'
    actions = MODULE.start(context)[0].get_sub_entities()
    include = next(a for a in actions if isinstance(a, IncludeLaunchDescription))
    assert dict(include.launch_arguments)['config_file'] == 'rubi_custommsg.yaml'
    cfg = yaml.safe_load((PACKAGE / 'config/rubi_custommsg.yaml').read_text())['/**']['ros__parameters']
    assert cfg['preprocess']['lidar_type'] == 1
    assert cfg['common']['lid_topic'] == '/livox/lidar'
    assert cfg['common']['imu_topic'] == '/livox/imu'


def test_mount_chain_matches_corrected_lidar_extrinsic(context):
    c = context.launch_configurations
    total = [float(c['imu_to_base_'+axis]) + float(c['base_to_lidar_'+axis]) for axis in 'xyz']
    assert total == pytest.approx([-.011, .02329, -.04412])
