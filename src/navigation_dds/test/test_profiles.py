"""Check actual launch scoping without starting any node or sending any command."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
from launch import LaunchContext
from launch.actions import ExecuteProcess, OpaqueFunction
from launch.utilities import perform_substitutions, visit_all_entities_and_collect_futures
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.actions import Node

from navigation_dds.profiles import dds_group, profile_environment, PROFILE_ENV_KEYS


ROOT = Path(__file__).resolve().parents[2]
NS = {'f': 'http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles'}


def test_profile_transport_sizes_and_no_endpoint_qos_changes():
    for filename, mib in [('fastdds_common.xml', 32), ('fastdds_large.xml', 128)]:
        tree = ET.parse(ROOT / 'navigation_dds/config' / filename)
        transports = tree.findall('.//f:transport_descriptor', NS)
        assert [t.find('f:type', NS).text for t in transports] == ['SHM', 'UDPv4']
        assert int(transports[0].find('f:segment_size', NS).text) == mib * 1024**2
        assert tree.find('.//f:useBuiltinTransports', NS).text == 'false'
        assert tree.find('f:participant', NS).get('is_default_profile') == 'true'
        assert tree.find('.//f:data_writer', NS) is None
        assert tree.find('.//f:data_reader', NS) is None


def test_incompatible_environment_fails_explicitly():
    with pytest.raises(RuntimeError, match='ROS_LOCALHOST_ONLY'):
        profile_environment('common', {'ROS_LOCALHOST_ONLY': '1'})
    with pytest.raises(RuntimeError, match='Fast DDS'):
        profile_environment('common', {'RMW_IMPLEMENTATION': 'rmw_cyclonedds_cpp'})


def test_nested_large_profile_does_not_leak_to_later_nodes_or_parent():
    context = LaunchContext()
    context.launch_configurations['dds_profile'] = 'navigation'
    context.environment['ROS_LOCALHOST_ONLY'] = '0'
    context.environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    for key in PROFILE_ENV_KEYS:
        context.environment[key] = '/original.xml'
    observed = []

    def capture(ctx):
        observed.append(ctx.environment[PROFILE_ENV_KEYS[0]])

    visit_all_entities_and_collect_futures(dds_group([
        OpaqueFunction(function=capture),
        dds_group([OpaqueFunction(function=capture)], 'PCD', 'large'),
        OpaqueFunction(function=capture),
    ], 'Common'), context)
    assert [Path(p).name for p in observed] == [
        'fastdds_common.xml', 'fastdds_large.xml', 'fastdds_common.xml']
    assert all(context.environment[key] == '/original.xml' for key in PROFILE_ENV_KEYS)


@pytest.mark.parametrize('relative,overrides,expected_count,has_pcd', [
    ('FAST_LIO_ROS2/launch/mapping.launch.py', {}, 2, False),
    ('rubi_height_mapping/launch/mapping.launch.py', {}, 3, False),
    ('legged_navigation/launch/height_only_navigation.launch.py', {}, 8, False),
    ('legged_navigation/launch/height_only_navigation.launch.py', {'rviz':'true'}, 9, False),
    ('legged_navigation/launch/height_hybrid_navigation.launch.py', {}, 9, False),
    ('legged_navigation/launch/height_hybrid_navigation.launch.py', {'rviz':'true'}, 10, False),
    ('FAST_LIO_LOCALIZATION2/launch/localization_nav_rubi.launch.py', {}, 9, True),
    ('legged_bringup/launch/rubi_height_navigation.launch.py', {'mode':'nav2','rviz':'true'}, 11, False),
    ('legged_bringup/launch/rubi_height_navigation.launch.py', {'mode':'replay','launch_localization':'true','rviz':'true'}, 19, True),
])
@pytest.mark.parametrize('mode', ['navigation', 'default'])
def test_actual_launches_choose_correct_profiles_without_running_nodes(
        monkeypatch, tmp_path, relative, overrides, expected_count, has_pcd, mode):
    context = LaunchContext()
    context.launch_configurations['dds_profile'] = mode
    context.launch_configurations.update(overrides)
    pcd = tmp_path/'map.pcd'
    pcd.write_text('fixture: only path validation, no pointcloud node runs')
    context.launch_configurations['pcd_map'] = str(pcd)
    for name, filename in [('height_map','map.rghm'),('map_yaml','map.yaml')]:
        fixture=tmp_path/filename
        fixture.write_text('fixture: only path validation; no node runs')
        context.launch_configurations[name]=str(fixture)
    context.environment['ROS_LOCALHOST_ONLY'] = '0'
    context.environment['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    for key in PROFILE_ENV_KEYS:
        context.environment[key] = '/inherited.xml'
    seen = []

    def capture(node, ctx):
        seen.append((perform_substitutions(
                         ctx, normalize_to_list_of_substitutions(node.node_executable)),
                     {k: ctx.environment.get(k) for k in PROFILE_ENV_KEYS}))

    def forbidden(*args, **kwargs):
        raise AssertionError('Test attempted to run a real process')

    monkeypatch.setattr(ExecuteProcess, 'execute', forbidden)
    monkeypatch.setattr(Node, 'execute', capture)
    spec = importlib.util.spec_from_file_location('launch_under_test', ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if hasattr(module, 'load_site'):
        monkeypatch.setattr(module, 'load_site', lambda _: ({'id':'test-site'}, {
            'pcd':str(pcd), 'height':str(tmp_path/'map.rghm'), 'occupancy':str(tmp_path/'map.yaml')}))
    visit_all_entities_and_collect_futures(module.generate_launch_description(), context)
    assert len(seen) == expected_count
    if relative.startswith('legged_navigation/'):
        assert sum(executable == 'rviz2' for executable, _ in seen) == int(overrides.get('rviz') == 'true')
    assert sum(executable == 'pcd_to_pointcloud' for executable, _ in seen) == int(has_pcd)
    for executable, environment in seen:
        expected = ('inherited.xml' if mode == 'default' else
                    'fastdds_large.xml' if executable == 'pcd_to_pointcloud' else
                    'fastdds_common.xml')
        assert all(Path(value).name == expected for value in environment.values())
    assert all(context.environment[key] == '/inherited.xml' for key in PROFILE_ENV_KEYS)


def test_operation_replay_resolves_paths_and_preserves_explicit_arguments(tmp_path, monkeypatch):
    from navigation_dds.operation import operation_arguments

    monkeypatch.setenv('LEGGED_WS', str(tmp_path))
    context = LaunchContext()
    context.launch_configurations.update(operation='replay', rviz='true')
    keys = ('rviz use_fastlio_velocity cloud_topic lidar_frame filtered_cloud_topic '
            'local_cloud_topic map_frame local_frame base_frame height_map cmd_vel_topic '
            'use_sim_time global_cloud_topic map_yaml').split()
    for action in operation_arguments(dict.fromkeys(keys, ''), 'navigation', 'hybrid'):
        visit_all_entities_and_collect_futures(action, context)
    result = context.launch_configurations
    assert result['rviz'] == 'true'
    assert result['use_sim_time'] == 'true'
    assert result['cmd_vel_topic'] == '/legged/replay/cmd_vel'
    assert result['local_cloud_topic'] == result['global_cloud_topic'] == result['filtered_cloud_topic']
    assert Path(result['height_map']) == tmp_path / 'maps/rubi_1f/map.rghm'
    assert result['launch-prefix'] == 'taskset -c 6,7'


def test_custom_operation_file_resolves_relative_site_and_cli_overrides(tmp_path):
    from navigation_dds.operation import operation_arguments

    file = tmp_path / 'settings.yaml'
    file.write_text("site: './two sites/A'\ncommon:\n  navigation:\n"
                    "    height_map: ${SITE}/map.rghm\n    cpu_affinity: '4,5'\n"
                    "    cmd_vel_topic: /from_file\nlive: {}\nreplay: {}\n")
    context = LaunchContext()
    context.launch_configurations.update(operation='live', operation_file=str(file),
                                         cmd_vel_topic='/explicit', cpu_affinity='')
    for action in operation_arguments({'height_map': '', 'cmd_vel_topic': ''}, 'navigation'):
        visit_all_entities_and_collect_futures(action, context)
    cfg = context.launch_configurations
    assert Path(cfg['height_map']) == tmp_path / 'two sites/A/map.rghm'
    assert cfg['cmd_vel_topic'] == '/explicit'
    assert not cfg.get('launch-prefix')


@pytest.mark.parametrize('relative,section,profile,affinity', [
    ('livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py', 'livox', 'livox', 'taskset -c 0,1'),
    ('pongbot_lcm/launch/joy_lcm.launch.py', 'lcm', 'navigation', ''),
])
def test_live_hardware_launch_wiring_without_starting_nodes(monkeypatch, relative, section, profile, affinity):
    context = LaunchContext()
    context.launch_configurations['operation'] = 'live'
    context.environment.update(ROS_LOCALHOST_ONLY='0', RMW_IMPLEMENTATION='rmw_fastrtps_cpp')
    seen = []

    def capture(node, ctx):
        cfg = ctx.launch_configurations
        assert cfg['dds_profile'] == profile and cfg['use_sim_time'] == 'false'
        assert cfg.get('launch-prefix', '') == affinity
        if section == 'livox':
            assert cfg['config_file'] == 'MID360s_config.json' and cfg['lidar_frame'] == 'livox_frame'
        else:
            assert cfg['cmd_vel_topic'] == '/cmd_vel' and cfg['lcm_channel'] == 'CMD_VEL'
            assert cfg['joy_topic'] == '/joy_wireless' and cfg['lcm_url'].endswith('?ttl=1')
        seen.append(node)

    monkeypatch.setattr(Node, 'execute', capture)
    spec = importlib.util.spec_from_file_location('hardware_launch', ROOT / relative)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    visit_all_entities_and_collect_futures(module.generate_launch_description(), context)
    assert len(seen) == 1
