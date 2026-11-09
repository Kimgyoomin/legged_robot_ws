"""RUBI integration contracts, independent of real sensor hardware."""
from pathlib import Path
import importlib.util
import yaml
import pytest
from legged_bringup.rubi_navigation import configuration

SHARE = Path(__file__).resolve().parents[1]
WS = SHARE.parents[1]


def config(sim=True, velocity=True):
    return configuration(SHARE, sim=sim, velocity=velocity, budget=5000,
                         height_limit=.11, height_cost='geometry')


def test_feedback_selects_current_velocity_and_control_period():
    p = config()['controller_server']['ros__parameters']
    assert p['odom_topic'] == '/fastlio/odometry_base'
    assert p['controller_frequency'] == 5
    assert p['FollowPath']['trajectory_generator_name'] == 'dwb_plugins::LimitedAccelGenerator'
    assert p['FollowPath']['sim_period'] == .2
    assert config(velocity=False)['controller_server']['ros__parameters']['FollowPath']['trajectory_generator_name'] == 'dwb_plugins::StandardTrajectoryGenerator'


def test_all_nodes_share_clock_and_costmaps_share_filtered_cloud():
    def visit(d):
        if isinstance(d, dict):
            if 'ros__parameters' in d: assert d['ros__parameters']['use_sim_time'] is True
            for v in d.values(): visit(v)
    cfg = config(); visit(cfg)
    filt = cfg['costmap_sector_filter']['ros__parameters']
    for key in ('local_costmap', 'global_costmap'):
        p = cfg[key][key]['ros__parameters']
        assert p['obstacle_layer']['livox_cloud']['topic'] == filt['output_topic']
        assert set(map(tuple, yaml.safe_load(p['footprint']))) == {(-.275,-.275),(.275,-.275),(.275,.275),(-.275,.275)}
    assert filt['front_half_angle_deg'] == 60
    assert filt['front_min_range'] == .3 and filt['other_min_range'] == 1


def test_terrain_limits_and_height_planner_preserved():
    p = config()['planner_server']['ros__parameters']
    assert p['planner_plugins'] == ['Height']
    h = p['Height']
    assert h['costmap_clearance_m'] == .3
    assert h['evaluation.max_crossable_height_jump_m'] == .11
    assert h['evaluation.local_relief_threshold_m'] == .11
    assert h['evaluation.grid_cot_height_cost_enabled'] is False
    assert h['static_heightmap'] is True
    assert h['grid_max_planning_time_ms'] == 5000


def test_rosbag_helper_excludes_derived_topics(tmp_path):
    spec = importlib.util.spec_from_file_location('replay_raw', WS/'tools/replay_height_bag.py')
    m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
    entries = [{'topic_metadata': {'name': n, 'type': t}, 'message_count': 10} for n,t in m.TOPICS.items()]
    entries.append({'topic_metadata': {'name':'/cmd_vel','type':'geometry_msgs/msg/Twist'}, 'message_count':100})
    (tmp_path/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'topics_with_message_count':entries}}))
    cmd = m.command(tmp_path,.5)
    assert cmd[cmd.index('--topics')+1:] == list(m.TOPICS)
    assert '/cmd_vel' not in cmd and '/tf' not in cmd and '/Odometry' not in cmd
    assert '--clock' in cmd and '--start-paused' in cmd
    entries[0]['topic_metadata']['type']='sensor_msgs/msg/PointCloud2'
    (tmp_path/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{'topics_with_message_count':entries}}))
    with pytest.raises(ValueError,match='expected'): m.command(tmp_path,.5)


def load_launch(name, monkeypatch):
    spec = importlib.util.spec_from_file_location(name, SHARE/'launch'/name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    def package_share(package):
        return str(SHARE if package == 'legged_bringup' else SHARE.parent/'FAST_LIO_LOCALIZATION2')
    monkeypatch.setattr(module, 'get_package_share_directory', package_share)
    return module


def launch_context(module, **overrides):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


@pytest.mark.parametrize('clock', ['true', 'false'])
def test_localization_opens_shared_view_before_height_is_loaded(monkeypatch, clock):
    from launch.actions import IncludeLaunchDescription
    module = load_launch('rubi_localization.launch.py', monkeypatch)
    requests = []
    def site(filename, require_height=True):
        requests.append(require_height)
        return {}, {'pcd':'/test/reference.pcd'}
    monkeypatch.setattr(module, 'load_site', site)
    context = launch_context(module, use_sim_time=clock)
    include = next(a for a in module.start(context) if isinstance(a, IncludeLaunchDescription))
    args = dict(include.launch_arguments)
    assert args['use_rviz'] == 'true'
    assert args['use_sim_time'] == clock
    assert Path(args['rviz_config']) == SHARE/'rviz/rubi_height_navigation.rviz'
    assert requests == [False]  # Height loading belongs to navigation, not initial localization.
    include.launch_description_source.get_launch_description(context)
    assert 'localization_nav_rubi.launch.py' in include.launch_description_source.location


@pytest.mark.parametrize('mode, combined, explicit, expected', [
    ('replay','false',None,False), ('nav2','false',None,False),
    ('replay','true',None,True), ('replay','false','true',True),
    ('nav2','false','false',False),
])
def test_navigation_rviz_ownership_and_replay_clock(monkeypatch, mode, combined, explicit, expected):
    import legged_bringup.rubi_navigation as navigation
    module = load_launch('rubi_height_navigation.launch.py', monkeypatch)
    monkeypatch.setattr(module, 'load_site', lambda _: ({'id':'test'}, {'height':'/test/height','pcd':'/test/map.pcd'}))
    calls = []
    def nodes(*args, **kwargs):
        calls.append(kwargs)
        return []
    monkeypatch.setattr(navigation, 'nodes', nodes)
    overrides = dict(mode=mode, launch_localization=combined, use_sim_time='false')
    if explicit is not None:
        overrides['rviz'] = explicit
    context = launch_context(module, **overrides)
    module.start(context)  # Construct actions only; no ROS processes or goals.
    assert calls[0]['rviz'] is expected
    assert calls[0]['sim'] is (mode == 'replay')
    assert calls[0]['replay'] is (mode == 'replay')
    assert calls[0]['velocity'] is True


def test_planner_only_review_keeps_its_window(monkeypatch):
    module = load_launch('rubi_height_navigation.launch.py', monkeypatch)
    context = launch_context(module)
    assert context.launch_configurations['rviz'].lower() == 'true'


def test_shared_rviz_receives_frozen_height_and_incremental_costmaps():
    cfg = yaml.safe_load((SHARE/'rviz/rubi_height_navigation.rviz').read_text())
    manager = cfg['Visualization Manager']
    def flattened(items):
        for item in items:
            yield item
            yield from flattened(item.get('Displays', []))
    displays = list(flattened(manager['Displays']))
    by_topic = {d.get('Topic', {}).get('Value'):d for d in displays}
    assert manager['Global Options']['Fixed Frame'] == 'map'
    for topic in ('/cloud_pcd','/localization','/Odometry','/legged/height','/map','/plan','/local_plan'):
        assert by_topic[topic]['Enabled']
    height = by_topic['/legged/height']['Topic']
    assert height['Durability Policy'] == 'Transient Local'
    assert height['Reliability Policy'] == 'Reliable' and height['Depth'] == 1
    for scope in ('local', 'global'):
        topic = f'/{scope}_costmap/costmap'
        assert sum(d.get('Topic', {}).get('Value') == topic for d in displays) == 1
        display = by_topic[topic]
        assert display['Topic']['Durability Policy'] == 'Transient Local'
        assert display['Update Topic']['Value'] == topic+'_updates'
    tools = {a['Class']:a for a in manager['Tools']}
    assert tools['rviz_default_plugins/SetInitialPose']['Topic']['Value'] == '/initialpose'
    assert 'nav2_rviz_plugins/GoalTool' in tools
