"""RUBI frozen height + occupancy integration. Review mode starts only planning."""
from pathlib import Path
import tempfile
import math
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription, SetEnvironmentVariable, LogInfo, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from legged_bringup.site import load_site


def default_site(share):
    for parent in Path(share).resolve().parents:
        candidate = parent / 'maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml'
        if candidate.is_file():
            return str(candidate)
    return str(Path.home() / 'legged_robot_ws/maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml')


def start(context):
    get = lambda k: LaunchConfiguration(k).perform(context)
    share = Path(get_package_share_directory('legged_bringup'))
    mode = get('mode')
    if mode not in ('review', 'nav2', 'replay'):
        raise ValueError('mode must be review, nav2 or replay')
    profile = get('dds_profile')
    if profile not in ('navigation', 'default'):
        raise ValueError('dds_profile must be navigation or default')
    actions = []
    if profile == 'navigation':
        xml = str(share/'config/fastdds_common.xml')
        actions += [SetEnvironmentVariable('FASTRTPS_DEFAULT_PROFILES_FILE', xml),
                    SetEnvironmentVariable('FASTDDS_DEFAULT_PROFILES_FILE', xml),
                    LogInfo(msg='Height navigation DDS: 32 MiB SHM + UDP per participant; TX/RX request 4 MiB.')]
    site, assets = load_site(get('site'))
    actions += [LogInfo(msg=f"Height site: {site['id']} | {assets['height']} | mode={mode}")]
    if mode in ('nav2', 'replay'):
        from legged_bringup.rubi_navigation import nodes
        from navigation_dds.profiles import dds_group
        replay = mode == 'replay'
        sim = replay or get('use_sim_time').lower() == 'true'
        if get('controller') != 'dwb':
            raise ValueError('RUBI baseline uses DWB; alternative controllers use navigate.launch.py')
        children = [LogInfo(msg=f"Height site: {site['id']} | {assets['height']} | mode={mode}")]
        if replay:
            children.append(LogInfo(msg='ROS BAG REPLAY: recorded motion is fixed; new commands go only to /legged/replay/cmd_vel. No LCM.'))
        if replay and get('launch_localization').lower() == 'true':
            fast = Path(get_package_share_directory('fast_lio_localization'))
            children.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(fast/'launch/localization_nav_rubi.launch.py')),
                launch_arguments={'pcd_map':assets['pcd'], 'input_format':'custommsg',
                    'use_sim_time':'true', 'use_rviz':'false', 'dds_profile':profile}.items()))
        children += nodes(share, assets, sim=sim, replay=replay,
            velocity=get('use_fastlio_velocity').lower()=='true', rviz=get('rviz').lower()=='true',
            budget=int(get('planning_time_ms')), height_limit=float(get('height_limit_m')),
            height_cost=get('height_cost'), prefix=get('navigation_prefix'), cloud_topic=get('cloud_topic'))
        return [dds_group(children, 'RUBI height navigation')]
    sim = get('use_sim_time').lower() == 'true'
    planner = yaml.safe_load((share/'config/rubi/planner.yaml').read_text())
    review_budget = int(get('review_planning_time_ms'))
    if not 1 <= review_budget <= 60000:
        raise ValueError('review_planning_time_ms must be between 1 and 60000')
    planner['grid_max_planning_time_ms'] = review_budget
    # Optional comparison overrides apply to planner-only review, never mode=nav2.
    cost_mode = get('review_height_cost')
    if cost_mode not in ('configured', 'cot', 'geometry'):
        raise ValueError('review_height_cost must be configured, cot, or geometry')
    if cost_mode != 'configured':
        planner['evaluation.grid_cot_height_cost_enabled'] = cost_mode == 'cot'
    height_limit = get('review_height_limit_m').strip()
    if height_limit:
        limit = float(height_limit)
        if not math.isfinite(limit) or limit <= planner['evaluation.height_noise_floor_m']:
            raise ValueError('review_height_limit_m must be finite and exceed height_noise_floor_m')
        if (planner.get('evaluation.grid_cot_height_cost_enabled', False) and
                limit > max(planner['evaluation.grid_cot_step_heights_m'])):
            raise ValueError('review_height_limit_m exceeds the CoT calibration table')
        planner['evaluation.max_crossable_height_jump_m'] = limit
        planner['evaluation.local_relief_threshold_m'] = limit
    actions.append(LogInfo(msg=(
        f"Height review only: cost={'cot' if planner.get('evaluation.grid_cot_height_cost_enabled', False) else 'geometry'}; "
        f"jump_limit={planner['evaluation.max_crossable_height_jump_m']:.3f} m; "
        f"relief_limit={planner['evaluation.local_relief_threshold_m']:.3f} m. "
        'Production planner.yaml unchanged.')))
    cfg = yaml.safe_load((share/'config/rubi/nav2.yaml').read_text())
    glob = cfg['global_costmap']['global_costmap']['ros__parameters']
    glob.update(use_sim_time=sim, robot_base_frame='map', update_frequency=1.0,
                publish_frequency=1.0, plugins=['static_layer','inflation_layer'])
    # Explicit start/goal poses come from RViz: no synthetic robot TF or sensor data.
    cfg = {'global_costmap': cfg['global_costmap'], 'planner_server': {'ros__parameters': {
        'use_sim_time':sim, 'expected_planner_frequency':1.0, 'planner_plugins':['Height'],
        'Height':dict(planner,plugin='legged_navigation/HeightPlanner')}}}
    with tempfile.NamedTemporaryFile('w',prefix='rubi_height_review_',suffix='.yaml',delete=False) as f:
        yaml.safe_dump(cfg,f); runtime=f.name
    actions += [RegisterEventHandler(OnShutdown(on_shutdown=lambda *_:Path(runtime).unlink(missing_ok=True))),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen',
             parameters=[{'use_sim_time':sim,'yaml_filename':assets['occupancy'],'frame_id':'map'}]),
        Node(package='legged_bringup', executable='frozen_map', name='frozen_map', output='screen',
             parameters=[{'use_sim_time':sim,'file':assets['height'],'frame':'map'}]),
        Node(package='nav2_planner', executable='planner_server', name='planner_server', output='screen',parameters=[runtime]),
        Node(package='legged_bringup', executable='height_review', name='height_review', output='screen',parameters=[{'use_sim_time':sim}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_height_review', output='screen',
             parameters=[{'use_sim_time':sim,'autostart':True,'node_names':['map_server','planner_server'],'bond_timeout':4.0}])]
    if get('rviz').lower()=='true':
        actions.append(Node(package='rviz2',executable='rviz2',name='height_review_rviz',
            arguments=['-d',str(share/'rviz/height_review.rviz')],parameters=[{'use_sim_time':sim}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('legged_bringup')
    defaults = {'site':default_site(share),'mode':'review','controller':'dwb','use_sim_time':'false',
                'review_planning_time_ms':'800','review_height_cost':'configured',
                'review_height_limit_m':'','planning_time_ms':'800', 'height_limit_m':'0.11',
                'height_cost':'geometry', 'launch_localization':'false', 'use_fastlio_velocity':'true', 'navigation_prefix':'',
                # Separate localization owns the shared RViz. Review/combined replay open their own.
                'rviz':PythonExpression(["'", LaunchConfiguration('mode'), "' == 'review' or '",
                                         LaunchConfiguration('launch_localization'), "'.lower() == 'true'"]),
                'cloud_topic':'/livox/lidar_pointcloud','dds_profile':'navigation'}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=start)])
