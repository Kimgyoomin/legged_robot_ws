"""Shared local-control wiring; global planner configurations remain separate."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo, EmitEvent
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from navigation_dds.profiles import dds_argument, dds_group


def configuration(share, mode, params_file, common_file, sim, velocity, cloud_topic):
    cfg = yaml.safe_load(Path(common_file).read_text())
    specific = yaml.safe_load(Path(params_file).read_text())
    cfg.update(specific)
    pid = 'HeightOnly' if mode == 'only' else 'Height'
    server = cfg['planner_server']['ros__parameters']
    expected = 'legged_navigation/' + ('HeightOnlyPlanner' if mode == 'only' else 'HeightPlanner')
    if server['planner_plugins'] != [pid] or server[pid]['plugin'] != expected:
        raise ValueError(f'{mode} launch requires {expected}; use the matching launch/config')
    if mode == 'only':
        global_params = cfg['global_costmap']['global_costmap']['ros__parameters']
        if (global_params['plugins'] != ['height_context'] or
                global_params['height_context']['plugin'] != 'legged_navigation/HeightContextLayer'):
            raise ValueError('Height-only global costmap permits only the no-input HeightContextLayer')
    cfg['costmap_sector_filter']['ros__parameters']['input_topic'] = cloud_topic
    controller = cfg['controller_server']['ros__parameters']
    if velocity:
        controller['odom_topic'] = '/fastlio/odometry_base'
        controller['FollowPath'].update(trajectory_generator_name='dwb_plugins::LimitedAccelGenerator',
                                       sim_period=1./controller['controller_frequency'])
    bt = cfg['bt_navigator']['ros__parameters']
    bt['odom_topic'] = controller['odom_topic']
    bt['default_nav_to_pose_bt_xml'] = str(share / f'config/navigate_{mode}.xml')
    bt['default_nav_through_poses_bt_xml'] = str(share / f'config/navigate_many_{mode}.xml')
    def set_clock(obj):
        if isinstance(obj, dict):
            if 'ros__parameters' in obj: obj['ros__parameters']['use_sim_time'] = sim
            for value in obj.values(): set_clock(value)
    set_clock(cfg)
    return cfg


def _start(context, mode):
    get = lambda key: LaunchConfiguration(key).perform(context)
    def boolean(key):
        value = get(key).lower()
        if value not in ('true', 'false'): raise ValueError(key + ' must be true or false')
        return value == 'true'
    share = Path(get_package_share_directory('legged_navigation'))
    height = Path(get('height_map')).expanduser()
    if not height.is_file(): raise ValueError(f'height_map must be an existing map.rghm: {height}')
    occupancy = Path(get('map_yaml')).expanduser() if mode == 'hybrid' else None
    if occupancy is not None and not occupancy.is_file():
        raise ValueError(f'Hybrid navigation requires an existing map_yaml: {occupancy}')
    sim, velocity = boolean('use_sim_time'), boolean('use_fastlio_velocity')
    cfg = configuration(share, mode, get('params_file'), get('common_params_file'), sim, velocity, get('cloud_topic'))
    pid = 'HeightOnly' if mode == 'only' else 'Height'
    resolution = cfg['planner_server']['ros__parameters'][pid]['map_resolution_m']
    with tempfile.NamedTemporaryFile('w', prefix=f'rubi_height_{mode}_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
        runtime = f.name
    frozen = Node(package='rubi_global_heightmap_wrapper', executable='frozen_heightmap_node',
                  name='frozen_heightmap', output='screen', parameters=[{'use_sim_time':sim,
                    'file':str(height.resolve()), 'frame':'map', 'expected_resolution':resolution}])
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=lambda *_: Path(runtime).unlink(missing_ok=True))),
        RegisterEventHandler(OnProcessExit(target_action=frozen,
            on_exit=[EmitEvent(event=Shutdown(reason='Frozen height publisher exited'))])),
        LogInfo(msg=f'RUBI {pid}: global={mode}, local=LiDAR+DWB, cmd_vel={get("cmd_vel_topic")}; wait for terrain READY.'),
        frozen,
        Node(package='legged_navigation', executable='costmap_sector_filter',
             name='costmap_sector_filter', output='screen', parameters=[runtime])]
    lifecycle = []
    if occupancy is not None:
        actions.append(Node(package='nav2_map_server', executable='map_server', name='map_server',
            output='screen', parameters=[{'use_sim_time':sim,'yaml_filename':str(occupancy.resolve()),'frame_id':'map'}]))
        lifecycle.append('map_server')
    if velocity:
        actions.append(Node(package='legged_navigation', executable='fastlio_velocity_feedback',
            name='fastlio_velocity_feedback', output='screen', parameters=[{'use_sim_time':sim}]))
    for name, package in [('planner_server','nav2_planner'),('controller_server','nav2_controller'),
                          ('behavior_server','nav2_behaviors'),('bt_navigator','nav2_bt_navigator')]:
        actions.append(Node(package=package, executable=name, name=name, output='screen', parameters=[runtime],
                            remappings=[('/cmd_vel',get('cmd_vel_topic'))]))
        lifecycle.append(name)
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name=f'lifecycle_height_{mode}', output='screen',
        parameters=[{'use_sim_time':sim,'autostart':True,'node_names':lifecycle}]))
    if boolean('rviz'):
        actions.append(Node(package='rviz2', executable='rviz2', name=f'height_{mode}_rviz',
            arguments=['-d',str(share/f'rviz/height_{mode}.rviz')], parameters=[{'use_sim_time':sim}]))
    return [dds_group(actions, f'Height {mode} navigation')]


def navigation_launch(mode):
    if mode not in ('hybrid','only'): raise ValueError('Unknown navigation mode: ' + mode)
    share = Path(get_package_share_directory('legged_navigation'))
    defaults = {'height_map':'','use_sim_time':'false','use_fastlio_velocity':'true','rviz':'true',
        'cloud_topic':'/livox/lidar_pointcloud','cmd_vel_topic':'/cmd_vel',
        'params_file':str(share/f'config/height_{mode}.yaml'),
        'common_params_file':str(share/'config/nav2_common.yaml')}
    if mode == 'hybrid': defaults['map_yaml'] = ''
    return LaunchDescription([dds_argument(),
        *[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
        OpaqueFunction(function=_start,args=[mode])])
