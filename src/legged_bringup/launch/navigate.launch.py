"""One frozen site, one selected controller, one guarded command output."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from legged_bringup.site import load_site
from legged_bringup.costmap_guard import configure_costmap_footprint

# Hold this descriptor for the launch process lifetime. Both modes share /cmd_vel.
_navigation_lock = None


def claim_navigation_instance():
    import fcntl
    import os
    global _navigation_lock
    lock = open(f'/tmp/legged_navigation_{os.getuid()}_{os.environ.get("ROS_DOMAIN_ID", "0")}.lock', 'a+')
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        lock.close()
        raise RuntimeError('Navigation is already running in this ROS domain. Stop the existing navigate.launch.py before starting another controller.')
    _navigation_lock = lock


def start(context):
    get = lambda k: LaunchConfiguration(k).perform(context)
    share = get_package_share_directory('legged_bringup')
    sim = get('use_sim_time').lower() == 'true'
    mode = get('mode')
    if mode not in ('nav2','pursuit'): raise ValueError('mode must be nav2 or pursuit')
    site, assets = load_site(get('site'))
    if abs(site['height_resolution']-.05)>1e-9:
        raise ValueError('This delivery profile uses 0.05 m height cells')
    claim_navigation_instance()
    planner = yaml.safe_load(Path(get('planner')).read_text())
    costmaps = yaml.safe_load(Path(get('costmap')).read_text())
    # One geometry for Nav2 footprint, near-cloud stop and final map sweep.
    collision = yaml.safe_load(Path(get('collision')).read_text())['collision_monitor']['ros__parameters']
    motion = yaml.safe_load(Path(get('motion')).read_text())
    half_length, half_width = configure_costmap_footprint(
        costmaps, collision['Stop']['points'], collision['base_frame_id'])
    # Preserve the planner YAML's explicit clearance. Forcing the stop
    # rectangle's circumcircle here requires full-turn space everywhere and
    # disconnects otherwise usable corridors. The full rectangle and its
    # predicted motion remain checked by the watchdog on the original map.
    nav = yaml.safe_load(Path(get('nav2')).read_text())
    controller = get('controller')
    if controller not in ('dwb', 'rpp'):
        raise ValueError('controller must be dwb or rpp')
    if mode != 'nav2' and controller != 'dwb':
        raise ValueError('controller:=rpp requires mode:=nav2 (Nav2 local costmap)')
    if mode == 'nav2' and controller == 'rpp':
        # Replace only the plugin configuration: the height planner and Nav2 servers stay shared.
        nav['controller_server']['ros__parameters']['FollowPath'] = yaml.safe_load(
            Path(get('rpp')).read_text())
    if mode == 'pursuit':
        pursuit = yaml.safe_load(Path(get('pursuit')).read_text())[
            'simple_pure_pursuit_controller']['ros__parameters']
        stop_distance = pursuit['max_cross_track_error_m']
        recovery_distance = planner.setdefault('replanning.max_tracking_error_m', stop_distance)
        if not planner.get('replanning.enabled') or not 0.0 < recovery_distance <= stop_distance:
            raise ValueError('Pursuit requires replanning.max_tracking_error_m > 0 and <= max_cross_track_error_m')
    nav.update(costmaps)
    nav['planner_server']['ros__parameters']['Height'] = dict(planner,
        plugin='legged_navigation/HeightPlanner')
    nav['bt_navigator']['ros__parameters'].update(
        default_nav_to_pose_bt_xml=share+'/config/navigate.xml',
        default_nav_through_poses_bt_xml=share+'/config/navigate_many.xml', odom_topic=get('odom_topic'))
    nav['controller_server']['ros__parameters']['odom_topic'] = get('odom_topic')
    # Nested costmap nodes receive their fully qualified ROS parameter names.
    import tempfile
    from launch.actions import RegisterEventHandler
    from launch.event_handlers import OnShutdown
    def clock_params(value):
        if isinstance(value,dict):
            if 'ros__parameters' in value: value['ros__parameters']['use_sim_time']=sim
            for child in value.values(): clock_params(child)
    clock_params(nav)
    with tempfile.NamedTemporaryFile('w',prefix='legged_nav_',suffix='.yaml',delete=False) as out:
        yaml.safe_dump(nav,out,sort_keys=False); runtime=out.name
    nodes=[RegisterEventHandler(OnShutdown(on_shutdown=lambda *_:Path(runtime).unlink(missing_ok=True))),
      Node(package='nav2_map_server',executable='map_server',name='map_server',output='screen',
        parameters=[{'use_sim_time':sim,'yaml_filename':assets['occupancy'],'frame_id':'map'}]),
      Node(package='legged_bringup',executable='frozen_map',name='frozen_map',output='screen',
        parameters=[{'use_sim_time':sim,'file':assets['height'],'frame':'map'}]),
      Node(package='legged_navigation',executable='obstacle_cloud',name='obstacle_cloud',
        parameters=[share+'/config/obstacle.yaml',{'use_sim_time':sim}],
        remappings=[('/livox/lidar',get('cloud_topic'))]),
      Node(package='nav2_collision_monitor',executable='collision_monitor',name='collision_monitor',
        output='screen',parameters=[collision,{'use_sim_time':sim}]),
      Node(package='legged_bringup',executable='command_watchdog',name='command_watchdog',
        parameters=[motion, {'use_sim_time':sim,'odom_topic':get('odom_topic'),
                     'base_frame':collision['base_frame_id'],
                     'footprint_half_length':half_length,'footprint_half_width':half_width}])]
    lifecycle=['map_server']
    if mode=='nav2':
        for name,pkg in [('planner_server','nav2_planner'),('controller_server','nav2_controller'),
                         ('bt_navigator','nav2_bt_navigator')]:
            nodes.append(Node(package=pkg,executable=name,name=name,output='screen',
                parameters=[runtime],remappings=[('cmd_vel','/legged/cmd_raw')]))
            lifecycle.append(name)
    else:
        nodes += [Node(package='rubi_heightmap_step_wavefront_planner',executable='hybrid_global_costmap_node',
            name='global_costmap',namespace='global_costmap',output='screen',parameters=[runtime]),
          Node(package='rubi_heightmap_step_wavefront_planner',executable='hybrid_planner_comparison_node',
            name='rubi_hybrid_planner_comparison',output='screen',parameters=[planner,{'use_sim_time':sim}]),
          Node(package='legged_navigation',executable='pure_pursuit',name='simple_pure_pursuit_controller',
            output='screen',parameters=[get('pursuit'),{'use_sim_time':sim}])]
        nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
            name='lifecycle_costmap',output='screen',parameters=[{'use_sim_time':sim,
            'autostart':True,'node_names':['global_costmap/global_costmap'],'bond_timeout':0.0}]))
    lifecycle.append('collision_monitor')
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='lifecycle_navigation',output='screen',parameters=[{'use_sim_time':sim,
        'autostart':True,'node_names':lifecycle,'bond_timeout':4.0}]))
    if get('rviz').lower()=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',share+'/rviz/nav.rviz'],
            parameters=[{'use_sim_time':sim}]))
    return nodes


def generate_launch_description():
    share=get_package_share_directory('legged_bringup')
    defaults={'site':'','mode':'nav2','controller':'dwb','use_sim_time':'false','rviz':'true',
      'odom_topic':'/Odometry','cloud_topic':'/livox/lidar',
      **{k:share+'/config/'+k+'.yaml' for k in ['planner','costmap','nav2','pursuit','collision','rpp','motion']}}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+
                             [OpaqueFunction(function=start)])
