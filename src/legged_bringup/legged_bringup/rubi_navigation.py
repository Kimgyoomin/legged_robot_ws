"""Build the real RUBI profile; replay and live share all navigation settings."""
from pathlib import Path
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.actions import LogInfo, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch_ros.actions import Node


def configuration(share, *, sim, velocity, budget, height_limit, height_cost, cloud_topic="/livox/lidar_pointcloud"):
    cfg = yaml.safe_load((share / 'config/rubi/nav2.yaml').read_text())
    cfg['costmap_sector_filter']['ros__parameters']['input_topic'] = cloud_topic
    planner = yaml.safe_load((share / 'config/rubi/planner.yaml').read_text())
    if not 1 <= budget <= 60000:
        raise ValueError('planning_time_ms must be 1..60000')
    if not .01 < height_limit <= .30:
        raise ValueError('height_limit_m must exceed .01 and be <= .30')
    if height_cost not in ('geometry', 'cot'):
        raise ValueError('height_cost must be geometry or cot')
    if height_cost == 'cot' and height_limit > max(planner['evaluation.grid_cot_step_heights_m']):
        raise ValueError('height limit exceeds CoT calibration')
    planner.update({'grid_max_planning_time_ms': budget,
                    'evaluation.max_crossable_height_jump_m': height_limit,
                    'evaluation.local_relief_threshold_m': height_limit,
                    'evaluation.grid_cot_height_cost_enabled': height_cost == 'cot'})
    cfg['planner_server']['ros__parameters']['Height'] = dict(planner, plugin='legged_navigation/HeightPlanner')
    ctrl = cfg['controller_server']['ros__parameters']
    if velocity:
        ctrl['odom_topic'] = '/fastlio/odometry_base'
        ctrl['FollowPath'].update(trajectory_generator_name='dwb_plugins::LimitedAccelGenerator',
                                 sim_period=1.0/ctrl['controller_frequency'])
    bt = cfg['bt_navigator']['ros__parameters']
    bt['odom_topic'] = ctrl['odom_topic']
    bt['default_nav_to_pose_bt_xml'] = str(share/'config/rubi/navigate.xml')
    # Through-poses uses the existing sequential height planning tree.
    bt['default_nav_through_poses_bt_xml'] = str(share/'config/navigate_many.xml')
    def clock(d):
        if isinstance(d, dict):
            if 'ros__parameters' in d: d['ros__parameters']['use_sim_time'] = sim
            for v in d.values(): clock(v)
    clock(cfg)
    return cfg


def nodes(share, assets, *, sim, replay, velocity, rviz, budget, height_limit, height_cost, prefix='', cloud_topic='/livox/lidar_pointcloud'):
    cfg = configuration(share, sim=sim, velocity=velocity, budget=budget,
                        height_limit=height_limit, height_cost=height_cost, cloud_topic=cloud_topic)
    with tempfile.NamedTemporaryFile('w', prefix='rubi_height_nav_', suffix='.yaml', delete=False) as f:
        yaml.safe_dump(cfg, f, sort_keys=False); runtime = f.name
    command = '/legged/replay/cmd_vel' if replay else '/cmd_vel'
    actions = [RegisterEventHandler(OnShutdown(on_shutdown=lambda *_: Path(runtime).unlink(missing_ok=True))),
        LogInfo(msg=f'RUBI height navigation: DWB 5 Hz, velocity={velocity}, command={command}; '
                    f'height={height_limit} m, cost={height_cost}, budget={budget} ms. '
                    '1 Hz replanning is a target, not a runtime guarantee.'),
        Node(package='nav2_map_server', executable='map_server', name='map_server', output='screen', prefix=prefix,
             parameters=[{'use_sim_time': sim, 'yaml_filename': assets['occupancy'], 'frame_id':'map'}]),
        Node(package='legged_bringup', executable='frozen_map', name='frozen_map', output='screen', prefix=prefix,
             parameters=[{'use_sim_time': sim, 'file': assets['height'], 'frame':'map'}]),
        Node(package='legged_navigation', executable='costmap_sector_filter', name='costmap_sector_filter',
             output='screen', prefix=prefix, parameters=[runtime])]
    if velocity:
        actions.append(Node(package='legged_navigation', executable='fastlio_velocity_feedback',
                            name='fastlio_velocity_feedback', output='screen', prefix=prefix,
                            parameters=[{'use_sim_time': sim}]))
    lifecycle = ['map_server']
    for name, package in [('planner_server','nav2_planner'), ('controller_server','nav2_controller'),
                          ('smoother_server','nav2_smoother'), ('behavior_server','nav2_behaviors'),
                          ('bt_navigator','nav2_bt_navigator')]:
        actions.append(Node(package=package, executable=name, name=name, output='screen', prefix=prefix,
                            parameters=[runtime], remappings=[('/cmd_vel', command)]))
        lifecycle.append(name)
    actions.append(Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                        name='lifecycle_rubi_height', output='screen', prefix=prefix,
                        parameters=[{'use_sim_time':sim, 'autostart':True, 'node_names':lifecycle}]))
    if rviz:
        actions.append(Node(package='rviz2', executable='rviz2', name='rubi_height_navigation_rviz',
                            arguments=['-d',str(share/'rviz/rubi_height_navigation.rviz')],
                            parameters=[{'use_sim_time':sim}]))
    return actions
