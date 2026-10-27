"""Select one localization owner. Neither branch starts a second map server."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from legged_bringup.site import load_site


def start(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    share = get_package_share_directory('legged_bringup')
    _, assets = load_site(get('site'), require_height=False)
    profile = get('profile')
    if profile not in ('hardware', 'sim'):
        raise ValueError('profile must be hardware or sim')
    def flag(name, automatic):
        value = get(name).lower()
        if value == 'auto': return automatic
        if value not in ('true', 'false'): raise ValueError(name+' must be true, false or auto')
        return value == 'true'
    sim = flag('use_sim_time', profile == 'sim')
    mode = get('method')
    if mode not in ('fastlio','genz'): raise ValueError('method must be fastlio or genz')
    nodes = [LogInfo(msg=f'Localization: profile={profile}, method={mode}, use_sim_time={sim}')]
    frames_file = get('frames') or share+'/config/'+('frames_sim.yaml' if profile == 'sim' else 'frames.yaml')
    frames = yaml.safe_load(Path(frames_file).expanduser().read_text())
    robot_body = frames.get('robot_body_frame', 'base_link')
    def fixed(parent, child, xyz):
        return Node(package='tf2_ros',executable='static_transform_publisher',
            name=parent+'_to_'+child,parameters=[{'use_sim_time':sim}],
            arguments=['--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),
                       '--frame-id',parent,'--child-frame-id',child])
    # The simulator owns BODY -> livox_frame. It does not own the localization
    # body bridge. Keep these switches independent to avoid a disconnected TF
    # tree when an existing robot_state_publisher supplies the sensor mounting.
    if flag('publish_mount', profile == 'hardware'):
        nodes.append(fixed(robot_body,'livox_frame',frames['base_to_lidar']))
    if flag('publish_body_tf', True):
        if mode == 'fastlio':
            nodes.append(fixed('body',robot_body,frames['imu_to_base']))
            if robot_body != 'base_link':
                nodes.append(fixed(robot_body,'base_link',[0.,0.,0.]))
        elif robot_body != 'base_link':
            nodes.append(fixed('base_link',robot_body,[0.,0.,0.]))
    if mode=='fastlio':
        fastlio = get('fastlio') or share+'/config/'+('fastlio_sim.yaml' if profile == 'sim' else 'fastlio.yaml')
        nodes += [Node(package='fast_lio_localization',executable='fastlio_mapping',output='screen',
            parameters=[str(Path(fastlio).expanduser()),{'use_sim_time':sim}]),
          Node(package='fast_lio_localization',executable='global_localization.py',
            name='global_localization',output='screen',parameters=[{'use_sim_time':sim,
            'pcd_map_path':assets['pcd'],'pcd_map_topic':'/laser_map',
            'map_voxel_size':.2,'scan_voxel_size':.08,'freq_localization':1.0,
            'freq_global_map':.25,'localization_threshold':.8,'fov':6.28319,'fov_far':300}]),
          Node(package='fast_lio_localization',executable='transform_fusion.py',
            name='transform_fusion',output='screen',parameters=[{'use_sim_time':sim}])]
    else:
        common={'use_sim_time':sim,'use_sensor_stamp':True,'config_file':share+'/config/genz.yaml',
            'odom_frame':'camera_init','base_frame':'base_link','visualize':False}
        for executable,name,extra in [
          ('odometry_node','genz_odometry',{'publish_odom_tf':True}),
          ('localization_node','genz_localization',{'map_path':assets['pcd'],'map_frame':'map',
           'publish_map_to_odom_tf':True,'publish_map_to_base_tf':False,
           'transform_publish_tolerance':0.0,'require_initial_pose':True})]:
            nodes.append(Node(package='genz_icp',executable=executable,name=name,output='screen',
                parameters=[common,extra],remappings=[('pointcloud_topic',
                    '/livox/lidar_PointCloud2' if profile == 'sim' else '/livox/lidar')]))
    return nodes


def generate_launch_description():
    defaults={'site':'','method':'fastlio','profile':'hardware',
              'use_sim_time':'auto','publish_mount':'auto','publish_body_tf':'true',
              'frames':'','fastlio':''}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+
                             [OpaqueFunction(function=start)])
