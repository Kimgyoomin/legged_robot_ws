#!/usr/bin/env python3
"""Standalone upside-down MID-360S localization, without legged_bringup.

The driver runs separately. Navigation owns /map by default. Mount translations
are legacy RUBI values and must be measured again if the bracket has changed.
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import yaml
from navigation_dds.profiles import dds_argument, dds_group


def default_pcd_map(share):
    relative = Path('maps/rubi_1f_legacy/data/1f_fast_lio_260731_segmented.pcd')
    # Works with both source symlinks and isolated/merged colcon installs.
    # Find the tracked manifest even when the ignored map data is not restored yet.
    for parent in share.resolve().parents:
        if (parent / 'maps/rubi_1f_legacy/site.yaml').is_file():
            return str(parent / relative)
    return str(Path.home() / 'legged_robot_ws' / relative)


def start(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    share = Path(get_package_share_directory('fast_lio_localization'))
    pcd = Path(get('pcd_map')).expanduser()
    if not pcd.is_file():
        raise ValueError(f'pcd_map must be an existing PCD file: {pcd}')
    input_format = get('input_format')
    if input_format not in ('pointcloud2', 'custommsg'):
        raise ValueError('input_format must be pointcloud2 or custommsg')
    config = Path(get('config') or str(share / 'config' / ('rubi_custommsg.yaml' if input_format == 'custommsg' else 'mid360s_nav.yaml'))).expanduser()
    params = yaml.safe_load(config.read_text())['/**']['ros__parameters']
    expected_type = 1 if input_format == 'custommsg' else 4
    if params['preprocess']['lidar_type'] != expected_type:
        raise ValueError(f'input_format={input_format} requires preprocess.lidar_type: {expected_type}')
    if get('publish_2d_map').lower() == 'true' and not Path(get('map_yaml')).is_file():
        raise ValueError('publish_2d_map:=true requires an existing map_yaml')

    sim = LaunchConfiguration('use_sim_time')
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(share / 'launch/localization.launch.py')),
        launch_arguments={
            'use_sim_time': sim, 'map': str(pcd.resolve()), 'dds_profile': get('dds_profile'),
            'config_path': str(config.resolve().parent), 'config_file': config.name,
            'pcd_map_topic': LaunchConfiguration('pcd_map_topic'),
            'map_yaml': LaunchConfiguration('map_yaml'),
            'publish_2d_map': LaunchConfiguration('publish_2d_map'), 'rviz': 'false',
        }.items())

    def mount(name, parent, child, prefix):
        return Node(package='tf2_ros', executable='static_transform_publisher',
                    name=name, output='screen',
                    condition=IfCondition(LaunchConfiguration('publish_mount')),
                    parameters=[{'use_sim_time': ParameterValue(sim, value_type=bool)}],
                    arguments=['--x', LaunchConfiguration(prefix + '_x'),
                               '--y', LaunchConfiguration(prefix + '_y'),
                               '--z', LaunchConfiguration(prefix + '_z'),
                               '--roll', '0', '--pitch', '0', '--yaw', '0',
                               '--frame-id', parent, '--child-frame-id', child])

    return [dds_group([localization,
            mount('body_to_robot_body', 'body', 'BODY', 'imu_to_base'),
            Node(package='tf2_ros', executable='static_transform_publisher',
                 name='BODY_to_base_link', condition=IfCondition(LaunchConfiguration('publish_mount')),
                 parameters=[{'use_sim_time': ParameterValue(sim, value_type=bool)}],
                 arguments=['--frame-id','BODY','--child-frame-id','base_link']),
            mount('base_link_to_livox_frame', 'base_link', 'livox_frame', 'base_to_lidar'),
            Node(package='rviz2', executable='rviz2', name='rviz2_fastlio_localization',
                 output='screen', arguments=['-d', LaunchConfiguration('rviz_config')],
                 parameters=[{'use_sim_time': ParameterValue(sim, value_type=bool)}],
                 condition=IfCondition(LaunchConfiguration('use_rviz')))], 'RUBI localization')]


def generate_launch_description():
    share = Path(get_package_share_directory('fast_lio_localization'))
    defaults = {
        'pcd_map': default_pcd_map(share),
        'input_format': 'pointcloud2', 'config': '',
        'pcd_map_topic': '/cloud_pcd', 'map_yaml': '', 'publish_2d_map': 'false',
        'use_sim_time': 'false', 'use_rviz': 'true',
        'rviz_config': str(share / 'rviz/fastlio_localization.rviz'),
        'publish_mount': 'true',
        'imu_to_base_x': '-0.1105', 'imu_to_base_y': '0.02329', 'imu_to_base_z': '0.09338',
        'base_to_lidar_x': '0.0995', 'base_to_lidar_y': '0.0', 'base_to_lidar_z': '-0.1375',
    }
    return LaunchDescription([dds_argument(),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=start),
    ])
