"""Standalone FAST-LIO SLAM with an explicit PCD destination and DDS profile."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from navigation_dds.operation import operation_arguments
from navigation_dds.profiles import dds_group


def generate_launch_description():
    share = Path(get_package_share_directory('fast_lio'))
    maps = Path(get_package_share_directory('fast_lio_localization')) / 'maps'
    defaults = {'use_sim_time':'false','config_path':str(share/'config'),'config_file':'mid360.yaml',
        'lidar_topic':'', 'imu_topic':'', 'registered_topic':'', 'registered_body_topic':'',
        'odom_output_topic':'', 'rviz':'true','rviz_cfg':str(share/'rviz/fastlio.rviz'),
        'pcd_map':str(maps.resolve()/'slam_map.pcd'),'pcd_overwrite':'false'}
    return LaunchDescription([*operation_arguments(defaults, 'slam'),
                              OpaqueFunction(function=start)])


def start(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)

    topics = {key: value(name) for name, key in [('lidar_topic', 'common.lid_topic'),
              ('imu_topic', 'common.imu_topic')] if value(name)}
    outputs = [('/cloud_registered', value('registered_topic')),
               ('/cloud_registered_body', value('registered_body_topic')),
               ('/Odometry', value('odom_output_topic'))]
    clock = ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    return [dds_group([
            Node(package='fast_lio',executable='fastlio_mapping',output='screen',
                 remappings=[(src, dst) for src, dst in outputs if dst and src != dst], parameters=[
                PathJoinSubstitution([LaunchConfiguration('config_path'),LaunchConfiguration('config_file')]),
                {**topics, 'use_sim_time':clock,'map_file_path':LaunchConfiguration('pcd_map'),
                 'pcd_overwrite':ParameterValue(LaunchConfiguration('pcd_overwrite'),value_type=bool)}]),
            Node(package='rviz2',executable='rviz2',arguments=['-d',LaunchConfiguration('rviz_cfg')],
                 parameters=[{'use_sim_time':clock}],condition=IfCondition(LaunchConfiguration('rviz')))
        ], 'FAST-LIO SLAM')]
