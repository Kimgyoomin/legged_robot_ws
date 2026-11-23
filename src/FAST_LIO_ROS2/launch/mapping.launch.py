"""Standalone FAST-LIO SLAM with an explicit PCD destination and DDS profile."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from navigation_dds.profiles import dds_argument, dds_group


def generate_launch_description():
    share = Path(get_package_share_directory('fast_lio'))
    maps = Path(get_package_share_directory('fast_lio_localization')) / 'maps'
    defaults = {'use_sim_time':'false','config_path':str(share/'config'),'config_file':'mid360.yaml',
        'rviz':'true','rviz_cfg':str(share/'rviz/fastlio.rviz'),
        'pcd_map':str(maps.resolve()/'slam_map.pcd'),'pcd_overwrite':'false'}
    clock = ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)
    return LaunchDescription([dds_argument(),
        *[DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()],
        dds_group([
            Node(package='fast_lio',executable='fastlio_mapping',output='screen',parameters=[
                PathJoinSubstitution([LaunchConfiguration('config_path'),LaunchConfiguration('config_file')]),
                {'use_sim_time':clock,'map_file_path':LaunchConfiguration('pcd_map'),
                 'pcd_overwrite':ParameterValue(LaunchConfiguration('pcd_overwrite'),value_type=bool)}]),
            Node(package='rviz2',executable='rviz2',arguments=['-d',LaunchConfiguration('rviz_cfg')],
                 parameters=[{'use_sim_time':clock}],condition=IfCondition(LaunchConfiguration('rviz')))
        ], 'FAST-LIO SLAM')])
