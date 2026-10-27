"""Direct RUBI simulation localization, with package-local configuration and RViz.

The simulator supplies BODY -> livox_frame. This launch supplies the localizer
body bridge. Both cloud and IMU already have upright axes at the input topics.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('fast_lio_localization')
    arg = LaunchConfiguration
    declarations = [
        DeclareLaunchArgument('pcd_map', description='Path to the localization PCD map'),
        DeclareLaunchArgument('pcd_map_topic', default_value='/cloud_pcd'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        DeclareLaunchArgument('publish_2d_map', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(share,'rviz','fastlio_localization.rviz')),
        DeclareLaunchArgument('config_path', default_value=os.path.join(share,'config')),
        DeclareLaunchArgument('config_file', default_value='mid360.yaml'),
    ]
    localizer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share,'launch','localization.launch.py')),
        launch_arguments={
            'use_sim_time':arg('use_sim_time'), 'map':arg('pcd_map'),
            'pcd_map_topic':arg('pcd_map_topic'), 'map_yaml':arg('map_yaml'),
            'publish_2d_map':arg('publish_2d_map'), 'rviz':'false',
            'config_path':arg('config_path'), 'config_file':arg('config_file'),
        }.items())
    def fixed(name, parent, child, xyz):
        return Node(package='tf2_ros', executable='static_transform_publisher',
            name=name, output='screen', parameters=[{'use_sim_time':arg('use_sim_time')}],
            arguments=['--x',str(xyz[0]),'--y',str(xyz[1]),'--z',str(xyz[2]),
                       '--frame-id',parent,'--child-frame-id',child])
    return LaunchDescription(declarations+[
        localizer,
        fixed('body_to_robot_body','body','BODY',[-.112,.02329,.07278]),
        fixed('robot_body_to_base_link','BODY','base_link',[0.,0.,0.]),
        Node(package='rviz2', executable='rviz2', name='rviz2_fastlio_localization',
            output='screen', arguments=['-d',arg('rviz_config')],
            parameters=[{'use_sim_time':arg('use_sim_time')}],
            condition=IfCondition(arg('use_rviz'))),
    ])
