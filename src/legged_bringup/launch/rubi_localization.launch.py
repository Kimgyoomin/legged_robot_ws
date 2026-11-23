"""Standalone FAST-LIO Localization, using the PCD from the selected height site."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from legged_bringup.site import load_site

def start(context):
    get=lambda k:LaunchConfiguration(k).perform(context)
    _,assets=load_site(get('site'), require_height=False)
    share=Path(get_package_share_directory('fast_lio_localization'))
    return [LogInfo(msg='FAST-LIO Localization first: use 2D Pose Estimate in RViz; start height navigation after alignment.'),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/'launch/localization_nav_rubi.launch.py')),
        launch_arguments={'pcd_map':assets['pcd'],'input_format':get('input_format'),
                          'use_sim_time':get('use_sim_time'),'use_rviz':get('use_rviz'),
                          'dds_profile':get('dds_profile'), 'rviz_config':get('rviz_config')}.items())]

def generate_launch_description():
    share=Path(get_package_share_directory('legged_bringup')).resolve()
    site=next((p/'maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml' for p in share.parents
               if (p/'maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml').is_file()),
              Path.home()/'legged_robot_ws/maps/rubi_1f_height_20261109_wall_gap_v6/site.yaml')
    defaults={'site':str(site),
              'input_format':'custommsg','use_sim_time':'true','use_rviz':'true','dds_profile':'navigation',
              'rviz_config':str(share/'rviz/rubi_height_navigation.rviz')}
    return LaunchDescription([DeclareLaunchArgument(k,default_value=v) for k,v in defaults.items()]+[OpaqueFunction(function=start)])
