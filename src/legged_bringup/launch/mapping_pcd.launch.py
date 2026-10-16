"""Separate PCD survey session; do not run alongside localization."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    output=Path(LaunchConfiguration('output').perform(context)).expanduser().resolve()
    if output.exists(): raise ValueError('PCD output already exists; choose a new survey filename')
    output.parent.mkdir(parents=True,exist_ok=True)
    share=get_package_share_directory('legged_bringup')
    sim=LaunchConfiguration('use_sim_time').perform(context).lower()=='true'
    return [Node(package='fast_lio',executable='fastlio_mapping',output='screen',
        parameters=[share+'/config/fastlio.yaml',{'use_sim_time':sim,'map_file_path':str(output),
            'publish.map_en':True,'publish.dense_publish_en':True,'pcd_save.pcd_save_en':True}])]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('output'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),OpaqueFunction(function=start)])
