from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config=Path(get_package_share_directory('rubi_heightmap_postprocess'))/'config/comparison.rviz'
    return LaunchDescription([
        DeclareLaunchArgument('result_dir',description='Directory containing metrics.json and layers.npz'),
        DeclareLaunchArgument('rviz',default_value='true'),
        Node(package='rubi_heightmap_postprocess',executable='preview',output='screen',
             parameters=[{'result_dir':LaunchConfiguration('result_dir')}]),
        Node(package='rviz2',executable='rviz2',name='rubi_postprocess_rviz',
             arguments=['-d',str(config)],condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
