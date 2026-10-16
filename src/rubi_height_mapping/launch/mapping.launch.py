"""RUBI height mapper + global accumulator + optional RViz. No environment overrides."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    own = get_package_share_directory('rubi_height_mapping')
    defaults = {'use_sim_time': 'false', 'rviz': 'true',
                'input_scan': '/livox/lidar_PointCloud2', 'min_range': '0.6', 'max_range': '3.0',
                'save_dir': os.path.expanduser('~/legged_robot_ws/maps/raw'),
                'odom_topic': '', 'map_frame': 'map', 'base_frame': 'base_link',
                'odom_max_gap': '0.2',
                'tf_wait_timeout': '1.5',
                'ray_mode': 'all', 'ray_voxel_size': '0.0',
                'config': os.path.join(own, 'config', 'height_mapping.yaml'),
                'wrapper_config': os.path.join(own, 'config', 'global_heightmap.yaml'),
                'rviz_config': os.path.join(own, 'config', 'rubi_mapping.rviz')}
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([DeclareLaunchArgument(k, default_value=v) for k,v in defaults.items()] + [
        Node(package='rubi_height_mapping', executable='height_mapping_node', name='rubi_height_mapping',
             parameters=[LaunchConfiguration('config'), {'use_sim_time': clock,
                'input_topic': LaunchConfiguration('input_scan'),
                'ray_mode': ParameterValue(LaunchConfiguration('ray_mode'), value_type=str),
                'ray_voxel_size': ParameterValue(LaunchConfiguration('ray_voxel_size'), value_type=float),
                'odom_topic': ParameterValue(LaunchConfiguration('odom_topic'), value_type=str),
                'map_frame': LaunchConfiguration('map_frame'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_max_gap': ParameterValue(LaunchConfiguration('odom_max_gap'), value_type=float),
                'min_range': ParameterValue(LaunchConfiguration('min_range'), value_type=float),
                'max_range': ParameterValue(LaunchConfiguration('max_range'), value_type=float),
                'tf_wait_timeout': ParameterValue(LaunchConfiguration('tf_wait_timeout'), value_type=float)}], output='screen'),
        Node(package='rubi_global_heightmap_wrapper', executable='global_heightmap_node',
             name='rubi_global_heightmap_wrapper',
             parameters=[LaunchConfiguration('wrapper_config'), {'use_sim_time': clock,
                 'map_frame': LaunchConfiguration('map_frame'),
                 'output_directory': LaunchConfiguration('save_dir')}], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz2_rubi_height_mapping',
             arguments=['-d', LaunchConfiguration('rviz_config'), '-f', LaunchConfiguration('map_frame')],
             parameters=[{'use_sim_time': clock}],condition=IfCondition(LaunchConfiguration('rviz')),output='screen'),
    ])
