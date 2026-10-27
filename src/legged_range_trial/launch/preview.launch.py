"""Publish diagnostic clouds only; no planner, controller or velocity output."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    # The normal profile targets /obstacle_cloud. Pass its values explicitly
    # so the diagnostic namespace does not silently miss that YAML node key.
    parameters = yaml.safe_load(Path(get('filter')).read_text())['obstacle_cloud']['ros__parameters']
    return [Node(package='legged_navigation', executable='obstacle_cloud',
                 name='obstacle_cloud', namespace='range_trial', output='screen',
                 parameters=[parameters, {'use_sim_time': get('use_sim_time').lower() == 'true'}],
                 remappings=[
                     ('/livox/lidar', get('cloud_topic')),
                     ('/legged/obstacles', '/range_trial/obstacles'),
                     ('/legged/collision_points', '/range_trial/collision_points'),
                     ('/legged/collision_input_state', '/range_trial/input_state')])]


def generate_launch_description():
    share = get_package_share_directory('legged_range_trial')
    return LaunchDescription([
        DeclareLaunchArgument('filter', default_value=share+'/config/range.yaml'),
        DeclareLaunchArgument('cloud_topic', default_value='/livox/lidar_PointCloud2'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=start)
    ])
