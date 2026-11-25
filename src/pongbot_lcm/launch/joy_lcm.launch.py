"""Bridge direct ROS joystick and Nav2 inputs to the robot's LCM channel."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from navigation_dds.operation import operation_arguments
from navigation_dds.profiles import dds_group


def _start(context):
    get = lambda key: LaunchConfiguration(key).perform(context)
    overrides = {key: ParameterValue(get(key), value_type=str)
                 for key in ('joy_topic', 'cmd_vel_topic', 'lcm_channel', 'lcm_url') if get(key)}
    overrides['use_sim_time'] = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    node = Node(package='pongbot_lcm', executable='joy_lcm_node', name='joy_to_lcm',
                output='screen', parameters=[get('params_file'), overrides])
    return [dds_group([node], 'LCM ROS bridge')]


def generate_launch_description():
    share = Path(get_package_share_directory('pongbot_lcm'))
    defaults = {'params_file': str(share / 'config/joy_lcm.yaml'), 'use_sim_time': 'false',
                'joy_topic': '', 'cmd_vel_topic': '', 'lcm_channel': '', 'lcm_url': ''}
    return LaunchDescription([*operation_arguments(defaults, 'lcm', modes=('package', 'live')),
                              OpaqueFunction(function=_start)])
