"""MID-360S: one upright PointCloud2/IMU contract for all consumers."""
from pathlib import Path
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    filename = LaunchConfiguration('config').perform(context)
    cfg = json.loads(Path(filename).read_text())
    for item in cfg['lidar_configs']:
        p = item['extrinsic_parameter']
        if any(p[k] != 0 for k in ('x','y','z')):
            raise ValueError('Keep driver translation zero: IMU and LiDAR retain distinct origins; use frames.yaml')
    return [Node(package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', output='screen', parameters=[{
        'xfer_format':0, 'multi_topic':0, 'data_src':0, 'publish_freq':10.0,
        'output_data_type':0, 'frame_id':'livox_frame', 'user_config_path':filename,
        'lvx_file_path':'', 'cmdline_input_bd_code':'livox0000000001'}])]


def generate_launch_description():
    share = get_package_share_directory('legged_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('config',default_value=share+'/config/sensor.json'),
        OpaqueFunction(function=start)])
