"""Run mapping.yaml: GPU height mapper, global accumulator and optional RViz."""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start(context):
    filename = Path(LaunchConfiguration('params_file').perform(context)).expanduser().resolve()
    settings = yaml.safe_load(filename.read_text())
    required = {'use_sim_time', 'rviz', 'input_scan', 'odom_topic', 'map_frame',
                'base_frame', 'min_range', 'max_range', 'ray_mode', 'ray_voxel_size',
                'save_dir', 'config', 'wrapper_config'}
    optional = {'rviz_config', 'tf_wait_timeout', 'odom_max_gap'}
    if not isinstance(settings, dict):
        raise ValueError(f'{filename}: expected mapping settings as YAML key/value pairs')
    missing, unknown = required - settings.keys(), settings.keys() - required - optional
    if missing or unknown:
        raise ValueError(f'{filename}: missing keys={sorted(missing)}, unknown keys={sorted(unknown)}')
    for key in ('use_sim_time', 'rviz'):
        if not isinstance(settings[key], bool):
            raise ValueError(f'{filename}: {key} must be true or false')
    actions = []
    for key, value in settings.items():
        if value is None or isinstance(value, (dict, list)):
            raise ValueError(f'{filename}: {key} must be a scalar; use "" for an empty topic')
        if key in ('config', 'wrapper_config', 'rviz_config', 'save_dir'):
            path = Path(str(value)).expanduser()
            value = str(path if path.is_absolute() else filename.parent / path)
        value = str(value).lower() if isinstance(value, bool) else str(value)
        # Explicit CLI arguments remain available and win over this YAML.
        actions.append(DeclareLaunchArgument(key, default_value=value))
    mapper = get_package_share_directory('rubi_height_mapping')
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapper + '/launch/mapping.launch.py'),
        launch_arguments={key: LaunchConfiguration(key) for key in settings}.items()))
    return actions


def generate_launch_description():
    own = get_package_share_directory('legged_bringup')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=own + '/config/mapping.yaml',
                              description='Mapping session YAML; relative paths are relative to this file.'),
        OpaqueFunction(function=start),
    ])
