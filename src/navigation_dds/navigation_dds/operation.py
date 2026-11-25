"""Load shared site settings before ordinary launch defaults. CLI arguments win."""
from pathlib import Path
import os
import re

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
import yaml

from .profiles import dds_argument


def default_operation_file():
    return Path(get_package_share_directory('navigation_dds')) / 'config' / 'operation.yaml'


def workspace_root(environment):
    share = Path(get_package_share_directory('navigation_dds'))
    return environment.get('LEGGED_WS') or next(
        (str(p) for p in share.parents if (p / 'src').is_dir() and (p / 'maps').is_dir()),
        str(Path.home() / 'legged_robot_ws'))


def site_directory(path, data, workspace):
    value = data.get('site')
    if value is None:
        return None  # Older operation files may use explicit asset paths throughout.
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f'{path}: site must be a directory path')
    return (path.parent / Path(value.replace('${LEGGED_WS}', workspace)).expanduser()).resolve()


def default_site_manifest():
    path = default_operation_file()
    directory = site_directory(path, yaml.safe_load(path.read_text()), workspace_root(os.environ))
    if directory is None:
        raise ValueError(f'{path}: set site to the directory containing site.yaml')
    return str(directory / 'site.yaml')


def affinity_prefix(value):
    if not value:
        return ''
    if not re.fullmatch(r'[0-9]+(?:-[0-9]+)?(?:,[0-9]+(?:-[0-9]+)?)*', str(value)):
        raise ValueError(f'Invalid cpu_affinity: {value!r} (example: "2,3,4,5")')
    return f'taskset -c {value}'


def _apply(context, sections, allowed):
    mode = LaunchConfiguration('operation').perform(context)
    if mode == 'package':
        return []
    path = Path(LaunchConfiguration('operation_file').perform(context)).expanduser()
    data = yaml.safe_load(path.read_text())
    values = {}
    for group in ('common', mode):
        for section in ('all', *sections):
            values.update(data[group].get(section, {}))
    unknown = values.keys() - allowed
    if unknown:
        raise ValueError(f'{path} ({mode}): unknown launch arguments {sorted(unknown)}')
    workspace = workspace_root(context.environment)
    site = site_directory(path, data, workspace)
    for name, value in values.items():
        if not isinstance(value, (bool, str, float, int)):
            raise ValueError(f'{path}: expected a scalar for {name}')
        value = str(value).lower() if isinstance(value, bool) else str(value)
        value = value.replace('${LEGGED_WS}', workspace)
        if '${SITE}' in value:
            if site is None:
                raise ValueError(f'{path}: {name} uses ${{SITE}} but site is missing')
            value = value.replace('${SITE}', str(site))
        context.launch_configurations.setdefault(name, value)
    return [LogInfo(msg=f'Operation {mode} ({", ".join(sections)}): {path}')]


def _affinity(context):
    prefix = affinity_prefix(LaunchConfiguration('cpu_affinity').perform(context))
    if prefix and not context.launch_configurations.get('launch-prefix'):
        context.launch_configurations['launch-prefix'] = prefix
    return []


def operation_arguments(defaults, *sections, dds=None, modes=('package', 'live', 'replay')):
    return [
        DeclareLaunchArgument('operation', default_value='package', choices=modes,
                              description='Select site YAML settings or the original package arguments'),
        DeclareLaunchArgument('operation_file', default_value=str(default_operation_file())),
        OpaqueFunction(function=_apply, args=[sections, set(defaults) | {'dds_profile', 'cpu_affinity'}]),
        DeclareLaunchArgument('cpu_affinity', default_value='', description='Linux CPU list; empty inherits affinity'),
        OpaqueFunction(function=_affinity),
        dds if dds is not None else dds_argument(),
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
    ]
