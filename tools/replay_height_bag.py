#!/usr/bin/env python3
"""Replay only raw sensors for RUBI localization + height navigation review."""
import argparse
import math
import os
from pathlib import Path
import shlex
import sys
import yaml

TOPICS = {'/livox/lidar': 'livox_ros_driver2/msg/CustomMsg',
          '/livox/imu': 'sensor_msgs/msg/Imu',
          '/livox/lidar_pointcloud': 'sensor_msgs/msg/PointCloud2'}


def command(bag, rate, paused=True):
    if not math.isfinite(rate) or rate <= 0:
        raise ValueError('--rate must be finite and positive')
    info = yaml.safe_load((bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    found = {i['topic_metadata']['name']: i for i in info['topics_with_message_count']}
    for name, expected in TOPICS.items():
        entry = found.get(name)
        if not entry or entry['message_count'] <= 0:
            raise ValueError(f'{name} missing or empty: this three-topic replay requires the recorded PointCloud2 as well as CustomMsg and IMU')
        if entry['topic_metadata']['type'] != expected:
            raise ValueError(f'{name}: expected {expected}, found {entry["topic_metadata"]["type"]}')
    cmd = ['ros2', 'run', 'navigation_dds', 'with_profile', 'common', 'ros2', 'bag', 'play',
           str(bag), '--clock', '100', '--rate', str(rate), '--read-ahead-queue-size', '3000']
    if paused:
        cmd.append('--start-paused')
    return cmd + ['--topics', *TOPICS]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--rate', type=float, default=.5)
    parser.add_argument('--no-pause', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    args = parser.parse_args()
    try:
        cmd = command(args.bag.expanduser().resolve(), args.rate, not args.no_pause)
        if os.environ.get('ROS_DOMAIN_ID', '0') in ('0', '13'):
            raise ValueError('Use a separate replay domain, e.g. export ROS_DOMAIN_ID=87, in BOTH terminals. Domain 13 is the real robot domain.')
    except (OSError, KeyError, TypeError, ValueError) as e:
        parser.error(str(e))
    print('Only raw sensors are replayed; /clock is generated. Recorded TF/Odometry/cmd_vel are excluded.', flush=True)
    print(shlex.join(cmd), flush=True)
    if not args.dry_run:
        os.execvp(cmd[0], cmd)

if __name__ == '__main__':
    main()
