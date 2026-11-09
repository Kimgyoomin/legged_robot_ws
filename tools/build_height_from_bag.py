#!/usr/bin/env python3
"""Replay only LiDAR/IMU into a private localhost ROS domain; save a draft height map.

Source tools/height_env.sh first. No driver, Nav2 or LCM process is started.
The output remains camera_init until alignment against the real site is reviewed.
"""
import argparse
import csv
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import time

import yaml


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True, help='New directory; existing outputs are never overwritten')
    parser.add_argument('--rate', type=float, default=1.0)
    parser.add_argument('--seconds', type=float, default=0, help='0 replays the full bag; otherwise a short sensor-time trial')
    parser.add_argument('--domain', type=int, default=171)
    args = parser.parse_args()
    if args.rate <= 0 or args.seconds < 0 or not 0 <= args.domain <= 232:
        parser.error('Invalid rate, duration or ROS domain')
    if args.output.exists():
        parser.error('Output already exists; choose a new directory')
    info = yaml.safe_load((args.bag / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
    topics = {v['topic_metadata']['name']: v['topic_metadata']['type'] for v in info['topics_with_message_count']}
    if topics.get('/livox/lidar') != 'sensor_msgs/msg/PointCloud2' or topics.get('/livox/imu') != 'sensor_msgs/msg/Imu':
        parser.error('This replay requires timestamped Livox PointCloud2 + Imu')
    # Imports and a GPU smoke check happen before starting any subprocess.
    import cupy as cp
    from grid_map_msgs.msg import GridMap
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import Trigger
    assert cp.arange(10).sum().get() == 45
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['RMW_IMPLEMENTATION'] = 'rmw_fastrtps_cpp'
    rclpy.init()
    node = rclpy.create_node('offline_height_supervisor', parameter_overrides=[Parameter('use_sim_time', value=True)])
    args.output.mkdir(parents=True)
    processes, logs, trajectory, commands = [], [], [], []
    stats = {'odom_messages': 0, 'body_cloud_messages': 0, 'local_grid_messages': 0,
             'global_cloud_points': 0, 'last_clock_ns': 0, 'last_odom_ns': 0}
    begin_ns = info['starting_time']['nanoseconds_since_epoch']

    def odom(msg):
        stats['odom_messages'] += 1
        stats['last_odom_ns'] = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        trajectory.append((stats['last_odom_ns'], p.x, p.y, p.z, q.x, q.y, q.z, q.w))

    def count(key):
        def callback(_):
            stats[key] += 1
        return callback

    node.create_subscription(Odometry, '/Odometry', odom, qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/cloud_registered_body', count('body_cloud_messages'), qos_profile_sensor_data)
    node.create_subscription(GridMap, '/rubi/local_elevation/grid_map', count('local_grid_messages'), qos_profile_sensor_data)
    node.create_subscription(PointCloud2, '/rubi/global_elevation/cloud',
                             lambda m: stats.update(global_cloud_points=m.width*m.height), qos_profile_sensor_data)
    node.create_subscription(Clock, '/clock', lambda m: stats.update(
        last_clock_ns=m.clock.sec*10**9+m.clock.nanosec), qos_profile_sensor_data)
    save = node.create_client(Trigger, '/rubi/global_elevation/save_map')

    def start(name, command):
        commands.append({'name': name, 'argv': command})
        log = (args.output / (name + '.log')).open('w')
        logs.append(log)
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append(proc)
        return proc

    def stop(proc):
        if proc.poll() is not None:
            return
        # launch forwards SIGINT itself; signaling its whole group would also
        # interrupt Python child cleanup a second time. ros2 run needs the group.
        if proc.args[:2] == ['ros2', 'launch']:
            proc.send_signal(signal.SIGINT)
        else:
            os.killpg(proc.pid, signal.SIGINT)
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(proc.pid, signal.SIGTERM)
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()

    report = {'bag': str(args.bag.resolve()), 'frame': 'camera_init', 'alignment_verified': False,
              'navigation_ready': False, 'rate': args.rate, 'requested_seconds': args.seconds}
    try:
        config_archive = args.output / 'config'
        config_archive.mkdir()
        for relative in ('src/FAST_LIO_LOCALIZATION2/config/mid360s_nav.yaml',
                         'src/rubi_height_mapping/config/height_mapping.yaml',
                         'src/rubi_height_mapping/config/bag_260731_global.yaml'):
            source = root / relative
            shutil.copy2(source, config_archive / source.name)
        # Do not share this domain with an existing robot graph.
        deadline = time.monotonic() + 2
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        existing = [name for name in node.get_node_names() if name != node.get_name()]
        if existing:
            raise RuntimeError(f'ROS domain is not empty: {existing}')
        lio = start('fastlio', ['ros2', 'run', 'fast_lio_localization', 'fastlio_mapping', '--ros-args',
            '--params-file', str(root / 'src/FAST_LIO_LOCALIZATION2/config/mid360s_nav.yaml'),
            '-p', 'use_sim_time:=true', '-p', 'publish.path_en:=false'])
        mapper = start('height', ['ros2', 'launch', 'rubi_height_mapping', 'mapping.launch.py',
            'use_sim_time:=true', 'rviz:=false', 'input_scan:=/cloud_registered_body',
            'odom_topic:=/Odometry', 'map_frame:=camera_init', 'base_frame:=body',
            'min_range:=0.6', 'max_range:=3.0', 'tf_wait_timeout:=5.0',
            f'save_dir:={args.output.resolve()}',
            f'wrapper_config:={root / "src/rubi_height_mapping/config/bag_260731_global.yaml"}'])
        deadline = time.monotonic() + 120
        while not (save.service_is_ready() and node.count_subscribers('/livox/imu')
                   and node.count_subscribers('/livox/lidar')
                   and node.count_subscribers('/cloud_registered_body') >= 2):
            rclpy.spin_once(node, timeout_sec=0.2)
            if lio.poll() is not None or mapper.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Startup failed or timed out; inspect fastlio.log and height.log')
        player = start('replay', ['ros2', 'bag', 'play', str(args.bag), '--clock', '100',
            '--rate', str(args.rate), '--disable-keyboard-controls',
            '--read-ahead-queue-size', '500', '--topics', '/livox/lidar', '/livox/imu'])
        progress = time.monotonic()
        duration = info['duration']['nanoseconds'] / 1e9
        expected_duration = min(args.seconds, duration) if args.seconds else duration
        replay_deadline = progress + expected_duration / args.rate + 120
        while player.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
            elapsed = max(0, (stats['last_clock_ns'] - begin_ns) / 1e9)
            if time.monotonic() - progress >= 10:
                print(f'bag={elapsed:.1f}s stats={stats}', flush=True)
                progress = time.monotonic()
            if lio.poll() is not None or mapper.poll() is not None:
                raise RuntimeError('A mapping process exited during replay')
            if time.monotonic() > replay_deadline:
                raise RuntimeError('Bag playback timed out; inspect replay.log')
            if args.seconds and elapsed >= args.seconds:
                stop(player)
                break
        if player.returncode not in (0, -signal.SIGINT) and not args.seconds:
            raise RuntimeError(f'Bag replay exited with {player.returncode}')
        # Allow queued final scans to finish; no robot command publishers exist.
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not stats['local_grid_messages'] or not stats['global_cloud_points']:
            raise RuntimeError('No usable height cells were produced; refusing to report success')
        result = save.call_async(Trigger.Request())
        deadline = time.monotonic() + 45
        while not result.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not result.done() or not result.result().success:
            raise RuntimeError(f'Map save failed: {result.result() if result.done() else "timeout"}')
        report['save_result'] = result.result().message
        report['saved_maps'] = [str(p.resolve()) for p in args.output.glob('run_*/map.rghm')]
        report['success'] = True
        print(report['save_result'], flush=True)
    except BaseException as exc:
        report.update(success=False, error=str(exc))
        raise
    finally:
        for proc in reversed(processes):
            stop(proc)
        report['stats'] = stats
        report['commands'] = commands
        report['elapsed_bag_seconds'] = max(0, (stats['last_clock_ns'] - begin_ns) / 1e9)
        (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
        with (args.output / 'trajectory.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(('stamp_ns', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'))
            writer.writerows(trajectory)
        for log in logs:
            log.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
