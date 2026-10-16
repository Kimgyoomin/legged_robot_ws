#!/usr/bin/env python3
"""Replay a captured costmap/height pair into the planner, never the live robot.

Requires costmap.cdr, height.cdr and capture.json from a read-only capture.
Only the planner runs. No controller, watchdog or velocity publisher is started.
"""
import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import time

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path as RosPath
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformBroadcaster
import yaml
from legged_bringup.costmap_guard import CostmapGuard

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--goal', nargs=2, type=float, required=True)
    parser.add_argument('--radii', nargs='+', type=float, default=[0., .45, math.hypot(.5, .45)])
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '179':
        raise RuntimeError('Replay requires isolated ROS_DOMAIN_ID=179')
    cost = deserialize_message((args.capture/'costmap.cdr').read_bytes(), Costmap)
    height = deserialize_message((args.capture/'height.cdr').read_bytes(), PointCloud2)
    captured = json.loads((args.capture/'capture.json').read_text())
    pose = captured['tf']['transform']
    config = yaml.safe_load((ROOT/'src/legged_bringup/config/planner_smooth.yaml').read_text())
    config.update(use_sim_time=False, input_costmap_topic='/replay/costmap',
                  input_heightmap_topic='/replay/height', comparison_goal_topic='/replay/goal')
    config['replanning.enabled'] = False
    results = []
    rclpy.init()
    try:
        for radius in args.radii:
            n = rclpy.create_node('planner_clearance_replay')
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            pub_cost = n.create_publisher(Costmap, '/replay/costmap', qos)
            pub_height = n.create_publisher(PointCloud2, '/replay/height', 10)
            pub_goal = n.create_publisher(PoseStamped, '/replay/goal', 10)
            broadcaster = TransformBroadcaster(n)
            paths = []
            sub = n.create_subscription(RosPath, '/rubi/planner_comparison/grid/tracking_path',
                                        paths.append, qos)

            def tick():
                stamp = n.get_clock().now().to_msg()
                cost.header.stamp = stamp
                height.header.stamp = stamp
                pub_cost.publish(cost)
                pub_height.publish(height)
                tf = TransformStamped()
                tf.header.frame_id = 'map'; tf.child_frame_id = 'base_link'; tf.header.stamp = stamp
                for name in ('x', 'y', 'z'):
                    setattr(tf.transform.translation, name, pose['translation'][name])
                for name in ('x', 'y', 'z', 'w'):
                    setattr(tf.transform.rotation, name, pose['rotation'][name])
                broadcaster.sendTransform(tf)

            timer = n.create_timer(.1, tick)

            def spin(seconds):
                end = time.monotonic()+seconds
                while time.monotonic() < end:
                    rclpy.spin_once(n, timeout_sec=.02)

            name = f'radius_{radius:.3f}'
            params = args.capture/(name+'.yaml')
            config['costmap_clearance_m'] = radius
            params.write_text(yaml.safe_dump({'rubi_hybrid_planner_comparison': {'ros__parameters': config}}))
            log_path = args.capture/(name+'.log')
            with log_path.open('w') as log:
                proc = subprocess.Popen([str(ROOT/'install/rubi_heightmap_step_wavefront_planner/lib/'
                    'rubi_heightmap_step_wavefront_planner/hybrid_planner_comparison_node'),
                    '--ros-args', '--params-file', str(params)], stdout=log, stderr=subprocess.STDOUT)
                try:
                    spin(2.)
                    paths.clear()
                    goal = PoseStamped(); goal.header.frame_id = 'map'
                    goal.header.stamp = n.get_clock().now().to_msg()
                    goal.pose.position.x, goal.pose.position.y = args.goal
                    goal.pose.orientation.w = 1.
                    pub_goal.publish(goal)
                    spin(3.)
                    points = [[p.pose.position.x, p.pose.position.y] for p in paths[-1].poses] if paths else []
                    guard = CostmapGuard(.5, .45, 1.)
                    assert guard.update(cost)
                    collisions = []
                    # Tangent-pose checks are diagnostic only: not the swept
                    # closed-loop trajectory or the body's initial heading.
                    for i, ((x,y), (nx,ny)) in enumerate(zip(points, points[1:])):
                        if guard.check((x,y,math.atan2(ny-y,nx-x)), (0.,0.,0.)) != 'pass':
                            collisions.append(i)
                    results.append(dict(radius_m=radius, path_points=len(points),
                                        path=points, tangent_rectangle_blocked_indices=collisions))
                finally:
                    if proc.poll() is None:
                        proc.send_signal(signal.SIGINT)
                    try:
                        proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        proc.terminate(); proc.wait(timeout=5)
                    n.destroy_node()
            terms = re.findall(r'termination\s+: (\w+)', log_path.read_text())
            results[-1]['termination'] = terms[-1] if terms else 'no_result'
            print({k:v for k,v in results[-1].items() if k!='path'}, flush=True)
        (args.capture/'replay.json').write_text(json.dumps(results, indent=2)+'\n')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
