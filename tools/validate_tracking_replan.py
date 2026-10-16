#!/usr/bin/env python3
"""Synthetic fixed-map recovery test; never publishes into the robot domain."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Path as RosPath
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import yaml

ROOT = Path(__file__).resolve().parents[1]
assert os.environ.get('ROS_DOMAIN_ID') == '176', 'Use isolated domain 176 only'
rclpy.init()
n = rclpy.create_node('tracking_replan_test')
out = ROOT / 'validation/navigation_recovery'
log = (out / 'tracking_replan.log').open('w')
params = yaml.safe_load((ROOT / 'src/legged_bringup/config/planner.yaml').read_text())
params.update(input_costmap_topic='/test/costmap', input_heightmap_topic='/test/height',
              comparison_goal_topic='/test/goal')
param_file = out / 'tracking_replan_params.yaml'
param_file.write_text(yaml.safe_dump({'rubi_hybrid_planner_comparison': {'ros__parameters': params}}))
procs = []
paths, commands = [], []
robot_y = 1.0
blocked = False
pose_fresh = True
tf = TransformBroadcaster(n)
qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
cost_pub = n.create_publisher(Costmap, '/test/costmap', qos)
height_pub = n.create_publisher(PointCloud2, '/test/height', 10)
goal_pub = n.create_publisher(PoseStamped, '/test/goal', 10)
path_sub = n.create_subscription(RosPath, '/rubi/planner_comparison/grid/tracking_path',
                                 lambda m: paths.append(m), qos)
cmd_sub = n.create_subscription(Twist, '/test/cmd',
                                lambda m: commands.append((time.monotonic(), m.linear.x, m.angular.z)), 10)
xy = np.arange(120) * .05 + .025
xx, yy = np.meshgrid(xy, xy)
height = create_cloud_xyz32(Header(frame_id='map'), np.column_stack((xx.ravel(), yy.ravel(), np.zeros(xx.size))))
height.header.stamp.sec = 1


def tick():
    stamp = n.get_clock().now().to_msg()
    t = TransformStamped(); t.header.frame_id = 'map'; t.child_frame_id = 'base_link'
    t.header.stamp = stamp
    if not pose_fresh:
        t.header.stamp.sec -= 2
    t.transform.translation.x = 1.; t.transform.translation.y = robot_y
    t.transform.rotation.w = 1.; tf.sendTransform(t)
    c = Costmap(); c.header.frame_id = 'map'; c.header.stamp = stamp
    c.metadata.resolution = .05; c.metadata.size_x = 120; c.metadata.size_y = 120
    c.metadata.origin.orientation.w = 1.
    data = np.zeros((120, 120), dtype=np.uint8)
    if blocked:
        data[(xx-1.)**2+(yy-robot_y)**2 < .3**2] = 254
    c.data = data.ravel().tolist(); cost_pub.publish(c); height_pub.publish(height)


timer = n.create_timer(.1, tick)


def spin(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=.02)


def start(args):
    procs.append(subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT, start_new_session=True))


try:
    start(['ros2', 'run', 'rubi_heightmap_step_wavefront_planner', 'hybrid_planner_comparison_node',
           '--ros-args', '--params-file', str(param_file)])
    start(['ros2', 'run', 'legged_navigation', 'pure_pursuit', '--ros-args',
           '--params-file', str(ROOT/'src/legged_bringup/config/pursuit.yaml'),
           '-p', 'cmd_vel_topic:=/test/cmd'])
    spin(3)
    goal = PoseStamped(); goal.header.frame_id = 'map'
    goal.pose.position.x = 4.; goal.pose.position.y = 1.; goal.pose.orientation.w = 1.
    goal_pub.publish(goal); spin(3)
    assert paths and paths[-1].poses, 'Initial route missing'
    assert any(v > .01 for _, v, _ in commands), 'Initial controller did not run'
    before = len(paths); robot_y = 1.65
    spin(3)
    replacements = paths[before:]
    assert any(not p.poses for p in replacements), 'No stop before recovery'
    good = [p for p in replacements if p.poses]
    assert good and abs(good[-1].poses[0].pose.position.y - robot_y) < .05, 'No route from current pose'
    assert abs(good[-1].poses[-1].pose.position.y - 1.) < .05, 'Recovery changed goal'
    count = len(paths); spin(1.5)
    assert len(paths) == count, 'Recovery loop on unchanged route'
    # A blocked current location must not bypass the ordinary traversability checks.
    robot_y = 2.3; blocked = True; spin(3)
    assert paths and not paths[-1].poses, 'Unsafe recovery route accepted'
    assert max(abs(v)+abs(w) for t,v,w in commands if t > time.monotonic()-.5) < 1.e-8
    blocked = False; spin(3)
    assert paths[-1].poses and abs(paths[-1].poses[0].pose.position.y - robot_y) < .05
    # Let cached TF age before moving the synthetic body again.
    pose_fresh = False; spin(1)
    before = len(paths); robot_y = 3.0; spin(1)
    assert len(paths) == before, 'Recovery used stale pose'
    result = dict(unchanged_map_deviation_replan=True, stop_before_replan=True,
                  goal_preserved=True, no_replan_loop=True, blocked_start_stays_stopped=True,
                  clear_start_retries=True, stale_pose_not_used=True, domain=176)
    (out/'tracking_replan.json').write_text(json.dumps(result, indent=2)+'\n')
    print(result)
finally:
    for p in procs:
        if p.poll() is None: os.killpg(p.pid, signal.SIGINT)
    for p in procs:
        try: p.wait(timeout=5)
        except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGTERM); p.wait(timeout=5)
    n.destroy_node(); rclpy.shutdown(); log.close()
