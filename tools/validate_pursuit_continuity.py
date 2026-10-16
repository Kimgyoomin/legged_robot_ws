#!/usr/bin/env python3
"""Isolated domain 178: soft replans and stop/turn hysteresis, no robot bridge."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav_msgs.msg import Path as RosPath
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID') == '178', 'Use isolated ROS_DOMAIN_ID=178'
root = Path(__file__).resolve().parents[1]
out = root/'validation/stop_turn_20261016'
out.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('pursuit_continuity_inputs')
tf = TransformBroadcaster(node)
pub = node.create_publisher(RosPath, '/test/path',
    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
commands, states = [], []
subs = [node.create_subscription(Twist, '/test/command',
    lambda m: commands.append((time.monotonic(), m.linear.x, m.angular.z)), 10),
    node.create_subscription(String, '/legged/tracking_state',
    lambda m: states.append((time.monotonic(), m.data)), 10)]
yaw = 0.


def tick():
    t = TransformStamped(); t.header.frame_id = 'map'; t.child_frame_id = 'base_link'
    t.header.stamp = node.get_clock().now().to_msg()
    t.transform.rotation.z = math.sin(yaw/2); t.transform.rotation.w = math.cos(yaw/2)
    tf.sendTransform(t)


def spin(seconds):
    end = time.monotonic()+seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=.02)


def path(empty=False):
    p = RosPath(); p.header.frame_id = 'map'; p.header.stamp = node.get_clock().now().to_msg()
    if not empty:
        for i in range(61):
            v = PoseStamped(); v.header = p.header
            v.pose.position.x = .05*i; v.pose.orientation.w = 1.; p.poses.append(v)
    return p


timer = node.create_timer(.03, tick)
log = (out/'pursuit_continuity.log').open('w')
proc = subprocess.Popen([str(root/'install/legged_navigation/lib/legged_navigation/pure_pursuit'),
    '--ros-args', '--params-file', str(root/'src/legged_bringup/config/pursuit.yaml'),
    '-p', 'path_topic:=/test/path', '-p', 'cmd_vel_topic:=/test/command'],
    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
try:
    spin(1.5); pub.publish(path()); spin(2.)
    assert any(v > .65 for t,v,w in commands if t > time.monotonic()-.4)
    before = time.monotonic()
    for _ in range(6):
        pub.publish(path()); spin(.18)
    recent = [v for t,v,w in commands if t > before]
    assert recent and min(recent) > .65, recent
    # A moderate 52-degree error can be followed with a moving arc.
    yaw = .9; before = time.monotonic(); spin(1.5)
    assert all(v > .05 for t,v,w in commands if t > before+.3)
    assert not any(s == 'aligning_heading' for t,s in states if t > before+.3)
    # A rearward goal still requires braking and stationary rotation.
    yaw = 2.; before = time.monotonic(); spin(2.)
    assert any(abs(v) < 1.e-8 and w < -.1 for t,v,w in commands if t > before+1.)
    yaw = .45; before = time.monotonic(); spin(1.5)
    assert any(v > .15 for t,v,w in commands if t > before+.5)
    # Hard replans retain their stop; a valid replacement cannot inherit speed.
    pub.publish(path(empty=True)); spin(.3)
    assert all(abs(v)+abs(w) < 1.e-8 for t,v,w in commands if t > time.monotonic()-.15)
    yaw = 0.; spin(.15); before = time.monotonic(); pub.publish(path()); spin(.4)
    restarted = [(v,w) for t,v,w in commands if t > before and v > 0.]
    assert restarted and restarted[0][0] <= .15, restarted
    result = dict(domain=178, soft_replan_min_velocity_mps=min(recent),
        moderate_heading_keeps_translation=True, rear_target_still_stops_and_turns=True,
        alignment_exits_at_26_degrees=True, hard_replan_resets_speed=True)
    (out/'pursuit_continuity.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
finally:
    if proc.poll() is None: proc.send_signal(signal.SIGINT)
    try: proc.wait(timeout=5)
    except subprocess.TimeoutExpired: proc.terminate(); proc.wait(timeout=5)
    node.destroy_node(); rclpy.shutdown(); log.close()
