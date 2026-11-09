#!/usr/bin/env python3
"""Read-only tracking recorder. Does not send goals, commands or start navigation."""
import argparse
from collections import Counter
import json
from pathlib import Path
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path as RosPath
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker

p = argparse.ArgumentParser()
p.add_argument('output', type=Path)
p.add_argument('--seconds', type=float, default=45.)
args = p.parse_args()
args.output.parent.mkdir(parents=True, exist_ok=True)
rclpy.init()
n = rclpy.create_node('tracking_readonly_recorder', parameter_overrides=[Parameter('use_sim_time', value=True)])
buffer = Buffer(); listener = TransformListener(buffer, n)
data = []; subscriptions = []; start = time.monotonic()


def record(m, topic):
    d = dict(topic=topic, wall=time.monotonic()-start, ros=n.get_clock().now().nanoseconds/1e9)
    if isinstance(m, Odometry):
        pose = m.pose.pose
        d.update(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
                 frame=m.header.frame_id, child=m.child_frame_id,
                 xyz=[pose.position.x, pose.position.y, pose.position.z],
                 q=[pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w],
                 vx=m.twist.twist.linear.x, vy=m.twist.twist.linear.y, wz=m.twist.twist.angular.z)
    elif isinstance(m, Twist):
        d.update(vx=m.linear.x, vy=m.linear.y, wz=m.angular.z)
    elif isinstance(m, RosPath):
        d.update(stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9, frame=m.header.frame_id,
                 points=[[v.pose.position.x, v.pose.position.y] for v in m.poses])
    elif isinstance(m, Joy):
        d.update(axes=list(m.axes), buttons=list(m.buttons))
    elif isinstance(m, String):
        d['state'] = m.data
    else:
        d.update(x=m.pose.position.x, y=m.pose.position.y)
    data.append(d)


topics = [(t, Odometry) for t in ['/Odometry', '/localization', '/ground_truth/odom']]
topics += [(t, Twist) for t in ['/legged/cmd_raw', '/legged/cmd_safe', '/cmd_vel']]
topics += [(t, String) for t in ['/legged/tracking_state', '/legged/motion_status']]
topics += [('/joy', Joy), ('/rubi/planner_comparison/grid/tracking_path', RosPath)]
topics += [(t, Marker) for t in ['/rubi/simple_pp/lookahead', '/rubi/simple_pp/nearest_path_point']]
for topic, typ in topics:
    qos = QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL) if typ == RosPath else qos_profile_sensor_data
    subscriptions.append(n.create_subscription(typ, topic, lambda m, t=topic: record(m, t), qos))


def pose_tick():
    try:
        t = buffer.lookup_transform('map', 'base_link', Time())
        v=t.transform.translation; q=t.transform.rotation
        data.append(dict(topic='map_base_tf', wall=time.monotonic()-start,
                         ros=n.get_clock().now().nanoseconds/1e9,
                         stamp=t.header.stamp.sec+t.header.stamp.nanosec/1e9,
                         xyz=[v.x,v.y,v.z], q=[q.x,q.y,q.z,q.w]))
    except Exception:
        pass


timer = n.create_timer(.05, pose_tick)
try:
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(n, timeout_sec=.02)
finally:
    args.output.write_text(json.dumps(data))
    print(json.dumps(dict(Counter(d['topic'] for d in data))))
    n.destroy_node(); rclpy.shutdown()
