#!/usr/bin/env python3
"""Exercise the final watchdog on isolated domain 177; no robot or goal launch."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

assert os.environ.get('ROS_DOMAIN_ID') == '177', 'Use isolated ROS_DOMAIN_ID=177'
root = next(p for p in Path(__file__).resolve().parents if (p / 'src/legged_bringup').is_dir())
out = root/'validation/rectangle_speed_20261015'
out.mkdir(exist_ok=True)
rclpy.init()
n = rclpy.create_node('guard_test_inputs')
pubs = {topic: n.create_publisher(typ, topic, 10) for topic, typ in [
    ('/legged/cmd_safe', Twist), ('/legged/collision_points', PointCloud2),
    ('/legged/collision_input_state', String), ('/Odometry', Odometry),
    ('/global_costmap/costmap_raw', Costmap)]}
tf = TransformBroadcaster(n)
commands, states = [], []
subs = [n.create_subscription(Twist, '/cmd_vel', lambda m: commands.append(
    (time.monotonic(), m.linear.x)), 10),
    n.create_subscription(String, '/legged/motion_status', lambda m: states.append(
    (time.monotonic(), json.loads(m.data)['reason'])), 10)]
costmap = Costmap(); costmap.header.frame_id = 'map'
costmap.metadata.size_x = costmap.metadata.size_y = 80
costmap.metadata.resolution = .05; costmap.metadata.origin.orientation.w = 1.
costmap.data = [0]*6400
publish_pose = publish_cloud = True


def tick():
    now = n.get_clock().now().to_msg()
    cmd = Twist(); cmd.linear.x = .7; pubs['/legged/cmd_safe'].publish(cmd)
    odom = Odometry(); odom.header.stamp = now; pubs['/Odometry'].publish(odom)
    pubs['/legged/collision_input_state'].publish(String(data='ready'))
    if publish_cloud:
        cloud = PointCloud2(); cloud.header.stamp = now; cloud.header.frame_id = 'base_link'
        pubs['/legged/collision_points'].publish(cloud)
    costmap.header.stamp = now; pubs['/global_costmap/costmap_raw'].publish(costmap)
    if publish_pose:
        t = TransformStamped(); t.header.stamp = now; t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'; t.transform.translation.x = t.transform.translation.y = 1.
        t.transform.rotation.w = 1.; tf.sendTransform(t)


def spin(seconds):
    until = time.monotonic()+seconds
    while time.monotonic() < until:
        rclpy.spin_once(n, timeout_sec=.01)


def require(reason, moving=False):
    spin(.85)
    latest = [v for t, v in commands if t > time.monotonic()-.3]
    assert latest and (all(abs(v-.7) < 1.e-8 for v in latest) if moving else
                       all(abs(v) < 1.e-8 for v in latest)), (reason, latest)
    assert any(s == reason for t, s in states if t > time.monotonic()-.85), states[-4:]


timer = n.create_timer(.05, tick)
log = (out/'watchdog.log').open('w')
proc = subprocess.Popen(['ros2', 'run', 'legged_bringup', 'command_watchdog', '--ros-args',
                         '-p', 'max_pose_age:=0.15'], stdout=log, stderr=subprocess.STDOUT,
                        start_new_session=True)
try:
    spin(2.5); require('pass', True)
    # Side wall beyond the 0.45 m half-width stays clear.
    for x in range(12, 55): costmap.data[31*80+x] = 254
    require('pass', True)
    # The enlarged rectangle now blocks a wall inside that half-width.
    for x in range(12, 55): costmap.data[28*80+x] = 254
    require('costmap_collision')
    costmap.data = [0]*6400
    costmap.data[20*80+31] = 254  # 0.575 m ahead: outside rectangle, inside sensor blind zone.
    require('predicted_costmap_collision')
    costmap.data = [253]*6400  # Inflation alone must not be counted as an obstacle again.
    require('pass', True)
    costmap.data = [0]*6400; costmap.data[20*80+37] = 255
    require('predicted_costmap_collision')
    costmap.data = [0]*6400; publish_pose = False
    require('costmap_pose_stale')
    publish_pose = True; require('pass', True)
    publish_cloud = False; require('collision_cloud_timeout')
    publish_cloud = True; require('pass', True)
    costmap.data = []
    require('invalid_costmap')
    result = dict(domain=177, clear_command_07=True, clear_side_wall_pass=True,
                  inside_width_stop=True, stop_length_m=1.0, stop_width_m=.9,
                  mapped_blind_zone_stop=True, inflation_not_double_counted=True,
                  unknown_stop=True, stale_pose_stop=True, stale_cloud_stop=True,
                  malformed_costmap_stop=True)
    (out/'watchdog.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps(result))
finally:
    if proc.poll() is None: os.killpg(proc.pid, signal.SIGINT)
    try: proc.wait(timeout=5)
    except subprocess.TimeoutExpired: os.killpg(proc.pid, signal.SIGTERM); proc.wait(timeout=5)
    n.destroy_node(); rclpy.shutdown(); log.close()
