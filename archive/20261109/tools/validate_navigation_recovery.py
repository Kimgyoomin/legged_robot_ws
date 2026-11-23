#!/usr/bin/env python3
"""Isolated regression for mounting-only collision data, fail-closed guard and heading recovery."""
import os,subprocess,signal,time,json,math
from pathlib import Path
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy,qos_profile_sensor_data
from nav_msgs.msg import Odometry,Path as RosPath
from nav2_msgs.msg import Costmap
from geometry_msgs.msg import Twist,PoseStamped,TransformStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header,String
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32,read_points
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
ROOT=next(p for p in Path(__file__).resolve().parents if (p / 'src/legged_bringup').is_dir())
assert os.environ.get('ROS_DOMAIN_ID')=='175','Use isolated domain 175 only'
rclpy.init();n=rclpy.create_node('navigation_recovery_test');procs=[];out=ROOT/'validation/navigation_recovery';out.mkdir(exist_ok=True);log=(out/'integration.log').open('w');ss=[];clouds=[];states=[];commands=[];pp=[];ppstate=[];sensor=True;yaw=0.
def start(pkg,exe,params=(),remaps=()):
 cmd=['ros2','run',pkg,exe,'--ros-args']
 for p in params:cmd+=['-p',p]
 for p in remaps:cmd+=['-r',p]
 procs.append(subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
def sub(typ,topic,cb,qos=10):ss.append(n.create_subscription(typ,topic,cb,qos))
def spin(sec):
 end=time.monotonic()+sec
 while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.01)
sub(Twist,'/cmd_vel',lambda m:commands.append((time.monotonic(),m.linear.x,m.angular.z)))
sub(Twist,'/test/pp_cmd',lambda m:pp.append((time.monotonic(),m.linear.x,m.angular.z)))
sub(String,'/legged/tracking_state',lambda m:ppstate.append(m.data))
sub(String,'/legged/collision_input_state',lambda m:states.append(m.data))
sub(PointCloud2,'/legged/collision_points',lambda m:clouds.append(m),qos_profile_sensor_data)
raw=n.create_publisher(PointCloud2,'/livox/lidar',10);safe=n.create_publisher(Twist,'/legged/cmd_safe',10)
odom=n.create_publisher(Odometry,'/Odometry',10);cost=n.create_publisher(Costmap,'/global_costmap/costmap_raw',10)
pathpub=n.create_publisher(RosPath,'/test/path',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
tf=TransformBroadcaster(n);static=StaticTransformBroadcaster(n)
def tick():
 now=n.get_clock().now().to_msg()
 if sensor:raw.publish(create_cloud_xyz32(Header(stamp=now,frame_id='livox_frame'),[[.8,.1,.3],[.1,0.,0.]]))
 o=Odometry();o.header.stamp=now;odom.publish(o)
 c=Costmap();c.header.stamp=now;c.header.frame_id='map'
 c.metadata.resolution=.05;c.metadata.size_x=120;c.metadata.size_y=120
 c.metadata.origin.position.x=-3.;c.metadata.origin.position.y=-3.;c.metadata.origin.orientation.w=1.
 c.data=[0]*14400;cost.publish(c)
 cmd=Twist();cmd.linear.x=.2;safe.publish(cmd)
 t=TransformStamped();t.header.stamp=now;t.header.frame_id='map';t.child_frame_id='base_link';t.transform.rotation.z=math.sin(yaw/2);t.transform.rotation.w=math.cos(yaw/2);tf.sendTransform(t)
 # A dynamic mount is intentionally not an acceptable mounting contract.
 t=TransformStamped();t.header.stamp=now;t.header.frame_id='base_link';t.child_frame_id='livox_frame';t.transform.rotation.w=1.;tf.sendTransform(t)
timer=n.create_timer(.05,tick)
def path():
 p=RosPath();p.header.frame_id='map';p.header.stamp=n.get_clock().now().to_msg()
 for x in [0.,-.25,-.5,-1.,-2.]:
  v=PoseStamped();v.header=p.header;v.pose.position.x=x;v.pose.orientation.w=1.;p.poses.append(v)
 return p
try:
 start('legged_navigation','obstacle_cloud');start('legged_bringup','command_watchdog')
 spin(2.5)
 assert 'missing_static_mount' in states,states[-5:]
 assert not clouds and commands and max(abs(v) for t,v,w in commands)==0.,'Missing mount failed open'
 t=TransformStamped();t.header.stamp=n.get_clock().now().to_msg();t.header.frame_id='base_link';t.child_frame_id='livox_frame';t.transform.rotation.x=1.;t.transform.rotation.w=0.;static.sendTransform(t)
 spin(1.5)
 assert clouds and clouds[-1].header.frame_id=='base_link'
 pts=list(read_points(clouds[-1],field_names=('x','y','z')))
 print('mount points',pts,flush=True)
 assert len(pts)==1 and abs(float(pts[0][1])+.1)<1.e-5 and abs(float(pts[0][2])+.3)<1.e-5
 assert any(v>.1 for t,v,w in commands if t>time.monotonic()-.5),'Valid collision data not released'
 sensor=False;spin(.9)
 assert max(abs(v) for t,v,w in commands if t>time.monotonic()-.25)==0.,'Stale sensor failed open'
 sensor=True;spin(.5)
 start('legged_navigation','pure_pursuit',params=['path_topic:=/test/path','cmd_vel_topic:=/test/pp_cmd','max_tf_age_s:=0.5'])
 spin(1.5);pathpub.publish(path());begin=time.monotonic();spin(1.5)
 moving=[(v,w) for t,v,w in pp if t>begin+.4]
 assert moving and all(abs(v)<1.e-8 for v,w in moving) and any(w>.1 for v,w in moving),'Behind target did not rotate without translation'
 assert 'aligning_heading' in ppstate
 yaw=math.pi;begin=time.monotonic();spin(1.2)
 assert any(v>.05 for t,v,w in pp if t>begin+.5),'Did not resume after heading alignment'
 empty=RosPath();empty.header.frame_id='map';pathpub.publish(empty);spin(.4)
 assert max(abs(v)+abs(w) for t,v,w in pp if t>time.monotonic()-.2)<1.e-8,'Empty path did not stop'
 result=dict(static_mount_required=True,inverted_mount_transform=True,range_filter_preserved=True,stale_sensor_stop=True,behind_target_rotation=True,alignment_resume=True,empty_path_stop=True,domain=175)
 (out/'integration.json').write_text(json.dumps(result,indent=2));print(result)
finally:
 for p in procs:
  if p.poll() is None:os.killpg(p.pid,signal.SIGINT)
 for p in procs:
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
 n.destroy_node();rclpy.shutdown();log.close()
