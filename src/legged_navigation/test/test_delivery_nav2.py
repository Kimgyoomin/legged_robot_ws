#!/usr/bin/env python3
"""Synthetic, localhost-only Nav2 integration; never launch robot hardware."""
import os,sys,time,signal,subprocess,tempfile,math
from pathlib import Path
assert os.environ.get('ROS_LOCALHOST_ONLY')=='1'
assert os.environ.get('ROS_DOMAIN_ID')=='227'
os.environ.pop('FASTRTPS_DEFAULT_PROFILES_FILE',None)
os.environ.pop('FASTDDS_DEFAULT_PROFILES_FILE',None)
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped,Vector3Stamped,Twist,PoseStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from lifecycle_msgs.srv import GetState
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header
from rcl_interfaces.srv import GetParameters

mode=sys.argv[1] if len(sys.argv)>1 else 'only'
out=Path(tempfile.mkdtemp(prefix='legged_nav2_'+mode+'_'))
print('Test artifacts:',out,flush=True)
# Centered 5cm lattice, plenty of observed support around start and goal.
height=out/'map.rghm';cols,rows=120,80
with height.open('w') as f:
 f.write(f'RUBI_GLOBAL_HEIGHTMAP 1\nmap 0.05 -2 -2 {cols} {rows} 0.04 1000000000 {cols*rows}\n')
 for i in range(cols*rows):f.write(f'{i} 0 0.001 1000000000 1\n')
(out/'map.pgm').write_bytes(f'P5\n{cols} {rows}\n255\n'.encode()+bytes([254])*(cols*rows))
(out/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [-2., -2., 0.]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
for label, path, resolution in [('missing',out/'missing.rghm',.05),('resolution',height,.1)]:
 check=subprocess.run(['ros2','run','rubi_global_heightmap_wrapper','frozen_heightmap_node','--ros-args',
                       '-p','file:='+str(path),'-p','expected_resolution:='+str(resolution)],
                       capture_output=True,text=True,timeout=10)
 assert check.returncode!=0, label+' map should be rejected'
 assert ('Cannot open' if label=='missing' else 'resolution does not match') in check.stderr+check.stdout
print(mode,'invalid frozen inputs rejected',flush=True)
rclpy.init();node=rclpy.create_node('synthetic_delivery_check')
static=StaticTransformBroadcaster(node);dynamic=TransformBroadcaster(node)
def tf(parent,child,x=0.,y=0.):
 t=TransformStamped();t.header.stamp=node.get_clock().now().to_msg();t.header.frame_id=parent;t.child_frame_id=child
 t.transform.translation.x=x;t.transform.translation.y=y;t.transform.rotation.w=1.;return t
static.sendTransform([tf('map','camera_init'),tf('body','BODY'),tf('BODY','base_link'),tf('base_link','livox_frame')])
up=node.create_publisher(Vector3Stamped,'/fastlio/up_body',qos_profile_sensor_data)
vel=node.create_publisher(Odometry,'/fastlio/odometry_velocity',qos_profile_sensor_data)
odom=node.create_publisher(Odometry,'/Odometry',10)
cloud=node.create_publisher(PointCloud2,'/livox/lidar_pointcloud',qos_profile_sensor_data)
command=[];filtered=[]
node.create_subscription(Twist,'/delivery_test/cmd_vel',lambda m:command.append(m),10)
node.create_subscription(PointCloud2,'/livox/lidar_costmap',lambda m:filtered.append(m.width),qos_profile_sensor_data)
def publish():
 stamp=node.get_clock().now().to_msg();dynamic.sendTransform(tf('camera_init','body'))
 g=Vector3Stamped();g.header.stamp=stamp;g.header.frame_id='body';g.vector.z=1.;up.publish(g)
 o=Odometry();o.header.stamp=stamp;o.header.frame_id='camera_init';o.child_frame_id='body';o.pose.pose.orientation.w=1.;vel.publish(o);odom.publish(o)
 h=Header();h.stamp=stamp;h.frame_id='livox_frame'
 # Obstacle is away from the tested x-axis path but proves the live local pipeline.
 cloud.publish(create_cloud_xyz32(h,[[1.,1.5,.5],[1.05,1.5,.5]]))
node.create_timer(.05,publish)
def spin_until(predicate,seconds=45):
 deadline=time.monotonic()+seconds
 while time.monotonic()<deadline:
  rclpy.spin_once(node,timeout_sec=.05)
  if predicate():return True
  if 'process' in globals() and process.poll() is not None: raise AssertionError('Nav2 launch exited; see launch.log')
 return False
def future(f,seconds=45):
 assert spin_until(f.done,seconds),'future timeout';return f.result()
def pose(x):
 p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg();p.pose.position.x=x;p.pose.orientation.w=1.;return p
cmd=['ros2','launch','legged_navigation',f'height_{mode}_navigation.launch.py','dds_profile:=default','rviz:=false',
     'use_fastlio_velocity:=true',f'height_map:={height}','cmd_vel_topic:=/delivery_test/cmd_vel']
if mode=='hybrid':cmd.append(f'map_yaml:={out}/map.yaml')
log=(out/'launch.log').open('w');process=subprocess.Popen(cmd,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
 for name in ('planner_server','controller_server','behavior_server','bt_navigator'):
  client=node.create_client(GetState,name+'/get_state')
  assert spin_until(lambda:client.service_is_ready(),60),name+' service missing'
  end=time.monotonic()+60
  while time.monotonic()<end:
   response=future(client.call_async(GetState.Request()))
   if response.current_state.id==3:break
   spin_until(lambda:False,.3)
  else:raise AssertionError(name+' did not activate')
 print(mode,'all lifecycle nodes active',flush=True)
 assert spin_until(lambda:'terrain READY' in (out/'launch.log').read_text(),60),'cache not ready'
 assert spin_until(lambda:bool(filtered),10),'No filtered live obstacle cloud'
 actions=ActionClient(node,ComputePathToPose,'compute_path_to_pose')
 assert spin_until(actions.server_is_ready,10)
 req=ComputePathToPose.Goal();req.start=pose(0.);req.goal=pose(1.);req.use_start=True;req.planner_id='HeightOnly' if mode=='only' else 'Height'
 handle=future(actions.send_goal_async(req));assert handle.accepted
 result=future(handle.get_result_async());assert result.status==4,(result.status,result.result)
 assert len(result.result.path.poses)>10
 print(mode,'ComputePathToPose succeeded, poses=',len(result.result.path.poses),flush=True)
 names=node.get_node_names();assert ('map_server' in names)==(mode=='hybrid')
 # Height-only has no Occupancy input, even with unrelated /map on the graph.
 if mode=='only':
  topic_names=dict(node.get_topic_names_and_types());assert '/map' not in topic_names
 nav=ActionClient(node,NavigateToPose,'navigate_to_pose');assert spin_until(nav.server_is_ready,10)
 goal=NavigateToPose.Goal();goal.pose=pose(1.)
 goal_handle=future(nav.send_goal_async(goal));assert goal_handle.accepted
 assert spin_until(lambda:any(m.linear.x>0 for m in command),20),'No forward DWB command'
 cancellation=future(goal_handle.cancel_goal_async());assert cancellation.goals_canceling
 final=future(goal_handle.get_result_async(),20);assert final.status==5,final.status
 print(mode,'NavigateToPose -> DWB command -> cancel succeeded; live filter samples=',len(filtered),flush=True)
 # Out-of-map goal must fail, not produce a path through missing terrain.
 req.goal=pose(20.);handle=future(actions.send_goal_async(req));result=future(handle.get_result_async())
 assert result.status!=4,'Unobserved goal incorrectly succeeded'
 print(mode,'unknown/out-of-map goal rejected; PASS',flush=True)
finally:
 if process.poll() is None:
  os.killpg(process.pid,signal.SIGINT)
  try:process.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=10)
 log.close();node.destroy_node();rclpy.shutdown()
