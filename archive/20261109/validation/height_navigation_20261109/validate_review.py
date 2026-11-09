import os,time,signal,subprocess,json
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from lifecycle_msgs.srv import GetState
out=Path('/tmp/height_nav_integration'); env=os.environ.copy()
assert env['ROS_DOMAIN_ID']=='186' and env['ROS_LOCALHOST_ONLY']=='1'
log=(out/'review_launch.log').open('w')
p=subprocess.Popen(['ros2','launch','legged_bringup','rubi_height_navigation.launch.py','rviz:=false','mode:=review'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
rclpy.init();n=Node('height_integration_probe');c=ActionClient(n,ComputePathToPose,'/compute_path_to_pose');result=[]
def pose(x,y):
 m=PoseStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.pose.position.x=x;m.pose.position.y=y;m.pose.orientation.w=1.;return m
try:
 end=time.monotonic()+90
 while time.monotonic()<end:
  if p.poll() is not None:raise RuntimeError('launch exited')
  if c.wait_for_server(timeout_sec=1.):break
 else:raise RuntimeError('no planner action')
 check=n.create_client(GetState,'/planner_server/get_state')
 for i in range(80):
  if check.wait_for_service(timeout_sec=.5):
   f=check.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=1)
   if f.done() and f.result().current_state.id==3:break
  time.sleep(.5)
 else:raise RuntimeError('planner not active')
 time.sleep(8)
 for label,start,goal in [('hall', (18.975,42.975),(22.975,42.975)),('hall_repeat',(18.975,42.975),(22.975,42.975)),('near_steps',(27.975,43.975),(37.975,44.025)),('unknown',(18.975,42.975),(-20.,-20.))]:
  g=ComputePathToPose.Goal();g.planner_id='Height';g.use_start=True;g.start=pose(*start);g.goal=pose(*goal)
  t=time.monotonic();f=c.send_goal_async(g);rclpy.spin_until_future_complete(n,f,timeout_sec=10)
  if not f.done() or not f.result().accepted:raise RuntimeError('goal rejected')
  f=f.result().get_result_async();rclpy.spin_until_future_complete(n,f,timeout_sec=120)
  if not f.done():raise RuntimeError('plan timeout')
  r=f.result();d={'label':label,'start':start,'goal':goal,'status':r.status,'wall_seconds':time.monotonic()-t,'poses':[[v.pose.position.x,v.pose.position.y,v.pose.position.z] for v in r.result.path.poses]}
  result.append(d);(out/'review_results.json').write_text(json.dumps(result,indent=2));print({k:v for k,v in d.items() if k!='poses'},'points',len(d['poses']),flush=True)
 topics=n.get_topic_names_and_types();assert not any(t in ['/cmd_vel','/legged/cmd_raw','/legged/cmd_safe'] for t,_ in topics)
 print('No command topics in review domain',flush=True)
finally:
 p.send_signal(signal.SIGINT)
 try:p.wait(timeout=15)
 except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=10)
 log.close();n.destroy_node();rclpy.shutdown()
