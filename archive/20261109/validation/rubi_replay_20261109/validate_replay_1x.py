import os, sys, time, signal, subprocess, json, math, statistics
from pathlib import Path
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3Stamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath, OccupancyGrid
from sensor_msgs.msg import PointCloud2
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState

assert os.environ.get('ROS_DOMAIN_ID') == '188'
out=Path('/tmp/rubi_integration/runtime_1x');out.mkdir(exist_ok=True)
procs=[]; logs=[]; received=defaultdict(list); poses=[]; last={}
rclpy.init(); n=Node('raw_bag_integration_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
start=time.monotonic()
def record(name,msg):
    now=n.get_clock().now().nanoseconds/1e9
    stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9 if hasattr(msg,'header') else now
    received[name].append((time.monotonic()-start, now, stamp))
    last[name]=msg
    if name=='/localization': poses.append([now,msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
    if name=='/legged/replay/cmd_vel':
        commands.append([now,msg.linear.x,msg.angular.z])
commands=[]
subs=[]
for topic,typ in [('/Odometry',Odometry),('/localization',Odometry),('/map_to_odom',Odometry),('/fastlio/odometry_velocity',Odometry),('/fastlio/odometry_base',Odometry),('/fastlio/up_body',Vector3Stamped),('/livox/lidar_costmap',PointCloud2),('/local_plan',NavPath),('/plan',NavPath),('/legged/replay/cmd_vel',Twist),('/cloud_registered',PointCloud2)]:
    subs.append(n.create_subscription(typ,topic,lambda m,t=topic:record(t,m),qos_profile_sensor_data))
q=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
for topic in ['/local_costmap/costmap','/global_costmap/costmap']:
    subs.append(n.create_subscription(OccupancyGrid,topic,lambda m,t=topic:record(t,m),q))
initial=n.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
nav=ActionClient(n,NavigateToPose,'/navigate_to_pose')
plan=ActionClient(n,ComputePathToPose,'/compute_path_to_pose')
results={};goalhandle=None

def launch(cmd,name):
    f=(out/name).open('w');logs.append(f)
    p=subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True);procs.append(p);return p

def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)

try:
    stack=launch(['ros2','launch','legged_bringup','rubi_height_navigation.launch.py','mode:=replay','rviz:=false','dds_profile:=navigation'], 'launch.log')
    spin(12)
    if stack.poll() is not None: raise RuntimeError('stack launch exited')
    player=launch(['python3','/home/rclab/legged_robot_ws/tools/replay_height_bag.py','/home/rclab/rosbag_record/[26.11.06]Height_map_data','--rate','1.0','--no-pause'],'bag.log')
    deadline=time.monotonic()+50
    while time.monotonic()<deadline and len(received['/cloud_registered'])<30:spin(.2)
    if len(received['/cloud_registered'])<30: raise RuntimeError('no scan stream')
    m=PoseWithCovarianceStamped();m.header.frame_id='map';m.header.stamp=n.get_clock().now().to_msg();m.pose.pose.orientation.w=1.;initial.publish(m)
    spin(20)
    for name in ['planner_server','controller_server','bt_navigator']:
        c=n.create_client(GetState,'/'+name+'/get_state')
        f=c.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=5)
        results[name]=f.result().current_state.label if f.done() and f.result() else 'timeout'
    if not received['/map_to_odom']: raise RuntimeError('initial ICP did not produce map_to_odom; inspect fitness')
    loc=last['/localization'];print('localized', loc.pose.pose.position,flush=True)
    # Request a short in-room path; recorded motion remains unchanged.
    goal=NavigateToPose.Goal();goal.pose.header.frame_id='map';goal.pose.header.stamp=n.get_clock().now().to_msg()
    goal.pose.pose.position.x=2.8;goal.pose.pose.position.y=.17;goal.pose.pose.orientation.w=1.
    if not nav.wait_for_server(timeout_sec=5):raise RuntimeError('Nav2 action not ready')
    future=nav.send_goal_async(goal);rclpy.spin_until_future_complete(n,future,timeout_sec=5)
    if not future.done():raise RuntimeError('goal request timeout')
    goalhandle=future.result();results['goal_accepted']=goalhandle.accepted
    spin(85)
    topics=n.get_topic_names_and_types();results['command_topics']=[t for t,_ in topics if 'cmd_vel' in t]
    assert '/cmd_vel' not in results['command_topics']
    pc=n.create_client(GetParameters,'/controller_server/get_parameters');q=GetParameters.Request();q.names=['use_sim_time','odom_topic','controller_frequency','FollowPath.trajectory_generator_name','FollowPath.sim_period']
    f=pc.call_async(q);rclpy.spin_until_future_complete(n,f,timeout_sec=5)
    if f.done(): results['controller_parameters']=[str(v) for v in f.result().values]
finally:
    if goalhandle and goalhandle.accepted:
        f=goalhandle.cancel_goal_async();rclpy.spin_until_future_complete(n,f,timeout_sec=3)
    for p in reversed(procs):
        if p.poll() is None:
            p.send_signal(signal.SIGINT)
            try:p.wait(timeout=12)
            except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=8)
    results['process_exit_codes']=[p.returncode for p in procs]
    for f in logs:f.close()
    for topic, entries in received.items():
        e=[v for v in entries if v[0]>25]; ages=[now-stamp for _,now,stamp in e if stamp>0 and now>0]
        stamps=sorted(set(v[2] for v in e));dt=[b-a for a,b in zip(stamps,stamps[1:]) if b>a]
        wall_dt=[b[0]-a[0] for a,b in zip(e,e[1:]) if b[0]>a[0]]
        results[topic]={'count':len(entries),'wall_mean_hz':(len(e)-1)/(e[-1][0]-e[0][0]) if len(e)>1 else None,'wall_gap_max':max(wall_dt) if wall_dt else None,'stamp_hz':1/statistics.median(dt) if dt else None,'age_median':statistics.median(ages) if ages else None,'age_max':max(ages) if ages else None}
    results['nonzero_commands']=sum(abs(x)+abs(w)>1e-4 for _,x,w in commands)
    (out/'summary.json').write_text(json.dumps(results,indent=2));(out/'localization.json').write_text(json.dumps(poses));(out/'commands.json').write_text(json.dumps(commands))
    print(json.dumps(results,indent=2),flush=True)
    n.destroy_node();rclpy.shutdown()
