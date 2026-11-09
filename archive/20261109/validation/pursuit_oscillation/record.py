import time,json,rclpy
from pathlib import Path
from nav_msgs.msg import Odometry,Path as RosPath
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy
rclpy.init();n=rclpy.create_node('pursuit_readonly_recorder');data=[];subs=[];t=time.monotonic()
def cb(m,topic):
 d={'topic':topic,'wall':time.monotonic()-t}
 if isinstance(m,Odometry):
  p=m.pose.pose;d.update(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,frame=m.header.frame_id,child=m.child_frame_id,xyz=[p.position.x,p.position.y,p.position.z],q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w],vx=m.twist.twist.linear.x,wz=m.twist.twist.angular.z)
 elif isinstance(m,Twist):d.update(vx=m.linear.x,wz=m.angular.z)
 elif isinstance(m,RosPath):d.update(stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,frame=m.header.frame_id,points=[[p.pose.position.x,p.pose.position.y] for p in m.poses])
 else:d.update(x=m.pose.position.x,y=m.pose.position.y)
 data.append(d)
for topic,typ in [('/Odometry',Odometry),('/localization',Odometry),('/ground_truth/odom',Odometry),('/legged/cmd_raw',Twist),('/legged/cmd_safe',Twist),('/cmd_vel',Twist),('/rubi/planner_comparison/grid/tracking_path',RosPath),('/rubi/simple_pp/lookahead',Marker)]:
 qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if typ==RosPath else qos_profile_sensor_data
 subs.append(n.create_subscription(typ,topic,lambda m,t=topic:cb(m,t),qos))
while time.monotonic()-t<35:rclpy.spin_once(n,timeout_sec=.03)
Path('/home/kim/legged_robot_ws/validation/pursuit_oscillation/samples.json').write_text(json.dumps(data))
from collections import Counter
print(dict(Counter(d['topic'] for d in data)));n.destroy_node();rclpy.shutdown()
