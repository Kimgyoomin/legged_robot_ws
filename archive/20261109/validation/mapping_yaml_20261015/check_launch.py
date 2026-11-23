"""Expand the actual installed launch without starting any ROS processes."""
import importlib.util,json
from pathlib import Path
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.utilities import evaluate_parameters
from ament_index_python.packages import get_package_share_directory

share=Path(get_package_share_directory('legged_bringup'))
spec=importlib.util.spec_from_file_location('mapping_launch',share/'launch/mapping.launch.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
result={}
for name,overrides in [('default',{}),('hardware',{'use_sim_time':'false','rviz':'false'}),('legacy_raw',{'input_scan':'/livox/lidar_PointCloud2','odom_topic':'/ground_truth/odom','map_frame':'world','base_frame':'BODY'})]:
 c=LaunchContext();c.launch_configurations.update(overrides)
 m.generate_launch_description().entities[0].execute(c)
 actions=m.start(c)
 for a in actions[:-1]:a.execute(c)
 child=actions[-1].launch_description_source.get_launch_description(c)
 values=[]
 for a in child.entities:
  if isinstance(a,DeclareLaunchArgument):a.execute(c)
  elif isinstance(a,Node):
   params=evaluate_parameters(c,a._Node__parameters)
   values.append([str(p) if isinstance(p,Path) else p for p in params])
 mapper,wrapper,rviz=values
 assert mapper[-1]['input_topic']==overrides.get('input_scan','/cloud_registered_body')
 assert mapper[-1]['odom_topic']==overrides.get('odom_topic','/localization')
 assert mapper[-1]['use_sim_time']==(name!='hardware')
 assert wrapper[-1]['use_sim_time']==(name!='hardware')
 assert mapper[-1]['map_frame']==wrapper[-1]['map_frame']==overrides.get('map_frame','map')
 assert Path(mapper[0]).is_file() and Path(wrapper[0]).is_file()
 assert wrapper[-1]['output_directory']==str(Path('~/legged_robot_ws/maps/sim_lio_snapshot').expanduser())
 result[name]=values
print(json.dumps(result,indent=2))
