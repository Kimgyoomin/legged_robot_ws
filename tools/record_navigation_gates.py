#!/usr/bin/env python3
"""Read-only synchronized navigation-gate, obstacle and map snapshot recorder."""
import argparse
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from rclpy.time import Time
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from legged_bringup.costmap_guard import CostmapGuard, planar_yaw

parser = argparse.ArgumentParser()
parser.add_argument('out', type=Path)
parser.add_argument('--seconds', type=float, default=60.)
args = parser.parse_args()
args.out.mkdir(parents=True, exist_ok=True)
rclpy.init()
node = rclpy.create_node('navigation_gates_readonly', parameter_overrides=[Parameter('use_sim_time', value=True)])
buffer = Buffer(); listener = TransformListener(buffer, node)
guard = CostmapGuard(half_length=.5, half_width=.45, horizon=1.)
messages, received, samples, saved, subs = {}, {}, [], [], []
start = time.monotonic()


def receive(topic, message):
    messages[topic] = message; received[topic] = time.monotonic()
    if topic == '/global_costmap/costmap_raw': guard.update(message)


for topic, typ in [('/global_costmap/costmap_raw', Costmap), ('/map', OccupancyGrid),
                   ('/legged/collision_points', PointCloud2),
                   ('/legged/cmd_raw', Twist), ('/legged/cmd_safe', Twist), ('/cmd_vel', Twist),
                   ('/legged/motion_status', String), ('/legged/tracking_state', String),
                   ('/legged/collision_input_state', String), ('/goal_pose', PoseStamped),
                   ('/rubi/planner_comparison/grid/tracking_path', RosPath)]:
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL) if typ in (OccupancyGrid, RosPath) else qos_profile_sensor_data
    subs.append(node.create_subscription(typ, topic, lambda m, t=topic: receive(t, m), qos))


def stamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec/1.e9


def grid_stats(grid, resolution, origin, yaw, pose, threshold):
    x,y,heading=pose; dx,dy=x-origin[0],y-origin[1]
    c,s=math.cos(yaw),math.sin(yaw)
    gx,gy=(c*dx+s*dy)/resolution,(-s*dx+c*dy)/resolution
    ix,iy=math.floor(gx),math.floor(gy)
    result={'center':int(grid[iy,ix]) if 0<=iy<grid.shape[0] and 0<=ix<grid.shape[1] else None}
    radius=int(math.ceil(2./resolution))
    lx,hx=max(0,ix-radius),min(grid.shape[1],ix+radius+1)
    ly,hy=max(0,iy-radius),min(grid.shape[0],iy+radius+1)
    if hx<=lx or hy<=ly: return result
    crop=grid[ly:hy,lx:hx]
    rr,cc=np.nonzero(crop>=threshold)
    px=(cc+lx+.5-gx)*resolution; py=(rr+ly+.5-gy)*resolution
    a=heading-yaw; ca,sa=math.cos(a),math.sin(a)
    bx=ca*px+sa*py; by=-sa*px+ca*py
    reach=.5*resolution*(abs(ca)+abs(sa))
    hits=(abs(bx)<=.5+reach)&(abs(by)<=.45+reach)
    hits &= (abs(px)<=.5*abs(ca)+.45*abs(sa)+.5*resolution)
    hits &= (abs(py)<=.5*abs(sa)+.45*abs(ca)+.5*resolution)
    result.update(nearest_blocked_center_m=float(np.hypot(px,py).min()) if len(px) else None,
                  footprint_blocked_cells=int(hits.sum()), crop_origin_grid=[lx,ly])
    return result


def sample():
    wall=time.monotonic()-start; now=node.get_clock().now().nanoseconds/1.e9
    record={'wall':wall,'ros':now,'age_wall':{k:time.monotonic()-v for k,v in received.items()}}
    for topic in ('/legged/cmd_raw','/legged/cmd_safe','/cmd_vel'):
        m=messages.get(topic)
        if m: record[topic]=[m.linear.x,m.linear.y,m.angular.z]
    for topic in ('/legged/motion_status','/legged/tracking_state','/legged/collision_input_state'):
        m=messages.get(topic)
        if m:
            try: record[topic]=json.loads(m.data)
            except ValueError: record[topic]=m.data
    cloud=messages.get('/legged/collision_points')
    if cloud:
        record['cloud_age_ros']=now-stamp(cloud)
        record['cloud_frame']=cloud.header.frame_id
        try:
            fields={f.name:f for f in cloud.fields}
            endian='>' if cloud.is_bigendian else '<'
            dtype=np.dtype({'names':['x','y','z'], 'formats':[endian+('f4' if fields[k].datatype==7 else 'f8') for k in ('x','y','z')],
                            'offsets':[fields[k].offset for k in ('x','y','z')], 'itemsize':cloud.point_step})
            points=np.ndarray((cloud.height,cloud.width),dtype=dtype,buffer=cloud.data,strides=(cloud.row_step,cloud.point_step))
            x,y,z=(points[k].ravel() for k in ('x','y','z'))
            mask=(abs(x)<=.5)&(abs(y)<=.45)&(z>=-.35)&(z<=1.5)
            record['stop_cloud_points']=int(mask.sum())
            if mask.any(): record['stop_points_bbox']=[list(map(float,[x[mask].min(),y[mask].min(),z[mask].min()])),list(map(float,[x[mask].max(),y[mask].max(),z[mask].max()]))]
        except (ValueError, KeyError) as e: record['cloud_error']=str(e)
    try:
        tf=buffer.lookup_transform('map','base_link',Time())
        pose=(tf.transform.translation.x,tf.transform.translation.y,planar_yaw(tf.transform.rotation))
        record['pose']=pose; record['pose_age_ros']=now-stamp(tf)
        if guard.grid is not None:
            record['costmap']=grid_stats(guard.grid,guard.resolution,guard.origin[:2],guard.origin[2],pose,254)
            record['stationary_guard']=guard.check(pose,(0.,0.,0.))
            raw=record.get('/legged/cmd_raw',(0.,0.,0.))
            record['raw_guard']=guard.check(pose,raw)
            record['counterfactual_regulation']=guard.regulate(pose,raw)
            record['costmap_age_ros']=now-stamp(messages['/global_costmap/costmap_raw'])
        static=messages.get('/map')
        if static:
            grid=np.asarray(static.data,dtype=np.int8).reshape(static.info.height,static.info.width)
            origin=static.info.origin
            record['static']=grid_stats(grid,static.info.resolution,(origin.position.x,origin.position.y),planar_yaw(origin.orientation),pose,65)
        path=messages.get('/rubi/planner_comparison/grid/tracking_path')
        if path is not None:
            record['path_count']=len(path.poses)
            if path.poses:
                xy=np.array([[p.pose.position.x,p.pose.position.y] for p in path.poses])
                distance=np.linalg.norm(xy-np.array(pose[:2]),axis=1)
                nearest=int(distance.argmin())
                record.update(nearest_path_index=nearest,nearest_vertex_distance=float(distance[nearest]),
                              goal_distance=float(distance[-1]),path_goal=xy[-1].tolist())
        if guard.grid is not None and (not saved or wall-saved[-1]['wall']>=5.):
            name=f'grid_{len(saved):03d}.npz'
            np.savez_compressed(args.out/name,grid=guard.grid,origin=guard.origin,resolution=guard.resolution,
                                pose=pose,ros=now,wall=wall,
                                path=np.array([[p.pose.position.x,p.pose.position.y] for p in path.poses]) if path is not None else np.empty((0,2)))
            saved.append({'wall':wall,'ros':now,'file':name})
    except Exception as e: record['pose_or_map_error']=str(e)
    samples.append(record)


timer=node.create_timer(.5,sample,clock=Clock(clock_type=ClockType.STEADY_TIME))
try:
    while time.monotonic()-start<args.seconds:
        rclpy.spin_once(node,timeout_sec=.05)
finally:
    (args.out/'gates.json').write_text(json.dumps(samples,indent=2)+'\n')
    (args.out/'snapshots.json').write_text(json.dumps(saved,indent=2)+'\n')
    print(json.dumps({'samples':len(samples),'snapshots':len(saved),'last':samples[-1] if samples else {}}))
    node.destroy_node();rclpy.shutdown()
