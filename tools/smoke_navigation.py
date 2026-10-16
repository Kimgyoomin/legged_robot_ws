#!/usr/bin/env python3
"""Isolated ROS integration: frozen maps, both planners, obstacle stop and reroute.

Never run on a robot domain. This script publishes synthetic TF/sensors and does
not start Gazebo, Livox hardware, or the robot actuator bridge.
"""
import argparse
import math
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped, PoseStamped, Twist
from nav_msgs.msg import Path as RosPath, Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from legged_bringup.costmap_guard import rectangle_half_extents

ROOT = Path(__file__).resolve().parents[1]


class Inputs(Node):
    def __init__(self, mode):
        super().__init__('delivery_smoke_inputs')
        self.mode=mode;self.obstacle=False;self.near=False;self.sensor_enabled=True
        self.paths=[];self.commands=[];self.heights=[]
        self.tf=TransformBroadcaster(self);self.static=StaticTransformBroadcaster(self)
        t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg()
        t.header.frame_id='map';t.child_frame_id='camera_init';t.transform.rotation.w=1.
        world_tf=t
        t=TransformStamped();t.header.stamp=self.get_clock().now().to_msg()
        t.header.frame_id='base_link';t.child_frame_id='livox_frame';t.transform.rotation.w=1.
        self.static.sendTransform([world_tf,t])
        self.cloud=self.create_publisher(PointCloud2,'/livox/lidar',10)
        self.odom=self.create_publisher(Odometry,'/Odometry',10)
        self.goal=self.create_publisher(PoseStamped,'/goal_pose',10)
        self.raw=self.create_publisher(Twist,'/legged/cmd_raw',10)
        topic='/rubi/planner_comparison/grid/tracking_path' if mode=='pursuit' else '/plan'
        qos=QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL) if mode=='pursuit' else 10
        self.create_subscription(RosPath,topic,lambda m:self.paths.append(m),qos)
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.commands.append((time.monotonic(),m.linear.x)),10)
        self.create_subscription(PointCloud2,'/legged/height',lambda m:self.heights.append(m.header.stamp),10)
        self.create_timer(.05,self.tick)

    def tick(self):
        now=self.get_clock().now().to_msg()
        t=TransformStamped();t.header.stamp=now;t.header.frame_id='camera_init';t.child_frame_id='base_link'
        t.transform.translation.x=1.;t.transform.translation.y=1.;t.transform.translation.z=.65
        t.transform.rotation.w=1.;self.tf.sendTransform(t)
        o=Odometry();o.header=t.header;o.child_frame_id='base_link';o.pose.pose.position.x=1.
        o.pose.pose.position.y=1.;o.pose.pose.position.z=.65;o.pose.pose.orientation.w=1.;self.odom.publish(o)
        if self.sensor_enabled:
            # Point coordinates are relative to the virtual upright sensor.
            # Far returns allow the costmap to ray-clear earlier synthetic obstacles.
            points=[[3.,dy,-.65] for dy in [-.5,0.,.5]]
            if self.obstacle: points += [[1.2+.05*i,dy,0.] for i in range(3) for dy in [-.15,0.,.15]]
            # Tall nearby returns remain outside the 0.6 m spherical sensor
            # filter while lying inside the configured stop rectangle in XY.
            if self.near: points += [[.2,dy,.65] for dy in [-.1,0.,.1,.15]]
            self.cloud.publish(create_cloud_xyz32(Header(stamp=now,frame_id='livox_frame'),points))

    def spin_for(self, seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(self,timeout_sec=.05)

    def require_active(self, name):
        client=self.create_client(GetState,name+'/get_state')
        assert client.wait_for_service(timeout_sec=5),name+' missing'
        f=client.call_async(GetState.Request());rclpy.spin_until_future_complete(self,f,timeout_sec=5)
        assert f.done() and f.result().current_state.id==3,name+' is not active'

    def parameters(self, name, names):
        client=self.create_client(GetParameters,name+'/get_parameters')
        assert client.wait_for_service(timeout_sec=5),name+' parameters unavailable'
        f=client.call_async(GetParameters.Request(names=names))
        rclpy.spin_until_future_complete(self,f,timeout_sec=5)
        assert f.done() and f.result(),name+' parameter request failed'
        return f.result().values

    def target(self):
        g=PoseStamped();g.header.frame_id='map';g.header.stamp=self.get_clock().now().to_msg()
        g.pose.position.x=4.;g.pose.position.y=1.;g.pose.orientation.w=1.;return g

    def plan(self):
        if self.mode=='pursuit':
            before=len(self.paths);self.goal.publish(self.target());self.spin_for(4)
            candidates=[p for p in self.paths[before:] if p.poses]
            assert candidates,'Pure Pursuit route was not published';return candidates[-1]
        client=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        assert client.wait_for_server(timeout_sec=10),'Planner action unavailable'
        goal=ComputePathToPose.Goal();goal.goal=self.target();goal.planner_id='Height';goal.use_start=False
        f=client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self,f,timeout_sec=10)
        assert f.done() and f.result().accepted,'ComputePath rejected'
        result=f.result().get_result_async();rclpy.spin_until_future_complete(self,result,timeout_sec=12)
        assert result.done() and result.result().status==4,'ComputePath failed'
        assert result.result().result.path.poses
        return result.result().result.path


def main():
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['nav2','pursuit'])
    parser.add_argument('--controller', choices=['dwb','rpp'], default='dwb');args=parser.parse_args()
    domain=int(os.environ.get('ROS_DOMAIN_ID','0'))
    if domain not in (173,174,175):raise SystemExit('Set ROS_DOMAIN_ID=174 or 175; robot domains are refused')
    out=ROOT/'validation';out.mkdir(exist_ok=True)
    label=args.mode+('_rpp' if args.controller=='rpp' else '')
    log=(out/(label+'_launch.log')).open('w')
    rclpy.init();node=Inputs(args.mode)
    proc=subprocess.Popen(['ros2','launch','legged_bringup','navigate.launch.py',
        'site:='+str(ROOT/'maps/demo/site.yaml'),'mode:='+args.mode,
        'controller:='+args.controller,'rviz:=false'],
        stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        node.spin_for(8);assert proc.poll() is None,'Launch exited'
        node.require_active('/collision_monitor')
        node.require_active('/map_server')
        points=node.parameters('/collision_monitor',['Stop.points'])[0].double_array_value
        half_length,half_width=rectangle_half_extents(points)
        guard=node.parameters('/command_watchdog',['footprint_half_length','footprint_half_width'])
        assert abs(guard[0].double_value-half_length)<1.e-9
        assert abs(guard[1].double_value-half_width)<1.e-9
        planner_node='/rubi_hybrid_planner_comparison' if args.mode=='pursuit' else '/planner_server'
        clearance_name='costmap_clearance_m' if args.mode=='pursuit' else 'Height.costmap_clearance_m'
        clearance=node.parameters(planner_node,[clearance_name])[0].double_value
        # Launch must honor the planner profile, not enlarge it to the stop
        # rectangle's circumcircle (which closed real narrow corridors).
        expected_clearance=yaml.safe_load((ROOT/'src/legged_bringup/config/planner.yaml').read_text())['costmap_clearance_m']
        assert abs(clearance-expected_clearance)<1.e-9
        if args.mode=='nav2':
            node.require_active('/bt_navigator')
            node.require_active('/controller_server')
            node.require_active('/local_costmap/local_costmap')
        original=node.plan()
        if args.mode=='nav2':
            client=ActionClient(node,NavigateToPose,'/navigate_to_pose')
            assert client.wait_for_server(timeout_sec=5)
            g=NavigateToPose.Goal();g.pose=node.target()
            f=client.send_goal_async(g);rclpy.spin_until_future_complete(node,f,timeout_sec=5)
            assert f.done() and f.result().accepted
            node.spin_for(3)
            assert any(v>.02 for t,v in node.commands if t>time.monotonic()-2),'Nav2 did not produce motion'
            cancel=f.result().cancel_goal_async();rclpy.spin_until_future_complete(node,cancel,timeout_sec=5)
        else:
            assert any(v>.02 for t,v in node.commands if t>time.monotonic()-2),'Pursuit did not produce motion'
        before=len(node.paths);node.obstacle=True;node.spin_for(5)
        if args.mode=='pursuit':
            changed=[p for p in node.paths[before:] if p.poses]
            assert changed,'No automatic obstacle replan';rerouted=changed[-1]
        else:rerouted=node.plan()
        deviation=max(abs(p.pose.position.y-1.) for p in rerouted.poses)
        assert deviation>.3, f'Route did not detour around obstacle: {deviation}'
        assert node.heights and all(s.sec==1 and s.nanosec==0 for s in node.heights), 'Frozen stamps changed'
        # Direct raw commands test the common collision-monitor + watchdog stage.
        node.near=True
        for _ in range(30):
            t=Twist();t.linear.x=.2;node.raw.publish(t);node.spin_for(.1)
        assert node.commands and max(abs(v) for t,v in node.commands if t>time.monotonic()-.4)<1e-8,'Near obstacle did not stop output'
        node.sensor_enabled=False
        for _ in range(15):
            t=Twist();t.linear.x=.2;node.raw.publish(t);node.spin_for(.1)
        assert max(abs(v) for t,v in node.commands if t>time.monotonic()-.4)<1e-8,'Stale sensor did not stop output'
        result={'mode':args.mode,'controller':args.controller,'first_path_points':len(original.poses),'rerouted_points':len(rerouted.poses),
            'detour_max_y_m':deviation,'frozen_acquisition_stamp_preserved':True,
            'shared_stop_length_m':2*half_length,'shared_stop_width_m':2*half_width,
            'planner_obstacle_clearance_m':clearance,
            'max_forward_command_mps':max(v for _,v in node.commands),
            'controller_produced_motion':True,'near_obstacle_stop':True,'stale_sensor_stop':True,'domain':domain}
        (out/(label+'_smoke.json')).write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result))
    finally:
        # Let ros2 launch forward one SIGINT to its nodes. Signalling the whole
        # group as well causes duplicate interrupts during rclpy teardown.
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        log.close();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
