#!/usr/bin/env python3
"""Isolated profile/TF/input checks; synthetic odometry is not accuracy validation."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, PointCloud2
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer, TransformListener, StaticTransformBroadcaster

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--direct', action='store_true', help='Test the native FAST-LIO launch, including offscreen RViz')
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '177':
        raise SystemExit('Use isolated ROS_DOMAIN_ID=177')
    results = {}
    out = ROOT/'validation'/('direct_fastlio_20261019' if args.direct else 'localization_profiles_20261019')
    out.mkdir(parents=True, exist_ok=True)
    prefix = subprocess.check_output(['ros2','pkg','prefix','fast_lio_localization'], text=True).strip()
    assert Path(prefix) == ROOT/'install/fast_lio_localization', prefix
    cases = [('sim','fastlio')] if args.direct else [('hardware','fastlio'),('sim','fastlio'),('hardware','genz'),('sim','genz')]
    for profile, method in cases:
        label = profile+'_'+method
        rclpy.init(); node = rclpy.create_node('localization_profile_check')
        buffer = Buffer(); listener = TransformListener(buffer,node)
        synthetic_mount = StaticTransformBroadcaster(node)
        clock = node.create_publisher(Clock,'/clock',10)
        imu_pub = node.create_publisher(Imu,'/livox/imu',10)
        scan_pub = node.create_publisher(CustomMsg,'/livox/lidar',10) if label=='sim_fastlio' else None
        odoms, clouds, maps = [], [], []
        subscriptions = [node.create_subscription(Odometry,'/Odometry',odoms.append,10),
            node.create_subscription(PointCloud2,'/cloud_registered_body',clouds.append,10),
            node.create_subscription(PointCloud2,'/cloud_pcd',maps.append,10)]
        begin = time.monotonic()
        def stamp(sec):
            from builtin_interfaces.msg import Time
            ns = int(sec*1e9); return Time(sec=ns//10**9,nanosec=ns%10**9)
        def tick():
            if profile == 'sim':
                now = stamp(10.+time.monotonic()-begin)
                clock.publish(Clock(clock=now))
                if scan_pub:
                    imu = Imu(); imu.header.stamp=now; imu.header.frame_id='livox_frame'
                    imu.linear_acceleration.z=9.81; imu.orientation.w=1.
                    imu_pub.publish(imu)
        points = []
        for i in range(1200):
            angle=2*math.pi*i/1200; radius=2.+.3*math.sin(5*angle)
            points.append(CustomPoint(offset_time=i*80000,x=radius*math.cos(angle),
                y=radius*math.sin(angle),z=-.4+.4*(i%7),reflectivity=80,tag=16,line=i%4))
        def scan():
            if scan_pub:
                m=CustomMsg(); m.header.frame_id='livox_frame'
                m.header.stamp=stamp(9.9+time.monotonic()-begin)
                m.timebase=m.header.stamp.sec*10**9+m.header.stamp.nanosec
                m.points=points; m.point_num=len(points);scan_pub.publish(m)
        timers=[node.create_timer(.005,tick),node.create_timer(.1,scan)]
        if profile=='sim':
            mount=TransformStamped();mount.header.frame_id='BODY';mount.child_frame_id='livox_frame'
            mount.transform.translation.x=.101;mount.transform.translation.z=-.1169
            mount.transform.rotation.w=1.;synthetic_mount.sendTransform(mount)
        def spin(seconds):
            end=time.monotonic()+seconds
            while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
        def parameters(name, keys):
            client=node.create_client(GetParameters,name+'/get_parameters')
            assert client.wait_for_service(timeout_sec=3),name
            future=client.call_async(GetParameters.Request(names=keys))
            end=time.monotonic()+3
            while not future.done() and time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.01)
            assert future.done(),name
            return future.result().values
        with (out/(label+'.log')).open('w') as log:
            command=['ros2','launch','legged_bringup','localize.launch.py',
                'method:='+method,'profile:='+profile,'site:='+str(ROOT/'maps/demo/site.yaml')]
            env=os.environ.copy()
            if args.direct:
                command=['ros2','launch','fast_lio_localization','localization_nav_rubi.launch.py',
                    'pcd_map:='+str(ROOT/'maps/sim_site_v1/localization.pcd'),
                    'use_sim_time:=true','publish_2d_map:=false','use_rviz:=true']
                env.setdefault('QT_QPA_PLATFORM','offscreen')
            proc=subprocess.Popen(command,env=env,
                stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                spin(7.)
                names={name for name,_ in node.get_node_names_and_namespaces()}
                required={'laser_mapping','global_localization','transform_fusion'} if method=='fastlio' else {'genz_odometry','genz_localization'}
                assert required<=names,(label,names)
                root='body' if method=='fastlio' else 'base_link'
                tf=buffer.lookup_transform(root,'livox_frame',rclpy.time.Time())
                expected=(-.011,.02329,-.04412) if method=='fastlio' else (.101,0.,-.1169)
                xyz=[getattr(tf.transform.translation,k) for k in ('x','y','z')]
                assert all(abs(a-b)<1e-8 for a,b in zip(xyz,expected)),(label,xyz)
                assert abs(tf.transform.rotation.w-1.)<1e-8, 'Unexpected second sensor rotation'
                if profile=='sim':
                    assert 'BODY_to_livox_frame' not in names, 'Simulator mount duplicated'
                if args.direct:
                    assert 'rviz2_fastlio_localization' in names,names
                    assert parameters('/rviz2_fastlio_localization',['use_sim_time'])[0].bool_value
                    assert maps and maps[-1].width>0 and maps[-1].header.frame_id=='map', 'PCD map not published'
                    assert 'world_to_map' not in names,'Unrequested world/map alignment'
                    assert 'map_server' not in names,'Duplicate 2D map publisher'
                if method=='fastlio':
                    v=parameters('/laser_mapping',['preprocess.lidar_type','use_sim_time'])
                    assert v[0].integer_value==(1 if profile=='sim' else 4)
                    assert v[1].bool_value==(profile=='sim')
                    endpoints=node.get_subscriptions_info_by_topic('/livox/lidar')
                    actual={e.topic_type for e in endpoints if e.node_name=='laser_mapping'}
                    expected_type='livox_ros_driver2/msg/CustomMsg' if profile=='sim' else 'sensor_msgs/msg/PointCloud2'
                    assert actual=={expected_type},(label,actual)
                    if profile=='sim':
                        assert odoms and clouds, 'CustomMsg + IMU did not produce odometry/body cloud'
                else:
                    topic='/livox/lidar_PointCloud2' if profile=='sim' else '/livox/lidar'
                    endpoints=node.get_subscriptions_info_by_topic(topic)
                    assert {'genz_odometry','genz_localization'}<={e.node_name for e in endpoints}
                log.flush(); text=(out/(label+'.log')).read_text()
                assert proc.poll() is None and 'process has died' not in text and 'Traceback' not in text,text
                results[label]=dict(startup=True,tf_translation=xyz,local_package_prefix=prefix,
                    odometry_samples=len(odoms),body_cloud_samples=len(clouds),accuracy_tested=False)
                if args.direct:
                    results[label].update(rviz_started=True,qt_platform=env['QT_QPA_PLATFORM'],pcd_messages=len(maps),
                        pcd_points=maps[-1].width*maps[-1].height,pcd_frame=maps[-1].header.frame_id,
                        duplicate_map_server=False,world_map_identity=False)
                print(label,results[label],flush=True)
            finally:
                if proc.poll() is None:proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
                node.destroy_node();rclpy.shutdown()
    (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')


if __name__=='__main__':main()
