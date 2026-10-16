"""Actual GPU + wrapper: timestamped odometry, mounting TF, conflicting dynamic TF."""
from pathlib import Path
import os, signal, subprocess, tempfile, time, unittest
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from rubi_height_mapping.pointcloud import pointcloud_xyz


class DirectOdometryPipeline(unittest.TestCase):
    def test_direct_pose_interpolation_mounts_and_rejections(self):
        os.environ['ROS_DOMAIN_ID']='184'
        rclpy.init(); node=rclpy.create_node('rubi_odom_input_test')
        with tempfile.TemporaryDirectory(prefix='rubi_odom_test_') as td:
            logpath=Path(td)/'launch.log'
            clock=node.create_publisher(Clock,'/clock',10)
            scan=node.create_publisher(PointCloud2,'/livox/lidar_PointCloud2',qos_profile_sensor_data)
            odom=node.create_publisher(Odometry,'/diagnostic/odom',qos_profile_sensor_data)
            static=node.create_publisher(TFMessage,'/tf_static',QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            dynamic=node.create_publisher(TFMessage,'/tf',10)
            maps=[]
            node.create_subscription(PointCloud2,'/rubi/global_elevation/cloud',maps.append,10)
            with logpath.open('w') as log:
                proc=subprocess.Popen(['ros2','launch','rubi_height_mapping','mapping.launch.py',
                    'use_sim_time:=true','rviz:=false','ray_mode:=voxel','odom_topic:=/diagnostic/odom',
                    'map_frame:=world','base_frame:=gt_body','tf_wait_timeout:=0.7'],
                    stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                now=100.
                def stamp(t):
                    ns=round(t*1e9); return Time(sec=ns//10**9,nanosec=ns%10**9)
                def pump(seconds):
                    nonlocal now
                    end=time.monotonic()+seconds
                    while time.monotonic()<end:
                        self.assertIsNone(proc.poll(),logpath.read_text())
                        now+=.01; clock.publish(Clock(clock=stamp(now)))
                        rclpy.spin_once(node,timeout_sec=.02)
                def pose(t,p,q,parent='world'):
                    m=Odometry();m.header.stamp=stamp(t);m.header.frame_id=parent;m.child_frame_id='gt_body'
                    m.pose.pose.position.x,m.pose.pose.position.y,m.pose.pose.position.z=map(float,p)
                    m.pose.pose.orientation.x,m.pose.pose.orientation.y,m.pose.pose.orientation.z,m.pose.pose.orientation.w=map(float,q)
                    odom.publish(m)
                # The scan is expressed in an upside-down sensor with a 10 cm
                # forward mount. Correct midpoint base pose is [0.5,0.2,0.8], yaw 45 deg.
                axis=np.arange(-.9,.91,.05); x,y=np.meshgrid(axis+.5,axis+.2)
                z=np.where(x<.25,0,np.where(x<.75,.05,.15))
                world=np.column_stack([x.ravel(),y.ravel(),z.ravel()])
                c=2**-.5;rotation=np.array([[c,c,0],[c,-c,0],[0,0,-1]])
                xyz=((world-[.5+.1*c,.2+.1*c,.9])@rotation).astype('<f4')
                cloud=PointCloud2();cloud.header.frame_id='test_lidar';cloud.height=1;cloud.width=len(xyz)
                cloud.point_step=12;cloud.row_step=12*len(xyz);cloud.is_dense=True;cloud.data=xyz.tobytes()
                cloud.fields=[PointField(name=n,offset=4*i,datatype=7,count=1) for i,n in enumerate('xyz')]
                try:
                    end=time.monotonic()+25
                    while not scan.get_subscription_count() and time.monotonic()<end:pump(.1)
                    self.assertGreater(scan.get_subscription_count(),0,logpath.read_text())
                    mount=TransformStamped();mount.header.frame_id='gt_body';mount.child_frame_id='test_lidar'
                    mount.transform.translation.x=.1;mount.transform.translation.z=.1
                    mount.transform.rotation.x=1.;mount.transform.rotation.w=0.
                    static.publish(TFMessage(transforms=[mount]));pump(.25)
                    for _ in range(8):
                        now+=.3;t=now;cloud.header.stamp=stamp(t)
                        pose(t-.05,[.4,.2,.8],[0,0,0,1])
                        # Wrong dynamic TF would displace the entire surface by 20 m.
                        bad=TransformStamped();bad.header.frame_id='world';bad.header.stamp=stamp(t)
                        bad.child_frame_id='test_lidar';bad.transform.translation.z=20.;bad.transform.rotation.w=1.
                        dynamic.publish(TFMessage(transforms=[bad]));scan.publish(cloud);pump(.12)
                        self.assertFalse(any(m.header.stamp==stamp(t) for m in maps),'Fused without future odometry')
                        pose(t+.05,[.6,.2,.8],[0,0,1,1])
                        deadline=time.monotonic()+5
                        while not any(m.header.stamp==stamp(t) for m in maps) and time.monotonic()<deadline:pump(.05)
                        self.assertTrue(any(m.header.stamp==stamp(t) for m in maps),logpath.read_text())
                    result=maps[-1];self.assertEqual(result.header.frame_id,'world')
                    points=pointcloud_xyz(result)
                    self.assertGreater(len(points),100)
                    for lo,hi,height in [(-.3,.1,0),(.4,.6,.05),(.9,1.2,.15)]:
                        values=points[(points[:,0]>lo)&(points[:,0]<hi)&(abs(points[:,1]-.2)<.5),2]
                        self.assertGreater(len(values),20,f'ROI {lo,hi}, cloud bounds {points.min(0)} .. {points.max(0)}')
                        self.assertLess(abs(np.median(values)-height),.005)
                    # Neither bad odometry frames nor a fresh dynamic TF may fill a missing bracket.
                    now+=.4;t=now;cloud.header.stamp=stamp(t)
                    pose(t,[0,0,0],[0,0,0,1],parent='wrong_frame')
                    bad.header.stamp=stamp(t);dynamic.publish(TFMessage(transforms=[bad]));scan.publish(cloud)
                    pump(1.2)
                    self.assertFalse(any(m.header.stamp==stamp(t) for m in maps))
                    self.assertIn('Rejected odometry',logpath.read_text())
                    self.assertIn('Stamped TF timeout',logpath.read_text())
                    now=1.;pump(.3);maps.clear();pump(.3)
                    self.assertFalse(maps)
                    self.assertIn('ROS time moved backwards',logpath.read_text())
                    print(f'Direct odometry passed: {len(points)} cells, interpolation, upside-down mounts, TF isolation, frame rejection, clock reset')
                finally:
                    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
                    try:proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    if os.environ.get('RUBI_ODOM_TEST_LOG'):Path(os.environ['RUBI_ODOM_TEST_LOG']).write_text(logpath.read_text())
                    node.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main()
