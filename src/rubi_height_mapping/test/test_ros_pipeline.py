"""Run the actual lean launch in an isolated DDS domain with delayed stamped TF."""
from pathlib import Path
import json,os,signal,subprocess,tempfile,time,unittest
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2,PointField
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from rosgraph_msgs.msg import Clock
from grid_map_msgs.msg import GridMap
from std_srvs.srv import Trigger

class Pipeline(unittest.TestCase):
    def test_mapping_save_restore_and_clock_reset(self):
        os.environ['ROS_DOMAIN_ID']='183'
        rclpy.init();node=rclpy.create_node('rubi_height_delivery_test')
        with tempfile.TemporaryDirectory(prefix='rubi_height_pipeline_') as td:
            directory=Path(td);logpath=directory/'launch.log'
            # Excluded modules must stay unnecessary even during ROS callbacks.
            (directory/'sitecustomize.py').write_text("import sys,importlib.abc\nclass Block(importlib.abc.MetaPathFinder):\n def find_spec(self,fullname,path=None,target=None):\n  if fullname.split('.')[0] in {'torch','chainer','cv_bridge','cv2','semantic_sensor','scipy','shapely','simple_parsing','elevation_mapping_cupy','elevation_map_msgs','ros2_numpy','rosbag2_py'}: raise ModuleNotFoundError('Excluded: '+fullname)\nsys.meta_path.insert(0,Block())\n")
            config=directory/'wrapper.yaml'
            config.write_text('/rubi_global_heightmap_wrapper:\n  ros__parameters:\n    input_topic: /rubi/local_elevation/grid_map\n    publish_fps: 10.0\n    output_directory: '+str(directory/'maps')+'\n    load_path: '+str(directory/'restore.rghm')+'\n')
            env=os.environ.copy()
            env['PYTHONPATH']=str(directory)+':'+':'.join(p for p in env.get('PYTHONPATH','').split(':') if '.python_deps' not in p)
            for key in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS'):env.pop(key,None)
            clock=node.create_publisher(Clock,'/clock',10);scan=node.create_publisher(PointCloud2,'/livox/lidar_PointCloud2',qos_profile_sensor_data);tf=node.create_publisher(TFMessage,'/tf',20)
            local=[];global_msgs=[]
            node.create_subscription(GridMap,'/rubi/local_elevation/grid_map',local.append,2)
            node.create_subscription(PointCloud2,'/rubi/global_elevation/cloud',global_msgs.append,2)
            with logpath.open('w') as log:
                proc=subprocess.Popen(['ros2','launch','rubi_height_mapping','mapping.launch.py','use_sim_time:=true','rviz:=false',f'wrapper_config:={config}'],env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
                try:
                    now=100.0
                    def stamp(t):
                        from builtin_interfaces.msg import Time
                        return Time(sec=int(t),nanosec=int(round((t-int(t))*1e9)))
                    def pump(seconds):
                        nonlocal now
                        end=time.monotonic()+seconds
                        while time.monotonic()<end:
                            self.assertIsNone(proc.poll(),logpath.read_text())
                            now += .01
                            c=Clock();c.clock=stamp(now);clock.publish(c)
                            rclpy.spin_once(node,timeout_sec=.02)
                    for _ in range(100):
                        pump(.05)
                        if scan.get_subscription_count():break
                    self.assertGreater(scan.get_subscription_count(),0,logpath.read_text())
                    first_width=None;counts=[]
                    for i in range(12):
                        now+=.3;t=stamp(now);base_x=0.0 if i<6 else 5.0
                        axis=np.arange(-1.2,1.2,.06,dtype=np.float32);x,y=np.meshgrid(axis,axis)
                        z=np.where(x<-.3,0,np.where(x<.3,.05,.15)).astype(np.float32)
                        # Sensor is upside down: R=diag(1,-1,-1), at world z=.8.
                        xyz=np.stack([x.ravel(),-y.ravel(),.8-z.ravel()],axis=1)
                        if i>=9:xyz=xyz[xyz[:,1]<0]  # Simulated loss of observation.
                        xyz=np.concatenate([xyz,np.array([[.1,0,0],[4.,0,0]],np.float32)])
                        msg=PointCloud2();msg.header.frame_id='test_lidar';msg.header.stamp=t;msg.height=1;msg.width=len(xyz);msg.point_step=12;msg.row_step=12*len(xyz);msg.is_dense=True
                        msg.fields=[PointField(name=n,offset=4*k,datatype=7,count=1) for k,n in enumerate('xyz')];msg.data=xyz.astype('<f4').tobytes()
                        scan.publish(msg);pump(.12)
                        transforms=[]
                        for child,zoff,roll in [('base_link',0.,False),('test_lidar',.8,True)]:
                            tr=TransformStamped();tr.header.frame_id='map';tr.header.stamp=t;tr.child_frame_id=child;tr.transform.translation.x=base_x;tr.transform.translation.z=zoff
                            if roll:tr.transform.rotation.x=1.;tr.transform.rotation.w=0.
                            else:tr.transform.rotation.w=1.
                            transforms.append(tr)
                        tf.publish(TFMessage(transforms=transforms));now+=.15;pump(.22)
                        deadline=time.monotonic()+8
                        seen=[m for m in global_msgs if m.header.stamp==t]
                        while not seen and time.monotonic()<deadline:
                            pump(.05)
                            seen=[m for m in global_msgs if m.header.stamp==t]
                        self.assertTrue(seen, f'local_stats={[(np.isfinite(m.data[0].data).sum(), np.nanmin(m.data[1].data), np.nanmax(m.data[1].data)) for m in local[-1:]]} local={[(m.header.stamp, m.layers) for m in local[-2:]]} global={[(m.header.stamp,m.width) for m in global_msgs[-2:]]} expected={t}\n'+logpath.read_text())
                        if seen:counts.append(seen[-1].width)
                        if i==5:
                            self.assertTrue(seen, f'local_stats={[(np.isfinite(m.data[0].data).sum(), np.nanmin(m.data[1].data), np.nanmax(m.data[1].data)) for m in local[-1:]]} local={[(m.header.stamp, m.layers) for m in local[-2:]]} global={[(m.header.stamp,m.width) for m in global_msgs[-2:]]} expected={t}\n'+logpath.read_text());first_width=seen[-1].width
                    pump(.2)
                    self.assertTrue(local,logpath.read_text());self.assertTrue(global_msgs,logpath.read_text())
                    self.assertEqual(local[-1].header.stamp,t)
                    self.assertEqual(global_msgs[-1].header.stamp,t)
                    self.assertEqual(local[-1].layers,['elevation','variance'])
                    self.assertGreater(global_msgs[-1].width,first_width*1.5)
                    self.assertTrue(all(b>=a for a,b in zip(counts,counts[1:])),counts)
                    def service(name):
                        c=node.create_client(Trigger,'/rubi/global_elevation/'+name);self.assertTrue(c.wait_for_service(timeout_sec=4))
                        f=c.call_async(Trigger.Request());end=time.monotonic()+5
                        while not f.done() and time.monotonic()<end:pump(.02)
                        self.assertTrue(f.done(),logpath.read_text());r=f.result();self.assertTrue(r.success,r.message);node.destroy_client(c);return r.message
                    saved=Path(service('save_map'));self.assertTrue((saved/'surface.pcd').is_file())
                    (directory/'restore.rghm').write_bytes((saved/'map.rghm').read_bytes())
                    service('clear_map');service('load_map')
                    restored=Path(service('save_map'))
                    self.assertEqual((saved/'map.rghm').read_bytes(),(restored/'map.rghm').read_bytes())
                    now=10.;pump(.3);local.clear();global_msgs.clear();pump(.3)
                    self.assertFalse(local);self.assertFalse(global_msgs)
                    self.assertIn('ROS time moved backwards',logpath.read_text())
                    self.assertNotIn('ModuleNotFoundError',logpath.read_text())
                    print(json.dumps({'initial_global_cells':first_width,'counts':counts,'clock_reset_stops_output':True,'save_load_exact':True,'excluded_imports_blocked':True}))
                finally:
                    if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
                    try:proc.wait(timeout=8)
                    except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
                    if os.environ.get('RUBI_TEST_LOG'):
                        Path(os.environ['RUBI_TEST_LOG']).write_text(logpath.read_text())
                    node.destroy_node();rclpy.shutdown()
if __name__=='__main__':unittest.main()
