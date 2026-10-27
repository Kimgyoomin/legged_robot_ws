"""Offline provenance of the two-cell obstacle above the cube; no ROS publishers."""
from pathlib import Path
import pickle, json, xml.etree.ElementTree as E
import numpy as np
from scipy.spatial.transform import Rotation
from scipy.spatial import cKDTree
import rclpy
from rclpy.duration import Duration
from rclpy.serialization import deserialize_message as de
from tf2_ros import Buffer
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from nav2_msgs.msg import Costmap
from sensor_msgs.msg import PointCloud2

P=Path(__file__).parent; D=pickle.loads((P/'live/capture.pkl').read_bytes())
rclpy.init(); buf=Buffer(cache_time=Duration(seconds=40))
for topic,static in [('/tf_static',True),('/tf',False)]:
    for _,b in D[topic]:
        for t in de(b,TFMessage).transforms:
            (buf.set_transform_static if static else buf.set_transform)(t,'record')
def seconds(s): return s.sec+s.nanosec*1e-9
def matrix(p,q):
    m=np.eye(4);m[:3,:3]=Rotation.from_quat([q.x,q.y,q.z,q.w]).as_matrix();m[:3,3]=[p.x,p.y,p.z];return m
gt=[de(b,Odometry) for _,b in D['/ground_truth/odom']];ts=np.array([seconds(m.header.stamp) for m in gt])
rows=[];align=[];skew=[]
for _,b in D['/legged/collision_points']:
    m=de(b,PointCloud2);t=seconds(m.header.stamp);i=np.argmin(abs(ts-t))
    if abs(ts[i]-t)>.01:continue
    try:tr=buf.lookup_transform('map',m.header.frame_id,rclpy.time.Time.from_msg(m.header.stamp)).transform
    except Exception:continue
    tm=matrix(tr.translation,tr.rotation);tw=matrix(gt[i].pose.pose.position,gt[i].pose.pose.orientation)
    f={x.name:x.offset for x in m.fields};dt=np.dtype(dict(names=list('xyz'),formats=['<f4']*3,offsets=[f[x] for x in 'xyz'],itemsize=m.point_step))
    a=np.ndarray((m.height,m.width),dtype=dt,buffer=m.data,strides=(m.row_step,m.point_step));xyz=np.column_stack([a[k].ravel() for k in 'xyz'])
    # Costmap marking is limited to 4 m from the sensor.
    xyz=xyz[np.linalg.norm(xyz-[.101,0,-.1169],axis=1)<4.]
    pm=xyz@tm[:3,:3].T+tm[:3,3];pw=xyz@tw[:3,:3].T+tw[:3,3]
    rows.append(np.column_stack([pm,pw,xyz,np.full(len(xyz),t)]));align.append(tm@np.linalg.inv(tw));skew.append(abs(ts[i]-t))
a=np.concatenate(rows);world=a[:,3:6];shapes=[]
def pose(text):
    v=list(map(float,(text or '0 0 0 0 0 0').split()));m=np.eye(4);m[:3,:3]=Rotation.from_euler('xyz',v[3:]).as_matrix();m[:3,3]=v[:3];return m
root=E.parse('/tmp/rubi_sensor_world_frp15jju/world.sdf').getroot()
for model in root.findall('world/model'):
    if model.get('name')=='RUBI':continue
    for link in model.findall('link'):
        for c in link.findall('collision'):
            g=c.find('geometry');tm=pose(model.findtext('pose'))@pose(link.findtext('pose'))@pose(c.findtext('pose'));kind=next(iter(g)).tag
            if kind=='box':size=np.array(list(map(float,g.findtext('box/size').split())))/2
            elif kind=='cylinder':size=np.array([float(g.findtext('cylinder/radius')),float(g.findtext('cylinder/length'))/2])
            elif kind=='plane':size=None
            else:continue
            shapes.append((model.get('name')+'::'+link.get('name'),kind,tm,size))
tm=np.eye(4);tm[:3,3]=[2.960624528171306,4.734544296403779,.5598924933607592];tm[:3,:3]=Rotation.from_quat([.06413827003669478,.000007020914401063,.0001092399588123455,.9979410154583478]).as_matrix()
shapes.append(('unit_box','box',tm,np.array([.5,.5,.5])))
best=np.full(len(a),np.inf);ids=np.full(len(a),-1)
for k,(name,kind,tm,size) in enumerate(shapes):
    q=(world-tm[:3,3])@tm[:3,:3]
    if kind=='plane':dist=abs(q[:,2])
    else:
        v=abs(q)-size if kind=='box' else np.column_stack([np.linalg.norm(q[:,:2],axis=1),abs(q[:,2])])-size
        dist=abs(np.linalg.norm(np.maximum(v,0),axis=1)+np.minimum(v.max(axis=1),0))
    use=dist<best;best[use]=dist[use];ids[use]=k
upper=np.linalg.norm(a[:,:2]-[8.95,5.25],axis=1)<.12
out=dict(samples=len(rows),points=len(a),gt_time_skew_max_s=max(skew),upper_roi_center_map=[8.95,5.25],upper_roi_radius_m=.12,upper_points=int(upper.sum()),upper_surface_matches={name:int((upper&(ids==k)&(best<.04)).sum()) for k,(name,*_) in enumerate(shapes) if np.any(upper&(ids==k))},upper_unmatched=int((upper&(best>=.04)).sum()))
if upper.any():
    out['upper_world_xyz_median']=np.median(world[upper],axis=0).tolist();out['upper_world_z_range']=[float(world[upper,2].min()),float(world[upper,2].max())];out['upper_body_z_range']=[float(a[upper,8].min()),float(a[upper,8].max())];out['upper_nearest_surface_error_max_m']=float(best[upper].max())
(P/'upper_obstacle.json').write_text(json.dumps(out,indent=2));np.savez_compressed(P/'provenance.npz',points=a,model=ids,surface_error=best,names=[s[0] for s in shapes],T_map_world=align[0]);print(json.dumps(out,indent=2));rclpy.shutdown()
