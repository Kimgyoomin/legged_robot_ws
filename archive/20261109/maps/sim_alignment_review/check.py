from pathlib import Path
import sys,json
sys.path.insert(0,'/home/kim/legged_robot_ws/src/legged_bringup')
from legged_bringup.map_io import read_snapshot
import numpy as np,yaml
from PIL import Image
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
s=read_snapshot('/home/kim/legged_robot_ws/maps/sim_lio_snapshot/run_1792052351250393480/map.rghm')
op=Path('/home/kim/ros2_ws_nav/src/navigation_real/pongbot_navigation/maps/RUBI_occupancy_map.yaml')
o=yaml.safe_load(op.read_text()); im=np.flipud(np.asarray(Image.open(op.parent/o['image'])))
p=Path('/home/kim/ros2_ws_nav/src/navigation_real/FAST_LIO_ROS2/PCD/2026_07_22_10_43_57.pcd').read_bytes();raw=p.split(b'DATA binary\n',1)[1];pc=np.frombuffer(raw,dtype='<f4').reshape(-1,8)[:,:3];pc=pc[np.isfinite(pc).all(axis=1)]
x,y=s.xy();ok=np.isfinite(s.grid);x=x[ok];y=y[ok];z=s.grid[ok]
ix=np.floor((x-o['origin'][0])/o['resolution']).astype(int);iy=np.floor((y-o['origin'][1])/o['resolution']).astype(int);inside=(ix>=0)&(ix<im.shape[1])&(iy>=0)&(iy<im.shape[0]);v=im[iy[inside],ix[inside]]
metrics={'height_cells':len(z),'height_bounds':[[float(x.min()),float(x.max())],[float(y.min()),float(y.max())]],'height_inside_occupancy_fraction':float(inside.mean()),'inside_cells_free_fraction':float((v>191).mean()),'inside_cells_occupied_fraction':float((v<90).mean()),'inside_cells_unknown_fraction':float(((v>=90)&(v<=191)).mean()),'note':'Footprint overlap screening only; not landmark registration or height accuracy certification.'}
Path('/home/kim/legged_robot_ws/maps/sim_alignment_review/metrics.json').write_text(json.dumps(metrics,indent=2))
fig,axs=plt.subplots(1,2,figsize=(14,7),sharex=True,sharey=True)
ext=[o['origin'][0],o['origin'][0]+im.shape[1]*o['resolution'],o['origin'][1],o['origin'][1]+im.shape[0]*o['resolution']]
for a in axs:a.imshow(im,cmap='gray',origin='lower',extent=ext,vmin=0,vmax=255);a.set_aspect('equal');a.set_xlabel('map x [m]');a.set_ylabel('map y [m]')
axs[0].scatter(pc[::2,0],pc[::2,1],s=.25,c='dodgerblue',alpha=.4);axs[0].set_title('Existing occupancy + localization PCD (blue)')
q=axs[1].scatter(x,y,c=z,s=.35,cmap='viridis',vmin=np.percentile(z,2),vmax=np.percentile(z,98));axs[1].set_title('Existing occupancy + frozen height (no transform)');fig.colorbar(q,ax=axs[1],label='height [m]',shrink=.6)
fig.tight_layout();fig.savefig('/home/kim/legged_robot_ws/maps/sim_alignment_review/overlay.png',dpi=160)
print(json.dumps(metrics))
from scipy.ndimage import distance_transform_edt
D=distance_transform_edt(im<90)*o['resolution']
d=D[iy[inside],ix[inside]]
print('distance inside occupied cells quantiles',np.percentile(d[d>0],[50,95,100]))
print('pcd bounds',pc.min(0),pc.max(0))
