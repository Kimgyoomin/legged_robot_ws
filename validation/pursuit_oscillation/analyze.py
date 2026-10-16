import json,numpy as np
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from scipy.spatial import cKDTree
root=Path(__file__).parent;d=json.loads((root/'samples.json').read_text());groups={t:[v for v in d if v['topic']==t] for t in set(v['topic'] for v in d)}
loc=groups.get('/localization',[]);gt=groups.get('/ground_truth/odom',[]);paths=groups.get('/rubi/planner_comparison/grid/tracking_path',[])
def yaw(v):
 x,y,z,w=v['q'];return np.arctan2(2*(w*z+x*y),1-2*(y*y+z*z))
fig,ax=plt.subplots(2,2,figsize=(13,9));metrics={'samples':{k:len(v) for k,v in groups.items()},'path_messages':len(paths),'nonempty_paths':sum(bool(p['points']) for p in paths)}
for p in paths:
 if p['points']:
  a=np.array(p['points']);ax[0,0].plot(a[:,0],a[:,1],color='gray',alpha=.5)
if loc:
 xy=np.array([v['xyz'][:2] for v in loc]);tt=np.array([v['wall'] for v in loc]);ys=np.unwrap([yaw(v) for v in loc]);ax[0,0].plot(xy[:,0],xy[:,1],label='Localization (IMU body)',color='tab:blue');metrics['localization_travel_m']=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
 ax[0,1].plot(tt,np.degrees(ys-ys[0]),label='Localization yaw change')
 if gt:
  stamps=np.array([v['stamp'] for v in loc]);gs=np.array([v['stamp'] for v in gt]);gy=np.unwrap([yaw(v) for v in gt]);interp=np.interp(stamps,gs,gy);valid=(stamps>=gs[0])&(stamps<=gs[-1]);res=np.angle(np.exp(1j*(ys-interp)));res=np.unwrap(res);res-=np.median(res[valid]) if valid.any() else np.median(res)
  ax[0,1].plot(tt,np.degrees(interp-interp[0]),label='GT yaw change (different frame)');metrics['yaw_difference_demeaned_p95_deg']=float(np.percentile(np.abs(np.degrees(res[valid])),95)) if valid.any() else None
  ax[1,1].plot(tt[valid],np.degrees(res[valid]),label='LIO-GT yaw, constant offset removed')
 cte=[]
 for v in loc:
  ps=[p for p in paths if p['wall']<=v['wall'] and p['points']]
  if ps:
   a=np.array(ps[-1]['points']);dist,i=cKDTree(a).query(v['xyz'][:2]);cte.append(float(dist))
 if cte:metrics['body_to_nearest_path_point_p50_p95_m']=np.percentile(cte,[50,95]).tolist()
for topic in ['/legged/cmd_raw','/legged/cmd_safe','/cmd_vel']:
 vs=groups.get(topic,[])
 if not vs:continue
 t=[v['wall'] for v in vs];w=np.array([v['wz'] for v in vs]);x=np.array([v['vx'] for v in vs]);ax[1,0].plot(t,w,label=topic,alpha=.8);metrics[topic]={'vx_median':float(np.median(x)),'zero_command_fraction':float(np.mean((abs(x)<.001)&(abs(w)<.001))),'wz_p05_p95':np.percentile(w,[5,95]).tolist()}
ax[0,0].set(title='Frozen path(s) and measured body trajectory',xlabel='map x [m]',ylabel='map y [m]');ax[0,0].axis('equal')
ax[0,1].set(title='Yaw evolution',xlabel='wall time [s]',ylabel='degrees relative to first sample')
ax[1,0].set(title='Yaw-rate commands before/after guards',xlabel='wall time [s]',ylabel='rad/s')
ax[1,1].set(title='Yaw discrepancy (not absolute alignment error)',xlabel='wall time [s]',ylabel='degrees')
for a in ax.flat:a.legend(fontsize=8);a.grid(alpha=.2)
fig.tight_layout();fig.savefig(root/'comparison.png',dpi=150);(root/'metrics.json').write_text(json.dumps(metrics,indent=2));print(json.dumps(metrics,indent=2))
