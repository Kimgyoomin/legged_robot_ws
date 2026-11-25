"""V6: wall-bounded floor gap at (45.2512,46.7622), based on V5.

Usage: python3 repair_regions.py OUTPUT_DIRECTORY (must not exist).
Fill only unknown cells supported by current measured floor and legacy PCD.
All pre-existing records, walls and placed steps remain unchanged.
"""
import os,sys,json,shlex
from pathlib import Path
os.environ.setdefault('MPLCONFIGDIR','/tmp/height-target-mpl')
sys.path.insert(0,'/home/rclab/legged_robot_ws/src/legged_bringup')
from legged_bringup.map_io import read_snapshot,write_pcd
import numpy as np
from scipy import ndimage as ndi
from scipy.spatial import cKDTree
import yaml
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.path import Path as Polygon
from matplotlib.colors import ListedColormap

base=Path('/home/rclab/legged_robot_ws/maps/rubi_1f_height_20261109_targeted_floor_v5')
reference=Path('/home/rclab/legged_robot_ws/maps/processed/run_1794037249460292244_postprocess_20261107/audits/reference/reference_masks.npz')
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
s=read_snapshot(base/'map.rghm');z=s.grid;x,y=s.xy();r=s.resolution;valid=np.isfinite(z)
d=np.load(reference);p=np.load(base/'provenance.npz')
assert s.digest=='92aadca91c74a48ab22e28482097a65f9b6c459071f87e681ca88eea0b5046ea'
assert np.array_equal(d['offset'],s.offset) and d['old_free'].shape==z.shape
areas=[dict(id='E_wall_gap',click=[45.2512,46.7622],bounds=[44.35,46.25,46.25,47.05],operation='fill')]
labels=np.zeros(z.shape,np.uint8)
for k,a in enumerate(areas,1):
 b=a['bounds'];mask=(x>=b[0])&(x<=b[1])&(y>=b[2])&(y<=b[3])
 if 'polygon' in a:mask&=Polygon(a['polygon']).contains_points(np.c_[x.ravel(),y.ravel()]).reshape(z.shape)
 labels[mask]=k
step=d['step_protected']|p['v3_top_flattened']
# Data fill reaches the last FREE cell (5cm) adjacent to the occupancy wall.
# Planner clearance is unchanged. Occupied/unknown occupancy remains untouched.
safe=d['old_free']&(d['free_margin_m']>=.049)&~step
pcd=d['old_pcd_floor_adjusted_z']
pr=np.isfinite(pcd)&safe&(d['bias_support_distance_m']<=2.)
pd,pi=ndi.distance_transform_edt(~pr,sampling=r,return_indices=True);pz=pcd[tuple(pi)]
original=p['original_observed']&~p['confirmed_artifact_repaired']&~step
floor=original&valid&safe&(pd<=.30)&(abs(z-pz)<=.035)
nonfloor=valid&(pd<=.4)&(abs(z-pz)>.07)&~p['confirmed_artifact_repaired']
guard=ndi.distance_transform_edt(~ndi.binary_fill_holes(nonfloor),sampling=r)<.10
iy,ix=np.indices(z.shape)
def dataset(mask,arr):
 pts=np.c_[x[mask],y[mask]]
 return pts,arr[mask],cKDTree(pts)
current=dataset(floor&(iy%2==0)&(ix%2==0),z)
legacy=dataset(pr&(iy%2==0)&(ix%2==0),pcd)
def fit(q,data,min_n=20,exclude=0.):
 pts,h,tree=data;ds,ids=tree.query(q,k=min(250,len(h)),distance_upper_bound=1.3)
 ok=np.isfinite(ds)&(ds>exclude);ds=ds[ok];ids=ids[ok]
 if len(ids)<min_n:return None
 delta=pts[ids]-q;A=np.c_[np.ones(len(ids)),delta];b=h[ids];weights=1/(.12+ds)**2
 keep=np.ones(len(ids),bool)
 for _ in range(5):
  coef=np.linalg.lstsq(A[keep]*np.sqrt(weights[keep,None]),b[keep]*np.sqrt(weights[keep]),rcond=None)[0]
  residual=b-A@coef;med=np.median(residual[keep]);mad=np.median(abs(residual[keep]-med))
  keep=abs(residual-med)<=max(.010,3*1.4826*mad)
  if keep.sum()<min_n:return None
 coef=np.linalg.lstsq(A[keep]*np.sqrt(weights[keep,None]),b[keep]*np.sqrt(weights[keep]),rcond=None)[0]
 rms=np.sqrt(np.mean((b[keep]-A[keep]@coef)**2));slope=np.linalg.norm(coef[1:])
 if rms>.018 or slope>.08:return None
 ang=np.sort(np.arctan2(delta[keep,1],delta[keep,0]));gap=np.max(np.diff(np.r_[ang,ang[0]+2*np.pi]))
 return dict(z=float(coef[0]),coef=coef.tolist(),rms=float(rms),slope=float(slope),bracket=bool(gap<np.pi-.08),nearest=float(ds[keep].min()),n=int(keep.sum()),left=int((delta[keep,0]<-.15).sum()),right=int((delta[keep,0]>.15).sum()),below=int((delta[keep,1]<-.15).sum()))

new=z.copy();fills=np.zeros(z.shape,bool);repairs=fills.copy();rms=np.full(z.shape,np.nan,np.float32);difference=rms.copy();models=[];rejected={};small_hole_fill=fills.copy()
fill_candidates=(labels>0)&(labels!=3)&~valid&safe&~guard&(pd<=.30)
for j,i in zip(*np.where(fill_candidates)):
 q=[x[j,i],y[j,i]];f=fit(q,current);rf=fit(q,legacy)
 why=None
 if f is None or rf is None:why='unsupported_plane'
 elif abs(f['z']-rf['z'])>.035 or abs(f['z']-pz[j,i])>.04:why='surface_disagreement'
 elif not f['bracket'] and not (rf['bracket'] and f['nearest']<=.65):
  # This user-identified gap ends at a wall. Requiring observations across the
  # wall is impossible: accept closely supported floor from left/right/below
  # only with tighter current-plane and independent legacy-plane agreement.
  wall_supported=(d['free_margin_m'][j,i]<=.4 and f['nearest']<=.4 and f['rms']<=.012 and f['slope']<=.05 and abs(f['z']-rf['z'])<=.025 and f['left']>=5 and f['right']>=5 and f['below']>=10)
  if not wall_supported:why='one_sided_support'
 if why:rejected[why]=rejected.get(why,0)+1;continue
 new[j,i]=f['z'];fills[j,i]=True;rms[j,i]=f['rms'];difference[j,i]=abs(f['z']-rf['z'])
 models.append(dict(region=int(labels[j,i]),x=float(q[0]),y=float(q[1]),operation='fill',current=f,legacy=rf))



changed=fills
assert fills.any() and not repairs.any()
assert np.array_equal(new[valid],z[valid])
assert not np.any(fills&(~safe|step))
assert np.array_equal(np.isfinite(new),valid|fills)

# A withheld observation validation of this exact local interpolation method.
rng=np.random.default_rng(202611091516);validation=[]
jj,ii=np.where(floor&(x>=44.35)&(x<=46.25)&(y>=45.9)&(y<=46.8))
for radius in [.15,.30]:
 for n in rng.choice(len(ii),min(30,len(ii)),replace=False):
  j,i=jj[n],ii[n];q=[x[j,i],y[j,i]];f=fit(q,current,exclude=radius);rf=fit(q,legacy)
  support=f is not None and rf is not None
  if support:
   ordinary=f['bracket'] or (rf['bracket'] and f['nearest']<=.65)
   wall=(d['free_margin_m'][j,i]<=.4 and f['nearest']<=.4 and f['rms']<=.012 and f['slope']<=.05 and abs(f['z']-rf['z'])<=.025 and f['left']>=5 and f['right']>=5 and f['below']>=10)
   support=abs(f['z']-rf['z'])<=.035 and (ordinary or wall)
  validation.append(dict(radius_m=radius,x=float(q[0]),y=float(q[1]),accepted=bool(support),error_m=float(f['z']-z[j,i]) if support else None))

lines=s.text.splitlines();header=shlex.split(lines[1]);header[-1]=str(len(s.index)+int(fills.sum()));nx=s.dimensions[0]
result=[lines[0],' '.join(header)]+lines[2:]
existing=set(s.index.tolist())
for j,i in zip(*np.where(fills)):
 idx=(j+s.offset[1])*nx+i+s.offset[0];assert idx not in existing
 result.append(f'{idx} {new[j,i]:.17g} 0.0009 {s.stamp_ns} 1')
(out/'map.rghm').write_text('\n'.join(result)+'\n');saved=read_snapshot(out/'map.rghm')
assert saved.offset==s.offset and saved.grid.shape==z.shape and np.allclose(saved.grid,new,equal_nan=True)
assert result[2:2+len(s.index)]==lines[2:]
write_pcd(s,new,out/'surface.pcd')
prov={k:p[k] for k in p.files};prov.update(inferred_floor=p['inferred_floor']|fills,candidate_height=new,v6_filled=fills,v6_before=z,v6_region_labels=labels,v6_fit_rms=rms,v6_reference_agreement=difference)
np.savez_compressed(out/'provenance.npz',**prov)
cx,cy=areas[0]['click'];cj=int((cy-s.origin[1])/r)-s.offset[1];ci=int((cx-s.origin[0])/r)-s.offset[0]
edges=[]
for dj,di in [(1,0),(-1,0),(0,1),(0,-1)]:
 neigh=ndi.shift(new,[dj,di],order=0,mode='constant',cval=np.nan,prefilter=False)
 old=ndi.shift(valid,[dj,di],order=0,mode='constant',cval=False,prefilter=False)
 m=fills&old&np.isfinite(neigh);edges.extend(abs(new[m]-neigh[m]).tolist())
errors=[abs(t['error_m']) for t in validation if t['accepted']]
metrics=dict(base_map=str(base/'map.rghm'),base_sha256=s.digest,candidate_sha256=saved.digest,region=areas[0],base_usable_cells=int(valid.sum()),result_usable_cells=int(np.isfinite(new).sum()),filled_cells=int(fills.sum()),filled_area_m2=float(fills.sum()*r*r),repaired_existing_cells=0,preserved_existing_records=len(s.index),click_before=None,click_after=float(new[cj,ci]),click_occupancy_margin_m=float(d['free_margin_m'][cj,ci]),region_free_unknown_before=int(((labels>0)&d['old_free']&~valid).sum()),region_free_unknown_after=int(((labels>0)&d['old_free']&~np.isfinite(new)).sum()),region_occupied_unknown_preserved=int(((labels>0)&d['old_occupied']&~valid).sum()),fit_rms_p50_p95_max_mm=(np.percentile(rms[fills],[50,95,100])*1000).tolist(),reference_agreement_p50_p95_max_mm=(np.percentile(difference[fills],[50,95,100])*1000).tolist(),join_abs_difference_p50_p95_max_mm=(np.percentile(edges,[50,95,100])*1000).tolist(),holdout_trials=len(validation),holdout_accepted=len(errors),holdout_abs_error_p50_p95_max_mm=(np.percentile(errors,[50,95,100])*1000).tolist(),holdout=validation,step_changes=0,occupied_or_unknown_changes=0,rejected=rejected)
(out/'metrics.json').write_text(json.dumps(metrics,indent=2));(out/'fill_support.json').write_text(json.dumps(models))
site=yaml.safe_load((base/'site.yaml').read_text());site['id']='rubi-1f-height-wall-gap-v6-20261109'
for k in ['pcd','occupancy','image']:site['assets'][k]['path']=str((base/site['assets'][k]['path']).resolve())
site['assets']['height']={'path':'map.rghm','sha256':saved.digest}
site['alignment']['evidence']=str(reference.with_name('reference_audit.json'))
site['alignment']['method']='Inherited V5 XY alignment. User-designated wall-adjacent free-space gap filled from measured local floor with independent legacy PCD checks; all existing heights preserved.'
site['provenance'].update(height='V5 plus bounded wall-adjacent floor gap; all pre-existing heights preserved',base_map=str(base/'map.rghm'),base_sha256=s.digest,v5_report='../rubi_1f_height_20261109_targeted_floor_v5/REPORT.md',v6_report='REPORT.md')
(out/'site.yaml').write_text(yaml.safe_dump(site,sort_keys=False))
extent=[x.min()-r/2,x.max()+r/2,y.min()-r/2,y.max()+r/2]
fig,axs=plt.subplots(1,3,figsize=(15,5.8),layout='constrained')
for ax,arr,title in zip(axs,[z,new,np.where(fills,1,np.nan)],['Before: V5','After: V6','Cyan = inferred floor; black = occupancy wall']):
 ax.imshow(np.where(d['old_occupied'],0,np.where(d['old_free'],1,.5)),origin='lower',extent=extent,cmap='gray',vmin=0,vmax=1)
 if ax is axs[2]:
  ax.imshow(np.where(valid,1,np.nan),origin='lower',extent=extent,cmap=ListedColormap(['#c4bdb4']))
  ax.imshow(arr,origin='lower',extent=extent,cmap=ListedColormap(['#00bbc9']))
 else:im=ax.imshow(arr,origin='lower',extent=extent,vmin=-.49,vmax=-.30,cmap='viridis')
 ax.contour(x,y,d['old_occupied'],[.5],colors=['black'],linewidths=.7)
 ax.plot(cx,cy,'rx');ax.set(xlim=[44.,46.7],ylim=[45.8,47.3],title=title,aspect='equal',xlabel='map X [m]',ylabel='map Y [m]')
fig.colorbar(im,ax=axs[:2],shrink=.6,label='Height [m], clipped for display')
fig.suptitle(f"Wall gap: {int(fills.sum())} filled cells / 0 existing heights changed")
fig.savefig(out/'wall_gap_before_after.png',dpi=150)
print(json.dumps({k:v for k,v in metrics.items() if k!='holdout'},indent=2),flush=True)
