"""Bounded manual floor repairs, using V4 + legacy occupancy/PCD evidence.

Usage: python3 repair_regions.py OUTPUT_DIRECTORY (must not exist).
Unknown cells remain unknown unless both current observations and legacy floor
support the height. Existing height changes are confined to user-confirmed C.
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

base=Path('/home/rclab/legged_robot_ws/maps/rubi_1f_height_20261109_floorcomplete_v4')
reference=Path('/home/rclab/legged_robot_ws/maps/processed/run_1794037249460292244_postprocess_20261107/audits/reference/reference_masks.npz')
out=Path(sys.argv[1]);out.mkdir(parents=True,exist_ok=False)
s=read_snapshot(base/'map.rghm');z=s.grid;x,y=s.xy();r=s.resolution;valid=np.isfinite(z)
d=np.load(reference);p=np.load(base/'provenance.npz')
assert s.digest=='8a6fb20d85292cbfa3a51de7c345fc2fe5829dba4b87f926fec2e1af292fde30'
assert np.array_equal(d['offset'],s.offset) and d['old_free'].shape==z.shape
areas=[
 dict(id='A_hall',click=[13.406,43.5112],bounds=[11.15,15.5,42.45,45.5],operation='fill'),
 dict(id='B_entrance_fill',click=[5.92883,4.87223],bounds=[5.5,7.1,4.5,5.65],operation='fill'),
 dict(id='C_entrance_flat',click=[5.11579,-.0449459],bounds=[4.4,6.12,-.72,1.1],operation='flatten_artifacts',polygon=[[4.42,-.65],[5.78,-.72],[6.12,-.35],[6.1,.55],[5.6,1.1],[4.75,1.05],[4.4,.45]]),
 dict(id='D_upper',click=[31.0846,51.578],bounds=[29.7,32.2,50.7,53.6],operation='fill')]
labels=np.zeros(z.shape,np.uint8)
for k,a in enumerate(areas,1):
 b=a['bounds'];mask=(x>=b[0])&(x<=b[1])&(y>=b[2])&(y<=b[3])
 if 'polygon' in a:mask&=Polygon(a['polygon']).contains_points(np.c_[x.ravel(),y.ravel()]).reshape(z.shape)
 labels[mask]=k
step=d['step_protected']|p['v3_top_flattened']
# A 10-cm data-edit margin is NOT a robot clearance change: all planner
# footprint/clearance settings remain unchanged. Occupied/unknown cells untouched.
safe=d['old_free']&(d['free_margin_m']>=.10)&~step
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
 return dict(z=float(coef[0]),coef=coef.tolist(),rms=float(rms),slope=float(slope),bracket=bool(gap<np.pi-.08),nearest=float(ds[keep].min()),n=int(keep.sum()))

new=z.copy();fills=np.zeros(z.shape,bool);repairs=fills.copy();rms=np.full(z.shape,np.nan,np.float32);difference=rms.copy();models=[];rejected={};small_hole_fill=fills.copy()
fill_candidates=(labels>0)&(labels!=3)&~valid&safe&~guard&(pd<=.30)
for j,i in zip(*np.where(fill_candidates)):
 q=[x[j,i],y[j,i]];f=fit(q,current);rf=fit(q,legacy)
 why=None
 if f is None or rf is None:why='unsupported_plane'
 elif abs(f['z']-rf['z'])>.035 or abs(f['z']-pz[j,i])>.04:why='surface_disagreement'
 elif not f['bracket'] and not (rf['bracket'] and f['nearest']<=.65):why='one_sided_support'
 if why:rejected[why]=rejected.get(why,0)+1;continue
 new[j,i]=f['z'];fills[j,i]=True;rms[j,i]=f['rms'];difference[j,i]=abs(f['z']-rf['z'])
 models.append(dict(region=int(labels[j,i]),x=float(q[0]),y=float(q[1]),operation='fill',current=f,legacy=rf))

# Small fully enclosed gaps: legacy surfaces can disagree by a few cm while the
# current scan unambiguously surrounds a tiny missing patch. Permit that explicit
# manual-floor case only for <=16-cell holes and a clean measured floor rim.
holes,count=ndi.label(~valid);sizes=np.bincount(holes.ravel())
for hid in np.unique(holes[fill_candidates&~fills]):
 if hid==0 or sizes[hid]>16:continue
 m=holes==hid;rim=ndi.binary_dilation(m)&~m
 if not np.all(safe[m]) or np.any(guard[m]) or not np.all(valid[rim]):continue
 if np.ptp(z[rim])>.04:continue
 for j,i in zip(*np.where(m&fill_candidates&~fills)):
  q=[x[j,i],y[j,i]];f=fit(q,current);rf=fit(q,legacy)
  if f is None or rf is None or not f['bracket'] or f['rms']>.012 or abs(f['z']-rf['z'])>.06:continue
  new[j,i]=f['z'];fills[j,i]=True;small_hole_fill[j,i]=True;rms[j,i]=f['rms'];difference[j,i]=abs(f['z']-rf['z'])
  models.append(dict(region=int(labels[j,i]),x=float(q[0]),y=float(q[1]),operation='enclosed_hole',current=f,legacy=rf,hole_cells=int(sizes[hid]),rim_height_span_m=float(np.ptp(z[rim]))))

# C is explicitly identified as human/moving-object floor contamination by user.
# Estimate a single local plane from clean surrounding observations, then replace
# positive artifacts >3 cm and a one-cell rim, keeping the unchanged floor outside.
C=(labels==3)&safe;cc=np.array(areas[2]['click']);outer=ndi.distance_transform_edt(~C,sampling=r)
support=(outer>=.1)&(outer<=.8)&floor&(iy%2==0)&(ix%2==0)
pts=np.c_[x[support],y[support]];h=z[support];A=np.c_[np.ones(len(h)),pts-cc]
keep=np.ones(len(h),bool)
for _ in range(5):
 coef=np.linalg.lstsq(A[keep],h[keep],rcond=None)[0];err=h-A@coef;med=np.median(err[keep]);mad=np.median(abs(err[keep]-med));keep=abs(err-med)<=max(.012,3*1.4826*mad)
coef=np.linalg.lstsq(A[keep],h[keep],rcond=None)[0];crms=float(np.sqrt(np.mean((h[keep]-A[keep]@coef)**2)))
assert keep.sum()>=40 and crms<=.018 and np.linalg.norm(coef[1:])<=.08
plane=coef[0]+coef[1]*(x-cc[0])+coef[2]*(y-cc[1])
artifact=C&valid&(z-plane>.030)&(z-plane<.40)&(pd<=.30)
artifact=ndi.binary_dilation(artifact,iterations=1)&C&valid&(pd<=.30)&(abs(z-plane)<.40)
# Use the robust reference surface, not a single noisy legacy point per cell.
for j,i in zip(*np.where(artifact)):
 rf=fit([x[j,i],y[j,i]],legacy)
 if rf is None or abs(plane[j,i]-rf['z'])>.04:artifact[j,i]=False
 else:difference[j,i]=abs(plane[j,i]-rf['z'])
new[artifact]=plane[artifact];repairs|=artifact;rms[artifact]=crms
flat_model=dict(center=cc.tolist(),coefficients_intercept_dx_dy=coef.tolist(),support=int(keep.sum()),rms=crms,artifact_threshold_m=.030,max_correction_m=.40)
changed=fills|repairs
assert np.any(fills) and np.any(repairs)
assert not np.any(changed&(~safe|step))
assert np.array_equal(z[valid&~repairs],new[valid&~repairs])
assert not np.any(repairs&(labels!=3))
assert np.array_equal(np.isfinite(new),valid|fills)

# Local spatial holdout: omit circles of current measurements near each repair
# region, rather than checking training residuals only.
rng=np.random.default_rng(202611091430);validation=[]
for k,a in enumerate(areas,1):
 jj,ii=np.where(floor&(labels==k))
 for radius in [.20,.40]:
  for n in rng.choice(len(ii),min(20,len(ii)),replace=False):
   j,i=jj[n],ii[n];q=[x[j,i],y[j,i]];f=fit(q,current,exclude=radius);rf=fit(q,legacy)
   accept=f is not None and rf is not None and abs(f['z']-rf['z'])<=.035 and (f['bracket'] or (rf['bracket'] and f['nearest']<=.65))
   validation.append(dict(region=k,radius_m=radius,x=float(q[0]),y=float(q[1]),accepted=bool(accept),error_m=float(f['z']-z[j,i]) if accept else None))

lines=s.text.splitlines();header=shlex.split(lines[1]);header[-1]=str(len(s.index)+int(fills.sum()));nx=s.dimensions[0]
result=[lines[0],' '.join(header)]
for n,idx in enumerate(s.index):
 j=idx//nx-s.offset[1];i=idx%nx-s.offset[0]
 if s.usable[n] and repairs[j,i]:
  parts=lines[n+2].split();parts[1]=format(new[j,i],'.17g');result.append(' '.join(parts))
 else:result.append(lines[n+2])
existing=set(s.index.tolist())
for j,i in zip(*np.where(fills)):
 idx=(j+s.offset[1])*nx+i+s.offset[0];assert idx not in existing
 result.append(f'{idx} {new[j,i]:.17g} 0.0009 {s.stamp_ns} 1')
(out/'map.rghm').write_text('\n'.join(result)+'\n');saved=read_snapshot(out/'map.rghm')
assert saved.offset==s.offset and saved.grid.shape==z.shape and np.allclose(saved.grid,new,equal_nan=True)
# The entirety of each unmodified original record, including whitespace, persists.
for n,idx in enumerate(s.index):
 j=idx//nx-s.offset[1];i=idx%nx-s.offset[0]
 if not(s.usable[n] and repairs[j,i]):assert result[n+2]==lines[n+2]
 else:assert result[n+2].split()[2:]==lines[n+2].split()[2:]
write_pcd(s,new,out/'surface.pcd')
prov={k:p[k] for k in p.files};prov.update(inferred_floor=p['inferred_floor']|fills,confirmed_artifact_repaired=p['confirmed_artifact_repaired']|repairs,candidate_height=new,v5_filled=fills,v5_repaired=repairs,v5_before=z,v5_region_labels=labels,v5_fit_rms=rms,v5_reference_agreement=difference,v5_small_enclosed_hole_fill=small_hole_fill)
np.savez_compressed(out/'provenance.npz',**prov)
for k,a in enumerate(areas,1):
 m=labels==k;cx,cy=a['click'];j=int((cy-s.origin[1])/r)-s.offset[1];i=int((cx-s.origin[0])/r)-s.offset[0]
 a.update(filled_cells=int((fills&m).sum()),repaired_cells=int((repairs&m).sum()),unknown_before=int((m&~valid).sum()),unknown_after=int((m&~np.isfinite(new)).sum()),click_occupied=bool(d['old_occupied'][j,i]),click_before=float(z[j,i]) if valid[j,i] else None,click_after=float(new[j,i]) if np.isfinite(new[j,i]) else None)
errors=[abs(t['error_m']) for t in validation if t['accepted']]
edge=[]
for dj,di in [(1,0),(-1,0),(0,1),(0,-1)]:
 neigh=ndi.shift(new,[dj,di],order=0,mode='constant',cval=np.nan,prefilter=False)
 unchanged=ndi.shift(valid&~repairs,[dj,di],order=0,mode='constant',cval=False,prefilter=False)
 ok=changed&unchanged&np.isfinite(neigh);edge.extend(abs(new[ok]-neigh[ok]).tolist())
metrics=dict(base_map=str(base/'map.rghm'),base_sha256=s.digest,candidate_sha256=saved.digest,areas=areas,base_usable_cells=int(valid.sum()),result_usable_cells=int(np.isfinite(new).sum()),filled_cells=int(fills.sum()),filled_area_m2=float(fills.sum()*r*r),repaired_cells=int(repairs.sum()),repaired_area_m2=float(repairs.sum()*r*r),maximum_correction_m=float(abs(new[repairs]-z[repairs]).max()),fit_rms_p50_p95_max_mm=(np.percentile(rms[changed],[50,95,100])*1000).tolist(),reference_agreement_p50_p95_max_mm=(np.percentile(difference[changed],[50,95,100])*1000).tolist(),join_abs_difference_p50_p95_max_mm=(np.percentile(edge,[50,95,100])*1000).tolist(),holdout_trials=len(validation),holdout_accepted=len(errors),holdout_abs_error_p50_p95_max_mm=(np.percentile(errors,[50,95,100])*1000).tolist(),unchanged_original_records=len(s.index)-int(repairs.sum()),step_changes=0,occupancy_obstacle_unknown_changes=0,artifact_model=flat_model,fill_rejected=rejected,holdout=validation,limitations=['Manually bounded areas from user coordinates; actual pillar interiors and occupancy obstacles remain unfilled.','New heights are inferred from observed floor/legacy PCD, not measured ground truth. Legacy map predates the placed steps.','10 cm data-edit margin does not change the robot clearance or any navigation parameter.','Original variance/time on repaired cells retained as provenance, not calibrated uncertainty. Added variance 0.0009 and snapshot timestamps are serialization values, not new sensor observations.','V1 RGHM has no inference flag; preserve provenance.npz with map.rghm.'])
metrics['small_enclosed_hole_fill_cells']=int(small_hole_fill.sum())
(out/'metrics.json').write_text(json.dumps(metrics,indent=2));(out/'fill_support.json').write_text(json.dumps(models))
site=yaml.safe_load((base/'site.yaml').read_text());site['id']='rubi-1f-height-targeted-floor-v5-20261109'
for k in ['pcd','occupancy','image']:site['assets'][k]['path']=str((base/site['assets'][k]['path']).resolve())
site['assets']['height']={'path':'map.rghm','sha256':saved.digest}
site['alignment']['evidence']=str(reference.with_name('reference_audit.json'))
site['alignment']['method']='Inherited V4 XY alignment; user-designated floor gaps inferred from current observed floor with legacy PCD/occupancy checks; entrance artifacts flattened locally.'
site['provenance'].update(height='V4 plus four manually bounded floor repairs; step tops and structure preserved',base_map=str(base/'map.rghm'),base_sha256=s.digest,v5_report='REPORT.md',step_measurements='../rubi_1f_height_20261109_flatsteps_v3/metrics.json')
(out/'site.yaml').write_text(yaml.safe_dump(site,sort_keys=False))
extent=[x.min()-r/2,x.max()+r/2,y.min()-r/2,y.max()+r/2];bg=np.where(d['old_occupied'],0,np.where(d['old_free'],.9,.5))
for k,a in enumerate(areas,1):
 b=a['bounds'];bb=[b[0]-.45,b[1]+.45,b[2]-.45,b[3]+.45]
 fig,axes=plt.subplots(1,3,figsize=(14,5.7),layout='constrained')
 for ax,arr,title in zip(axes,[z,new,np.where(fills,1,np.where(repairs,2,np.nan))],['Before: V4','After: V5','Cyan: filled / orange: flattened']):
  ax.imshow(bg,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=1)
  if ax is axes[2]:
   ax.imshow(np.where(valid,1,np.nan),origin='lower',extent=extent,cmap=ListedColormap(['#c4bdb4']))
   ax.imshow(arr,origin='lower',extent=extent,cmap=ListedColormap(['#00bbc9','#ff7600']),vmin=1,vmax=2)
  else:im=ax.imshow(arr,origin='lower',extent=extent,cmap='viridis',vmin=-.52,vmax=-.28)
  ax.contour(x,y,d['old_occupied'],[.5],colors=['black'],linewidths=.7)
  ax.plot(*a['click'],'rx',ms=8,mew=1.5);ax.set(xlim=bb[:2],ylim=bb[2:],aspect='equal',title=title,xlabel='map X [m]',ylabel='map Y [m]')
 fig.suptitle(a['id']+f": +{a['filled_cells']} floor cells / {a['repaired_cells']} changed cells; black outline = old occupied")
 fig.colorbar(im,ax=axes[:2],shrink=.65,label='Absolute map Z [m] (display clipped)');fig.savefig(out/(a['id']+'_before_after.png'),dpi=150);plt.close(fig)
print(json.dumps({k:v for k,v in metrics.items() if k not in ['holdout','limitations']},indent=2),flush=True)
