import os,sys,json
os.environ['MPLCONFIGDIR']='/tmp/height-target-mpl'
sys.path.insert(0,'/home/rclab/legged_robot_ws/src/legged_bringup')
from legged_bringup.map_io import read_snapshot
import numpy as np
from scipy import ndimage as ndi
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from pathlib import Path
B=Path('/home/rclab/legged_robot_ws/maps/rubi_1f_height_20261109_floorcomplete_v4')
R=Path('/home/rclab/legged_robot_ws/maps/processed/run_1794037249460292244_postprocess_20261107/audits/reference/reference_masks.npz')
O=Path('/tmp/height_targeted_v5_20261109')
s=read_snapshot(B/'map.rghm');z=s.grid;x,y=s.xy();d=np.load(R);p=np.load(B/'provenance.npz');r=s.resolution
extent=[x.min()-r/2,x.max()+r/2,y.min()-r/2,y.max()+r/2]
areas=[('A_hall',13.406,43.5112,[10,17,41.5,46.5]),('B_entrance_fill',5.92883,4.87223,[3.5,9,2,7.5]),('C_entrance_flat',5.11579,-.0449459,[3.5,7.5,-2,2]),('D_upper',31.0846,51.578,[28,34.5,49.5,54])]
for name,cx,cy,bb in areas:
    m=(x>=bb[0])&(x<=bb[1])&(y>=bb[2])&(y<=bb[3])
    fig,axs=plt.subplots(1,4,figsize=(20,6),layout='constrained')
    for ax in axs:
        ax.set(xlim=bb[:2],ylim=bb[2:],aspect='equal',xlabel='map X [m]',ylabel='map Y [m]');ax.plot(cx,cy,'rx',ms=9,mew=2)
    im=axs[0].imshow(z,origin='lower',extent=extent,vmin=-.52,vmax=-.30,cmap='viridis');axs[0].set_title('V4 height, clipped to show floor');fig.colorbar(im,ax=axs[0],shrink=.6)
    occ=np.where(d['old_occupied'],0,np.where(d['old_free'],1,.5))
    axs[1].imshow(occ,origin='lower',extent=extent,cmap='gray',vmin=0,vmax=1);axs[1].contour(x,y,np.isfinite(z),[.5],colors=['#ff7800'],linewidths=.5);axs[1].set_title('Old occupancy + current observed outline')
    im=axs[2].imshow(d['old_pcd_floor_adjusted_z'],origin='lower',extent=extent,vmin=-.52,vmax=-.30,cmap='viridis');axs[2].set_title('Bias-corrected old PCD floor');fig.colorbar(im,ax=axs[2],shrink=.6)
    status=np.where(p['original_observed'],1,0)+2*p['inferred_floor']+4*p['confirmed_artifact_repaired']
    axs[3].imshow(status,origin='lower',extent=extent,cmap='tab10',vmin=0,vmax=9);axs[3].contour(x,y,d['old_free_margin_35cm'],[.5],colors=['red'],linewidths=.5);axs[3].set_title('0 unknown / 1 measured / 2 inferred / 5 repaired')
    fig.savefig(O/(name+'_inspect.png'),dpi=140);plt.close(fig)
    j=int((cy-s.origin[1])/r)-s.offset[1];i=int((cx-s.origin[0])/r)-s.offset[0]
    rr=(x-cx)**2+(y-cy)**2<=1
    vals=z[rr&np.isfinite(z)]
    print(name,'clicked',float(z[j,i]),'occ',bool(d['old_free'][j,i]),'pcd',float(d['old_pcd_floor_adjusted_z'][j,i]),'radius1 height percentiles',np.percentile(vals,[0,10,50,90,100]).tolist(),'missing',int((rr&~np.isfinite(z)).sum()),flush=True)
