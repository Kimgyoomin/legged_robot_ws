from pathlib import Path
import sys, json, hashlib
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

root = Path(__file__).parent
ws = Path('/home/rclab/legged_robot_ws')
sys.path.insert(0, str(ws/'src/legged_bringup'))
from legged_bringup.map_io import read_snapshot
s = read_snapshot(ws/'maps/rubi_1f_height_20261109_startfloor_v2/map.rghm')
x, y = s.xy()
old = json.loads((root/'current_10_cot.json').read_text())
limit_only = json.loads((root/'review_11_cot.json').read_text())
new = json.loads((root/'review_11_geometry.json').read_text())
fig, axes = plt.subplots(1, 5, figsize=(18, 4.8), layout='constrained')
summary = []
for ax, a, b, c, nominal in zip(axes, old, limit_only, new, [10, 10, 6, 29, 29]):
    cx, cy = c['center']
    rows = np.flatnonzero(abs(y[:, 0]-cy) < 1.2)
    cols = np.flatnonzero(abs(x[0]-cx) < 1.2)
    sl = np.ix_(rows, cols)
    xx, yy, zz = x[sl], y[sl], s.grid[sl]
    radius = np.hypot(xx-cx, yy-cy)
    ring = (radius > .65) & (radius < 1.1) & np.isfinite(zz)
    floor = np.median(zz[ring])
    cm = plt.get_cmap('viridis').copy(); cm.set_bad('#333333')
    im = ax.imshow(100*(zz-floor), origin='lower', cmap=cm, vmin=0, vmax=32,
        extent=[xx.min()-.025,xx.max()+.025,yy.min()-.025,yy.max()+.025])
    grid = np.asarray(c['grid'])
    gx, gy = np.unique(grid[:,0]), np.unique(grid[:,1])
    blocked = (grid[:,2] != 0).reshape(len(gy),len(gx))
    if blocked.any() and not blocked.all():
        ax.contour(gx, gy, blocked, levels=[.5], colors=['#ff5050'], linewidths=.8)
    for d, color, label in [(a,'white','10 cm + CoT'),(c,'#ff40ff','11 cm + geometry')]:
        path = np.asarray(d['default']['path'])
        ax.plot(path[:,0],path[:,1],color=color,lw=2.2,label=label)
    assert np.allclose(a['default']['path'], b['default']['path'])
    path = np.asarray(c['default']['path'])
    ax.plot(*path[0,:2], marker='o', color='#ff40ff', ms=6)
    ax.plot(*path[-1,:2], marker='x', color='#ff40ff', ms=7)
    ax.set(xlim=(cx-1.15,cx+1.15),ylim=(cy-1.0,cy+1.0),xlabel='map X [m]',ylabel='map Y [m]',
           title=f"{a['id']} | nominal {nominal} cm\n{a['default']['length']:.2f} m -> {c['default']['length']:.2f} m")
    summary.append(dict(id=a['id'],nominal_cm=nominal,center=c['center'],
        original=a['default'],limit_only=b['default'],geometry=c['default'],
        direct_invalid_edges=[d['straight_invalid_edges'] for d in [a,b,c]]))
axes[0].legend(loc='lower left',fontsize=8)
fig.colorbar(im,ax=axes,shrink=.65,label='Height above local ring median [cm]')
fig.suptitle('Same saved height map and costmap: planner core comparison\n11 cm + CoT gives the same paths as 10 cm + CoT. Red outline: rejected area at 11 cm.')
fig.savefig(root/'cost_paths.png',dpi=160);plt.close(fig)

fig,ax=plt.subplots(figsize=(8,4.5),layout='constrained')
h=np.linspace(0,.11,441);geom=np.where(h>.01,.11*np.clip((h-.01)/.1,0,1)**2,0)
cot=np.where(h>.01,np.interp(h,[0,.05,.07,.09,.11,.13,.15],[0,.361,.716,1.083,1.272,1.604,1.634]),0)
ax.plot(h*100,cot,label='CoT: one abrupt upward edge',lw=2)
ax.plot(h*100,geom,label='Geometry: one up OR down edge',lw=2)
ax.set(xlabel='Adjacent height difference [cm]',ylabel='Height penalty [equivalent m]',
       title='11 cm limit, height weight 1: ideal single-edge cost\nCoT event height assumed equal to jump; actual local relief can increase it.')
ax.legend();ax.grid(alpha=.25);fig.savefig(root/'cost_curves.png',dpi=160);plt.close(fig)
runtime=json.loads((root/'runtime/review_results.json').read_text())
for d in runtime:
    a=np.asarray(d['poses']);d['length_xy_m']=float(np.linalg.norm(np.diff(a[:,:2],axis=0),axis=1).sum())
    d['y_range_m']=[float(a[:,1].min()),float(a[:,1].max())]
(root/'summary.json').write_text(json.dumps(dict(map_sha256=s.digest,map=str(s.source),core=summary,nav2=runtime),indent=2))
print('Created cost_paths.png, cost_curves.png, summary.json')
