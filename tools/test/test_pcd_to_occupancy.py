from pathlib import Path
import importlib.util
import numpy as np
spec=importlib.util.spec_from_file_location('projection',Path(__file__).resolve().parents[1]/'pcd_to_occupancy.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_relative_height_band_unknowns_and_world_axes():
    ground=np.array([[1.,1.,np.nan],[1.,2.,np.nan]])
    points=np.array([[-.75,-.75,1.01],[-.25,-.75,1.4],[-.25,-.25,2.1],[-.25,-.25,4.],
                     [100.,100.,2.], [np.nan,0.,0.]])
    image=module.project(points,ground,.5,[-1.,-1.],.15,1.,.2)
    assert image.tolist()==[[254,0,205],[254,254,205]]


def test_unknown_does_not_become_free_from_neighbor_support():
    image=module.project(np.array([[.15,.05,.5]]),np.array([[0.,np.nan,np.nan]]),.1,[0.,0.],.15,1.,.15)
    assert image.tolist()==[[254,0,205]]
