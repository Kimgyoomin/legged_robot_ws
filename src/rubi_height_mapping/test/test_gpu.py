"""CUDA tests with excluded optional modules actively blocked from import."""
import importlib.abc
import sys
import unittest
from pathlib import Path
import numpy as np

class BlockOptional(importlib.abc.MetaPathFinder):
    forbidden={'torch','chainer','cv2','cv_bridge','semantic_sensor','scipy','shapely',
               'simple_parsing','elevation_mapping_cupy','elevation_map_msgs','ros2_numpy','rosbag2_py'}
    def find_spec(self, fullname, path=None, target=None):
        if fullname.split('.')[0] in self.forbidden:
            raise ModuleNotFoundError('Blocked deployment dependency: '+fullname)
sys.meta_path.insert(0,BlockOptional())
import cupy as cp
from rubi_height_mapping.core import ElevationMap
from rubi_height_mapping.parameter import Parameter
from rubi_height_mapping.traversability_cupy import CupyTraversability
WEIGHTS=Path(__file__).resolve().parents[1]/'config/traversability_weights.npz'

class GPU(unittest.TestCase):
    def test_validated_input_owns_buffer_and_batch_export_matches(self):
        p=Parameter(weights_path=str(WEIGHTS));p.update()
        old=ElevationMap(p);new=ElevationMap(p)
        axis=np.arange(-1.5,1.5,.05,dtype=np.float32)+.025
        x,y=np.meshgrid(axis,axis)
        points=np.stack([x.ravel(),y.ravel(),np.where(x.ravel()>0,.15,0)-.8],axis=1).astype(np.float32)
        before=points.copy();device=cp.asarray(points)
        rotation=np.eye(3,dtype=np.float32);sensor=np.array([0,0,.8],np.float32)
        # Legacy input must still discard NaNs. The trusted path owns both
        # host and device inputs because fusion kernels overwrite their scratch.
        dirty=np.concatenate([points,np.full((1,3),np.nan,np.float32)])
        for value in (points,device):
            old.input_pointcloud(dirty,['x','y','z'],rotation,sensor,0,0)
            new.input_pointcloud(value,['x','y','z'],rotation,sensor,0,0,points_are_finite=True)
        np.testing.assert_array_equal(points,before)
        np.testing.assert_array_equal(cp.asnumpy(device),before)
        np.testing.assert_allclose(cp.asnumpy(new.elevation_map),cp.asnumpy(old.elevation_map),atol=2e-6,rtol=2e-5)
        batch=np.empty((2,new.cell_n-2,new.cell_n-2),np.float32)
        new.get_raw_layers_ref(batch)
        for index,layer in enumerate(('elevation','variance')):
            single=np.empty(batch.shape[1:],np.float32)
            new.get_map_with_name_ref(layer,single)
            np.testing.assert_array_equal(batch[index],single)

    def test_long_grid_motion_preserves_global_cell_alignment(self):
        p=Parameter(weights_path=str(WEIGHTS));p.update();m=ElevationMap(p)
        result=np.empty((1,3),np.float32);rotation=np.eye(3,dtype=np.float32)
        # This route exceeded 0.004 cells of float32 drift in the old code;
        # the C++ wrapper permits only 0.001 cells before rejecting a patch.
        path=list(range(500))+list(range(499,-1,-1))*3
        for i in path:
            position=np.array([i*.05,-i*.05,.6],np.float32)
            m.move_to(position,rotation);m.get_center_position(result)
            phase=result[0,:2].astype(np.float64)/m.resolution
            self.assertLess(np.max(abs(phase-np.rint(phase))),.001)
            np.testing.assert_allclose(result[0],position,atol=3e-6,rtol=0)
    def test_filter_against_numpy_convolution(self):
        rng=np.random.default_rng(17)
        for n in (10,32,162):
            a=rng.normal(0,.2,(n,n)).astype(np.float32)
            expected=np.zeros((n-6,n-6),np.float32)
            with np.load(WEIGHTS,allow_pickle=False) as w:
                for b in range(3):
                    d=b+1
                    for c in range(4):
                        conv=np.zeros_like(expected)
                        for ky in range(3):
                            for kx in range(3):
                                yy=3+(ky-1)*d;xx=3+(kx-1)*d
                                conv+=a[yy:yy+n-6,xx:xx+n-6]*w[f'w{b+1}'][c,0,ky,kx]
                        expected+=np.abs(conv)*w['w_out'][0,b*4+c,0,0]
            expected=np.exp(-expected)
            result=cp.asnumpy(CupyTraversability(str(WEIGHTS))(cp.asarray(a)))[0,0]
            np.testing.assert_allclose(result,expected,rtol=2e-5,atol=2e-6)
    def test_height_and_moving_grid_without_excluded_packages(self):
        p=Parameter(weights_path=str(WEIGHTS));p.update();m=ElevationMap(p)
        axis=np.arange(-1.5,1.5,.05,dtype=np.float32)+.025
        x,y=np.meshgrid(axis,axis);z=np.where(x<-.4,0,np.where(x<.4,.05,.15)).astype(np.float32)
        world=np.stack([x.ravel(),y.ravel(),z.ravel()],axis=1)
        for base_x in (0,.1,.2,-.1):
            sensor=np.array([base_x,0,.8],np.float32)
            m.move_to(np.array([base_x,0,0],np.float32),np.eye(3,dtype=np.float32))
            m.input_pointcloud(world-sensor,['x','y','z'],np.eye(3,dtype=np.float32),sensor,0,0)
            m.update_time();m.update_variance()
        cp.cuda.Stream.null.synchronize()
        heights=cp.asnumpy(m.get_elevation());valid=np.isfinite(heights)
        self.assertGreater(valid.sum(),1000)
        for target in (0,.05,.15):self.assertGreater(np.sum(np.abs(heights[valid]-target)<.002),200)
        out=np.empty(heights.T.shape,np.float32);m.get_map_with_name_ref('elevation',out)
        np.testing.assert_allclose(out,heights.T[::-1,::-1],atol=0,equal_nan=True)
        self.assertFalse(BlockOptional.forbidden & set(sys.modules))
if __name__=='__main__':unittest.main()
