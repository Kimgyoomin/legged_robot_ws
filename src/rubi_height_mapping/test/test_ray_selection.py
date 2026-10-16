"""Actual CUDA: representative rays, fusion coverage, and cleanup behavior."""
from pathlib import Path
import cupy as cp
import numpy as np
import pytest
from rubi_height_mapping.core import ElevationMap
from rubi_height_mapping.parameter import Parameter
from rubi_height_mapping.ray_selection import VoxelRaySelector

WEIGHTS = str(Path(__file__).resolve().parents[1]/'config/traversability_weights.npz')
R = np.eye(3, dtype=np.float32)
SENSOR = np.array([0, 0, .8], np.float32)


def mapper(mode, **kw):
    p = Parameter(ray_mode=mode, weights_path=WEIGHTS, **kw)
    p.update()
    return ElevationMap(p)


def feed(m, points):
    m.input_pointcloud(points, ['x','y','z'], R, SENSOR, 0, 0, points_are_finite=True)


def test_original_endpoints_per_3d_voxel_and_reused_buffers():
    p = Parameter();p.update();s = VoxelRaySelector(p)
    # Same XY but different Z must not collapse: a 5 cm step retains a ray.
    world = np.array([[1.011,.011,.011],[1.021,.021,.021],
                      [1.011,.011,.061],[-1.021,-.021,.011]], np.float32)
    points = cp.asarray(np.tile(world-SENSOR, (100, 1)))
    s.select(points, cp.asarray(R), cp.asarray(SENSOR), cp.zeros(1, cp.float32))
    np.testing.assert_array_equal(cp.where(s.selected_mask())[0].get(), [0,2,3])
    before = (s.keys.data.ptr, s.winners.data.ptr, s.slots.data.ptr)
    s.select(points[:4], cp.asarray(R), cp.asarray(SENSOR), cp.zeros(1, cp.float32))
    assert before == (s.keys.data.ptr, s.winners.data.ptr, s.slots.data.ptr)
    assert s.statistics()['selected_rays'] == 3
    # Empty scans have no representatives, and the next scan clears old winners.
    s.select(points[:0], cp.asarray(R), cp.asarray(SENSOR), cp.zeros(1, cp.float32))
    assert s.statistics()['selected_rays'] == 0
    s.select(points[2:3], cp.asarray(R), cp.asarray(SENSOR), cp.zeros(1, cp.float32))
    assert s.statistics()['selected_rays'] == 1


def test_rotated_mount_negative_world_and_vertical_datum():
    p = Parameter();p.update();s = VoxelRaySelector(p)
    rotation = np.diag([1,-1,-1]).astype(np.float32)
    translation = np.array([-2.3,.7,.8], np.float32)
    center_z = np.array([1.5], np.float32)
    relative_world = np.array([[-1.011,.311,.011],[-1.021,.321,.021],[-1.011,.311,.061]],np.float32)
    sensor_points = (relative_world-translation) @ rotation
    s.select(cp.asarray(sensor_points),cp.asarray(rotation),cp.asarray(translation),cp.asarray(center_z))
    assert s.statistics()['selected_rays'] == 2
    np.testing.assert_array_equal(cp.where(s.selected_mask())[0].get(), [0,2])


def test_hash_exhaustion_and_coordinate_overflow_keep_original_rays():
    p = Parameter();p.update();s = VoxelRaySelector(p)
    # Force saturation of a two-slot table; unique endpoints must still all trace.
    points = cp.asarray(np.array([[.2+.1*i,.011,-.8] for i in range(40)],np.float32))
    s.count = len(points)
    s.keys = cp.full(2,cp.uint64(0xffffffffffffffff),cp.uint64)
    s.winners = cp.full(2,0x7fffffff,cp.int32)
    s.slots = cp.empty(len(points),cp.int32)
    s.kernel(points,cp.asarray(R),cp.asarray(SENSOR),cp.zeros(1,cp.float32),cp.uint64(1),s.keys,s.winners,s.slots,size=len(points))
    assert s.statistics()['selected_rays'] == 40
    assert s.statistics()['fallback_rays'] == 38
    points = cp.asarray([[60000.,0.,-.8]],dtype=cp.float32)
    s.capacity = 0
    s.select(points,cp.asarray(R),cp.asarray(SENSOR),cp.zeros(1,cp.float32))
    assert s.statistics()['fallback_rays'] == 1


def test_height_fusion_keeps_every_observation_and_steps():
    a = np.arange(-1.5,1.5,.025,dtype=np.float32)+.0125
    x,y = np.meshgrid(a,a)
    z = np.where(x<-.4,0,np.where(x<.4,.05,.15)).astype(np.float32)
    points = np.stack([x.ravel(),y.ravel(),z.ravel()-.8],axis=1)
    maps = {mode:mapper(mode) for mode in ('all','voxel')}
    for m in maps.values():
        for _ in range(3): feed(m,points)
        # New-map count comes from height fusion, not the selected-ray set.
        assert int(cp.sum(m.new_map[2]).get()) == len(points)
    a,b = (cp.asnumpy(m.get_elevation()) for m in maps.values())
    np.testing.assert_array_equal(np.isfinite(a),np.isfinite(b))
    np.testing.assert_allclose(a,b,atol=2e-6,rtol=2e-5,equal_nan=True)
    for height in (0,.05,.15): assert np.sum(abs(b-height)<.002)>200
    assert maps['voxel'].ray_selector.statistics()['selected_rays'] < len(points)/2


def cleanup_fixture(mode):
    m = mapper(mode)
    # Two old elevated cells: one intersected by a ray, one occluded off its path.
    row = m.cell_n//2; col = row+20
    m.elevation_map[0,row,col] = .7
    m.elevation_map[1,row,col] = .01
    m.elevation_map[2,row,col] = 1
    m.elevation_map[4,row,col] = 2
    m.elevation_map[5,row,col] = .7
    m.elevation_map[:,row+10,col] = m.elevation_map[:,row,col]
    return m,row,col


def test_cleanup_removes_ray_conflict_but_keeps_occluded_and_unknown():
    points = np.tile(np.array([[2.5,.011,-.8]],np.float32),(40,1))
    clearing = {}
    for mode in ('all','voxel'):
        m,row,col = cleanup_fixture(mode)
        for frame in range(1,21):
            # Known horizontal normal isolates cleanup from normal-estimation noise.
            m.normal_map[0:2] = 0;m.normal_map[2] = 1
            feed(m,points)
            assert float(m.elevation_map[2,row+10,col].get()) == 1
            assert float(m.elevation_map[2,row-10,col].get()) == 0
            if float(m.elevation_map[2,row,col].get()) < .5:
                clearing[mode] = frame
                break
        assert mode in clearing
    # Ray thinning changes evidence rate: report this rather than claiming identity.
    assert clearing['all'] <= clearing['voxel'] <= 12


def test_cleanup_disabled_does_not_allocate_selector():
    m = mapper('voxel',enable_visibility_cleanup=False)
    assert m.ray_selector is None
    feed(m,np.array([[1.01,.01,-.8]],np.float32))
