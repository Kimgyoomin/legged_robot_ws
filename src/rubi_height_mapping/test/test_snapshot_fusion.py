"""GPU behavioral regressions for scan-level state ownership.

These tests exercise actual CUDA kernels. They do not use localization and do
not claim real-world map accuracy or bitwise invariance across GPU models.
"""
from pathlib import Path

import cupy as cp
import numpy as np
import pytest

from rubi_height_mapping.core import ElevationMap
from rubi_height_mapping.parameter import Parameter

WEIGHTS = str(Path(__file__).resolve().parents[1] / 'config/traversability_weights.npz')
ROTATION = np.eye(3, dtype=np.float32)
SENSOR = np.array([0, 0, .8], dtype=np.float32)


def mapper(**kwargs):
    params = Parameter(weights_path=WEIGHTS, **kwargs)
    params.update()
    return ElevationMap(params)


def feed(m, points):
    m.input_pointcloud(points, ['x', 'y', 'z'], ROTATION, SENSOR, 0, 0,
                       points_are_finite=True)


def initialize_cell(m, height, variance=.01):
    row, col = m.cell_n // 2, m.cell_n // 2 + 20
    m.elevation_map[:, row, col] = cp.asarray(
        [height, variance, 1, 1, 2, height, 0], dtype=cp.float32)
    m.normal_map[2] = 1
    return row, col


@pytest.mark.parametrize('mode', ['all', 'voxel'])
def test_outliers_do_not_change_other_points_acceptance_mid_scan(mode):
    points = np.concatenate([
        np.tile(np.array([[1.011, .011, -.79]], np.float32), (31, 1)),
        np.tile(np.array([[1.011, .011, -.70]], np.float32), (4096, 1)),
    ])
    maps = []
    for cloud in (points, points[::-1].copy()):
        m = mapper(ray_mode=mode, enable_visibility_cleanup=False)
        row, col = initialize_cell(m, 0)
        before = m.elevation_map.copy()
        feed(m, cloud)
        # 10 cm outliers cannot inflate the gate and admit subsequent outliers.
        assert float(m.new_map[2, row, col].get()) == 31
        assert float(m.new_map[5, row, col].get()) == 4096
        assert 0 <= float(m.elevation_map[0, row, col].get()) < .02
        np.testing.assert_array_equal(m.map_snapshot.get(), before.get())
        maps.append(m.elevation_map.get())
    np.testing.assert_allclose(maps[0], maps[1], rtol=0, atol=1e-7)


@pytest.mark.parametrize('mode', ['all', 'voxel'])
def test_accepted_endpoint_wins_over_conflicting_cleanup(mode):
    m = mapper(ray_mode=mode)
    row, col = initialize_cell(m, .7)
    # Long downward rays cross the old elevated cell. A genuine accepted
    # endpoint in that same cell must win, independent of point scheduling.
    points = np.concatenate([
        np.tile(np.array([[2.5, .011, -.8]], np.float32), (80, 1)),
        np.array([[1.011, .011, -.1]], np.float32),
    ])
    before = m.elevation_map.copy()
    feed(m, points)
    assert float(m.visibility_map[0, row, col].get()) > 0
    assert float(m.new_map[2, row, col].get()) == 1
    cell = m.elevation_map[:, row, col].get()
    assert cell[2] == 1 and abs(cell[0] - .7) < .002
    assert cell[4] == 0 and cell[5] == cell[0] and cell[6] == 0
    np.testing.assert_array_equal(m.map_snapshot.get(), before.get())


def test_outlier_only_cell_changes_uncertainty_after_scan():
    m = mapper(enable_visibility_cleanup=False)
    row, col = initialize_cell(m, 0)
    feed(m, np.tile(np.array([[1.011, .011, -.7]], np.float32), (5, 1)))
    cell = m.elevation_map[:, row, col].get()
    assert cell[0] == 0 and cell[2] == 1 and cell[4] == 2
    assert abs(cell[1] - .06) < 1e-7
    assert float(m.map_snapshot[1, row, col].get()) == np.float32(.01)


def test_upper_bound_reduction_is_order_independent_and_does_not_fill_unknown():
    base = np.array([[2.5, .011, -.8], [2.5, .011, -1.8]], np.float32)
    outputs = []
    for points in (np.tile(base, (2048, 1)), np.tile(base[::-1], (2048, 1))):
        m = mapper()
        feed(m, points)
        outputs.append(m.elevation_map.get())
        # Ray-only evidence describes an upper bound, never observed ground.
        upper = (outputs[-1][6] > .5) & (outputs[-1][2] == 0)
        assert upper.sum() > 10
        assert outputs[-1][5][upper].min() < 0
    np.testing.assert_allclose(outputs[0], outputs[1], rtol=0, atol=1e-7)


@pytest.mark.parametrize('mode', ['all', 'voxel'])
def test_repeated_complex_scans_reproduce_and_reuse_buffers(mode):
    rng = np.random.default_rng(23)
    x = rng.uniform(-2.5, 2.5, 12000).astype(np.float32)
    y = rng.uniform(-2.5, 2.5, len(x)).astype(np.float32)
    z = np.where(x < -.4, 0, np.where(x < .4, .05, .15))
    z += rng.normal(0, .012, len(x))
    z[::17] += .25  # mixed-height cells and outliers
    points = np.stack([x, y, z - .8], axis=1).astype(np.float32)
    outputs = []
    for _ in range(3):
        m = mapper(ray_mode=mode)
        pointers = tuple(a.data.ptr for a in (
            m.map_snapshot, m.new_map, m.visibility_map, m._error, m._error_count))
        for scan in range(215):
            # Keep clocks identical, but exercise aged cells as well as fusion.
            feed(m, points if scan % 3 else points[:6000])
            m.update_time()
            m.update_variance()
        assert pointers == tuple(a.data.ptr for a in (
            m.map_snapshot, m.new_map, m.visibility_map, m._error, m._error_count))
        outputs.append(m.elevation_map.get())
    for other in outputs[1:]:
        np.testing.assert_array_equal(outputs[0][2] > .5, other[2] > .5)
        np.testing.assert_allclose(outputs[0], other, rtol=0, atol=2e-6)


def test_strided_xyz_empty_input_and_reset_leave_no_old_proposals():
    m = mapper()
    cloud = np.array([[1.011, .011, -.8, 999], [1.011, .011, -.8, -999]], np.float32)
    feed(m, cloud)
    assert float(cp.sum(m.new_map[2]).get()) == 2
    before = m.elevation_map.copy()
    feed(m, np.empty((0, 3), np.float32))
    np.testing.assert_array_equal(m.elevation_map.get(), before.get())
    m.clear()
    feed(m, cloud[:1])
    assert float(cp.sum(m.new_map[2]).get()) == 1
    assert np.all(m.map_snapshot[2].get() == 0)
