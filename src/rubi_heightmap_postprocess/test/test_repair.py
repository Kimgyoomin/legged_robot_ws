"""Offline editor contract: no automatic terrain approval or input overwrite."""
import hashlib
import json
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rubi_heightmap_postprocess.map_io import read_snapshot
from rubi_heightmap_postprocess.repair import RepairSession, RepairSettings, SupportFitError, polygon_mask


def snapshot_file(tmp_path, z, name='source.rghm', offset=(3, 7)):
    """Nonzero origin, cropped offset and exact integer timestamp fixture."""
    height, width = z.shape
    nx, ny = width+offset[0]+2, height+offset[1]+3
    stamp = 1785499140296331644
    rows, cols = np.where(np.isfinite(z))
    lines = ['RUBI_GLOBAL_HEIGHTMAP 1',
             f'"map" 0.05 -25 11 {nx} {ny} 0.04 {stamp} {len(rows)}']
    for y, x in zip(rows, cols):
        index = (y+offset[1])*nx+x+offset[0]
        lines.append(f'{index} {z[y, x]:.17g} 0.003 {stamp-101} 1')
    path = tmp_path/name
    path.write_text('\n'.join(lines)+'\n')
    return read_snapshot(path)


def floor(shape=(41, 53)):
    y, x = np.indices(shape)
    return -.43+.001*x-.0005*y


def patch(shape):
    mask = np.zeros(shape, bool)
    mask[18:23, 24:29] = True
    return mask


def test_preview_apply_undo_redo_preserve_original(tmp_path):
    truth = floor()
    z = truth.copy()
    mask = patch(z.shape)
    z[mask] += .12
    z[20, 26] = np.nan
    snapshot = snapshot_file(tmp_path, z)
    original = snapshot.source.read_bytes()
    session = RepairSession(snapshot)
    proposal = session.preview_region(mask)
    np.testing.assert_array_equal(session.working, z)
    assert proposal.details['newly_filled_cells'] == 1
    assert proposal.details['fit']['slope_deg'] > 0
    session.apply(proposal)
    np.testing.assert_allclose(session.working, truth, atol=1e-12)
    assert session.methods[mask].min() == 1
    assert session.undo()
    np.testing.assert_array_equal(session.working, z)
    assert not session.methods.any()
    assert session.redo()
    np.testing.assert_allclose(session.working, truth, atol=1e-12)
    assert snapshot.source.read_bytes() == original
    np.testing.assert_array_equal(snapshot.grid, z)


def test_polygon_coordinates_include_crop_origin_and_no_axis_flip(tmp_path):
    snap = snapshot_file(tmp_path, floor())
    x0, y0 = snap.origin[0]+snap.offset[0]*.05, snap.origin[1]+snap.offset[1]*.05
    vertices = [(x0+24*.05, y0+18*.05), (x0+29*.05, y0+18*.05),
                (x0+29*.05, y0+23*.05), (x0+24*.05, y0+23*.05)]
    np.testing.assert_array_equal(polygon_mask(snap, vertices), patch(snap.grid.shape))
    assert not polygon_mask(snap, [(0, 0), (1, 0), (1, 1)]).any()
    with pytest.raises(ValueError):
        polygon_mask(snap, [(0, 0), (np.nan, 1), (1, 1)])


@pytest.mark.parametrize('case', ['step', 'curved', 'edge', 'too_high', 'too_large', 'no_support', 'steep'])
def test_unsafe_region_rejected(tmp_path, case):
    z = floor()
    mask = patch(z.shape)
    if case == 'step':
        z[:, 26:] += .15
    elif case == 'curved':
        y, x = np.indices(z.shape)
        z += .003*(x-26)**2
    elif case == 'edge':
        mask[:] = False
        mask[:3, 24:29] = True
    elif case == 'too_high':
        z[mask] += .8
    elif case == 'too_large':
        mask[:] = True
    elif case == 'no_support':
        z[12:29, 18:35] = np.nan
    elif case == 'steep':
        z += np.indices(z.shape)[1]*.03
    session = RepairSession(snapshot_file(tmp_path, z))
    with pytest.raises(ValueError):
        session.preview_region(mask)
    np.testing.assert_array_equal(session.working, z)
    assert not session.undo_stack


def test_holes_interior_small_only_and_no_fake_observations(tmp_path):
    truth = floor()
    z = truth.copy()
    z[20:22, 25:27] = np.nan  # accepted 2x2 hole
    z[30:33, 40:43] = np.nan  # 3x3 rejected
    z[:3, 10] = np.nan  # boundary-connected rejected
    z[5, 10:13] = np.nan  # 3 cells but >10 cm span rejected
    session = RepairSession(snapshot_file(tmp_path, z))
    proposal = session.preview_holes()
    assert proposal.details['accepted_holes'] == 1
    assert len(proposal.indices) == 4
    np.testing.assert_allclose(proposal.values, truth.flat[proposal.indices], atol=1e-12)
    assert np.isnan(session.working[20, 25])
    session.apply(proposal)
    assert session.methods[20, 25] == 2
    assert np.isnan(session.working[31, 41])
    assert session.undo()
    np.testing.assert_array_equal(session.working, z)


def test_step_hole_is_not_a_fill_candidate(tmp_path):
    z = floor()
    z[:, 26:] += .1
    z[20, 26] = np.nan
    proposal = RepairSession(snapshot_file(tmp_path, z)).preview_holes()
    assert not len(proposal.indices)
    assert sum(proposal.details['rejected_holes'].values()) == 1


def test_holes_selection_restricts_candidates(tmp_path):
    z = floor()
    z[10, 10] = z[30, 40] = np.nan
    session = RepairSession(snapshot_file(tmp_path, z))
    mask = np.zeros(z.shape, bool)
    mask[9:12, 9:12] = True
    proposal = session.preview_holes(selection=mask)
    assert proposal.indices.tolist() == [10*z.shape[1]+10]


def test_modified_support_never_reused(tmp_path):
    z = floor()
    z[20, 26] = np.nan
    session = RepairSession(snapshot_file(tmp_path, z))
    # Exclude all edited/inferred support regardless of its plausible heights.
    session.methods[:] = 1
    session.methods[20, 26] = 0
    proposal = session.preview_holes()
    assert not len(proposal.indices)
    assert any(key.startswith('Insufficient original floor support')
               for key in proposal.details['rejected_holes'])


def manual_support(shape):
    support = np.zeros(shape, bool)
    support[15:26, 17:21] = True
    support[15:26, 32:36] = True
    return support


def test_manual_floor_excludes_neighbor_artifact_without_relaxing_limits(tmp_path):
    truth = floor()
    z = truth.copy()
    mask = patch(z.shape)
    z[mask] += .12
    z[20, 23] += .04  # just OUTSIDE target, contaminating the automatic ring
    session = RepairSession(snapshot_file(tmp_path, z))
    with pytest.raises(SupportFitError) as raised:
        session.preview_region(mask)
    assert 20*z.shape[1]+23 in raised.value.outlier_indices
    support = manual_support(z.shape)
    p = session.preview_region(mask, support_mask=support)
    assert p.details['fit']['support_mode'] == 'manual_floor'
    assert p.details['settings']['max_rms_m'] == .005
    assert p.details['fit']['support_peak_m'] < 1e-12
    session.apply(p)
    np.testing.assert_allclose(session.working[mask], truth[mask], atol=1e-12)
    np.testing.assert_array_equal(session.working[~mask], z[~mask])
    session.undo()
    np.testing.assert_array_equal(session.working, z)


@pytest.mark.parametrize('case', ['one_side', 'too_far', 'too_few', 'line', 'overlap',
                                  'edited', 'step', 'unknown', 'wrong_shape'])
def test_manual_support_guards(tmp_path, case):
    z = floor()
    mask = patch(z.shape)
    support = manual_support(z.shape)
    if case == 'one_side':
        support[:, 30:] = False
    elif case == 'too_far':
        support[0, 0] = True
    elif case == 'too_few':
        support[:] = False
        support[10, 10:15] = True
    elif case == 'line':
        support[:] = False
        support[20, 10:40] = True
    elif case == 'overlap':
        support = mask.copy()
    elif case == 'step':
        z[:, 26:] += .15
    elif case == 'unknown':
        z[support] = np.nan
    elif case == 'wrong_shape':
        support = support[:-1]
    session = RepairSession(snapshot_file(tmp_path, z))
    if case == 'edited':
        session.methods[support] = 1
    with pytest.raises(ValueError):
        session.preview_region(mask, support_mask=support)
    np.testing.assert_array_equal(session.working, z)
    assert not session.undo_stack


def test_manual_support_audit_roundtrip(tmp_path):
    z = floor()
    mask = patch(z.shape)
    z[mask] += .12
    session = RepairSession(snapshot_file(tmp_path, z))
    polygons = [[[1., 2.], [3., 2.], [3., 4.], [1., 4.]]]
    p = session.preview_region(mask, support_mask=manual_support(z.shape), support_vertices=polygons)
    session.apply(p)
    saved = session.save(tmp_path/'manual_review')
    restored = RepairSession.load(saved)
    assert restored.audit[0]['support_vertices_map_xy'] == polygons
    assert restored.audit[0]['fit']['support_mode'] == 'manual_floor'
    np.testing.assert_array_equal(restored.working, session.working)


def test_stale_preview_and_redo_branch(tmp_path):
    z = floor()
    mask = patch(z.shape)
    z[mask] += .1
    session = RepairSession(snapshot_file(tmp_path, z))
    preview = session.preview_region(mask)
    session.apply(preview)
    with pytest.raises(ValueError, match='stale'):
        session.apply(preview)
    session.undo()
    with pytest.raises(ValueError, match='stale'):
        session.apply(preview)
    session.apply(session.preview_region(mask))
    assert not session.redo()


def test_save_reopen_mask_time_provenance_and_no_overwrite(tmp_path):
    z = floor()
    mask = patch(z.shape)
    z[mask] += .12
    z[20, 26] = np.nan
    snapshot = snapshot_file(tmp_path, z)
    session = RepairSession(snapshot)
    session.apply(session.preview_region(mask))
    destination = session.save(tmp_path/'review')
    assert hashlib.sha256(snapshot.source.read_bytes()).hexdigest() == snapshot.digest
    observed = read_snapshot(destination/'observed_only/map.rghm')
    np.testing.assert_array_equal(np.isfinite(observed.grid), np.isfinite(z))
    np.testing.assert_array_equal(observed.cell_stamp_ns, snapshot.cell_stamp_ns)
    np.testing.assert_array_equal(observed.variance, snapshot.variance)
    np.testing.assert_array_equal(observed.usable, snapshot.usable)
    assert np.isnan(observed.grid[20, 26])
    with np.load(destination/'repair_layers.npz', allow_pickle=False) as data:
        assert data['inferred'].sum() == 1
        assert np.isfinite(data['repaired'][20, 26])
    manifest = json.loads((destination/'repair.json').read_text())
    assert manifest['navigation_approved'] is False
    assert manifest['inferred_cells'] == 1
    restored = RepairSession.load(destination)
    np.testing.assert_array_equal(restored.working, session.working)
    np.testing.assert_array_equal(restored.methods, session.methods)
    assert not restored.undo()  # reopening never fabricates historical undo state
    with pytest.raises(FileExistsError):
        session.save(destination)
    with pytest.raises(FileExistsError):
        session.save(snapshot.source.parent)


def test_changed_source_or_bundle_rejected(tmp_path):
    snapshot = snapshot_file(tmp_path, floor())
    session = RepairSession(snapshot)
    destination = session.save(tmp_path/'review')
    (destination/'repaired_preview.pcd').write_bytes(b'tampered')
    with pytest.raises(ValueError, match='hash mismatch'):
        RepairSession.load(destination)
    snapshot.source.write_text(snapshot.text+'\n')
    with pytest.raises(ValueError, match='source changed'):
        session.save(tmp_path/'not_created')
    assert not (tmp_path/'not_created').exists()


@pytest.mark.parametrize('changes', [{'ring_m': 0}, {'max_rms_m': float('nan')},
    {'max_correction_m': -1}, {'max_slope_deg': 90}, {'ring_m': 3},
    {'min_support': 2}, {'max_hole_cells': 4.5}, {'min_support': 20.5}])
def test_settings_validation(changes):
    with pytest.raises(ValueError):
        RepairSettings(**changes).validate()
