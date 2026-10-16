from pathlib import Path
import sys
import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from rubi_heightmap_postprocess.filters import Settings, process
from rubi_heightmap_postprocess.map_io import read_snapshot, write_snapshot


def test_known_noisy_steps_and_slope():
    yy,xx=np.mgrid[:90,:120];gt=.003*xx+.001*yy
    gt[:,40:80]+=.05;gt[:,80:]+=.15
    rng=np.random.default_rng(47);raw=gt+rng.normal(0,.0015,gt.shape)
    result=process(raw,.05)
    inner=result['eligible'];candidate=result['candidate']
    assert np.quantile(abs(candidate[inner]-gt[inner]),.95)<np.quantile(abs(raw[inner]-gt[inner]),.95)*.5
    assert np.max(abs(candidate-raw))<=Settings().max_change_m+1e-12
    np.testing.assert_array_equal(candidate[result['protected']],raw[result['protected']])
    # Ground-truth geometry appears only in tests, never algorithm input.
    for lo,hi in [(5,35),(45,75),(85,115)]:
        residual=candidate[5:-5,lo:hi]-gt[5:-5,lo:hi]
        assert abs(np.median(residual))<.0005


def test_unknown_holes_and_isolated_spike_preserved_for_review():
    raw=np.zeros((40,50));raw[10:17,14:20]=np.nan;raw[25,30]=.08
    result=process(raw,.05)
    for method in ('candidate','baseline'):
        np.testing.assert_array_equal(np.isfinite(result[method]),np.isfinite(raw))
    assert result['protected'][25,30]
    assert result['candidate'][25,30]==.08


def test_exact_ramp_no_axis_or_offset_change():
    yy,xx=np.mgrid[:31,:47];raw=-.47+.004*xx-.002*yy
    result=process(raw,.05)
    np.testing.assert_allclose(result['candidate'],raw,atol=1e-12)


def test_broad_curvature_is_not_flattened():
    yy,xx=np.mgrid[:70,:100];curve=.06*np.sin(xx/20.)+.02*np.cos(yy/20.)
    raw=curve+np.random.default_rng(20).normal(0,.001,curve.shape)
    result=process(raw,.05)
    assert any(not p['accepted_as_one_plane'] for p in result['regions'])
    assert np.std(result['candidate'])>.9*np.std(curve)
    assert np.max(abs(result['candidate']-raw))<=.01+1e-12
    assert np.any(result['method']==2)


def fixture_file(path):
    # Absolute nanoseconds beyond float64's exact integer range.
    path.write_text('RUBI_GLOBAL_HEIGHTMAP 1\n"map" 0.05 -2.5 3.5 4 3 0.04 1791948579928580337 4\n'
                    '0 0.10 0.003 1791948579928580331 1\n'
                    '3 0.15 0.004 1791948579928580332 1\n'
                    '6 0.20 0.005 1791948579928580333 0\n'
                    '11 0.30 0.006 1791948579928580337 1\n')


def test_roundtrip_non_square_geometry_and_exact_timestamps(tmp_path):
    source=tmp_path/'raw.rghm';fixture_file(source);before=source.read_bytes()
    a=read_snapshot(source);x,y=a.xy()
    assert x[0,0]==pytest.approx(-2.475);assert y[2,3]==pytest.approx(3.625)
    z=a.grid.copy();z[0,0]+=.001
    write_snapshot(a,z,tmp_path/'processed.rghm');b=read_snapshot(tmp_path/'processed.rghm')
    assert b.stamp_ns==1791948579928580337
    np.testing.assert_array_equal(a.cell_stamp_ns,b.cell_stamp_ns)
    np.testing.assert_array_equal(a.variance,b.variance)
    np.testing.assert_array_equal(a.usable,b.usable)
    assert b.height[2]==a.height[2]  # unusable record is retained
    assert source.read_bytes()==before
    with pytest.raises(ValueError):write_snapshot(a,z,source)
    z[1,1]=0.
    with pytest.raises(ValueError):write_snapshot(a,z,tmp_path/'filled.rghm')


@pytest.mark.parametrize('bad',[
    ('11 0.30','3 0.30'),('11 0.30','12 0.30'),('0.003','-0.003'),
    ('0.003','0.05'),('0.10','nan'),('0331 1','0340 1'),('0331 1','0331 2'),
])
def test_malformed_snapshot_rejected(tmp_path,bad):
    source=tmp_path/'raw.rghm';fixture_file(source)
    source.write_text(source.read_text().replace(*bad))
    with pytest.raises(ValueError):read_snapshot(source)


@pytest.mark.parametrize('changes',[{'local_window':4},{'max_change_m':-1},{'edge_guard_cells':1.5},{'local_min_points':100},{'region_p95_m':float('nan')}])
def test_invalid_settings(changes):
    with pytest.raises(ValueError):Settings(**changes).validate()
