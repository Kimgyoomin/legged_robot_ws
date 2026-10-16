"""Same input-defined masks/ROIs for every method; no world truth in filtering."""
import warnings
import numpy as np


def median_residual(z):
    windows = np.lib.stride_tricks.sliding_window_view(np.pad(z, 1, constant_values=np.nan), (3, 3))
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        return z-np.nanmedian(windows, axis=(-2, -1))


def quantiles(values):
    a = np.asarray(values); a = a[np.isfinite(a)]
    return None if len(a) == 0 else np.quantile(a, [.05, .5, .95]).tolist()


def roi_measurements(snapshot, z, rois):
    x, y = snapshot.xy(); output = []
    for item in rois:
        center = item['center_xy_m']; angle = np.deg2rad(item.get('yaw_deg', 0.))
        u = (x-center[0])*np.cos(angle) + (y-center[1])*np.sin(angle)
        v = -(x-center[0])*np.sin(angle) + (y-center[1])*np.cos(angle)
        half = item['core_half_extent_m']
        core = (abs(u) < half[0]) & (abs(v) < half[1]) & np.isfinite(z)
        if item['kind'] == 'step':
            inner, outer = item['ring_inner_half_extent_m'], item['ring_outer_half_extent_m']
            support = ((abs(u) < outer[0]) & (abs(v) < outer[1])
                       & ((abs(u) > inner[0]) | (abs(v) > inner[1])) & np.isfinite(z))
        else:
            support = core
        if core.sum() < 12 or support.sum() < 25:
            output.append({'name': item['name'], 'kind': item['kind'], 'status': 'insufficient_observations'})
            continue
        design = np.column_stack([u[support], v[support], np.ones(support.sum())])
        keep = np.ones(support.sum(), bool)
        for _ in range(6):
            coeff, _, rank, _ = np.linalg.lstsq(design[keep], z[support][keep], rcond=None)
            residual = z[support]-design@coeff
            keep = abs(residual) < max(.005, np.quantile(abs(residual), .8))
        if rank != 3:
            output.append({'name': item['name'], 'kind': item['kind'], 'status': 'degenerate_plane'})
            continue
        core_residual = z[core]-(coeff[0]*u[core]+coeff[1]*v[core]+coeff[2])
        output.append({'name': item['name'], 'kind': item['kind'], 'status': 'measured',
                       'core_cells': int(core.sum()), 'support_cells': int(support.sum()),
                       'relative_height_p05_p50_p95_m': quantiles(core_residual),
                       'core_plane_abs_residual_p95_m': float(np.quantile(abs(core_residual), .95)),
                       'support_plane_abs_residual_p95_m': float(np.quantile(abs(residual), .95)),
                       'plane_coeff_local_uv': coeff.tolist()})
    return output


def compare_metrics(snapshot, result, rois):
    raw = snapshot.grid; valid = np.isfinite(raw)
    eligible = result['eligible']  # selected from RAW only, never changed per method
    outputs = {}
    for name, z in [('raw', raw), ('baseline', result['baseline']), ('candidate', result['candidate'])]:
        delta = z-raw
        hf = median_residual(z)[eligible]
        outputs[name] = {
            'usable_cells': int(np.isfinite(z).sum()),
            'changed_cells_gt_1um': int((valid & (abs(delta) > 1e-6)).sum()),
            'abs_change_p95_m': float(np.quantile(abs(delta[valid]), .95)),
            'abs_change_max_m': float(np.max(abs(delta[valid]))),
            'protected_cells_changed_gt_1um': int((result['protected'] & (abs(delta) > 1e-6)).sum()),
            'unknown_mask_preserved': bool(np.array_equal(np.isfinite(z), valid)),
            'raw_selected_interior_cells': int(eligible.sum()),
            'interior_local_median_abs_residual_p95_m': float(np.quantile(abs(hf), .95)) if len(hf) else None,
            'rois': roi_measurements(snapshot, z, rois),
        }
    return outputs
