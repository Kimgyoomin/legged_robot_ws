"""Conservative piecewise plane denoising, independent of the online estimator.

Unknown cells and protected boundaries are immutable. Globally nonplanar regions
receive bounded local plane denoising only, not whole-region flattening.
"""
from dataclasses import dataclass, asdict
import numpy as np
from scipy import ndimage as ndi


@dataclass(frozen=True)
class Settings:
    mean_radius_m: float = .06
    edge_span_m: float = .025
    edge_guard_cells: int = 1
    min_region_cells: int = 25
    region_p95_m: float = .004
    local_window: int = 7
    local_min_points: int = 12
    local_rms_m: float = .003
    max_change_m: float = .010
    max_slope_degrees: float = 35.

    def validate(self):
        values = asdict(self)
        if not all(np.isfinite(v) for v in values.values()):
            raise ValueError('Settings must be finite')
        for k in ('edge_guard_cells', 'min_region_cells', 'local_window', 'local_min_points'):
            if not isinstance(values[k], int) or isinstance(values[k], bool):
                raise ValueError(f'{k} must be an integer')
        if (self.mean_radius_m <= 0 or self.edge_span_m <= 0 or self.region_p95_m <= 0
                or self.local_rms_m <= 0 or self.max_change_m <= 0
                or not 0 <= self.edge_guard_cells <= 10 or self.min_region_cells < 3
                or self.local_window < 3 or self.local_window > 21 or self.local_window % 2 != 1
                or not 3 <= self.local_min_points <= self.local_window**2
                or not 0 < self.max_slope_degrees < 90):
            raise ValueError('Invalid filter settings')


def radial_mean(z, resolution, radius):
    """Finite-neighbor circular mean; restore unknown mask after filtering.

    Corresponds to MeanInRadiusFilter with an observed-mask gate. The upstream
    filter by itself can also fill unknown centers; that is excluded here.
    """
    n = int(np.ceil(radius/resolution))
    yy, xx = np.mgrid[-n:n+1, -n:n+1]
    kernel = ((xx*resolution)**2 + (yy*resolution)**2 <= radius**2+1e-12).astype(float)
    valid = np.isfinite(z)
    total = ndi.convolve(np.where(valid, z, 0.), kernel, mode='constant', cval=0.)
    count = ndi.convolve(valid.astype(float), kernel, mode='constant', cval=0.)
    result = np.full_like(z, np.nan)
    np.divide(total, count, out=result, where=valid & (count > 0))
    return result


def masks(z, settings):
    valid = np.isfinite(z)
    lo = ndi.minimum_filter(np.where(valid, z, np.inf), size=3, mode='constant', cval=np.inf)
    hi = ndi.maximum_filter(np.where(valid, z, -np.inf), size=3, mode='constant', cval=-np.inf)
    edges = valid & ((hi-lo) > settings.edge_span_m)
    guard = edges.copy()
    if settings.edge_guard_cells:
        guard = ndi.binary_dilation(guard, iterations=settings.edge_guard_cells)
    complete = ndi.binary_erosion(valid, structure=np.ones((3, 3)), border_value=0)
    eligible = valid & complete & ~guard
    return edges, valid & ~eligible, eligible


def fit_region(x, y, z):
    design = np.column_stack([x-x.mean(), y-y.mean(), np.ones(len(z))])
    coeff = np.linalg.lstsq(design, z, rcond=None)[0]
    for _ in range(3):
        residual = z-design@coeff
        scale = max(.0005, 1.4826*np.median(abs(residual-np.median(residual))))
        weights = np.minimum(1., 2.5*scale/np.maximum(abs(residual), 1e-12))
        coeff = np.linalg.lstsq(design*np.sqrt(weights[:, None]), z*np.sqrt(weights), rcond=None)[0]
    return design@coeff, coeff, np.linalg.matrix_rank(design)


def process(z, resolution, settings=Settings()):
    settings.validate()
    if z.ndim != 2 or not np.isfinite(resolution) or resolution <= 0 or np.isinf(z).any():
        raise ValueError('Expected 2D finite/NaN heights and positive resolution')
    z = np.asarray(z, dtype=float)
    edges, protected, eligible = masks(z, settings)
    labels, count = ndi.label(eligible)
    result = z.copy(); method = np.zeros(z.shape, np.uint8)
    plane_id = np.zeros(z.shape, np.int32)
    regions = []
    slope_limit = np.tan(np.deg2rad(settings.max_slope_degrees))
    for label, sl in enumerate(ndi.find_objects(labels), start=1):
        if sl is None:
            continue
        yy, xx = np.where(labels[sl] == label)
        yy += sl[0].start; xx += sl[1].start
        if len(xx) < settings.min_region_cells:
            continue
        pred, coeff, rank = fit_region(xx*resolution, yy*resolution, z[yy, xx])
        error = abs(pred-z[yy, xx]); p95 = float(np.quantile(error, .95))
        accepted = (rank == 3 and p95 <= settings.region_p95_m
                    and error.max() <= settings.max_change_m
                    and np.linalg.norm(coeff[:2]) <= slope_limit)
        regions.append({'region': label, 'cells': len(xx), 'accepted_as_one_plane': bool(accepted),
                        'fit_abs_residual_p95_m': p95, 'fit_abs_residual_max_m': float(error.max())})
        if accepted:
            result[yy, xx] = pred; method[yy, xx] = 1; plane_id[yy, xx] = label

    # Failed large regions can be curved/warped. Fit overlapping small planes,
    # using only neighbors in the same protected-boundary-separated component.
    window = settings.local_window; half = window//2
    zw = np.lib.stride_tricks.sliding_window_view(np.pad(z, half, constant_values=np.nan), (window, window))
    lw = np.lib.stride_tricks.sliding_window_view(np.pad(labels, half), (window, window))
    dy, dx = np.mgrid[-half:half+1, -half:half+1]
    design = np.column_stack([dx.ravel(), dy.ravel(), np.ones(window**2)])
    yy, xx = np.where(eligible & (method == 0))
    local_rms = np.full(z.shape, np.nan)
    for start in range(0, len(xx), 2048):
        by, bx = yy[start:start+2048], xx[start:start+2048]
        values = zw[by, bx].reshape(len(by), -1)
        member = lw[by, bx].reshape(len(by), -1) == labels[by, bx, None]
        member &= np.isfinite(values)
        values = np.where(member, values, 0.)
        n = member.sum(axis=1)
        weights = member.astype(float)
        coeff = np.zeros((len(by), 3)); solvable = np.zeros(len(by), bool)
        for _ in range(3):
            normal = np.einsum('nk,ki,kj->nij', weights, design, design)
            rhs = np.einsum('nk,nk,ki->ni', weights, values, design)
            solvable = (n >= settings.local_min_points) & (np.linalg.det(normal) > 1e-6)
            coeff[solvable] = np.linalg.solve(normal[solvable], rhs[solvable])
            residual = values-coeff@design.T
            # Huber reweighting in meters; center is not required to be an inlier.
            weights = member*np.minimum(1., settings.local_rms_m/np.maximum(abs(residual), 1e-12))
        residual = values-coeff@design.T
        rms = np.sqrt(np.sum(member*residual**2, axis=1)/np.maximum(n, 1))
        local_rms[by, bx] = np.where(solvable, rms, np.nan)
        support = (np.any(member & (design[:, 0] < 0), axis=1)
                   & np.any(member & (design[:, 0] > 0), axis=1)
                   & np.any(member & (design[:, 1] < 0), axis=1)
                   & np.any(member & (design[:, 1] > 0), axis=1))
        accept = (solvable & support & (rms <= settings.local_rms_m)
                  & (abs(coeff[:, 2]-z[by, bx]) <= settings.max_change_m)
                  & (np.linalg.norm(coeff[:, :2], axis=1)/resolution <= slope_limit))
        result[by[accept], bx[accept]] = coeff[accept, 2]
        method[by[accept], bx[accept]] = 2
    assert np.array_equal(np.isfinite(result), np.isfinite(z))
    assert np.array_equal(result[protected], z[protected])
    return {'candidate': result, 'baseline': radial_mean(z, resolution, settings.mean_radius_m),
            'edges': edges, 'protected': protected, 'eligible': eligible, 'region_id': labels,
            'plane_id': plane_id, 'method': method, 'local_fit_rms': local_rms, 'regions': regions}
