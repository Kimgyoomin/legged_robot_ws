"""Explicit, undoable offline terrain repair. No live ROS or automatic approval."""
from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np
from scipy import ndimage as ndi
from scipy.spatial import ConvexHull, QhullError, cKDTree

from .map_io import read_snapshot, write_pcd, write_snapshot


@dataclass(frozen=True)
class RepairSettings:
    ring_m: float = .30
    max_rms_m: float = .005
    max_slope_deg: float = 15.
    max_correction_m: float = .30
    max_selection_span_m: float = 2.
    min_support: int = 20
    max_hole_cells: int = 4
    max_hole_span_m: float = .10
    max_support_distance_m: float = 1.

    def validate(self):
        if not all(np.isfinite(v) and v > 0 for v in asdict(self).values()):
            raise ValueError('Settings must be finite and positive.')
        if not 0 < self.max_slope_deg < 60 or self.ring_m > 2:
            raise ValueError('Slope must be below 60 deg; support ring at most 2 m.')
        if (type(self.min_support) is not int or self.min_support < 6 or
                type(self.max_hole_cells) is not int or self.max_hole_cells > 100):
            raise ValueError('Invalid support count / hole cell limit.')


@dataclass
class Proposal:
    revision: int
    indices: np.ndarray
    values: np.ndarray
    method: int  # 1: manually selected patch; 2: small-hole candidate
    details: dict


class SupportFitError(ValueError):
    """Rejected support with exact cells for a GUI diagnostic, never a proposal."""
    def __init__(self, message, support_indices, outlier_indices):
        super().__init__(message)
        self.support_indices = np.asarray(support_indices, dtype=np.int64)
        self.outlier_indices = np.asarray(outlier_indices, dtype=np.int64)


def polygon_mask(snapshot, vertices):
    """Select cell centers in map XY, accounting for cropped snapshot offsets."""
    from matplotlib.path import Path as MplPath
    vertices = np.asarray(vertices, dtype=float)
    if vertices.ndim != 2 or vertices.shape[1] != 2 or len(vertices) < 3 or not np.isfinite(vertices).all():
        raise ValueError('Select a polygon with at least three finite vertices.')
    r = snapshot.resolution
    ox = snapshot.origin[0] + snapshot.offset[0] * r
    oy = snapshot.origin[1] + snapshot.offset[1] * r
    h, w = snapshot.grid.shape
    x0 = max(0, int(np.floor((vertices[:, 0].min()-ox)/r)))
    x1 = min(w, int(np.ceil((vertices[:, 0].max()-ox)/r))+1)
    y0 = max(0, int(np.floor((vertices[:, 1].min()-oy)/r)))
    y1 = min(h, int(np.ceil((vertices[:, 1].max()-oy)/r))+1)
    mask = np.zeros((h, w), bool)
    if x1 <= x0 or y1 <= y0:
        return mask
    yy, xx = np.mgrid[y0:y1, x0:x1]
    points = np.column_stack((ox+(xx.ravel()+.5)*r, oy+(yy.ravel()+.5)*r))
    mask[y0:y1, x0:x1] = MplPath(vertices).contains_points(points).reshape(xx.shape)
    return mask


def _fit_patch(raw, working, methods, rows, cols, resolution, settings, support_mask=None):
    """Automatic ring or explicit clean floor; never fit to the target itself."""
    y0, y1, x0, x1 = int(rows.min()), int(rows.max()), int(cols.min()), int(cols.max())
    if support_mask is None:
        pad = max(1, int(np.ceil(settings.ring_m/resolution)))
        by0, by1 = max(0, y0-pad), min(raw.shape[0], y1+pad+1)
        bx0, bx1 = max(0, x0-pad), min(raw.shape[1], x1+pad+1)
        yy, xx = np.mgrid[by0:by1, bx0:bx1]
        selected = np.zeros(yy.shape, bool)
        selected[rows-by0, cols-bx0] = True
        ring = ndi.binary_dilation(selected, iterations=pad) & ~selected
        z = raw[by0:by1, bx0:bx1]
        support = ring & np.isfinite(z) & (methods[by0:by1, bx0:bx1] == 0)
        if support.sum() < settings.min_support or support.sum() < .5 * ring.sum():
            raise ValueError('Insufficient original floor support. Select clean floor manually on both sides.')
        for side in (xx < x0, xx > x1, yy < y0, yy > y1):
            if np.count_nonzero(support & side) < 3:
                raise ValueError('Automatic ring lacks floor on four sides. Select clean floor on both sides.')
        sy, sx = yy[support], xx[support]
    else:
        if support_mask.shape != raw.shape or support_mask.dtype != bool:
            raise ValueError('Invalid explicit floor support mask.')
        support = support_mask & np.isfinite(raw) & (methods == 0)
        support[rows, cols] = False
        sy, sx = np.where(support)
        if len(sy) < settings.min_support:
            raise ValueError(f'Choose more clean, ORIGINAL floor: at least {settings.min_support} cells '
                             '(target, unknown and already edited cells do not count).')
        target_xy = np.column_stack((cols, rows))*resolution
        floor_xy = np.column_stack((sx, sy))*resolution
        distances = cKDTree(target_xy).query(floor_xy)[0]
        if distances.max() > settings.max_support_distance_m+1e-9:
            raise ValueError(f'Floor too far away. Clear floor selection; pick patches within '
                             f'{settings.max_support_distance_m:g} m of the artifact.')
        # Unlike the automatic ring, two opposite wide patches can suffice.
        # Require interpolation inside their convex hull, not one-sided extrapolation.
        try:
            hull = ConvexHull(floor_xy)
        except QhullError as error:
            raise ValueError('Floor support is too narrow. Select wider patches on both sides.') from error
        if np.any(target_xy@hull.equations[:, :2].T+hull.equations[:, 2] > 1e-9):
            raise ValueError('Add clean floor on the OTHER side too. Floor patches must surround '
                             'the entire artifact (make them wider along its sides).')
    cx, cy = float(cols.mean()), float(rows.mean())
    design = np.column_stack(((sx-cx)*resolution, (sy-cy)*resolution, np.ones(len(sy))))
    values = raw[sy, sx]
    coeff, _, rank, _ = np.linalg.lstsq(design, values, rcond=None)
    if rank != 3:
        raise ValueError('Degenerate floor support.')
    for _ in range(3):
        residual = values-design@coeff
        scale = max(.0005, 1.4826*np.median(abs(residual-np.median(residual))))
        weights = np.sqrt(np.minimum(1., 2.5*scale/np.maximum(abs(residual), 1e-12)))
        coeff = np.linalg.lstsq(design*weights[:, None], values*weights, rcond=None)[0]
    residual = values-design@coeff
    rms = float(np.sqrt(np.mean(residual**2)))
    peak = float(np.max(abs(residual)))
    slope = float(np.rad2deg(np.arctan(np.linalg.norm(coeff[:2]))))
    # Check ALL support, not only RANSAC/robust inliers: don't bridge a step.
    support_indices = np.ravel_multi_index((sy, sx), raw.shape)
    peak_limit = max(.015, settings.max_rms_m*3)
    if rms > settings.max_rms_m or peak > peak_limit:
        bad = abs(residual) > (peak_limit if peak > peak_limit else settings.max_rms_m)
        raise SupportFitError(
            f'FLOOR support, not artifact height: RMS {rms*1000:.1f}/{settings.max_rms_m*1000:g} mm, '
            f'peak {peak*1000:.1f}/{peak_limit*1000:g} mm. Red crosses mark conflicting support. '
            'Select clean floor patches on both sides, excluding other footprints/steps.',
            support_indices, support_indices[bad])
    if slope > settings.max_slope_deg:
        raise ValueError(f'Floor slope {slope:.1f} deg exceeds limit.')
    predicted = coeff[0]*(cols-cx)*resolution+coeff[1]*(rows-cy)*resolution+coeff[2]
    old = working[rows, cols]
    known = np.isfinite(old)
    change = float(np.max(abs(predicted[known]-old[known]))) if known.any() else 0.
    if change > settings.max_correction_m:
        raise ValueError(f'Correction {change:.3f} m exceeds limit; possible real obstacle.')
    return predicted, {'support_cells': len(sy), 'support_rms_m': rms,
                       'support_peak_m': peak, 'slope_deg': slope,
                       'support_mode': 'manual_floor' if support_mask is not None else 'automatic_ring',
                       'support_indices': support_indices.tolist(),
                       'maximum_observed_change_m': change,
                       'center_crop_cells_xy': [cx, cy], 'plane_coeff_local_xy': coeff.tolist()}


class RepairSession:
    def __init__(self, snapshot):
        self.snapshot = snapshot
        self.raw = snapshot.grid.copy()
        self.working = self.raw.copy()
        self.methods = np.zeros(self.raw.shape, np.uint8)
        self.revision = 0
        self.undo_stack = []
        self.redo_stack = []
        self.audit = []

    def preview_region(self, mask, settings=RepairSettings(), vertices=None,
                       support_mask=None, support_vertices=None):
        settings.validate()
        if mask.shape != self.raw.shape or mask.dtype != bool:
            raise ValueError('Invalid selection mask.')
        rows, cols = np.where(mask)
        if not len(rows):
            raise ValueError('Select a nonempty patch first.')
        span = max(np.ptp(rows)+1, np.ptp(cols)+1)*self.snapshot.resolution
        if span > settings.max_selection_span_m + 1e-9:
            raise ValueError(f'Select a smaller patch (at most {settings.max_selection_span_m:g} m across).')
        values, quality = _fit_patch(self.raw, self.working, self.methods, rows, cols,
                                     self.snapshot.resolution, settings, support_mask=support_mask)
        details = {'kind': 'manual_region', 'settings': asdict(settings), 'fit': quality,
                   'vertices_map_xy': vertices, 'cells': len(rows),
                   'support_vertices_map_xy': support_vertices,
                   'newly_filled_cells': int((~np.isfinite(self.working[mask])).sum())}
        return Proposal(self.revision, np.ravel_multi_index((rows, cols), self.raw.shape), values, 1, details)

    def preview_holes(self, settings=RepairSettings(), selection=None):
        settings.validate()
        if selection is not None and (selection.shape != self.raw.shape or selection.dtype != bool):
            raise ValueError('Invalid selection mask.')
        labels, _ = ndi.label(~np.isfinite(self.working), structure=np.ones((3, 3)))
        exterior = set(np.unique(np.concatenate((labels[0], labels[-1], labels[:, 0], labels[:, -1]))))
        sizes = np.bincount(labels.ravel())
        indices, values, fits = [], [], []
        rejected = Counter()
        for label, sl in enumerate(ndi.find_objects(labels), start=1):
            if sl is None or label in exterior:
                continue
            if (sizes[label] > settings.max_hole_cells or
                    max(sl[0].stop-sl[0].start, sl[1].stop-sl[1].start)*self.snapshot.resolution
                    > settings.max_hole_span_m + 1e-9):
                continue
            rows, cols = np.where(labels[sl] == label)
            rows += sl[0].start
            cols += sl[1].start
            if selection is not None and not selection[rows, cols].all():
                continue
            try:
                predicted, fit = _fit_patch(self.raw, self.working, self.methods, rows, cols,
                                           self.snapshot.resolution, settings)
            except ValueError as error:
                rejected[str(error).split(':')[0]] += 1
                continue
            indices.extend(np.ravel_multi_index((rows, cols), self.raw.shape).tolist())
            values.extend(predicted.tolist())
            fits.append({'bbox_crop_yx': [sl[0].start, sl[0].stop, sl[1].start, sl[1].stop], **fit})
        details = {'kind': 'small_holes', 'settings': asdict(settings), 'accepted_holes': len(fits),
                   'rejected_holes': dict(rejected), 'fits': fits, 'cells': len(indices),
                   'newly_filled_cells': len(indices), 'selection_limited': selection is not None}
        return Proposal(self.revision, np.asarray(indices, dtype=np.int64), np.asarray(values), 2, details)

    def apply(self, proposal):
        if proposal.revision != self.revision:
            raise ValueError('Preview is stale; generate a new preview.')
        ix = proposal.indices
        if (not len(ix) or proposal.values.shape != ix.shape or not np.isfinite(proposal.values).all()
                or np.any(ix < 0) or np.any(ix >= self.raw.size) or len(np.unique(ix)) != len(ix)
                or proposal.method not in (1, 2)):
            raise ValueError('Empty or invalid proposal.')
        operation = (ix.copy(), self.working.flat[ix].copy(), self.methods.flat[ix].copy(),
                     proposal.values.copy(), proposal.method, proposal.details)
        self.working.flat[ix] = proposal.values
        self.methods.flat[ix] = proposal.method
        self.undo_stack.append(operation)
        self.redo_stack.clear()
        self.revision += 1
        self.audit.append({'action': 'apply', **proposal.details})

    def undo(self):
        if not self.undo_stack:
            return False
        op = self.undo_stack.pop()
        self.working.flat[op[0]], self.methods.flat[op[0]] = op[1], op[2]
        self.redo_stack.append(op)
        self.revision += 1
        self.audit.append({'action': 'undo', 'kind': op[5]['kind']})
        return True

    def redo(self):
        if not self.redo_stack:
            return False
        op = self.redo_stack.pop()
        self.working.flat[op[0]], self.methods.flat[op[0]] = op[3], op[4]
        self.undo_stack.append(op)
        self.revision += 1
        self.audit.append({'action': 'redo', 'kind': op[5]['kind']})
        return True

    def save(self, destination):
        """Review bundle only. NEVER forge acquisition times for filled cells."""
        destination = Path(destination)
        if hashlib.sha256(self.snapshot.source.read_bytes()).hexdigest() != self.snapshot.digest:
            raise ValueError('Original source changed on disk; refusing save.')
        destination.mkdir(parents=True, exist_ok=False)
        (destination/'original').mkdir()
        shutil.copyfile(self.snapshot.source, destination/'original/map.rghm')
        measured = self.working.copy()
        measured[~np.isfinite(self.raw)] = np.nan
        (destination/'observed_only').mkdir()
        write_snapshot(self.snapshot, measured, destination/'observed_only/map.rghm')
        write_pcd(self.snapshot, self.working, destination/'repaired_preview.pcd', provenance='repair.json')
        np.savez_compressed(destination/'repair_layers.npz', repaired=self.working, method=self.methods,
                            originally_observed=np.isfinite(self.raw),
                            inferred=(~np.isfinite(self.raw) & np.isfinite(self.working)))
        manifest = {'format': 'rubi_height_repair_review_v1', 'navigation_approved': False,
                    'original_source': str(self.snapshot.source), 'source_sha256': self.snapshot.digest,
                    'frame': self.snapshot.frame, 'resolution': self.snapshot.resolution,
                    'original_snapshot_stamp_ns': self.snapshot.stamp_ns,
                    'edited_cells': int((self.methods != 0).sum()),
                    'inferred_cells': int((~np.isfinite(self.raw) & np.isfinite(self.working)).sum()),
                    'audit': self.audit,
                    'semantics': 'PCD/NPZ include inferred heights. observed_only RGHM preserves the original '
                                 'observation mask and source timestamps/variance as provenance, NOT repaired '
                                 'measurement uncertainty. Filled cells have no measured time or calibrated '
                                 'variance and are NOT exported as observed RGHM cells.',
                    'files_sha256': {str(p.relative_to(destination)): hashlib.sha256(p.read_bytes()).hexdigest()
                                     for p in destination.rglob('*') if p.is_file()}}
        # Commit marker written last: incomplete exports cannot be reopened.
        (destination/'repair.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
        return destination

    @classmethod
    def load(cls, directory):
        directory = Path(directory)
        manifest = json.loads((directory/'repair.json').read_text())
        if manifest.get('format') != 'rubi_height_repair_review_v1':
            raise ValueError('Unsupported repair session.')
        for name in ('original/map.rghm', 'repair_layers.npz', 'observed_only/map.rghm', 'repaired_preview.pcd'):
            if hashlib.sha256((directory/name).read_bytes()).hexdigest() != manifest['files_sha256'].get(name):
                raise ValueError('Saved repair bundle hash mismatch: '+name)
        session = cls(read_snapshot(directory/'original/map.rghm'))
        if session.snapshot.digest != manifest['source_sha256']:
            raise ValueError('Saved original hash mismatch.')
        with np.load(directory/'repair_layers.npz', allow_pickle=False) as data:
            z, method = data['repaired'], data['method']
            if (z.shape != session.raw.shape or method.shape != z.shape or np.isinf(z).any()
                    or not np.isin(method, [0, 1, 2]).all()
                    or np.any(np.isfinite(session.raw) & ~np.isfinite(z))
                    or np.any((method != 0) & ~np.isfinite(z))
                    or not np.array_equal(z[method == 0], session.raw[method == 0], equal_nan=True)
                    or not np.array_equal(data['originally_observed'], np.isfinite(session.raw))
                    or not np.array_equal(data['inferred'], ~np.isfinite(session.raw) & np.isfinite(z))):
                raise ValueError('Invalid repair layers.')
            session.working = z.copy()
            session.methods = method.astype(np.uint8)
        session.audit = list(manifest['audit'])
        session.audit.append({'action': 'reopen', 'undo_history': 'new session only'})
        return session
