"""Strict reader for the wrapper's RGHM v1 snapshot, retaining integer times."""
from dataclasses import dataclass
from pathlib import Path
import hashlib
import shlex
import numpy as np


@dataclass
class Snapshot:
    source: Path
    text: str
    digest: str
    frame: str
    resolution: float
    origin: tuple
    dimensions: tuple
    stamp_ns: int
    index: np.ndarray
    height: np.ndarray
    variance: np.ndarray
    cell_stamp_ns: np.ndarray
    usable: np.ndarray
    offset: tuple
    grid: np.ndarray

    def xy(self):
        """Coordinates of cropped array cell centers; row=+Y, column=+X."""
        iy, ix = np.indices(self.grid.shape)
        return (self.origin[0] + (ix + self.offset[0] + .5) * self.resolution,
                self.origin[1] + (iy + self.offset[1] + .5) * self.resolution)


def read_snapshot(path):
    source = Path(path).resolve()
    payload = source.read_bytes()
    text = payload.decode('utf-8')
    lines = text.splitlines()
    if len(lines) < 2 or lines[0] != 'RUBI_GLOBAL_HEIGHTMAP 1':
        raise ValueError('Expected RUBI_GLOBAL_HEIGHTMAP version 1')
    h = shlex.split(lines[1])
    if len(h) != 9:
        raise ValueError('Malformed snapshot header')
    frame, res, ox, oy = h[0], *map(float, h[1:4])
    nx, ny = map(int, h[4:6])
    cutoff, stamp, count = float(h[6]), int(h[7]), int(h[8])
    if (not frame or not np.all(np.isfinite([res, ox, oy, cutoff])) or res <= 0
            or cutoff < 0 or nx < 1 or ny < 1 or nx * ny > 25_000_000
            or stamp < 0 or stamp > np.iinfo(np.int64).max or not 0 <= count <= nx*ny
            or len(lines) != count + 2):
        raise ValueError('Invalid geometry, timestamp or cell count')
    index = np.empty(count, np.int64)
    height = np.empty(count); variance = np.empty(count)
    times = np.empty(count, np.int64); usable = np.empty(count, bool)
    for n, line in enumerate(lines[2:]):
        p = line.split()
        if len(p) != 5:
            raise ValueError('Malformed cell record')
        i, z, v, t, u = int(p[0]), float(p[1]), float(p[2]), int(p[3]), int(p[4])
        if (not 0 <= i < nx*ny or not np.isfinite(z) or not np.isfinite(v)
                or not 0 <= v <= cutoff or not 0 < t <= stamp or u not in (0, 1)):
            raise ValueError('Invalid cell index, height, variance, timestamp or usability')
        index[n], height[n], variance[n], times[n], usable[n] = i, z, v, t, bool(u)
    if len(np.unique(index)) != count:
        raise ValueError('Duplicate cell index')
    if not usable.any():
        raise ValueError('Snapshot has no usable cells')
    xs, ys = index[usable] % nx, index[usable] // nx
    xmin, ymin = int(xs.min()), int(ys.min())
    grid = np.full((int(ys.max())-ymin+1, int(xs.max())-xmin+1), np.nan)
    grid[ys-ymin, xs-xmin] = height[usable]
    return Snapshot(source, text, hashlib.sha256(payload).hexdigest(), frame, res,
                    (ox, oy), (nx, ny), stamp, index, height, variance, times, usable,
                    (xmin, ymin), grid)


def write_snapshot(snapshot, grid, destination):
    """Change only usable Z fields; retain source state/variance/time verbatim.

    Source variance is provenance, NOT a calibrated posterior for processed Z.
    The comparison manifest records this limitation. Unknowns cannot be filled.
    """
    if grid.shape != snapshot.grid.shape or not np.array_equal(np.isfinite(grid), np.isfinite(snapshot.grid)):
        raise ValueError('Postprocessing must preserve the observed mask and geometry')
    if Path(destination).resolve() == snapshot.source:
        raise ValueError('Refusing to overwrite the input snapshot')
    lines = snapshot.text.splitlines()
    nx = snapshot.dimensions[0]
    for n in np.flatnonzero(snapshot.usable):
        i = snapshot.index[n]
        z = grid[i // nx - snapshot.offset[1], i % nx - snapshot.offset[0]]
        parts = lines[n+2].split()
        parts[1] = format(float(z), '.17g')
        lines[n+2] = ' '.join(parts)
    with Path(destination).open('x') as out:
        out.write('\n'.join(lines) + '\n')


def write_pcd(snapshot, grid, destination, provenance='metrics.json'):
    x, y = snapshot.xy(); valid = np.isfinite(grid)
    xyz = np.column_stack([x[valid], y[valid], grid[valid]]).astype('<f4')
    header = (f'# Offline RUBI preview; provenance in {provenance}\nVERSION .7\n'
              'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\n'
              f'POINTS {len(xyz)}\nDATA binary\n')
    with Path(destination).open('xb') as out:
        out.write(header.encode()); out.write(xyz.tobytes())
