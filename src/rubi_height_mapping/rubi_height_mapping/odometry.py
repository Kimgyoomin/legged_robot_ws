"""Bounded timestamped SE(3) poses. No ROS, SciPy, extrapolation or TF fallback."""
from bisect import bisect_left
from dataclasses import dataclass
import math
import numpy as np
from .pointcloud import quaternion_matrix


def unit_quaternion(q):
    q = np.asarray(q, dtype=np.float64)
    if q.shape != (4,) or not np.isfinite(q).all():
        raise ValueError('Odometry quaternion must contain four finite values')
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-10:
        raise ValueError('Invalid odometry quaternion norm')
    return q / norm


def slerp(a, b, fraction):
    a, b = unit_quaternion(a), unit_quaternion(b)
    dot = float(np.dot(a, b))
    if dot < 0:
        b, dot = -b, -dot
    dot = float(np.clip(dot, -1, 1))
    if dot > .9995:
        return unit_quaternion(a + fraction * (b - a))
    angle = math.acos(dot)
    return (math.sin((1-fraction)*angle)*a + math.sin(fraction*angle)*b) / math.sin(angle)


def compose_pose(position, quaternion, offset_position, offset_quaternion):
    """T_parent_child * T_child_sensor, quaternions are xyzw."""
    a, b = unit_quaternion(quaternion), unit_quaternion(offset_quaternion)
    xyz = a[3]*b[:3] + b[3]*a[:3] + np.cross(a[:3], b[:3])
    w = a[3]*b[3] - np.dot(a[:3], b[:3])
    p = np.asarray(position) + quaternion_matrix(a)[:3, :3] @ offset_position
    return p, unit_quaternion(np.r_[xyz, w])


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    position: np.ndarray
    quaternion: np.ndarray


class OdometryBuffer:
    def __init__(self, map_frame, cache_seconds=10.0, max_gap_seconds=.2, max_samples=2000):
        if not isinstance(map_frame, str) or not map_frame.strip():
            raise ValueError('Empty odometry parent frame')
        if not all(math.isfinite(v) and v > 0 for v in (cache_seconds, max_gap_seconds)):
            raise ValueError('Odometry cache and interpolation gap must be positive and finite')
        if cache_seconds < max_gap_seconds or max_samples < 2:
            raise ValueError('Odometry cache must hold an interpolation bracket')
        self.map_frame = map_frame
        self.cache_ns = int(cache_seconds * 1e9)
        self.max_gap_ns = int(max_gap_seconds * 1e9)
        self.max_samples = max_samples
        self.child_frame = None
        self.stamps = []
        self.poses = []

    def clear(self):
        self.stamps.clear()
        self.poses.clear()
        self.child_frame = None

    def add(self, stamp_ns, parent, child, position, quaternion):
        if not isinstance(stamp_ns, int) or stamp_ns < 0:
            raise ValueError('Invalid odometry timestamp')
        if parent != self.map_frame:
            raise ValueError(f"Odometry parent '{parent}' differs from map_frame '{self.map_frame}'")
        if not isinstance(child, str) or not child.strip() or child == parent:
            raise ValueError('Invalid odometry child frame')
        if self.child_frame is not None and child != self.child_frame:
            raise ValueError(f"Odometry child changed: '{self.child_frame}' -> '{child}'")
        p = np.asarray(position, dtype=np.float64)
        if p.shape != (3,) or not np.isfinite(p).all():
            raise ValueError('Invalid odometry position')
        q = unit_quaternion(quaternion)
        # A late duplicate must not rewrite a pose already used to fuse a scan.
        index = bisect_left(self.stamps, stamp_ns)
        if index < len(self.stamps) and self.stamps[index] == stamp_ns:
            return False
        if self.stamps and stamp_ns < self.stamps[-1] - self.cache_ns:
            return False
        self.child_frame = child
        self.stamps.insert(index, stamp_ns)
        self.poses.insert(index, PoseSample(stamp_ns, p.copy(), q))
        cut = max(bisect_left(self.stamps, self.stamps[-1] - self.cache_ns),
                  len(self.stamps) - self.max_samples)
        if cut > 0:
            del self.stamps[:cut]
            del self.poses[:cut]
        return True

    def lookup(self, stamp_ns):
        index = bisect_left(self.stamps, stamp_ns)
        if index < len(self.stamps) and self.stamps[index] == stamp_ns:
            return self.poses[index]
        if index == 0 or index == len(self.stamps):
            raise LookupError('No odometry bracket for scan timestamp; extrapolation is disabled')
        a, b = self.poses[index-1:index+1]
        gap = b.stamp_ns - a.stamp_ns
        if gap > self.max_gap_ns:
            raise LookupError(f'Odometry gap {gap/1e9:.3f}s exceeds {self.max_gap_ns/1e9:.3f}s')
        fraction = (stamp_ns - a.stamp_ns) / gap
        return PoseSample(stamp_ns, a.position + fraction*(b.position-a.position),
                          slerp(a.quaternion, b.quaternion, fraction))
