"""Small, bounded planar velocity fit for controller-only odometry.

Use a continuous odometry frame, never map poses corrected by global ICP.
Invalid/reset samples produce no estimate, rather than a fabricated zero.
"""
from collections import deque
import math


class PoseVelocity:
    def __init__(self, window=.4, min_span=.2, max_gap=.5,
                 max_speed=3., max_yaw_rate=4.):
        if not (0 < min_span <= window <= max_gap and
                max_speed > 0 and max_yaw_rate > 0):
            raise ValueError('Invalid velocity fit limits')
        self.window, self.min_span, self.max_gap = window, min_span, max_gap
        self.max_speed, self.max_yaw_rate = max_speed, max_yaw_rate
        self.samples = deque(maxlen=128)
        self.reason = 'warming_up'

    def reset(self, reason='reset'):
        self.samples.clear()
        self.reason = reason

    def add(self, stamp, x, y, yaw):
        if not all(math.isfinite(v) for v in (stamp, x, y, yaw)):
            self.reset('nonfinite_pose')
            return None
        if self.samples:
            t0, x0, y0, a0 = self.samples[-1]
            dt = stamp - t0
            if dt <= 0 or dt > self.max_gap:
                self.reset('time_reset_or_gap')
            else:
                da = math.remainder(yaw-a0, 2*math.pi)
                if (math.hypot(x-x0, y-y0)/dt > self.max_speed or
                        abs(da)/dt > self.max_yaw_rate):
                    self.reset('pose_jump')
                    return None
                yaw = a0 + da
        self.samples.append((stamp, x, y, yaw))
        while len(self.samples) > 1 and stamp-self.samples[0][0] > self.window+1e-7:
            self.samples.popleft()
        if len(self.samples) < 3 or stamp-self.samples[0][0] < self.min_span-1e-7:
            self.reason = 'warming_up'
            return None
        # Center time near zero, including long-running real sensor epochs.
        ts = [p[0]-stamp for p in self.samples]
        mt = sum(ts)/len(ts)
        denominator = sum((t-mt)**2 for t in ts)
        if denominator < 1e-12:
            self.reason = 'insufficient_time_span'
            return None
        slopes = [sum((t-mt)*p[axis] for t,p in zip(ts,self.samples))/denominator
                  for axis in (1,2,3)]
        vx, vy, wz = slopes
        c, s = math.cos(yaw), math.sin(yaw)
        self.reason = 'ready'
        return c*vx+s*vy, -s*vx+c*vy, wz
