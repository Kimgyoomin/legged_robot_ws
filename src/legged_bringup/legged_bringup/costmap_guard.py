"""Conservative rectangular stop-envelope sweep over occupied/unknown cells.

Costs 1..253 are soft/inflated costs, not new obstacle geometry: testing an
expanded footprint against them would count the robot radius twice. Only
lethal (254) and unknown (255) cells block this check. This is a constant-Twist
prediction, not a model of the biped's measured braking/gait dynamics.
"""
import math
import numpy as np


def planar_yaw(q):
    values = (q.x, q.y, q.z, q.w)
    if not all(math.isfinite(v) for v in values):
        raise ValueError('nonfinite quaternion')
    norm = math.sqrt(sum(v*v for v in values))
    if not math.isfinite(norm) or norm < 1.e-9:
        raise ValueError('invalid quaternion norm')
    x, y, z, w = (v/norm for v in values)
    return math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))


def rectangle_half_extents(points):
    """Read the same centered, axis-aligned rectangle used by Collision Monitor.

    Reject unsupported shapes instead of silently checking a different envelope.
    Vertices must follow the perimeter, clockwise or counterclockwise.
    """
    if len(points) != 8:
        raise ValueError('Stop.points must contain four rectangle vertices')
    values = [float(v) for v in points]
    if not all(math.isfinite(v) for v in values):
        raise ValueError('Stop.points must be finite')
    vertices = list(zip(values[::2], values[1::2]))
    hx, hy = max(abs(x) for x, _ in vertices), max(abs(y) for _, y in vertices)
    if hx <= 0 or hy <= 0 or len(set(vertices)) != 4:
        raise ValueError('Stop.points must be a nonzero centered rectangle')
    for i, (x, y) in enumerate(vertices):
        nx, ny = vertices[(i+1) % 4]
        close = lambda a, b: math.isclose(a, b, rel_tol=0., abs_tol=1.e-9)
        if (not close(abs(x), hx) or not close(abs(y), hy) or
                close(x, nx) == close(y, ny)):
            raise ValueError('Stop.points must follow a centered axis-aligned rectangle')
    return hx, hy


class CostmapGuard:
    def __init__(self, half_length=.5, half_width=.45, horizon=1.0):
        if not all(math.isfinite(v) and v > 0 for v in (half_length, half_width, horizon)):
            raise ValueError('invalid footprint/horizon')
        self.half_length, self.half_width, self.horizon = half_length, half_width, horizon
        self.grid = None
        self.frame = ''

    def update(self, msg):
        self.grid = None
        self.frame = ''
        m = msg.metadata
        w, h, r = m.size_x, m.size_y, m.resolution
        if (not msg.header.frame_id or w <= 0 or h <= 0 or w*h > 5_000_000 or
                len(msg.data) != w*h or not math.isfinite(r) or r <= 0):
            return False
        origin = m.origin
        if not all(math.isfinite(v) for v in (origin.position.x, origin.position.y)):
            return False
        # Nav2 grids are horizontal; accept origin yaw but reject tilted grids.
        if abs(origin.orientation.x) > 1.e-6 or abs(origin.orientation.y) > 1.e-6:
            return False
        try:
            yaw = planar_yaw(origin.orientation)
        except ValueError:
            return False
        self.grid = np.asarray(msg.data, dtype=np.uint8).reshape(h, w)
        self.resolution = r
        self.origin = (origin.position.x, origin.position.y, yaw)
        self.frame = msg.header.frame_id
        return True

    def check(self, pose, velocity):
        if self.grid is None:
            return 'invalid_costmap'
        if not all(math.isfinite(v) for v in (*pose, *velocity)):
            return 'invalid_guard_input'
        x, y, yaw = pose
        vx, vy, wz = velocity
        ox, oy, oyaw = self.origin
        co, so = math.cos(oyaw), math.sin(oyaw)
        x, y = co*(x-ox)+so*(y-oy), -so*(x-ox)+co*(y-oy)
        yaw -= oyaw
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return 'invalid_guard_input'
        speed_bound = math.hypot(vx, vy) + math.hypot(self.half_length, self.half_width)*abs(wz)
        # At most half a grid cell of corner displacement per interval.
        intervals = self.horizon*speed_bound/(self.resolution*.5)
        if not math.isfinite(intervals) or intervals > 1000:
            return 'guard_sampling_limit'
        steps = max(1, math.ceil(intervals))
        # Cover motion between sampled poses, including rectangle corners in turns.
        margin = self.horizon*speed_bound/(2*steps)
        for i in range(steps+1):
            t = self.horizon*i/steps
            angle = yaw+wz*t
            if abs(wz) < 1.e-8:
                px = x+t*(vx*math.cos(yaw)-vy*math.sin(yaw))
                py = y+t*(vx*math.sin(yaw)+vy*math.cos(yaw))
            else:
                px = x+(vx*(math.sin(angle)-math.sin(yaw)) +
                        vy*(math.cos(angle)-math.cos(yaw)))/wz
                py = y+(vx*(math.cos(yaw)-math.cos(angle)) +
                        vy*(math.sin(angle)-math.sin(yaw)))/wz
            reason = self._at_pose(px, py, angle, self.half_length+margin, self.half_width+margin)
            if reason:
                return reason if i == 0 else 'predicted_'+reason
        return 'pass'

    def _at_pose(self, x, y, yaw, hx, hy):
        r = self.resolution
        c, s = math.cos(yaw), math.sin(yaw)
        extent_x, extent_y = hx*abs(c)+hy*abs(s), hx*abs(s)+hy*abs(c)
        lo_x, hi_x = math.floor((x-extent_x)/r), math.floor((x+extent_x)/r)
        lo_y, hi_y = math.floor((y-extent_y)/r), math.floor((y+extent_y)/r)
        h, w = self.grid.shape
        if lo_x < 0 or lo_y < 0 or hi_x >= w or hi_y >= h:
            return 'costmap_boundary'
        region = self.grid[lo_y:hi_y+1, lo_x:hi_x+1]
        rows, cols = np.nonzero(region >= 254)
        if not len(rows):
            return None
        dx = (cols+lo_x+.5)*r-x
        dy = (rows+lo_y+.5)*r-y
        # Separating-axis test: rotated rectangle versus each grid-cell square.
        cell_reach = .5*r*(abs(c)+abs(s))
        hit = ((np.abs(dx*c+dy*s) <= hx+cell_reach) &
               (np.abs(-dx*s+dy*c) <= hy+cell_reach) &
               (np.abs(dx) <= extent_x+.5*r) & (np.abs(dy) <= extent_y+.5*r))
        if np.any(hit):
            return 'costmap_collision'
        return None
