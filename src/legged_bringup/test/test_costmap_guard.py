import math
import numpy as np
import pytest
from nav2_msgs.msg import Costmap
from legged_bringup.costmap_guard import CostmapGuard, rectangle_half_extents


def grid(value=0, yaw=0.):
    m = Costmap()
    m.header.frame_id = 'map'
    m.metadata.resolution = .05
    m.metadata.size_x = m.metadata.size_y = 80
    m.metadata.origin.orientation.z = math.sin(yaw/2)
    m.metadata.origin.orientation.w = math.cos(yaw/2)
    m.data = [value]*6400
    return m


def obstacle(m, x, y, value=254):
    m.data[int(y/.05)*80+int(x/.05)] = value


def guard(m):
    g = CostmapGuard(half_length=.31, half_width=.31, horizon=1.0)
    assert g.update(m)
    return g


def test_free_corridor_and_inflation_are_not_double_counted():
    m = grid(253)
    for x in np.arange(.2, 3., .05):
        obstacle(m, x, 1.43)
        obstacle(m, x, .55)
    assert guard(m).check((1., 1., 0.), (.6, 0., 0.)) == 'pass'


def test_known_obstacle_in_lidar_blind_zone_still_stops():
    m = grid()
    obstacle(m, 1.45, 1.)
    assert guard(m).check((1., 1., 0.), (.6, 0., 0.)) == 'predicted_costmap_collision'


def test_front_sweep_stops_while_obstacle_is_outside_current_square():
    m = grid()
    obstacle(m, 1.85, 1.)
    g = guard(m)
    assert g.check((1., 1., 0.), (0., 0., 0.)) == 'pass'
    assert g.check((1., 1., 0.), (.6, 0., 0.)) == 'predicted_costmap_collision'


def test_rear_obstacle_only_blocks_reverse_motion():
    m = grid()
    obstacle(m, .4, 1.)
    g = guard(m)
    assert g.check((1., 1., 0.), (.6, 0., 0.)) == 'pass'
    assert g.check((1., 1., 0.), (-.6, 0., 0.)) == 'predicted_costmap_collision'


def test_turning_square_corners_are_checked():
    m = grid()
    obstacle(m, 1.4, 1.)
    g = guard(m)
    assert g.check((1., 1., 0.), (0., 0., 0.)) == 'pass'
    assert g.check((1., 1., 0.), (0., 0., .8)) == 'predicted_costmap_collision'


def test_curved_motion_checks_arc_not_just_forward_line():
    m = grid()
    obstacle(m, 1.66, 1.58)
    g = guard(m)
    assert g.check((1., 1., 0.), (.6, 0., 0.)) == 'pass'
    assert g.check((1., 1., 0.), (.6, 0., .8)) == 'predicted_costmap_collision'


def test_rotated_grid_origin_and_body_heading():
    m = grid(yaw=math.pi/2)
    m.metadata.origin.position.x = 4.
    obstacle(m, 1.85, 1.)
    g = guard(m)
    assert g.check((3., 1., math.pi/2), (.6, 0., 0.)) == 'predicted_costmap_collision'


def test_unknown_and_map_boundary_fail_closed():
    m = grid()
    obstacle(m, 1.45, 1., 255)
    assert guard(m).check((1., 1., 0.), (.6, 0., 0.)) == 'predicted_costmap_collision'
    assert guard(grid()).check((3.5, 1., 0.), (.6, 0., 0.)) == 'predicted_costmap_boundary'


def test_malformed_new_grid_invalidates_previous_grid():
    g = guard(grid())
    m = grid(); m.data = [0]
    assert not g.update(m)
    assert g.check((1., 1., 0.), (.6, 0., 0.)) == 'invalid_costmap'
    m = grid(); m.metadata.origin.orientation.w = 0.
    assert not g.update(m)


def test_invalid_pose_command_and_unbounded_work_fail_closed():
    g = guard(grid())
    assert g.check((math.nan, 1., 0.), (.6, 0., 0.)) == 'invalid_guard_input'
    assert g.check((1., 1., 0.), (.6, 0., math.inf)) == 'invalid_guard_input'
    assert g.check((1., 1., 0.), (100., 0., 0.)) == 'guard_sampling_limit'
    assert g.check((1., 1., 0.), (1.e308, 0., 1.e308)) == 'guard_sampling_limit'


@pytest.mark.parametrize('yaw', [0., math.pi/2, math.pi, -math.pi/2])
def test_rectangle_longitudinal_and_lateral_extents_rotate_with_body(yaw):
    # Finer cells distinguish the 0.50 m longitudinal and 0.45 m lateral extent.
    for forward, expected in [(True, 'costmap_collision'), (False, 'pass')]:
        m = grid()
        m.metadata.resolution = .01
        m.metadata.size_x = m.metadata.size_y = 400
        m.data = [0]*160000
        bx, by = (.475, 0.) if forward else (0., .475)
        x, y = 2.+bx*math.cos(yaw)-by*math.sin(yaw), 2.+bx*math.sin(yaw)+by*math.cos(yaw)
        m.data[int(y/.01)*400+int(x/.01)] = 254
        g = CostmapGuard(half_length=.5, half_width=.45)
        assert g.update(m)
        assert g.check((2., 2., yaw), (0., 0., 0.)) == expected


def test_rectangle_rotation_checks_corner_outside_current_envelope():
    m = grid(); obstacle(m, 1.6, 1.)
    g = CostmapGuard(half_length=.5, half_width=.45)
    assert g.update(m)
    assert g.check((1., 1., 0.), (0., 0., 0.)) == 'pass'
    assert g.check((1., 1., 0.), (0., 0., 1.)) == 'predicted_costmap_collision'


@pytest.mark.parametrize('direction', [-1., 1.])
def test_rectangle_sweeps_full_speed_forward_and_reverse(direction):
    m = grid(); obstacle(m, 2.+direction*1.05, 2.)
    g = CostmapGuard(half_length=.5, half_width=.45)
    assert g.update(m)
    assert g.check((2., 2., 0.), (0., 0., 0.)) == 'pass'
    assert g.check((2., 2., 0.), (.7*direction, 0., 0.)) == 'predicted_costmap_collision'


def test_rectangle_configuration_accepts_both_vertex_orders():
    points = [.5, .45, .5, -.45, -.5, -.45, -.5, .45]
    assert rectangle_half_extents(points) == (.5, .45)
    reversed_points = [v for p in reversed(list(zip(points[::2], points[1::2]))) for v in p]
    assert rectangle_half_extents(reversed_points) == (.5, .45)


@pytest.mark.parametrize('points', [
    [.5, .45], [0., 0.]*4, [.5, math.nan]*4,
    [.5, .45, -.5, -.45, .5, -.45, -.5, .45],  # Crossing diagonals.
    [.6, .45, .6, -.45, -.4, -.45, -.4, .45],  # Off-center.
    [.5, .45, .5, -.45, -.5, -.45, -.5, .44],  # Not rectangular.
])
def test_unsupported_stop_geometry_is_rejected(points):
    with pytest.raises(ValueError, match='Stop.points'):
        rectangle_half_extents(points)
