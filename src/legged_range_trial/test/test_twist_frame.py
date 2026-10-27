import numpy as np
from legged_range_trial.twist_frame import transport


def test_rotating_offset_origin_and_covariance():
    twist, cov = transport(np.array([0., 0., 0., 0., 0., 2.]), np.eye(6),
                           np.eye(3), np.array([.2, 0., 0.]))
    np.testing.assert_allclose(twist, [0., .4, 0., 0., 0., 2.])
    assert np.isclose(cov[1, 1], 1.04)
    assert np.isclose(cov[1, 5], .2)
    np.testing.assert_allclose(cov, cov.T)


def test_child_axes_are_rotated_not_just_relabelled():
    r = np.array([[0., -1., 0.], [1., 0., 0.], [0., 0., 1.]])
    twist, _ = transport(np.array([0., 1., 0., 0., 0., 0.]), np.eye(6), r, np.zeros(3))
    np.testing.assert_allclose(twist, [1., 0., 0., 0., 0., 0.])
