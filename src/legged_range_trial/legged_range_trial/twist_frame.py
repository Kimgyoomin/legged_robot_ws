"""Rigid-body velocity transport; no differentiation or temporal filtering."""
import numpy as np


def skew(v):
    x, y, z = v
    return np.array([[0., -z, y], [z, 0., -x], [-y, x, 0.]])


def transport(twist, covariance, source_from_base, source_to_base):
    """Input twist at source origin; output at base origin in base axes.

    source_from_base is the rotation from base axes to source axes.
    source_to_base is the base origin expressed in source coordinates.
    """
    rotation = np.asarray(source_from_base, dtype=float).T
    jacobian = np.zeros((6, 6))
    jacobian[:3, :3] = rotation
    jacobian[:3, 3:] = -rotation @ skew(source_to_base)
    jacobian[3:, 3:] = rotation
    return jacobian @ twist, jacobian @ covariance @ jacobian.T
