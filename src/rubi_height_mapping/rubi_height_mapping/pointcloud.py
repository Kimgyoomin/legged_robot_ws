"""Strict PointCloud2 XYZ decoding; independent of image/semantic ROS helpers."""
import numpy as np


def pointcloud_xyz(msg):
    if msg.is_bigendian:
        raise ValueError('Expected little-endian FLOAT32 XYZ')
    if msg.width <= 0 or msg.height <= 0 or msg.point_step < 12:
        raise ValueError('Empty or invalid cloud geometry')
    if msg.row_step < msg.width * msg.point_step or len(msg.data) != msg.row_step * msg.height:
        raise ValueError('Invalid PointCloud2 row stride/payload length')
    xyz = []
    for name in ('x', 'y', 'z'):
        matches = [f for f in msg.fields if f.name == name]
        if len(matches) != 1:
            raise ValueError(f'Missing/duplicate field: {name}')
        field = matches[0]
        if field.datatype != 7 or field.count != 1 or not 0 <= field.offset <= msg.point_step-4:
            raise ValueError(f'{name} must be a valid FLOAT32 scalar')
        xyz.append(np.ndarray((msg.height, msg.width), dtype='<f4', buffer=msg.data,
                              offset=field.offset, strides=(msg.row_step, msg.point_step)))
    result = np.stack(xyz, axis=-1).reshape(-1, 3)
    return result[np.isfinite(result).all(axis=1)]


def quaternion_matrix(quaternion):
    q = np.asarray(quaternion, dtype=np.float64)
    norm2 = np.dot(q, q)
    if q.shape != (4,) or not np.isfinite(q).all() or norm2 < 1e-20:
        raise ValueError('Invalid TF quaternion')
    x, y, z, w = q / np.sqrt(norm2)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w), 0],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w), 0],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y), 0],
        [0, 0, 0, 1],
    ])
