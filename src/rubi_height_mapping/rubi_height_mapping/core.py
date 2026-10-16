# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under MIT; see LICENSE and UPSTREAM.md.
# RUBI height-only derivative, 2026-10-14.
from __future__ import annotations
import threading
from typing import List
import numpy as np
import cupy as cp
from .parameter import Parameter
from .ray_selection import VoxelRaySelector
from .traversability_cupy import CupyTraversability
from .kernels import propose_points_kernel, error_counting_kernel, finalize_map_kernel, dilation_filter_kernel, normal_filter_kernel
xp = cp

class ElevationMap:
    def __init__(self, param: Parameter):
        """

        Args:
            param (rubi_height_mapping.parameter.Parameter):
        """

        self.param = param
        self.data_type = self.param.data_type
        self.resolution = param.resolution
        self.center = xp.array([0, 0, 0], dtype=self.data_type)
        self.base_rotation = xp.eye(3, dtype=self.data_type)
        self.map_length = param.map_length
        self.cell_n = param.cell_n

        self.map_lock = threading.Lock()
        self.elevation_map = xp.zeros((7, self.cell_n, self.cell_n), dtype=self.data_type)
        self.layer_names = [
            "elevation",
            "variance",
            "is_valid",
            "traversability",
            "time",
            "upper_bound",
            "is_upper_bound",
        ]

        # buffers
        self._publish_layers = cp.empty((2, self.cell_n - 2, self.cell_n - 2), dtype=cp.float32)
        self.normal_map = xp.zeros((3, self.cell_n, self.cell_n), dtype=self.data_type)
        # Initial variance
        self.initial_variance = param.initial_variance
        self.elevation_map[1] += self.initial_variance
        self.elevation_map[3] += 1.0

        # overlap clearance
        cell_range = int(self.param.overlap_clear_range_xy / self.resolution)
        cell_range = np.clip(cell_range, 0, self.cell_n)
        self.cell_min = self.cell_n // 2 - cell_range // 2
        self.cell_max = self.cell_n // 2 + cell_range // 2

        # Initial mean_error
        self.mean_error = 0.0
        self.additive_mean_error = 0.0

        self.ray_selector = (VoxelRaySelector(param)
            if param.ray_mode == 'voxel' and param.enable_visibility_cleanup else None)
        self._error = cp.zeros(1, cp.float64)
        self._error_count = cp.zeros(1, cp.float64)
        self.compile_kernels()

        self.traversability_filter = CupyTraversability(param.weights_path)

    def clear(self):
        """Reset the internal elevation layers."""
        with self.map_lock:
            self.elevation_map *= 0.0
            # Initial variance
            self.elevation_map[1] += self.initial_variance

        self.mean_error = 0.0
        self.additive_mean_error = 0.0

    def get_center_position(self, position):
        """Return the position of the map center.

        Args:
            position (numpy.ndarray):

        """
        position[0][:] = xp.asnumpy(self.center)

    def move_to(self, position, R):
        """Shift the map to an absolute position and update the rotation of the robot.

        Args:
            position (numpy.ndarray):
            R (cupy._core.core.ndarray):
        """
        # Shift map to the center of robot.
        self.base_rotation = xp.asarray(R, dtype=self.data_type)
        position = xp.asarray(position)
        delta = position - self.center
        delta_pixel = xp.around(delta[:2] / self.resolution)
        # Reconstruct from whole-cell coordinates instead of repeatedly adding
        # float32 meter offsets. Accumulated rounding otherwise makes a valid
        # rolling grid fail the global wrapper's cell-alignment contract.
        center_pixel = xp.around(self.center[:2] / self.resolution) + delta_pixel
        self.center[:2] = center_pixel * self.resolution
        self.center[2] += delta[2]
        self.shift_map_xy(-delta_pixel)
        self.shift_map_z(-delta[2])

    def pad_value(self, x, shift_value, idx=None, value=0.0):
        """Create a padding of the map along x,y-axis according to amount that has shifted.

        Args:
            x (cupy._core.core.ndarray):
            shift_value (cupy._core.core.ndarray):
            idx (Union[None, int, None, None]):
            value (float):
        """
        if idx is None:
            if shift_value[0] > 0:
                x[:, : shift_value[0], :] = value
            elif shift_value[0] < 0:
                x[:, shift_value[0] :, :] = value
            if shift_value[1] > 0:
                x[:, :, : shift_value[1]] = value
            elif shift_value[1] < 0:
                x[:, :, shift_value[1] :] = value
        else:
            if shift_value[0] > 0:
                x[idx, : shift_value[0], :] = value
            elif shift_value[0] < 0:
                x[idx, shift_value[0] :, :] = value
            if shift_value[1] > 0:
                x[idx, :, : shift_value[1]] = value
            elif shift_value[1] < 0:
                x[idx, :, shift_value[1] :] = value

    def shift_map_xy(self, delta_pixel):
        """Shift the map along the horizontal axes according to the input.

        Args:
            delta_pixel (cupy._core.core.ndarray): Shift in [x, y] order (world coordinates).
                x corresponds to columns (axis 2), y corresponds to rows (axis 1).

        Note:
            The map array has shape (layers, height, width) = (layers, rows, cols).
            In row-major convention: axis 1 = rows = Y, axis 2 = cols = X.
            cp.roll with axis=(1, 2) expects [row_shift, col_shift] = [y_shift, x_shift].
            Since delta_pixel is [x, y], we swap to [y, x] for correct axis mapping.
        """
        # Swap [x, y] to [y, x] to match axis=(1, 2) = (rows=Y, cols=X)
        shift_value = cp.array([delta_pixel[1], delta_pixel[0]], dtype=cp.int32)
        if cp.abs(shift_value).sum() == 0:
            return
        with self.map_lock:
            self.elevation_map = cp.roll(self.elevation_map, shift_value, axis=(1, 2))
            self.pad_value(self.elevation_map, shift_value, value=0.0)
            self.pad_value(self.elevation_map, shift_value, idx=1, value=self.initial_variance)

    def shift_map_z(self, delta_z):
        """Shift the relevant layers along the vertical axis.

        Args:
            delta_z (cupy._core.core.ndarray):
        """
        with self.map_lock:
            # elevation
            self.elevation_map[0] += delta_z
            # upper bound
            self.elevation_map[5] += delta_z

    def compile_kernels(self):
        """Compile all kernels belonging to the elevation map."""

        self.new_map = cp.zeros(
            (self.elevation_map.shape[0], self.cell_n, self.cell_n),
            dtype=cp.float64,
        )
        # FP64 accumulation of float32 proposals reduces order-dependent rounding;
        # the seven published/internal state layers remain float32. Reuse buffers.
        self.map_snapshot = cp.empty_like(self.elevation_map)
        self.visibility_map = cp.empty((3, self.cell_n, self.cell_n), cp.float64)
        self.traversability_input = cp.zeros((self.cell_n, self.cell_n), dtype=self.data_type)
        self.traversability_mask_dummy = cp.zeros((self.cell_n, self.cell_n), dtype=self.data_type)

        self.propose_points_kernel = propose_points_kernel(
            self.resolution,
            self.cell_n,
            self.cell_n,
            self.param.sensor_noise_factor,
            self.param.mahalanobis_thresh,
            self.param.outlier_variance,
            self.param.wall_num_thresh,
            self.param.max_ray_length,
            self.param.cleanup_step,
            self.param.min_valid_distance,
            self.param.max_height_range,
            self.param.cleanup_cos_thresh,
            self.param.ramped_height_range_a,
            self.param.ramped_height_range_b,
            self.param.ramped_height_range_c,
            self.param.enable_edge_sharpen,
            self.param.enable_visibility_cleanup,
            self.ray_selector is not None,
        )
        self.error_counting_kernel = error_counting_kernel(
            self.resolution,
            self.cell_n,
            self.cell_n,
            self.param.sensor_noise_factor,
            self.param.mahalanobis_thresh,
            self.param.drift_compensation_variance_inlier,
            self.param.traversability_inlier,
            self.param.min_valid_distance,
            self.param.max_height_range,
            self.param.ramped_height_range_a,
            self.param.ramped_height_range_b,
            self.param.ramped_height_range_c,
            self.ray_selector is not None,
            self.param.ray_voxel_size or self.resolution,
        )
        self.finalize_map_kernel = finalize_map_kernel(
            self.cell_n, self.cell_n, self.param.max_variance, self.initial_variance,
            self.param.outlier_variance,
        )

        self.dilation_filter_kernel = dilation_filter_kernel(self.cell_n, self.cell_n, self.param.dilation_size)
        self.normal_filter_kernel = normal_filter_kernel(self.cell_n, self.cell_n, self.resolution)

    def update_map_with_kernel(self, points_all, channels, R, t, position_noise, orientation_noise):
        """Update map with new measurement.

        Args:
            points_all (cupy._core.core.ndarray):
            channels (List[str]):
            R (cupy._core.core.ndarray):
            t (cupy._core.core.ndarray):
            position_noise (float):
            orientation_noise (float):
        """
        points = cp.ascontiguousarray(points_all[:, :3])
        if points.shape[0] == 0:
            return
        with self.map_lock:
            self.new_map.fill(0)
            error, error_cnt = self._error, self._error_count
            error.fill(0)
            error_cnt.fill(0)
            self.map_snapshot[...] = self.elevation_map
            self.visibility_map[:2].fill(0)
            self.visibility_map[2].fill(cp.inf)
            t = t.copy()
            t[2] -= self.center[2]
            # The count pass only reads the snapshot and completes before the
            # proposal pass reads its cell counts (same CUDA stream).
            counting_inputs = (self.map_snapshot, points, self.center[:1], self.center[1:2], R, t)
            counting_outputs = (self.new_map, error, error_cnt)
            ray_arguments = ()
            if self.ray_selector is not None:
                ray_arguments = self.ray_selector.prepare(points.shape[0])
                counting_inputs += (self.center[2:3], cp.uint64(self.ray_selector.capacity - 1))
                counting_outputs += (self.ray_selector.keys, self.ray_selector.winners, self.ray_selector.slots)
            self.error_counting_kernel(*counting_inputs, *counting_outputs, size=points.shape[0])
            if (
                self.param.enable_drift_compensation
                and error_cnt > self.param.min_height_drift_cnt
                and (
                    position_noise > self.param.position_noise_thresh
                    or orientation_noise > self.param.orientation_noise_thresh
                )
            ):
                self.mean_error = error / error_cnt
                self.additive_mean_error += self.mean_error
                if np.abs(self.mean_error) < self.param.max_drift:
                    self.map_snapshot[0] += self.mean_error * self.param.drift_compensation_alpha
            # From here until finalization the snapshot is strictly read-only.
            self.propose_points_kernel(
                self.center[:1],
                self.center[1:2],
                R,
                t,
                self.map_snapshot,
                self.normal_map,
                *ray_arguments,
                points,
                self.new_map,
                self.visibility_map,
                size=(points.shape[0]),
            )

            self.finalize_map_kernel(
                self.map_snapshot, self.new_map, self.visibility_map,
                self.elevation_map, size=self.cell_n * self.cell_n,
            )

            if self.param.enable_overlap_clearance:
                self.clear_overlap_map(t)

            self.traversability_input *= 0.0
            self.dilation_filter_kernel(
                self.elevation_map[5],
                self.elevation_map[2] + self.elevation_map[6],
                self.traversability_input,
                self.traversability_mask_dummy,
                size=(self.cell_n * self.cell_n),
            )

            traversability = self.traversability_filter(self.traversability_input)
            self.elevation_map[3][3:-3, 3:-3] = traversability.reshape(
                (traversability.shape[2], traversability.shape[3])
            )

        self.update_normal(self.traversability_input)

    def clear_overlap_map(self, t):
        """Clear overlapping areas around the map center.

        Args:
            t (cupy._core.core.ndarray): Absolute point position
        """

        height_min = t[2] - self.param.overlap_clear_range_z
        height_max = t[2] + self.param.overlap_clear_range_z
        near_map = self.elevation_map[:, self.cell_min : self.cell_max, self.cell_min : self.cell_max]
        valid_idx = ~cp.logical_or(near_map[0] < height_min, near_map[0] > height_max)
        near_map[0] = cp.where(valid_idx, near_map[0], 0.0)
        near_map[1] = cp.where(valid_idx, near_map[1], self.initial_variance)
        near_map[2] = cp.where(valid_idx, near_map[2], 0.0)
        valid_idx = ~cp.logical_or(near_map[5] < height_min, near_map[5] > height_max)
        near_map[5] = cp.where(valid_idx, near_map[5], 0.0)
        near_map[6] = cp.where(valid_idx, near_map[6], 0.0)
        self.elevation_map[:, self.cell_min : self.cell_max, self.cell_min : self.cell_max] = near_map

    def update_variance(self):
        """Adds the time variacne to the valid cells."""
        self.elevation_map[1] += self.param.time_variance * self.elevation_map[2]

    def update_time(self):
        """adds the time interval to the time layer."""
        self.elevation_map[4] += self.param.time_interval

    def input_pointcloud(
        self,
        raw_points: cp._core.core.ndarray,
        channels: List[str],
        R: cp._core.core.ndarray,
        t: cp._core.core.ndarray,
        position_noise: float,
        orientation_noise: float,
        *,
        points_are_finite: bool = False,
    ):
        """Input the point cloud and fuse the new measurements to update the elevation map.

        Args:
            raw_points (cupy._core.core.ndarray):
            channels (List[str]):
            R  (cupy._core.core.ndarray):
            t (cupy._core.core.ndarray):
            position_noise (float):
            orientation_noise (float):
            points_are_finite (bool): Caller guarantees finite XYZ; skips GPU validation.

        Returns:
            None:
        """
        # The ROS decoder already removes non-finite XYZ on the CPU. Avoid a
        # second GPU compaction in that case. Kernels overwrite this buffer,
        # so it must remain owned even when the caller supplies a CuPy array.
        if points_are_finite:
            raw_points = cp.array(raw_points, dtype=self.data_type, copy=True)
        else:
            raw_points = cp.asarray(raw_points, dtype=self.data_type)
            raw_points = raw_points[~cp.isnan(raw_points[:, :3]).any(axis=1)]
        additional_channels = channels[3:]
        self.update_map_with_kernel(
            raw_points,
            additional_channels,
            cp.asarray(R, dtype=self.data_type),
            cp.asarray(t, dtype=self.data_type),
            position_noise,
            orientation_noise,
        )

    def update_normal(self, dilated_map):
        """Clear the normal map and then apply the normal kernel with dilated map as input.

        Args:
            dilated_map (cupy._core.core.ndarray):
        """
        with self.map_lock:
            self.normal_map *= 0.0
            self.normal_filter_kernel(
                dilated_map,
                self.elevation_map[2],
                self.normal_map,
                size=(self.cell_n * self.cell_n),
            )

    def process_map_for_publish(self, input_map, fill_nan=False, add_z=False, xp=cp):
        """Process the input_map according to the fill_nan and add_z flags.

        Args:
            input_map (cupy._core.core.ndarray):
            fill_nan (bool):
            add_z (bool):
            xp (module):

        Returns:
            cupy._core.core.ndarray:
        """
        m = input_map.copy()
        if fill_nan:
            m = xp.where(self.elevation_map[2] > 0.5, m, xp.nan)
        if add_z:
            m = m + self.center[2]
        return m[1:-1, 1:-1]

    def get_elevation(self):
        """Get the elevation layer.

        Returns:
            elevation layer

        """
        return self.process_map_for_publish(self.elevation_map[0], fill_nan=True, add_z=True)

    def get_variance(self):
        """Get the variance layer.

        Returns:
            variance layer
        """
        return self.process_map_for_publish(self.elevation_map[1], fill_nan=False, add_z=False)

    def _transform_to_grid_map_coordinate_convention(self, m):
        """Transform the map to the grid_map coordinate convention.

        elevation_mapping_cupy uses Row=Y, Col=X (see kernels/custom_kernels.py:35)
        grid_map uses Row→-X, Col→-Y (see grid_map_core/src/GridMapMath.cpp:64-67
        transformBufferOrderToMapFrame returns {-index[0], -index[1]})
        Required transformation:
           1. Transpose: swap axes so Row=X, Col=Y (matching grid_map's axis assignment)
           2. Flip axis 0: so increasing row → decreasing X (matching grid_map's -X)
           3. Flip axis 1: so increasing col → decreasing Y (matching grid_map's -Y)

        This is equivalent to: rot90(m.T, k=2) or flip(flip(m.T, 0), 1)

        Args:
            m (cupy._core.core.ndarray):

        Returns:
            cupy._core.core.ndarray:
        """
        m = m.T
        m = xp.flip(m, 0)
        m = xp.flip(m, 1)
        return m

    def get_map_with_name_ref(self, name, data):
        with self.map_lock:
            if name == 'elevation':
                layer = self.get_elevation()
            elif name == 'variance':
                layer = self.get_variance()
            else:
                raise ValueError('Only elevation and variance are published')
            data[...] = cp.asnumpy(self._transform_to_grid_map_coordinate_convention(layer))

    def get_raw_layers_ref(self, data):
        """Export elevation and variance in GridMap axes with one host transfer.

        ``data`` is a contiguous float32 array of shape (2, cell_n-2, cell_n-2).
        Layer order is elevation, variance. The legacy single-layer API remains.
        """
        with self.map_lock:
            self._publish_layers[0] = self._transform_to_grid_map_coordinate_convention(self.get_elevation())
            self._publish_layers[1] = self._transform_to_grid_map_coordinate_convention(self.get_variance())
            self._publish_layers.get(out=data)
