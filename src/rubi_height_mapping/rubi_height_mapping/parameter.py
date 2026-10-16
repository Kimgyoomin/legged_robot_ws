# Copyright (c) 2022, Takahiro Miki. All rights reserved.
# Licensed under MIT; see LICENSE and UPSTREAM.md.
# RUBI height-only derivative, 2026-10-14.
from dataclasses import dataclass
import math
import numpy as np

@dataclass
class Parameter:
    cleanup_cos_thresh: float = 0.2
    cleanup_step: float = 0.05
    dilation_size: int = 3
    drift_compensation_alpha: float = 0.1
    drift_compensation_variance_inlier: float = 0.1
    enable_drift_compensation: bool = False
    enable_edge_sharpen: bool = False
    enable_overlap_clearance: bool = False
    enable_visibility_cleanup: bool = True
    ray_mode: str = 'all'
    ray_voxel_size: float = 0.0  # 0 uses map resolution; applies to cleanup only
    initial_variance: float = 1000.0
    mahalanobis_thresh: float = 2.0
    map_length: float = 8.0
    max_drift: float = 0.1
    max_height_range: float = 2.5
    max_ray_length: float = 3.0
    max_variance: float = 100.0
    min_height_drift_cnt: int = 100
    min_valid_distance: float = 0.1
    orientation_noise_thresh: float = 0.01
    outlier_variance: float = 0.01
    overlap_clear_range_xy: float = 4.0
    overlap_clear_range_z: float = 2.0
    position_noise_thresh: float = 0.01
    ramped_height_range_a: float = 0.3
    ramped_height_range_b: float = 1.0
    ramped_height_range_c: float = 0.2
    resolution: float = 0.05
    sensor_noise_factor: float = 0.05
    time_interval: float = 0.1
    time_variance: float = 0.0001
    traversability_inlier: float = 0.9
    wall_num_thresh: int = 100
    weights_path: str = ''
    data_type = np.float32

    def update(self):
        for key, value in vars(self).items():
            if isinstance(value, (int, float)) and not isinstance(value, bool) and not math.isfinite(value):
                raise ValueError(f'{key} must be finite')
        if self.resolution <= 0 or self.map_length <= 0:
            raise ValueError('resolution/map_length must be positive')
        cells = self.map_length / self.resolution
        if abs(cells - round(cells)) > 1e-3 or not 8 <= round(cells) <= 1024:
            raise ValueError('map_length/resolution must be an integer in [8, 1024]')
        if self.initial_variance <= 0 or self.max_variance <= 0 or self.sensor_noise_factor < 0:
            raise ValueError('Invalid variance/noise parameters')
        if self.max_ray_length <= 0 or self.dilation_size < 1:
            raise ValueError('Invalid ray length/dilation size')
        if self.ray_mode not in ('all', 'voxel'):
            raise ValueError('ray_mode must be all or voxel')
        if self.ray_voxel_size < 0 or (self.ray_voxel_size and not self.resolution/4 <= self.ray_voxel_size <= self.resolution):
            raise ValueError('ray_voxel_size must be 0 or between resolution/4 and resolution')
        self.cell_n = int(round(cells)) + 2
