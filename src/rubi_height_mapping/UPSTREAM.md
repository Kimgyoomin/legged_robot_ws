# Provenance and retained behavior

Derived from `iit-DLSLab/elevation_mapping_gpu_ros2` commit
`e08063fe6937a768471fbf251fa30ed7eaae25b9`, via RUBI integration checkout
`58d1e98e22bacdefdcbf4534b2ad35513a0543ac` and 2026-10-14 fixes. Inherited MIT
license/notices are retained. This is an upstream-derived height estimator.

- `kernels.py`: inherited map utilities, error-counting, dilation and normal
  CUDA functions; scan-snapshot proposal/finalization replaces in-place fusion.
  Height/noise equations are retained; state update ordering is changed below.
- `core.py`: original rolling grid, seven internal layers, fusion and raw getters.
  Image input, plugin layers, polygon services, initializer and upstream bag/map
  editing services are omitted. Use CuPy's standard allocator; do not install the
  upstream process-global managed-memory allocator.
- `traversability_cupy.py`: same small four-convolution inference expressed as
  one CuPy kernel, replacing PyTorch/Chainer. Keep this internal score because it
  affects inlier/wall counts and visibility cleanup. No walking-safety layer is
  published. Original weights are converted from repository pickle to NPZ;
  runtime loading uses `allow_pickle=False`. Retain the license for these weights.
- `node.py`/`stamped_tf_queue.py`: retain original-stamp TF queue, bounds and
  clock-reset/fusion-failure stop behavior. Remove latest-TF, image/semantic,
  initializer and upstream bag service paths. New FLOAT32 decoder validates
  layout and supports organized row padding. A 3D sensor-origin distance mask
  defaults to 0.6–3.0 m; it is not deskew or complete robot self-filtering.
- GridMap uses standard -X/-Y matrix axes. Global persistence/save/load is provided
  by the separate C++ wrapper, with historical stamps preserved.

Runtime does not require elevation_mapping_cupy, elevation_map_msgs, PyTorch,
Chainer, semantic_sensor, cv_bridge, OpenCV, SciPy, Shapely, simple_parsing,
ros2_numpy or rosbag2_py. ROS/CuPy may have their own transitive dependencies.
CUDA, CuPy, NumPy and ROS 2 remain required.

## 0.3.0 — direct Odometry pose input (2026-10-14)

Optional `odom_topic` selects nav_msgs/Odometry poses instead of dynamic TF.
The ROS-independent bounded cache interpolates translation and quaternion SLERP
at the original scan stamp without extrapolation. Only static mounting TF is
used to compose the odometry child frame with sensor/tracking frames. Parent
frame must equal map_frame; a changing child frame is rejected. Existing TF
mode remains the default. No TF broadcaster is created. The launch forwards
map_frame consistently to the estimator, global wrapper, and RViz.

This changes the ROS pose adapter only, not CUDA height fusion. Covariance and
twist are not propagated into the height uncertainty model; no per-point
deskew or localization drift mitigation is claimed. Portable interpolation
tests and an isolated actual GPU/ROS odometry pipeline exercise delayed
samples, rotated/upside-down mounting, conflicting dynamic TF, frame rejection,
and simulation clock rollback.

## Rolling-grid center precision (2026-10-14)

`core.py::move_to` reconstructs XY center meters from whole-cell coordinates
after each integer cell shift, instead of repeatedly adding float32 meter
increments. A recorded 734-scan route accumulated 0.00113 cells (0.0565 mm at
5 cm resolution), causing the global wrapper to reject the last 24 scan
positions. The wrapper's alignment tolerance, height/noise equations, integer
shift operation and Z update are unchanged. A GPU long-motion regression and
the same GT/local/global/TF recorded-input replays verify the correction.

## GPU input/export overhead (2026-10-15)

The ROS node opts into `points_are_finite=True` after its CPU decoder has removed
non-finite XYZ. This skips redundant GPU compaction but retains an owned mutable
point buffer: the upstream fusion kernels overwrite their input scratch. The
legacy default still checks NaNs. A persistent two-layer GPU export buffer batches
elevation/variance transfer; the single-layer API remains available. Float32 ROS
serialization uses `array('f')` rather than creating/validating Python floats.
Fusion equations, cleanup, internal traversability, map axes, pose timing and
publication rates are unchanged. Tests cover input ownership, legacy NaN input,
layer parity, wire-byte parity, known step heights and actual ROS odometry fusion.
See workspace docs/VALIDATION.md for the scoped PC microbenchmark and limitations.

## Optional GPU representative rays (2026-10-15)

This section records the initial ray-selection implementation. Its in-place
fusion and buffer details are superseded by 0.3.1 below; voxel selection itself
is retained and remains optional.

`ray_mode=all` remains the default. `ray_mode=voxel` selects the lowest input
index in each map-aligned 3D voxel for visibility cleanup ONLY. All XYZ inputs
still enter the height estimator with the original noise/gating equations.
Selection is integrated into the existing error/inlier counting pass, reusing
its point transform; there is no point compaction, count readback, or extra
selection kernel launch on the production path. The existing combined fusion
kernel checks a selection slot before its cleanup loop. The all-mode CUDA
source and signature are preserved rather than adding unused selection inputs.

A reusable bounded GPU hash table stores 63-bit voxel keys and atomic-min point
indices. Probe exhaustion or coordinate overflow preserves the original ray.
No voxel centroid is generated. `ray_voxel_size=0` uses map resolution; explicit
values are restricted to [resolution/4, resolution]. Table memory grows with the
largest received scan, not global map size. The two one-element error buffers
are also reused in voxel mode; all mode keeps its legacy allocation path. No new runtime package or ROS message dependency was added.

This is an optional algorithmic tradeoff, not an identical cleanup replacement:
less ray evidence can delay ghost removal and change upper bounds, variance and
later height acceptance. Baseline estimator atomic updates can themselves be
order-sensitive; floating results are not claimed bit-identical on all scenes.
Keep `all` for deployment until site/Jetson validation supports `voxel`.

Architecture reference (independent implementation, no FastDEM code copied):
https://github.com/Ikhyeon-Cho/FastDEM/blob/main/fastdem/src/fastdem.cpp
FastDEM also separates height observations from a voxel-reduced ray input.
Its Kalman/P2 height estimators and raycasting evidence model are not ported.

## 0.3.1 — immutable scan snapshot and per-cell finalization (2026-10-15)

The proposal/finalization architecture follows the MIT-licensed upstream
`leggedrobotics/elevation_mapping_cupy` tag `v2.2.0` (commit prefix `f618dd8`).
`finalize_map_kernel` is adapted from its `kernels/custom_kernels.py`; the
existing MIT copyright/license matches that upstream and is retained here.

Source: https://github.com/leggedrobotics/elevation_mapping_cupy/blob/v2.2.0/elevation_mapping_cupy/elevation_mapping_cupy/kernels/custom_kernels.py

Our proposal pass retains RUBI's coordinate conventions, height/noise/gating
equations, fixed-step ray traversal and optional voxel selector. It reads the
snapshot and writes only endpoint statistics and separate visibility proposals.
The old height-inflation gate race and non-atomic upper-bound writes are removed.
Per-cell finalization gives accepted endpoints priority over same-scan cleanup;
outlier variance increments are applied only when no endpoint is accepted.
The finalized upper bound is the fused height, not the last writer's candidate.
Unknown ray-only cells remain unknown. New rules intentionally do not reproduce
the legacy order-dependent output on all scenes.

Unlike upstream's FP32 proposal buffers, we accumulate float32 candidate values
in FP64 buffers to reduce rounding sensitivity. Final map state/output stays
FP32. Counts/error counters and buffers are reused for both all/voxel modes.
This is not a universal bitwise-determinism guarantee. On the recorded fixed-pose
215-scan test, four runs per mode and an all-ray reversed-input test matched
exactly. The captured cloud uses a synthetic pose: no world/GT accuracy claim.

The upstream DDA ray traversal, geometry/dilation changes, semantic pipelines,
plugin system and ROS Jazzy dependencies are not imported. CUDA/CuPy/NumPy and
ROS Humble dependencies remain unchanged. FP64 proposal overhead needs target
Jetson measurement. Default ray_mode stays all.
