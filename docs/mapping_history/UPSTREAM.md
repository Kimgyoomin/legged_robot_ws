# Upstream provenance

## Offline postprocessing experiment (2026-10-14)

`rubi_heightmap_postprocess` is a separate NumPy/SciPy offline comparison package.
It reads frozen wrapper snapshots and does not change the upstream GPU estimator
or the live persistence path. The circular mean baseline is independently
implemented and checked against the installed `grid_map_filters/MeanInRadiusFilter`
on observed cells; unknown cells are restored to unknown. The candidate is an
original protected-boundary regional/local robust plane filter, not a copy or
execution of ETH's convex plane decomposition. Source map, XY, masks and integer
timestamps are preserved. Retained input variance is provenance, not a calibrated
postprocessing uncertainty estimate. See the package README for limits and tests.

## LiDAR-only deployment extraction (2026-10-14)

`rubi_height_mapping` is a separate ROS package derived from the existing GPU
backend. The original package stays available for comparison. The selected
fusion/noise/visibility CUDA functions and rolling-map operations are retained;
the image, semantic, plugin, polygon, initializer and upstream save/load paths
are excluded. The C++ global wrapper provides persistence and save/load.

The internal traversability score participates in height-update decisions, so
it is retained as an equivalent CuPy convolution kernel with the original weights,
instead of being removed along with PyTorch. Weights are packaged as NPZ with
pickle disabled at runtime. CuPy's standard allocator replaces the original
process-wide managed-memory allocator. The ROS adapter retains strict stamped
TF queuing and clock-reset failure handling, adds a validated XYZ decoder and
an integrated sensor-origin distance mask, and publishes only height/variance.
No CPU thread cap or desktop-specific PYTHONPATH is added by the launch file.

See [package provenance](../../rubi_height_mapping/UPSTREAM.md) and
[deployment instructions](../../rubi_height_mapping/README.md). This is a
source delivery for review, not Jetson performance or field-accuracy approval.
The sections below describe the earlier integration history; their PyTorch
dependency statements apply to the retained original backend.

- Repository: https://github.com/iit-DLSLab/elevation_mapping_gpu_ros2
- Imported commit: `e08063fe6937a768471fbf251fa30ed7eaae25b9`
- The original commit ancestry is preserved. `upstream` denotes the original
  project and `origin` denotes `Kimgyoomin/rubi_terrain_mapping` in the development checkout.
- Original README: [README.upstream.md](../../README.upstream.md).
- Original root MIT copyright/license remains in [LICENSE](../../LICENSE).
- Optional `plane_segmentation_ros2` remains pinned by the upstream submodule.
  It is not required for the initial raw-elevation wrapper path.

## RUBI changes to the existing backend

1. Publish actual cell-snapped XY map center rather than continuous robot XY.
   Keep neutral pose z/orientation because the elevation layer includes world z.
2. Track the last successfully processed input timestamp separately from the
   last received input. Failed TF lookup cannot refresh a published old map.
   This is still a map snapshot stamp, not a per-cell observation timestamp.
3. Add `allow_latest_tf_fallback`. The inherited core configuration retains true;
   RUBI sets false so a timestamp extrapolation does not register a moving cloud
   using a different pose time.
4. Add an opt-in, non-blocking stamped-TF PointCloud2 queue. RUBI enables the
   single-XYZ-LiDAR path with bounded FIFO count/payload and steady-time expiry;
   the inherited image/semantic/multi-sensor path remains separate with the queue
   disabled by default. Strict queue mode rejects latest fallback and zero stamps.
5. Couple rolling-map movement and point-cloud fusion in one queued operation:
   prepare finite XYZ, acquire sensor/base transforms for the same original stamp,
   call existing `move_to()`, then existing GPU `input_pointcloud()`. The separate
   pose timer is inert in queue mode. Failed/dropped scans cannot advance the map
   snapshot stamp; a fusion exception or ROS-clock reversal stops normal output.
6. Treat rclpy `ExternalShutdownException` as a normal executor shutdown in the
   mapper and delayed-TF synthetic publisher, so launch-controlled test teardown
   does not report a false node crash.

The queue and order controller do not alter GPU kernels, sensor-noise equations,
smoothing, inpainting or traversability. `fused` means the existing GPU API call
returned without a Python exception; no additional device synchronization was
added, so it is not proof that all asynchronous CUDA work completed successfully.

The RUBI global wrapper now also installs a ROS-time backward-jump callback. It
suppresses further input and XYZ publication after a reset until the wrapper and
local mapper are jointly restarted, preventing an old persistent grid from being
presented as normal output in a new simulation epoch.

GPU fusion kernels, the sensor noise model, traversability model and Mahalanobis-
named threshold computation are not rewritten in this bootstrap. Raw-only ROS
publishing does not remove internal traversability/PyTorch computation. Profiling
and a controlled same-bag comparison are required before changing those costs.

The existing backend load service historically stamps a restored map with load
time. Use the wrapper's save/load path for persistent maps with preserved times.
Backend service edits are not a supported live source of wrapper updates in this
bootstrap: the wrapper's duplicate policy assumes increasing sensor snapshot time.

## Wrapper coordinate correction (2026-10-14)

The backend already exports standard GridMap matrix axes (-X rows, -Y columns)
through `get_map_with_name_ref()`. The initial wrapper incorrectly treated this
as the backend's internal +Y rows / +X columns. The wrapper now converts standard
GridMap serialization to its internal storage, including transposed dimensions
for rectangular maps, for elevation and variance alike. The backend fusion and
export operations are unchanged. `input_convention: grid_map` replaces the old
native-axis setting; the old setting fails explicitly. Previously accumulated
maps from the incorrect convention must be regenerated.

ROS regression tests now exercise the actual backend getter, coordinate transform
and encoder with NumPy standing in for device operations before DDS transmission
to the C++ wrapper. This closes the earlier mock-getter validation gap without
claiming CUDA fusion coverage. A recorded local GridMap can also be replayed to
check actual exported data without moving the robot.

Inherited workflows target `ros2` and a project-specific GPU runner. The new
`rubi-humble.yml` checks the wrapper on a GitHub-hosted Humble container; it does
not publish a website/container or claim CUDA coverage.

## Global accumulation policy (2026-10-14)

The wrapper now defaults to `invalid_cell_policy: retain`. A missing or
variance-rejected local estimate leaves the last accepted global cell unchanged,
including its estimate timestamp. Previously `usable=false` propagated from the
rolling local window into global output and hid historical terrain during
occlusion/re-entry. The old behavior remains available as `invalidate`.
New valid posteriors still replace global values exactly; unknown cells are not
filled. File format v1 and saved usable flags are preserved on load. This is a
wrapper-only accumulation change, not a GPU estimator, visibility-cleanup or
dynamic-obstacle-removal change. Retained survey history must be distinguished
from current local observability. Portable and Humble regressions cover invalid
inputs, unseen areas, re-entry, lower-height replacement and unchanged timestamps.
# Direct odometry input — 2026-10-14

`rubi_height_mapping` 0.3.0 adds optional `odom_topic` (nav_msgs/Odometry).
The original-stamp input queue now supports an independent pose cache with
linear translation / quaternion SLERP interpolation and fixed mounting TF.
Dynamic localization TF is not read in this mode. The default TF path and
the GPU fusion equations remain unchanged. Odometry covariance/twist and
per-point deskew are outside this adapter change. See the package README for
frame contracts, timestamp limits, GT/local/global examples and test commands.

## Height-only rolling-grid precision — 2026-10-14

The derived `rubi_height_mapping/core.py` reconstructs its XY center from whole
cell indices instead of accumulating float32 meter increments. Long motion could
otherwise exceed the wrapper's 0.001-cell alignment tolerance and reject valid
patches. Cell shifts and height fusion equations remain unchanged. See the
package UPSTREAM.md and long-motion GPU regression; this corrects publication
continuity, not centimeter-scale localization-induced height distortion.
