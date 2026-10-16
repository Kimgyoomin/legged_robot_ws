# Global heightmap wrapper

This node persists the pinned CuPy backend's local elevation estimates. It is
not a second height estimator or a raw LiDAR mapper.

## Input contract

- `grid_map_msgs/msg/GridMap` on the configured `input_topic`. The legacy default is
  `/elevation_mapping_node/elevation_map_raw`; the LiDAR-only
  `rubi_height_mapping` integrated launch selects `/rubi/local_elevation/grid_map`.
- Exactly one `elevation` and one `variance` layer; finite elevation represents
  a locally valid estimate. Variance must be finite, nonnegative and below the
  configured cutoff. No smoothing/inpainting layer is consumed.
- **Standard GridMap axes:** exported matrix row increases toward map -X;
  column increases toward map -Y. The CuPy backend transposes and flips its
  internal +Y/+X array before publishing. The wrapper decodes this into its
  internal +Y/+X storage for both elevation and variance. A rectangular world
  grid of NY rows and NX columns has an NX-by-NY wire matrix.
- `input_convention` must be `grid_map` (the default). The old
  `cupy_rows_y_cols_x` setting is rejected: it silently misplaced terrain from
  this backend. Rebuild maps accumulated with that old interpretation; a single
  rotation of an accumulated moving-window map cannot repair it.
- Two labeled MultiArray dimensions, validated size/stride/data offset. Both
  column-major and row-major serialization are decoded. Circular start
  indices must be zero, as in this backend's publisher; others are rejected.
- Identity map orientation, pose.z=0, absolute elevation z. Published center must
  be the backend's actual snapped grid center. No extra robot pose transform.
- Resolution must equal the global grid. Cell boundary phase tolerance is
  0.001 cell (0.05 mm at 5 cm). Misaligned patches are rejected, not interpolated.
- Frame must match `map_frame`. Positive, increasing snapshot stamps; duplicates
  have no effect and backwards timestamps require an explicit experiment reset.
- With the RUBI backend queue, the stamp is the original input stamp of the last
  scan whose stamped sensor/base transforms were both found and whose GPU fusion
  call returned without an exception. Queue timeout/overflow/invalid inputs do
  not advance it. This submission acknowledgement is not a CUDA synchronization
  or a claim that every cell was observed at that time.

The subscriber is best effort with depth 2, compatible with the backend's
reliable publisher. A dropped local snapshot can lose terrain that exits the
window before another snapshot arrives; bag tests must check this coverage.

## Persistence rules

Accepted values replace the corresponding global cell's height and variance.
The variance is copied, not reduced by treating overlapping posteriors as
independent measurements. Cells outside the local window retain their state.

`invalid_cell_policy: retain` (default) leaves a global cell unchanged when a
local cell is NaN or fails the variance gate. Thus occlusion, local expiry or
rolling-window re-entry cannot remove an accepted historical surface. Height,
variance, usable flag and estimate timestamp all remain unchanged. Never-observed
cells remain unknown; this does not interpolate holes. New valid estimates still
replace older ones, including lower heights, without repeatedly fusing a posterior.

`invalid_cell_policy: invalidate` selects the legacy behavior: an invalid local
cell marks the global cell unusable for XYZ but preserves its historical values
in the saved state. The input layers do not distinguish occlusion/expiry from
actual removal. Neither policy performs evidence-based dynamic object removal.

The retained global layer is survey history, not a fresh local contact/obstacle
map. Keep the backend's live local map for changes and present observability.
Accumulation is not freezing: save a QC-reviewed snapshot after the survey for
offline use. A map reset, reload or explicit invalidation can still reduce coverage.

When ROS time moves backwards, the wrapper stops accepting local snapshots and
stops its wall-timer publication. It does not mix a new simulation epoch with
the retained grid or auto-recover after a clock jump; restart/reset it together
with the CuPy mapper and planner.

`estimate_snapshot_ns` means the snapshot carrying the estimate, not a measured
per-cell acquisition time. All scans within a local posterior are correlated.
This first wrapper does not estimate global pose covariance, correct an already
accumulated map after relocalization, or solve dynamic terrain removal.

## Output and services

`/rubi/global_elevation/cloud`: reliable/volatile depth 1, full packed FLOAT32
XYZ snapshot, height=1, one point at each usable global cell center, 1 Hz by
default. Frame is `map`; stamp is the last accepted local snapshot's stamp.
The node does not freshen old output with wall time. Export/packing is cached
between updates and skipped without subscribers.

```bash
ros2 service call /rubi/global_elevation/save_map std_srvs/srv/Trigger '{}'
ros2 service call /rubi/global_elevation/load_map std_srvs/srv/Trigger '{}'
ros2 service call /rubi/global_elevation/clear_map std_srvs/srv/Trigger '{}'
```

Save creates a new directory below `output_directory` containing `map.rghm`
(geometry, cutoff, timestamps, retained estimates and usable state) and
`surface.pcd` (usable XYZ only). Load reads the startup parameter `load_path`
and requires matching configuration; a malformed load leaves the current map
unchanged. Saved times are preserved. Save/load and input callbacks are serialized,
so stop bag playback during file operations. A failed save can leave a partial
new directory; the service reports failure and never overwrites an old run.

The v1 file format is unchanged. Loading preserves every saved usable flag,
including cells hidden by the legacy policy; selecting `retain` does not silently
reactivate them. The invalid-cell policy is a runtime setting, so archive the YAML
with the map for reproducibility. Any recovery of legacy hidden history must be
an explicit offline operation on a copy, with independent QC.

All configuration parameters are read-only after startup; use a YAML override
and restart to change `load_path`, geometry, cutoff or topics.

**Reset consumers between experiments.** Clear affects only the wrapper. CuPy
must also be cleared/restarted, or its next posterior repopulates the old terrain.
The current planner rejects empty clouds, so this node suppresses empty output
and cannot clear that planner's cache. Its normal freshness checks still apply.
Restart the planner before evaluating a new run. Omitting unusable cells does not
make them explicit hazards because the existing planner can query nearby heights.

The default fixed grid has 1 million cells. Storage is bounded, but full XYZ
output and planner snapshot reconstruction still scale with observed coverage.
Orin performance has not been measured.

The Humble ROS regression test executes the actual backend layer getter, axis
transform and message encoder with NumPy boundary doubles, then passes the result
through the compiled wrapper over DDS. It checks asymmetric 5/15 cm landmarks,
rectangular dimensions, a moving window, matching variance coordinates and
save/load. This covers the export contract, not CUDA height fusion or Orin speed.
