# Frozen planner evidence — 2026-11-09

See [report](../../../../docs/FROZEN_PLANNER.md) and `summary.json` / `latency.csv`.

- `nav2_review/`: 10 full-route action results, server timing log, exact costmap snapshot. Domain 187, localhost only, no controller/LCM.
- `raw_bag_replay/`: independent FAST-LIO Localization + navigation, raw bag 1x, about 68 seconds. Domain 188, commands only on `/legged/replay/cmd_vel`.
- `core_profile/`: profiling harness sources and output. Historical harness paths are examples; use current workspace/archive paths. `cached.cpp` accepts HEIGHT_XYZ64 and COSTMAP_U8. The latter is saved under `nav2_review/`; height input was derived from final map, three float64 x/y/z per observed cell.
- `core_tests_initial.log`: first 32-suite run; 31 passed, cache test failed because the test fixture enabled clearance without required relief rejection. Corrected fixture and final 5-case successful rerun is `core_cache_tests_final.xml`. Do not treat the initial failure as an unresolved implementation failure.
- `navigation_tests.log`, `bringup_tests.log`: final 4/4 and 4/4 passing CTest runs.
- `release_build.log`: final Release build of 3 changed packages.
- `source_sha256.json`: tested algorithm/launch/config source identities.

Old real-action evidence: `../rubi_replay_20261109/full_route/`. Same costmap bytes and all 1,652 x/y/z coordinates compare exactly equal. Workload was not exclusive or controlled between old/new runs. Startup preprocessing is excluded from warm request latency and reported separately.

Reproduction: build Release, source tools/navigation_env.sh, use a separate ROS domain. Run review with the final site, rviz:=false, review_planning_time_ms:=800, review_height_limit_m:=0.11, review_height_cost:=geometry, wait for READY. Request the documented full route. `validate_cached_review.py SITE OUTPUT_DIR` performs this on domain 187; it launches its own review and must not overlap another stack in that domain. The replay harness similarly assumes domain 188; inspect paths before running. Neither test publishes real robot commands.
