# Switch tracked vehicle lidar from 3D to 2D — Design

**Status:** approved by user.

## Goal

Change the tracked vehicle's lidar sensor from its current 16-vertical-beam 3D configuration
to a genuine 2D lidar (single flat scan ring), and reconfigure the SLAM/navigation stack
components that depend on lidar dimensionality accordingly — without regressing
`collision_monitor` into its historically-documented no-op state.

## Background

This vehicle's lidar has been a 3D `gpu_lidar` (16 vertical beams, ±15° FOV) since it was
first added in commit `dbbe3160` — there is no prior 2D-lidar state in git history to revert
to; this is a new forward change, not a literal revert.

**Real trap to avoid, documented in this project's own code**:
`tracked_vehicle_nav2_params.yaml`'s `collision_monitor` section has an explicit comment
recording that the *original* `collision_monitor` config used a 2D LaserScan
`"scan"` observation source, and this vehicle never published a `/scan` topic at all
(3D-lidar-only) — so that original config was a structurally-wired but completely inert
no-op for the entire time it existed, not a working detect-and-avoid mechanism. It was later
fixed by switching to a `pointcloud` source reading the real `/lidar/points` topic with a
height window. This design must not reintroduce that dead end.

## Scope

1. **Sensor** (`ros2_ws/src/cavex_tracked_vehicle/models/blueboat/model.sdf.tracked`):
   `lidar_sensor`'s vertical scan collapses from 16 samples (±15°, `-0.2618`/`0.2618` rad) to
   1 sample, `min_angle`/`max_angle` both `0`. Horizontal scan (360 samples, full circle),
   update rate (10Hz), range (0.3–12.0m), and `gz_frame_id` (`lidar_link`) are unchanged.

2. **Bridge** (`gazebo_tracked_vehicle_bridge.yaml`): no changes. `gpu_lidar` with 1 vertical
   sample still publishes a valid `PointCloud2` (just flatter) on the same
   `gz_topic_name`/`ros_topic_name` pair already bridging `/lidar/points` — every downstream
   consumer (`icp_odometry`, `rtabmap`, `collision_monitor`, RViz's `Lidar Points` display)
   keeps subscribing to that same topic unmodified.

3. **RTAB-Map** (`tracked_vehicle_slam.launch.py`):
   - `Grid/3D`: `'true'` → `'false'` (2D occupancy grid built by ray-casting the flat scan
     directly, not from 3D-derived normal segmentation).
   - `Icp/PointToPlane`: `'true'` → `'false'` (point-to-point ICP — a flat single-ring scan
     can't provide the reliable surface normals point-to-plane needs; point-to-point is
     RTAB-Map's standard mode for 2D-lidar SLAM).
   - **Remove** the `Grid/MaxGroundAngle: '65'` parameter added earlier this session — that
     fix addressed 3D normal-based ground/obstacle misclassification, which does not exist
     as a failure mode once `Grid/3D` is false and there's no vertical structure to
     misclassify in the first place.
   - `Grid/RangeMax` (60.0), `subscribe_scan_cloud` (`True`), and all remap topic names are
     unchanged — dimensionality-independent.

4. **`collision_monitor`** (`tracked_vehicle_nav2_params.yaml`): **keep** the `pointcloud`
   observation source (do not revert to the historically-broken `scan`/LaserScan source —
   see Background). Adjust `min_height`/`max_height` from the current `0.0`–`1.2m` (sized
   for the old ±15° cone's vertical spread at typical bubble-range distances) to a tight band
   around the lidar's real mount height, `lidar_link`'s pose relative to `base_link` is
   `z=0.55m` (confirmed directly from the SDF, not assumed) — use `0.45`–`0.65m`
   (±0.10m margin for real pitch/roll/physics jitter around the single flat ring, not an
   arbitrary number). `ObstacleBubble`/`FootprintApproach` polygon configs are unchanged.

5. **Explicitly unchanged**: Nav2's MPPI controller and its costmap layers (`static_layer`,
   `inflation_layer` — already costmap-content-only, indifferent to how the costmap's source
   data was built), `dead_end_backtrack_node.py` (reads the costmap, not the raw sensor),
   `sic_slam_node` (stays disabled per the prior session request — its 3D point-cloud
   colorization/clustering design is moot while disabled, out of scope for this change), the
   RViz Map-display shader fix (`Color Scheme: raw`) applied earlier this session.

## Real, accepted tradeoff

A true 2D lidar cannot detect overhangs, low obstacles below the scan plane, or anything
above/below the single flat ring — this is a genuine capability reduction versus the current
3D setup. This is standard, expected behavior for 2D-lidar-based navigation (not a bug to
work around), and the user has explicitly chosen this tradeoff.

## Out of scope

- Any change to `sic_slam_node`'s own code (stays disabled and untouched).
- Any change to `ardupilot`/`PX4-Autopilot` integration work.
- Re-enabling `dead_end_backtrack_node`'s dependency on `/sic_slam/instances` (already absent
  since `sic_slam_node` is disabled; no change here affects that).
