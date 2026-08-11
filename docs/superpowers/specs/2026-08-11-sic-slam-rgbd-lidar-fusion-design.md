# SIC-SLAM: RGB-D + 3D Lidar Fusion with Semantic Instance Clustering

Date: 2026-08-11

## Context

The tracked vehicle currently has:
- A 3D spinning lidar (`gpu_lidar`, 16 vertical channels, 360° horizontal, ±15° vertical FOV, 0.4-60m range) feeding `icp_odometry` and RTAB-Map's 3D occupancy grid (`Grid/3D: true`).
- A 2D RGB camera (`type="camera"`, no depth) feeding RTAB-Map's visual features/loop closure only (`subscribe_depth: False`).

There is currently no fusion between the two sensors, and no semantic/instance-level understanding of the environment — only raw occupancy cells and lidar/visual odometry.

## Goal

1. Upgrade the camera to RGB-D (color + depth).
2. Fuse the RGB-D camera with the 3D lidar by colorizing the lidar point cloud (projecting camera pixels onto lidar points via known extrinsics/intrinsics).
3. Cluster the colorized cloud into persistent-ID object/surface instances (geometric clustering, no deep-learning model).
4. Feed instance data into `dead_end_backtrack_node`'s 360° survey scoring, so it can distinguish a real corridor opening from a cluttered false opening. (Frontier/goal selection in `explore_lite` is an explicit future extension, not built in this pass.)

## Architecture

A new ROS2 package, `cavex_perception`, holds one C++ node: `sic_slam_node`.

```
camera (rgbd_camera)  ──color+depth+info──┐
                                            ├─▶ sic_slam_node ──▶ /sic_slam/instances (vision_msgs/Detection3DArray)
lidar (/lidar/points)  ────raw cloud──────┘         │                      │
                                                     └──▶ /sic_slam/colored_points (sensor_msgs/PointCloud2, RViz only)
                                                                            │
                                                            dead_end_backtrack_node (subscribes, scores openings)
```

`sic_slam_node` is independent of RTAB-Map's internal pipeline — it reads raw sensor topics and the existing static TF between `lidar_link`/`camera_link` directly. RTAB-Map gets its own RGB-D depth stream directly from the sensor bridge, in parallel; `sic_slam_node` does not sit in that path.

## Components

### 1. Sensor SDF change (`ros2_ws/src/cavex_tracked_vehicle/models/blueboat/model.sdf.tracked`)

`camera_sensor`'s `type` changes from `"camera"` to `"rgbd_camera"`, keeping existing resolution (800x800), horizontal FOV (1.3962634 rad), pose, and clip planes unchanged. `gazebo_tracked_vehicle_bridge.yaml` gets the additional depth image + depth camera_info topic bridges alongside the existing color ones.

### 2. RTAB-Map config (`ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py`)

`subscribe_depth` changes `False` → `True`, with the depth image/camera_info remappings added. RTAB-Map then fuses lidar geometry + depth-camera geometry + RGB visual features for its own map. `Grid/FromDepth` stays `false` (grid still built from lidar, per the existing, already-justified comment) — only the depth *subscription* is being added for RTAB-Map's own registration/loop-closure use, not to replace the lidar-based grid.

### 3. `sic_slam_node` (new, `cavex_perception` package)

- Subscribes: camera color image, camera depth image, camera_info, `/lidar/points`.
- Colorizes: for each lidar point, transforms it into the camera optical frame (static TF, computed once at startup), projects with the pinhole camera model from `camera_info`, and if the projection falls within image bounds, samples RGB from the color image. Points that project outside the image bounds keep a default gray color but are not dropped.
- Clusters: runs PCL `EuclideanClusterExtraction` on the colorized cloud (geometry-based; color is carried through as a point attribute for visualization/future use, not part of the clustering distance metric in v1 — keeps the first version simple and predictable).
- Tracks IDs: greedy nearest-centroid matching between this frame's clusters and the previous frame's, within a fixed distance threshold. Unmatched new clusters get a new ID; unmatched previous clusters are simply dropped (no ghost/decay bookkeeping in v1).
- Publishes: `/sic_slam/instances` (`vision_msgs/Detection3DArray`, one `Detection3D` per instance carrying centroid pose, bounding box size, and the persistent ID as `results[0].hypothesis.class_id`), and `/sic_slam/colored_points` (`sensor_msgs/PointCloud2`, XYZRGB, for RViz only).
- `--self-check` flag: feeds synthetic point sets (two well-separated clusters, one merged blob, a repeat frame with a small centroid shift) through the clustering + ID-matching logic in-process and asserts expected instance counts and ID continuity — no live sensor data or ROS graph needed.

### 4. `dead_end_backtrack_node.py` integration

Subscribes to `/sic_slam/instances`. During the existing 360° survey (`SURVEYING` state), for each candidate opening's angular sector, checks whether any instance centroid falls within that sector at close range (reusing the existing `OPENING_SCAN_RADIUS_M` constant as the distance cutoff) and applies a score penalty alongside the existing `ray_clear_distance` term, before selecting the best opening to face. This is an additive scoring change to the existing survey logic, not a rewrite of it.

## Error handling / edge cases

- No clusters found in a frame (open cave, sparse points): publish an empty `Detection3DArray` — dead-end scoring sees no penalty anywhere, safe no-op, behaves exactly as it does today.
- `sic_slam_node` not running or crashed: `dead_end_backtrack_node` simply never receives instance messages and falls back to its current ray-only scoring — no hard dependency, no crash.
- Camera-lidar projection: points outside image bounds are colored gray, not dropped, so geometry/clustering is unaffected by camera FOV being narrower than the lidar's 360°.

## Testing

- `sic_slam_node --self-check`: synthetic point-set clustering + ID-matching self-check, matching this project's established `dead_end_backtrack_node.py --self-check` pattern.
- Live verification (manual, post-implementation): run the full stack, confirm `/sic_slam/colored_points` shows real colored geometry in RViz, confirm `/sic_slam/instances` publishes stable IDs across a few seconds of a stationary scene, confirm `dead_end_backtrack_node`'s survey still completes normally with the new scoring term active.

## Explicit non-goals (this pass)

- No deep-learning semantic segmentation — clustering is purely geometric.
- No `explore_lite`/frontier-selection integration — dead-end scoring only.
- No persistent-ID decay/re-identification across occlusion gaps — a cluster that disappears for one frame gets a fresh ID if it reappears.
