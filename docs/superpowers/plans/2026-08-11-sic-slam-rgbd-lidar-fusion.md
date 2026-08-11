# SIC-SLAM: RGB-D + 3D Lidar Fusion Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade the tracked vehicle's camera to RGB-D, colorize the 3D lidar cloud from it, cluster the colorized cloud into persistent-ID instances, and use those instances to improve `dead_end_backtrack_node`'s 360° survey scoring.

**Architecture:** A new C++ ROS2 package `cavex_perception` holds `sic_slam_node`, which subscribes to the RGB-D camera and lidar topics, projects lidar points into the camera image to color them (via `image_geometry::PinholeCameraModel` + `tf2`), clusters the colorized cloud with PCL's `EuclideanClusterExtraction`, tracks instance IDs frame-to-frame by greedy nearest-centroid matching, and publishes `vision_msgs/Detection3DArray` + a colored `PointCloud2`. `dead_end_backtrack_node.py` subscribes to the instance array and applies a score penalty to survey directions with a nearby instance.

**Tech Stack:** ROS2 Jazzy, PCL 1.14 (via `pcl_conversions`), `image_geometry`, `cv_bridge`, `tf2_ros`, `vision_msgs`, Gazebo Harmonic (gz-sim8) `rgbd_camera` sensor type.

## Global Constraints

- Spec source: `docs/superpowers/specs/2026-08-11-sic-slam-rgbd-lidar-fusion-design.md`
- No deep-learning segmentation — clustering is purely geometric (PCL Euclidean clustering on XYZ, color carried as a point attribute only).
- No `explore_lite` integration in this pass — `dead_end_backtrack_node` only.
- No ID decay/re-identification across occlusion gaps — a cluster missing for one frame gets a fresh ID if it reappears.
- Every non-trivial pure-logic addition ships with a runnable self-check (`--self-check` CLI flag), matching this repo's existing convention in `dead_end_backtrack_node.py`.
- Do not guess Gazebo topic names or TF frame conventions — this codebase has been burned by this before (see `gazebo_tracked_vehicle_bridge.yaml`'s own header comment: `/lidar/points` in SDF actually publishes on gz topic `/lidar/points/points`, not the naive `/lidar/points`). Verify live with `gz topic -l` / `ros2 topic echo --once` before finalizing any new bridge or subscription topic name.

---

## File Structure

- Modify: `ros2_ws/src/cavex_tracked_vehicle/models/blueboat/model.sdf.tracked` — camera sensor `type="camera"` → `type="rgbd_camera"`.
- Modify: `ros2_ws/src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml` — add depth image + depth points bridge entries (exact gz topic names confirmed live in Task 1, not guessed).
- Modify: `ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py` — `subscribe_depth: True` on the `rtabmap` node, add depth remappings, add `sic_slam_node` to the launch description.
- Modify: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/dead_end_backtrack_node.py` — new pure `instance_penalty` function + self-check cases, `Detection3DArray` subscription, survey scoring integration.
- Modify: `ros2_ws/src/cavex_tracked_vehicle/package.xml` — add `vision_msgs` exec_depend (needed for the new subscription's message type).
- Create: `ros2_ws/src/cavex_perception/package.xml` — new package manifest.
- Create: `ros2_ws/src/cavex_perception/CMakeLists.txt` — new package build config.
- Create: `ros2_ws/src/cavex_perception/src/instance_clustering.hpp` — pure clustering/tracking types and function declarations (no ROS dependency, PCL/Eigen only).
- Create: `ros2_ws/src/cavex_perception/src/instance_clustering.cpp` — implementation of the above.
- Create: `ros2_ws/src/cavex_perception/src/sic_slam_node.cpp` — the ROS2 node (I/O, projection/colorization, wiring) plus `main()` with a `--self-check` mode.

---

## Task 1: Camera SDF → RGB-D, verify real Gazebo topic names, update the bridge

**Files:**
- Modify: `ros2_ws/src/cavex_tracked_vehicle/models/blueboat/model.sdf.tracked:943-961`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml`

**Interfaces:**
- Produces: real, confirmed gz topic names for the RGB-D camera's depth image and depth camera_info, and their ROS bridge entries (`/camera/depth/image_raw`, `/camera/depth/camera_info` — exact `gz_topic_name` values filled in from live output in Step 3, not assumed).

- [ ] **Step 1: Change the sensor type in the SDF**

In `ros2_ws/src/cavex_tracked_vehicle/models/blueboat/model.sdf.tracked`, change line 943 from:

```xml
      <sensor name="camera_sensor" type="camera">
```

to:

```xml
      <sensor name="camera_sensor" type="rgbd_camera">
```

Leave everything else in that sensor block (pose, update_rate, topic, gz_frame_id, horizontal_fov, image width/height/format, clip near/far) unchanged — `rgbd_camera` accepts the same `<camera>` sub-block as `type="camera"` and additionally publishes a depth image and depth-derived point cloud alongside the existing color image.

- [ ] **Step 2: Launch Gazebo alone and list the real topics**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash && source ardupilot_gazebo_env.sh
colcon build --packages-select cavex_tracked_vehicle --symlink-install
source install/setup.bash
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py &
sleep 30
gz topic -l | grep -i camera
```

Expected: alongside the existing `/camera/color` and `/camera/camera_info` gz topics, new topics appear for depth (typically `/camera/depth_image` and/or `/camera/points`, but confirm the exact strings from this real output — do not assume).

- [ ] **Step 3: Add the confirmed depth bridge entries**

In `ros2_ws/src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml`, following the existing pattern (see the `/camera/color/image_raw` and `/camera/color/camera_info` entries just above), add two new entries using the exact `gz_topic_name` values found in Step 2:

```yaml
- ros_topic_name: "/camera/depth/image_raw"
  gz_topic_name: "<exact gz topic from Step 2, e.g. /camera/depth_image>"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- ros_topic_name: "/camera/depth/camera_info"
  gz_topic_name: "<exact gz topic from Step 2 for depth camera_info>"
  ros_type_name: "sensor_msgs/msg/CameraInfo"
  gz_type_name: "gz.msgs.CameraInfo"
  direction: GZ_TO_ROS
```

If Gazebo publishes depth camera_info on the same topic as color camera_info (common for `rgbd_camera`, since intrinsics are shared), skip the second entry and reuse `/camera/color/camera_info` for depth in Task 2/4 instead — confirm from the Step 2 output which is actually true here.

- [ ] **Step 4: Rebuild and verify the ROS-side topics**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --packages-select cavex_tracked_vehicle --symlink-install
# kill the Gazebo launched in Step 2 first (find its PID via ps aux, kill -9)
source install/setup.bash && source ardupilot_gazebo_env.sh
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py &
sleep 30
ros2 topic echo /camera/depth/image_raw --once
```

Expected: a real `sensor_msgs/msg/Image` message (encoding likely `32FC1` or `16UC1` for depth), not a timeout. Kill the launched Gazebo process afterward (find PID via `ps aux | grep gz`, `kill -9`).

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/models/blueboat/model.sdf.tracked ros2_ws/src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml
git commit -m "Switch camera sensor to RGB-D, bridge depth topics"
```

---

## Task 2: RTAB-Map subscribes to depth

**Files:**
- Modify: `ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py:96-125` (the `rtabmap` Node definition)

**Interfaces:**
- Consumes: `/camera/depth/image_raw`, `/camera/depth/camera_info` (or the color camera_info topic, per Task 1 Step 3's finding) from Task 1.
- Produces: no new interface — RTAB-Map's own internal map now additionally uses depth-camera geometry; no other task depends on this directly.

- [ ] **Step 1: Read the current rtabmap Node block**

```bash
sed -n '96,130p' /home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py
```

Confirm the current `'subscribe_depth': False,` line and the existing `remappings=[...]` list (it currently remaps `rgb/image`, `rgb/camera_info`, `scan_cloud`).

- [ ] **Step 2: Flip subscribe_depth and add the depth remapping**

Change `'subscribe_depth': False,` to `'subscribe_depth': True,` and add to the `remappings` list:

```python
            ('depth/image', '/camera/depth/image_raw'),
```

(RTAB-Map's default depth camera_info topic name follows the `depth/image` remap's namespace automatically via its `camera_info` convention — if Task 1 found depth camera_info on a *different* topic than color's, also add `('depth/camera_info', '<that topic>')` here; if it's the same as color's camera_info, no extra remap line is needed since RTAB-Map already gets that from the existing `rgb/camera_info` remap.)

- [ ] **Step 3: Relaunch the full stack and verify no depth-related errors**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --packages-select cavex_tracked_vehicle --symlink-install
source install/setup.bash
# (Gazebo/ArduPilot already running from Task 1's verification, or relaunch gazebo_tracked_vehicle.launch.py first)
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py > /tmp/task2_slam.log 2>&1 &
sleep 30
grep -iE "error|exception" /tmp/task2_slam.log | grep -i depth
```

Expected: no depth-related errors in the log, and `rtabmap`'s own startup banner (grep for `"rtabmap:"` lines) shows depth subscription active. Kill the launched processes afterward.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py
git commit -m "Subscribe RTAB-Map to the new RGB-D depth stream"
```

---

## Task 3: Instance clustering + tracking — pure logic with self-check

**Files:**
- Create: `ros2_ws/src/cavex_perception/package.xml`
- Create: `ros2_ws/src/cavex_perception/CMakeLists.txt`
- Create: `ros2_ws/src/cavex_perception/src/instance_clustering.hpp`
- Create: `ros2_ws/src/cavex_perception/src/instance_clustering.cpp`
- Create: `ros2_ws/src/cavex_perception/src/sic_slam_node.cpp` (self-check entry point only in this task — full ROS I/O is Task 4)

**Interfaces:**
- Produces (used by Task 4 and Task 5):
  - `struct cavex_perception::Instance { int id; Eigen::Vector3f centroid; Eigen::Vector3f size; };`
  - `std::vector<pcl::PointIndices> cavex_perception::clusterPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud, double tolerance_m, int min_size, int max_size);`
  - `std::vector<cavex_perception::Instance> cavex_perception::clustersToInstances(const pcl::PointCloud<pcl::PointXYZRGB>& cloud, const std::vector<pcl::PointIndices>& clusters);`
  - `std::vector<cavex_perception::Instance> cavex_perception::matchAndAssignIds(const std::vector<cavex_perception::Instance>& new_instances, const std::vector<cavex_perception::Instance>& prev_instances, int& next_id, double match_distance_m);`

- [ ] **Step 1: Create the package manifest**

`ros2_ws/src/cavex_perception/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>cavex_perception</name>
  <version>1.0.0</version>
  <description>SIC-SLAM: RGB-D + 3D lidar fusion and geometric instance clustering for the CaveX tracked vehicle.</description>
  <maintainer email="petrisor.parvu@upb.ro">CaveX Team</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_eigen</depend>
  <depend>cv_bridge</depend>
  <depend>image_geometry</depend>
  <depend>pcl_conversions</depend>
  <depend>libpcl-all-dev</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

- [ ] **Step 2: Create the CMakeLists**

`ros2_ws/src/cavex_perception/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(cavex_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common search kdtree segmentation)

add_executable(sic_slam_node
  src/sic_slam_node.cpp
  src/instance_clustering.cpp
)
target_include_directories(sic_slam_node PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(sic_slam_node ${PCL_LIBRARIES})
ament_target_dependencies(sic_slam_node
  rclcpp sensor_msgs vision_msgs geometry_msgs tf2_ros tf2_eigen
  cv_bridge image_geometry pcl_conversions)

install(TARGETS sic_slam_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
```

- [ ] **Step 3: Write the header (function declarations)**

`ros2_ws/src/cavex_perception/src/instance_clustering.hpp`:

```cpp
#ifndef CAVEX_PERCEPTION__INSTANCE_CLUSTERING_HPP_
#define CAVEX_PERCEPTION__INSTANCE_CLUSTERING_HPP_

#include <vector>

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace cavex_perception
{

struct Instance
{
  int id;
  Eigen::Vector3f centroid;
  Eigen::Vector3f size;
};

// Segments `cloud` into geometric clusters. Points closer together than
// tolerance_m belong to the same cluster; clusters outside [min_size,
// max_size] points are discarded as noise/too-large-to-be-an-instance.
std::vector<pcl::PointIndices> clusterPoints(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  double tolerance_m, int min_size, int max_size);

// Computes the centroid and axis-aligned bounding-box size of each cluster.
// Returned instances have id = -1 (unassigned) -- matchAndAssignIds fills
// in real ids.
std::vector<Instance> clustersToInstances(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
  const std::vector<pcl::PointIndices> & clusters);

// Greedily matches each of new_instances to the closest prev_instances
// centroid within match_distance_m (each previous instance can match at
// most once). Matched instances keep the previous id; unmatched new
// instances get a fresh id from next_id (which is incremented). Returns
// new_instances with ids assigned, in the same order as the input.
std::vector<Instance> matchAndAssignIds(
  const std::vector<Instance> & new_instances,
  const std::vector<Instance> & prev_instances,
  int & next_id, double match_distance_m);

}  // namespace cavex_perception

#endif  // CAVEX_PERCEPTION__INSTANCE_CLUSTERING_HPP_
```

- [ ] **Step 4: Implement the pure logic**

`ros2_ws/src/cavex_perception/src/instance_clustering.cpp`:

```cpp
#include "instance_clustering.hpp"

#include <limits>

#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>

namespace cavex_perception
{

std::vector<pcl::PointIndices> clusterPoints(
  const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
  double tolerance_m, int min_size, int max_size)
{
  std::vector<pcl::PointIndices> clusters;
  if (cloud->empty()) {
    return clusters;
  }
  auto tree = pcl::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
  tree->setInputCloud(cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(tolerance_m);
  ec.setMinClusterSize(min_size);
  ec.setMaxClusterSize(max_size);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud);
  ec.extract(clusters);
  return clusters;
}

std::vector<Instance> clustersToInstances(
  const pcl::PointCloud<pcl::PointXYZRGB> & cloud,
  const std::vector<pcl::PointIndices> & clusters)
{
  std::vector<Instance> instances;
  instances.reserve(clusters.size());
  for (const auto & cluster : clusters) {
    Eigen::Vector3f min_pt(
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max(),
      std::numeric_limits<float>::max());
    Eigen::Vector3f max_pt(
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::lowest(),
      std::numeric_limits<float>::lowest());
    Eigen::Vector3f sum = Eigen::Vector3f::Zero();

    for (int idx : cluster.indices) {
      const auto & p = cloud.points[idx];
      Eigen::Vector3f v(p.x, p.y, p.z);
      sum += v;
      min_pt = min_pt.cwiseMin(v);
      max_pt = max_pt.cwiseMax(v);
    }

    Instance inst;
    inst.id = -1;
    inst.centroid = sum / static_cast<float>(cluster.indices.size());
    inst.size = max_pt - min_pt;
    instances.push_back(inst);
  }
  return instances;
}

std::vector<Instance> matchAndAssignIds(
  const std::vector<Instance> & new_instances,
  const std::vector<Instance> & prev_instances,
  int & next_id, double match_distance_m)
{
  std::vector<Instance> result = new_instances;
  std::vector<bool> prev_used(prev_instances.size(), false);

  for (auto & inst : result) {
    int best_idx = -1;
    float best_dist = static_cast<float>(match_distance_m);
    for (size_t j = 0; j < prev_instances.size(); ++j) {
      if (prev_used[j]) {
        continue;
      }
      float dist = (inst.centroid - prev_instances[j].centroid).norm();
      if (dist <= best_dist) {
        best_dist = dist;
        best_idx = static_cast<int>(j);
      }
    }
    if (best_idx >= 0) {
      inst.id = prev_instances[best_idx].id;
      prev_used[best_idx] = true;
    } else {
      inst.id = next_id++;
    }
  }
  return result;
}

}  // namespace cavex_perception
```

- [ ] **Step 5: Write the self-check (in `sic_slam_node.cpp`, this task's only content)**

`ros2_ws/src/cavex_perception/src/sic_slam_node.cpp` — self-check portion only for now (Task 4 adds the full node):

```cpp
#include <cstring>
#include <iostream>

#include "instance_clustering.hpp"

using cavex_perception::Instance;
using cavex_perception::clusterPoints;
using cavex_perception::clustersToInstances;
using cavex_perception::matchAndAssignIds;

namespace
{

pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeBlob(
  float cx, float cy, float cz, int n_points, float spread)
{
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (int i = 0; i < n_points; ++i) {
    pcl::PointXYZRGB p;
    float t = static_cast<float>(i) / static_cast<float>(n_points);
    p.x = cx + spread * (t - 0.5f);
    p.y = cy + spread * (t - 0.5f);
    p.z = cz;
    p.r = p.g = p.b = 200;
    cloud->push_back(p);
  }
  return cloud;
}

void selfCheck()
{
  // Two well-separated blobs (5m apart) should cluster into 2 instances.
  auto two_blobs = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  *two_blobs += *makeBlob(0.0f, 0.0f, 0.0f, 20, 0.3f);
  *two_blobs += *makeBlob(5.0f, 0.0f, 0.0f, 20, 0.3f);
  auto clusters = clusterPoints(two_blobs, 0.5, 5, 1000);
  if (clusters.size() != 2) {
    std::cerr << "FAIL: expected 2 clusters for two well-separated blobs, got "
              << clusters.size() << "\n";
    std::exit(1);
  }

  // A single merged blob (points spread continuously, gaps < tolerance)
  // should cluster into exactly 1 instance.
  auto merged = makeBlob(0.0f, 0.0f, 0.0f, 50, 2.0f);
  auto merged_clusters = clusterPoints(merged, 0.5, 5, 1000);
  if (merged_clusters.size() != 1) {
    std::cerr << "FAIL: expected 1 cluster for a continuous blob, got "
              << merged_clusters.size() << "\n";
    std::exit(1);
  }

  // Centroid of a symmetric blob at (5,0,0) should land near (5,0,0).
  auto instances = clustersToInstances(*two_blobs, clusters);
  bool found_near_five = false;
  for (const auto & inst : instances) {
    if (std::abs(inst.centroid.x() - 5.0f) < 0.3f) {
      found_near_five = true;
    }
  }
  if (!found_near_five) {
    std::cerr << "FAIL: expected one cluster centroid near x=5.0\n";
    std::exit(1);
  }

  // ID tracking: a repeat frame with a small centroid shift (0.1m, well
  // within match_distance_m=1.0) should keep the same ids; a frame with
  // only one of the two blobs should keep that one's id and not reuse the
  // other's id for a new, unrelated instance.
  int next_id = 0;
  auto frame1 = matchAndAssignIds(instances, {}, next_id, 1.0);
  if (frame1.size() != 2 || frame1[0].id == frame1[1].id) {
    std::cerr << "FAIL: expected 2 distinct fresh ids on first frame\n";
    std::exit(1);
  }

  std::vector<Instance> shifted = frame1;
  shifted[0].centroid.x() += 0.1f;
  shifted[1].centroid.x() += 0.1f;
  auto frame2 = matchAndAssignIds(shifted, frame1, next_id, 1.0);
  if (frame2[0].id != frame1[0].id || frame2[1].id != frame1[1].id) {
    std::cerr << "FAIL: expected ids to persist across a small centroid shift\n";
    std::exit(1);
  }

  std::vector<Instance> only_second = {frame1[1]};
  only_second[0].centroid.x() += 0.1f;
  auto frame3 = matchAndAssignIds(only_second, frame1, next_id, 1.0);
  if (frame3[0].id != frame1[1].id) {
    std::cerr << "FAIL: expected the surviving instance to keep its id\n";
    std::exit(1);
  }

  std::cout << "sic_slam_node self-check: OK\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--self-check") == 0) {
      selfCheck();
      return 0;
    }
  }
  std::cerr << "sic_slam_node: full ROS node not yet implemented "
               "(see Task 4). Run with --self-check for now.\n";
  return 1;
}
```

- [ ] **Step 6: Build and run the self-check, verify it passes**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cavex_perception --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run cavex_perception sic_slam_node --self-check
```

Expected: `sic_slam_node self-check: OK`. If any `FAIL:` line prints, fix `instance_clustering.cpp` (not the self-check) until it passes — the self-check encodes the correct behavior.

- [ ] **Step 7: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_perception/
git commit -m "Add cavex_perception package: pure instance clustering/tracking logic with self-check"
```

---

## Task 4: sic_slam_node — full ROS I/O (colorization, publishing, launch wiring)

**Files:**
- Modify: `ros2_ws/src/cavex_perception/src/sic_slam_node.cpp` (add the full node; keep the self-check from Task 3 working via the same `--self-check` flag)
- Modify: `ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py` (add `sic_slam_node` to the launch description)

**Interfaces:**
- Consumes: `Instance`, `clusterPoints`, `clustersToInstances`, `matchAndAssignIds` from Task 3.
- Produces: `/sic_slam/instances` (`vision_msgs/msg/Detection3DArray`, frame_id `"map"`) and `/sic_slam/colored_points` (`sensor_msgs/msg/PointCloud2`, frame_id `"map"`) — consumed by Task 5.

- [ ] **Step 1: Add the SicSlamNode class above `main()` in `sic_slam_node.cpp`**

Insert after the `selfCheck()` function and its closing `}  // namespace`, before `int main`:

```cpp
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <pcl_conversions/pcl_conversions.h>

namespace cavex_perception
{

class SicSlamNode : public rclcpp::Node
{
public:
  SicSlamNode()
  : Node("sic_slam_node"), next_id_(0)
  {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    color_sub_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", 10,
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(color_mutex_);
        latest_color_ = msg;
      });
    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/color/camera_info", 10,
      [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(color_mutex_);
        latest_info_ = msg;
      });
    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar/points", rclcpp::SensorDataQoS(),
      std::bind(&SicSlamNode::lidarCallback, this, std::placeholders::_1));

    instances_pub_ = create_publisher<vision_msgs::msg::Detection3DArray>(
      "/sic_slam/instances", 10);
    colored_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/sic_slam/colored_points", 10);

    RCLCPP_INFO(get_logger(), "sic_slam_node ready: colorizing lidar via camera "
      "projection, clustering into instances, publishing /sic_slam/instances "
      "and /sic_slam/colored_points.");
  }

private:
  void lidarCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg)
  {
    sensor_msgs::msg::Image::ConstSharedPtr color;
    sensor_msgs::msg::CameraInfo::ConstSharedPtr info;
    {
      std::lock_guard<std::mutex> lock(color_mutex_);
      color = latest_color_;
      info = latest_info_;
    }
    if (!color || !info) {
      return;  // no color frame cached yet -- skip this lidar frame
    }

    // Transform: lidar frame -> camera optical frame, for projection.
    geometry_msgs::msg::TransformStamped lidar_to_cam;
    try {
      lidar_to_cam = tf_buffer_->lookupTransform(
        color->header.frame_id, lidar_msg->header.frame_id,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF lookup %s -> %s failed: %s", lidar_msg->header.frame_id.c_str(),
        color->header.frame_id.c_str(), ex.what());
      return;
    }
    // Transform: lidar frame -> map, so published instances/cloud are in
    // the same frame dead_end_backtrack_node's pose/costmap use.
    geometry_msgs::msg::TransformStamped lidar_to_map;
    try {
      lidar_to_map = tf_buffer_->lookupTransform(
        "map", lidar_msg->header.frame_id, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "TF lookup %s -> map failed: %s", lidar_msg->header.frame_id.c_str(),
        ex.what());
      return;
    }

    cv_bridge::CvImageConstPtr cv_color;
    try {
      cv_color = cv_bridge::toCvShare(color, sensor_msgs::image_encodings::BGR8);
    } catch (const cv_bridge::Exception & ex) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "cv_bridge conversion failed: %s", ex.what());
      return;
    }

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(*info);

    pcl::PointCloud<pcl::PointXYZ> raw_cloud;
    pcl::fromROSMsg(*lidar_msg, raw_cloud);

    Eigen::Isometry3d lidar_to_cam_eigen = tf2::transformToEigen(lidar_to_cam);
    Eigen::Isometry3d lidar_to_map_eigen = tf2::transformToEigen(lidar_to_map);

    auto colored = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    colored->reserve(raw_cloud.size());
    for (const auto & p : raw_cloud.points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }
      Eigen::Vector3d p_lidar(p.x, p.y, p.z);
      Eigen::Vector3d p_cam = lidar_to_cam_eigen * p_lidar;

      pcl::PointXYZRGB cp;
      cp.r = cp.g = cp.b = 128;  // default gray, overwritten below if in view
      if (p_cam.z() > 0.0) {
        cv::Point2d uv = cam_model.project3dToPixel(
          cv::Point3d(p_cam.x(), p_cam.y(), p_cam.z()));
        int u = static_cast<int>(uv.x);
        int v = static_cast<int>(uv.y);
        if (u >= 0 && u < cv_color->image.cols && v >= 0 && v < cv_color->image.rows) {
          cv::Vec3b bgr = cv_color->image.at<cv::Vec3b>(v, u);
          cp.b = bgr[0];
          cp.g = bgr[1];
          cp.r = bgr[2];
        }
      }

      Eigen::Vector3d p_map = lidar_to_map_eigen * p_lidar;
      cp.x = static_cast<float>(p_map.x());
      cp.y = static_cast<float>(p_map.y());
      cp.z = static_cast<float>(p_map.z());
      colored->push_back(cp);
    }

    auto clusters = clusterPoints(colored, cluster_tolerance_m_, min_cluster_size_, max_cluster_size_);
    auto raw_instances = clustersToInstances(*colored, clusters);
    auto instances = matchAndAssignIds(raw_instances, prev_instances_, next_id_, match_distance_m_);
    prev_instances_ = instances;

    publishInstances(instances, lidar_msg->header.stamp);
    publishColoredCloud(*colored, lidar_msg->header.stamp);
  }

  void publishInstances(
    const std::vector<Instance> & instances, const builtin_interfaces::msg::Time & stamp)
  {
    vision_msgs::msg::Detection3DArray msg;
    msg.header.frame_id = "map";
    msg.header.stamp = stamp;
    for (const auto & inst : instances) {
      vision_msgs::msg::Detection3D det;
      det.header = msg.header;
      det.id = std::to_string(inst.id);
      det.bbox.center.position.x = inst.centroid.x();
      det.bbox.center.position.y = inst.centroid.y();
      det.bbox.center.position.z = inst.centroid.z();
      det.bbox.center.orientation.w = 1.0;
      det.bbox.size.x = inst.size.x();
      det.bbox.size.y = inst.size.y();
      det.bbox.size.z = inst.size.z();
      msg.detections.push_back(det);
    }
    instances_pub_->publish(msg);
  }

  void publishColoredCloud(
    const pcl::PointCloud<pcl::PointXYZRGB> & cloud, const builtin_interfaces::msg::Time & stamp)
  {
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = stamp;
    colored_pub_->publish(msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr instances_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  std::mutex color_mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr latest_color_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_info_;

  std::vector<Instance> prev_instances_;
  int next_id_;

  // Matches dead_end_backtrack_node's OPENING_SCAN_RADIUS_M (2.4m) scale --
  // clusters within 0.5m of each other merge into one instance, a plausible
  // single real object/wall-feature at this vehicle's ~0.3m footprint scale.
  double cluster_tolerance_m_ = 0.5;
  int min_cluster_size_ = 10;
  int max_cluster_size_ = 25000;
  double match_distance_m_ = 1.0;
};

}  // namespace cavex_perception
```

- [ ] **Step 2: Update `main()` to run the real node when not self-checking**

Replace the existing `main()` in `sic_slam_node.cpp` with:

```cpp
int main(int argc, char ** argv)
{
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--self-check") == 0) {
      selfCheck();
      return 0;
    }
  }
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<cavex_perception::SicSlamNode>());
  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 3: Rebuild, confirm the self-check still passes, and confirm the node starts**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cavex_perception --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 run cavex_perception sic_slam_node --self-check
```

Expected: `sic_slam_node self-check: OK` (unchanged from Task 3 — confirms the new ROS-node code didn't break the pure-logic build).

- [ ] **Step 4: Add sic_slam_node to the SLAM launch file**

In `ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py`, add near the other `Node(...)` definitions:

```python
    sic_slam_node = Node(
        package='cavex_perception',
        executable='sic_slam_node',
        name='sic_slam_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
```

and add `sic_slam_node` to the `LaunchDescription([...])` list returned at the end of the file (alongside the existing `icp_odometry`, `rtabmap`, etc. entries).

- [ ] **Step 5: Full-stack live verification**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source install/setup.bash && source ardupilot_gazebo_env.sh
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py &
sleep 30
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py > /tmp/task4_slam.log 2>&1 &
sleep 60
ros2 topic echo /sic_slam/instances --once
ros2 topic hz /sic_slam/colored_points
```

Expected: `/sic_slam/instances` returns a real (possibly empty, that's fine) `Detection3DArray`; `/sic_slam/colored_points` publishes at a nonzero rate. Check `/tmp/task4_slam.log` for `sic_slam_node` errors (TF lookup failures are expected/logged-not-fatal until `map` and both sensor frames are all live — if they persist past ~30s of driving, investigate the frame_id strings via `ros2 topic echo /camera/color/image_raw --once | grep frame_id` and `ros2 run tf2_ros tf2_echo <lidar frame> <camera frame>` rather than guessing). Kill all launched processes afterward.

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_perception/src/sic_slam_node.cpp ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py
git commit -m "Implement sic_slam_node ROS I/O: camera-lidar colorization, clustering, publishing"
```

---

## Task 5: dead_end_backtrack_node integration

**Files:**
- Modify: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/dead_end_backtrack_node.py`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/package.xml` (add `vision_msgs` exec_depend)

**Interfaces:**
- Consumes: `/sic_slam/instances` (`vision_msgs/msg/Detection3DArray`) from Task 4.

- [ ] **Step 1: Add the vision_msgs dependency**

In `ros2_ws/src/cavex_tracked_vehicle/package.xml`, add alongside the existing `<depend>` entries:

```xml
  <depend>vision_msgs</depend>
```

- [ ] **Step 2: Write the failing self-check case for the new pure function**

In `dead_end_backtrack_node.py`, find the `_self_check()` function (around line 653) and add, just before the final `print("dead_end_backtrack_node self-check: OK")` line:

```python
    # instance_penalty: an instance sitting directly in the scan direction,
    # within OPENING_SCAN_RADIUS_M, should reduce the score; one far off to
    # the side or beyond the radius should not affect it at all.
    no_instances_penalty = instance_penalty([], 0.0, 0.0, 0.0)
    assert no_instances_penalty == 0.0, \
        "no instances should mean zero penalty"
    blocking_instance = [(1.5, 0.0)]  # 1.5m straight ahead at yaw=0
    assert instance_penalty(blocking_instance, 0.0, 0.0, 0.0) > 0.0, \
        "an instance directly ahead within scan radius should add a penalty"
    off_to_side = [(0.0, 5.0)]  # 5m to the side, not ahead
    assert instance_penalty(off_to_side, 0.0, 0.0, 0.0) == 0.0, \
        "an instance off to the side should not be penalized"
    too_far = [(20.0, 0.0)]  # ahead, but beyond OPENING_SCAN_RADIUS_M
    assert instance_penalty(too_far, 0.0, 0.0, 0.0) == 0.0, \
        "an instance beyond the scan radius should not be penalized"
```

- [ ] **Step 3: Run the self-check and confirm it fails (function doesn't exist yet)**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
python3 src/cavex_tracked_vehicle/cavex_tracked_vehicle/dead_end_backtrack_node.py --self-check
```

Expected: `NameError: name 'instance_penalty' is not defined`.

- [ ] **Step 4: Implement `instance_penalty`**

In `dead_end_backtrack_node.py`, add this pure function right after `clearance_on_side` (around line 313, before `class DeadEndBacktrackNode`):

```python
INSTANCE_PENALTY_M = 2.0  # subtracted from a survey direction's clearance
                          # score if a known instance sits in that direction
                          # within OPENING_SCAN_RADIUS_M -- large enough to
                          # usually demote a cluttered opening below a real,
                          # instance-free one, without being an outright veto
                          # (ray_clear_distance's own SURVEY_FORWARD_CHECK_M
                          # cap is 5.0m, so this is a meaningful fraction of
                          # that range, not a rounding error).
INSTANCE_LATERAL_TOLERANCE_M = 0.5  # how far off the ray's centerline an
                                    # instance can be and still count as
                                    # "in" that direction -- roughly this
                                    # vehicle's own track width.


def instance_penalty(instance_centroids, x, y, yaw,
                      radius_m=OPENING_SCAN_RADIUS_M,
                      lateral_tolerance_m=INSTANCE_LATERAL_TOLERANCE_M,
                      penalty_m=INSTANCE_PENALTY_M):
    """How much to subtract from a survey direction's clearance score
    because a known instance (from /sic_slam/instances) sits in that
    direction. instance_centroids is a list of (x, y) tuples in the same
    frame as the pose ("map"). Pure function, no ROS dependency."""
    dx, dy = math.cos(yaw), math.sin(yaw)
    for (ix, iy) in instance_centroids:
        rel_x, rel_y = ix - x, iy - y
        along = rel_x * dx + rel_y * dy
        if along < 0.0 or along > radius_m:
            continue
        lateral = abs(rel_x * -dy + rel_y * dx)
        if lateral <= lateral_tolerance_m:
            return penalty_m
    return 0.0
```

- [ ] **Step 5: Run the self-check and confirm it passes**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
python3 src/cavex_tracked_vehicle/cavex_tracked_vehicle/dead_end_backtrack_node.py --self-check
```

Expected: `dead_end_backtrack_node self-check: OK`.

- [ ] **Step 6: Wire the subscription and apply the penalty in the survey tick**

Add the import near the top of the file, alongside the existing `from nav_msgs.msg import ...` line:

```python
from vision_msgs.msg import Detection3DArray
```

In `DeadEndBacktrackNode.__init__`, add alongside the existing `create_subscription` calls (after the costmap subscription):

```python
        self._instance_centroids = []
        self.create_subscription(Detection3DArray, '/sic_slam/instances',
                                  self._instances_cb, 10)
```

Add the callback method next to `_costmap_cb`:

```python
    def _instances_cb(self, msg: Detection3DArray):
        self._instance_centroids = [
            (d.bbox.center.position.x, d.bbox.center.position.y)
            for d in msg.detections
        ]
```

In `_tick_survey` (around line 495-501), change:

```python
            clearance = ray_clear_distance(self._latest_costmap, x, y, yaw, SURVEY_FORWARD_CHECK_M)
            if clearance > self._survey_best_clearance:
```

to:

```python
            clearance = ray_clear_distance(self._latest_costmap, x, y, yaw, SURVEY_FORWARD_CHECK_M)
            clearance -= instance_penalty(self._instance_centroids, x, y, yaw)
            if clearance > self._survey_best_clearance:
```

- [ ] **Step 7: Full-stack live verification**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source install/setup.bash
colcon build --packages-select cavex_tracked_vehicle --symlink-install
source install/setup.bash && source ardupilot_gazebo_env.sh
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py &
sleep 30
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py > /tmp/task5_slam.log 2>&1 &
sleep 60
grep -i "dead_end_backtrack_node ready" /tmp/task5_slam.log
grep -iE "error|traceback" /tmp/task5_slam.log | grep -i dead_end
```

Expected: `dead_end_backtrack_node ready:` banner present, no new errors attributable to `dead_end_backtrack_node` (a dead-end trigger itself doesn't need to fire in this short a window — the check here is "no crash/import error," not "observed a real dead-end encounter"). Kill all launched processes afterward.

- [ ] **Step 8: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/dead_end_backtrack_node.py ros2_ws/src/cavex_tracked_vehicle/package.xml
git commit -m "Penalize dead-end survey openings with a nearby SIC-SLAM instance"
```
