# CaveX Legged Walker (Phase 1) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A CHAMP-driven, Spot-proportioned quadruped autonomously explores the dry-cave section of `CaveX-Explorer-Pro`'s Gazebo Harmonic world, building a real 3D map (RTAB-Map) via Nav2 frontier exploration, with live telemetry on the existing web dashboard and a real ATE evaluation over a fixed sim-time exploration budget.

**Architecture:** Vendor CHAMP's ROS2 core packages (`champ`, `champ_base`, `champ_msgs` — proven working on Jazzy+Harmonic by a real community project, see Task 1) and `explore_lite` (via the `m-explore-ros2` ROS2 port) into `ros2_ws/src`. Build a new `cavex_walker_description` package with a Spot-proportioned URDF (CHAMP's own config format + our sensor blocks) driven through `gz_ros2_control`. Reuse `cavex_slam_nav`'s existing RTAB-Map/telemetry/ATE infrastructure, reconfigured for 3D lidar and autonomous exploration instead of scripted 2D driving.

**Tech Stack:** ROS2 Jazzy, Gazebo Harmonic (gz-sim), `ros2_control`/`gz_ros2_control`, CHAMP quadruped framework, `explore_lite` (m-explore-ros2 port), Nav2, RTAB-Map (3D/ICP mode), existing `web_telemetry_bridge.py`/React dashboard pattern.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-08-04-cavex-legged-walker-phase1-design.md` — every task below implements a section of it.
- No sonar in this phase (no water present).
- No `sic_slam_node.py` / "SIC-SLAM" labeling in this phase (no current to compensate for). It stays dormant.
- No literal "Spot" model — always label the robot "Spot-like quadruped (CHAMP)" in code comments, launch descriptions, and dashboard UI text.
- Build with `colcon build --symlink-install` from `/home/parvu/CaveX-Explorer-Pro/ros2_ws`, sourcing `/opt/ros/jazzy/setup.bash` first, matching every other package in this workspace.
- Verification in this codebase has consistently meant real `ros2`/`gz` CLI checks (`topic list`, `topic hz`, `topic echo`), not a pytest suite — this package has none, and none should be invented for this plan. Follow that established pattern.

---

### Task 1: Vendor and verify CHAMP's ROS2 core packages build

**Files:**
- Create: `ros2_ws/src/champ/` (vendored, git-cloned — not authored by us)
- Create: `ros2_ws/src/champ_base/` (vendored)
- Create: `ros2_ws/src/champ_msgs/` (vendored)

**Interfaces:**
- Produces: a working `champ_base` node that later tasks (Task 4, Task 5) launch, subscribing to `/cmd_vel` (`geometry_msgs/msg/Twist`) and publishing joint trajectory commands consumable by `gz_ros2_control`.

- [ ] **Step 1: Clone the proven Jazzy+Harmonic CHAMP core packages**

RobInLabUJI/unitree_go2_ros2_jazzy is a real, documented ROS2 Jazzy + Gazebo Harmonic project that already vendors and builds `champ`/`champ_base`/`champ_msgs` successfully (confirmed via its README this session — uses `ros-jazzy-gz-ros2-control`, targets Gazebo Harmonic explicitly). Clone just those three core packages out of it rather than upstream `chvmp/champ` (whose main-branch docs are ROS1/Kinetic-era and unverified for Jazzy):

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws/src
git clone --depth 1 https://github.com/RobInLabUJI/unitree_go2_ros2_jazzy.git /tmp/champ_source
cp -r /tmp/champ_source/champ /tmp/champ_source/champ_base /tmp/champ_source/champ_msgs .
rm -rf /tmp/champ_source
```

If any of those three directories don't exist at that exact path in the cloned repo, run `find /tmp/champ_source -maxdepth 2 -iname "champ*"` before deleting `/tmp/champ_source` to locate their real paths, and adjust the `cp` accordingly — the repo's exact layout wasn't byte-verified before writing this plan, only confirmed to exist via its README.

- [ ] **Step 2: Build just the three vendored packages**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/champ src/champ_base src/champ_msgs --ignore-src -r -y
colcon build --symlink-install --packages-select champ champ_msgs champ_base
```

Expected: `Summary: 3 packages finished`. If it fails, read the actual error — common failure modes for a vendored ROS1-adjacent package are missing `package.xml` `<buildtool_depend>ament_cmake</buildtool_depend>` (ROS1 packages use `catkin`) or Python 2 syntax. If the package.xml format is ROS1-style, this specific vendoring approach doesn't work and this task must switch to a from-scratch `champ_base`-equivalent node written against `champ`'s header-only gait library directly (a real fallback, but bigger — flag this back to the user before proceeding rather than silently improvising a large detour).

- [ ] **Step 3: Verify the built node actually starts and exposes the expected interface**

```bash
source install/setup.bash
ros2 run champ_base champ_base_node --ros-args -p gazebo:=true 2>&1 | head -20
```

(Exact executable name confirmed at Step 2's build output — if `champ_base_node` isn't the real executable name, check `ros2 pkg executables champ_base` and use the real one here and in Task 5.)

Expected: node starts without a Python/C++ traceback and without exiting immediately. Kill it (Ctrl-C / `pkill -f champ_base`) once confirmed — this step only proves the package is runnable, not correct behavior (that's Task 5's job, once it's wired to a real robot description).

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/champ ros2_ws/src/champ_base ros2_ws/src/champ_msgs
git commit -m "Vendor CHAMP ROS2 core packages (champ, champ_base, champ_msgs)"
```

---

### Task 2: Vendor and verify explore_lite (m-explore-ros2) builds

**Files:**
- Create: `ros2_ws/src/m-explore-ros2/` (vendored)

**Interfaces:**
- Produces: `explore_lite`'s `explore` executable, subscribing to a Nav2-style costmap topic and publishing `explore/frontiers` (`visualization_msgs/msg/MarkerArray`) and sending Nav2 `NavigateToPose` action goals directly (confirmed real behavior to verify at Task 9, not assumed here).

- [ ] **Step 1: Clone**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws/src
git clone --depth 1 https://github.com/robo-friends/m-explore-ros2.git
```

- [ ] **Step 2: Build**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/m-explore-ros2 --ignore-src -r -y
colcon build --symlink-install --packages-up-to explore_lite
```

Expected: `explore_lite` (and any dependency packages the repo also contains, e.g. a `nav2_map_server`-adjacent map-merge package) finish. If `explore_lite` isn't the exact top-level package name inside this repo, run `find src/m-explore-ros2 -name package.xml -exec grep -H "<name>" {} \;` first to get the real name and use that in `--packages-up-to` and in Task 9.

- [ ] **Step 3: Verify the executable exists**

```bash
source install/setup.bash
ros2 pkg executables explore_lite
```

Expected: lists an `explore` executable (per the repo's documented `ros2 run explore_lite explore` usage, confirmed via this session's research — not yet run against our own costmap, that's Task 9).

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/m-explore-ros2
git commit -m "Vendor explore_lite (m-explore-ros2 port) for frontier exploration"
```

---

### Task 3: Install ros2_control/Nav2 apt dependencies, declare them in package.xml

**Files:**
- Modify: `ros2_ws/src/cavex_slam_nav/package.xml`

**Interfaces:**
- Produces: nothing new at the code level — this task just makes the following apt packages available system-wide and declares them as real dependencies, so later tasks can assume they exist: `ros-jazzy-ros2-control`, `ros-jazzy-ros2-controllers`, `ros-jazzy-gz-ros2-control`, `ros-jazzy-nav2-bringup`, `ros-jazzy-nav2-costmap-2d`.

- [ ] **Step 1: Install (all four confirmed present in this session's apt cache — versions: ros2-control 4.45.2, ros2-controllers 4.40.1, gz-ros2-control 1.2.19, nav2-bringup 1.3.12)**

```bash
sudo apt-get install -y ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-gz-ros2-control ros-jazzy-nav2-bringup ros-jazzy-nav2-costmap-2d
```

- [ ] **Step 2: Add to package.xml**

Read `ros2_ws/src/cavex_slam_nav/package.xml` first. Add these lines next to the existing `<depend>ros_gz_sim</depend>`/`<depend>ros_gz_bridge</depend>` lines:

```xml
  <depend>ros2_control</depend>
  <depend>ros2_controllers</depend>
  <depend>gz_ros2_control</depend>
  <depend>nav2_bringup</depend>
```

(`nav2_bringup` is likely already listed — check before adding a duplicate; if present, skip adding it again.)

- [ ] **Step 3: Rebuild to confirm package.xml is still valid**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cavex_slam_nav
```

Expected: `Summary: 1 package finished`.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/package.xml
git commit -m "Add ros2_control/gz_ros2_control/nav2 dependencies for the legged walker"
```

---

### Task 4: Compose the Spot-like walker URDF with our sensors

**Files:**
- Create: `ros2_ws/src/cavex_slam_nav/urdf/cavex_walker.urdf.xacro`
- Create: `ros2_ws/src/cavex_slam_nav/config/cavex_walker_gait.yaml` (CHAMP gait config)
- Create: `ros2_ws/src/cavex_slam_nav/config/cavex_walker_ros2_control.yaml` (`gz_ros2_control` controller config)

**Interfaces:**
- Consumes: `champ_base` (Task 1) as the gait controller; `gz_ros2_control` (Task 3) as the joint-command bridge.
- Produces: a spawnable robot description (`robot_description` topic, same pattern as `cavex_robot.urdf.xacro`) with a `<gz_frame_id>` on every sensor, matching the existing convention in `cavex_robot.urdf.xacro` (needed for TF to line up, per this session's own hard-won finding).

- [ ] **Step 1: Locate CHAMP's bundled Spot leg/body dimensions**

CHAMP's upstream repo (`chvmp/champ`) advertises a pre-configured Spot example (confirmed via this session's research, not yet inspected byte-for-byte). Find its actual dimensions/joint layout:

```bash
git clone --depth 1 https://github.com/chvmp/robots.git /tmp/champ_robots
find /tmp/champ_robots -iname "*spot*"
```

Read whatever config/URDF file(s) that turns up (likely a `champ_config`-style YAML with body/leg link lengths under a `spot_config` directory) — use its body length/width/leg segment lengths as the real numbers for Step 2. If nothing Spot-named turns up in that repo, fall back to Boston Dynamics' publicly documented Spot dimensions (length 1.1m, width 0.5m, standing height ~0.84m — from Boston Dynamics' public spec sheet, not the proprietary CAD) and note in the xacro file's header comment which source the numbers came from.

- [ ] **Step 2: Write the xacro file**

Structure it exactly like the existing `ros2_ws/src/cavex_slam_nav/urdf/cavex_robot.urdf.xacro` (read it first for the sensor-block pattern): a `base_link` sized per Step 1's numbers, four legs each with hip/thigh/knee joints (CHAMP's expected joint naming convention — check `champ_base`'s source for the exact joint names it expects, likely `<leg>_hip_joint`, `<leg>_thigh_joint`, `<leg>_knee_joint` for `leg` in `lf/rf/lh/rh` or similar; grep the vendored `champ_base` config for the real names rather than guessing):

```bash
grep -rn "joint_name\|_hip_joint\|_thigh_joint\|_knee_joint" ros2_ws/src/champ_base/config/ ros2_ws/src/champ/ 2>/dev/null | head -30
```

Use those exact joint names in the xacro. Add sensor blocks copied from `cavex_robot.urdf.xacro`'s pattern:
- Camera: identical block to the existing `camera_link` sensor (same `<topic>camera/color</topic>`, same `<gz_frame_id>`).
- IMU: identical block to the existing `imu_sensor` (same `<topic>imu</topic>`).
- 3D lidar: same `gpu_lidar` sensor type as the existing 2D one, but with real 3D parameters instead of a single horizontal ring:
```xml
<gazebo reference="lidar_link">
  <sensor type="gpu_lidar" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <update_rate>10</update_rate>
    <topic>lidar/points</topic>
    <gz_frame_id>lidar_link</gz_frame_id>
    <ray>
      <scan>
        <horizontal><samples>360</samples><resolution>1</resolution><min_angle>-3.14</min_angle><max_angle>3.14</max_angle></horizontal>
        <vertical><samples>16</samples><resolution>1</resolution><min_angle>-0.2618</min_angle><max_angle>0.2618</max_angle></vertical>
      </scan>
      <range><min>0.3</min><max>12.0</max></range>
    </ray>
  </sensor>
</gazebo>
```
(16 vertical channels, ±15° vertical FOV — a real, if modest, 3D lidar; adjust later if RTAB-Map's ICP needs denser data.)

Add the `gz_ros2_control` plugin block (replaces `cavex_robot`'s `VelocityControl`/`OdometryPublisher` — this robot has real joints, not a free body):
```xml
<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find cavex_slam_nav)/config/cavex_walker_ros2_control.yaml</parameters>
  </plugin>
</gazebo>
```
(Exact plugin filename/class confirmed at Task 3's `gz-ros2-control` package description — verify with `ros2 pkg prefix gz_ros2_control && find $(ros2 pkg prefix gz_ros2_control) -iname "*.so"` and correct the filename here if it differs.)

- [ ] **Step 3: Write the ros2_control YAML**

Base structure (a `JointTrajectoryController` or CHAMP-compatible controller — check what `champ_base` actually publishes to (joint names + message type) from Step 2's grep, and match the controller's command interface to it):

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    champ_controller:
      type: joint_trajectory_controller/JointTrajectoryController

champ_controller:
  ros__parameters:
    joints:
      - lf_hip_joint
      - lf_thigh_joint
      - lf_knee_joint
      # ... one line per joint name found in Step 2's grep, all 12
    command_interfaces: [position]
    state_interfaces: [position, velocity]
```

Replace the joint list with the exact names found in Step 2 — do not leave the four abbreviated `lf_*` lines as the only ones; all 12 joints (4 legs × 3 joints) must be listed.

- [ ] **Step 4: Verify the xacro parses**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
xacro src/cavex_slam_nav/urdf/cavex_walker.urdf.xacro > /tmp/cavex_walker.urdf
echo "xacro exit code: $?"
grep -c "<joint" /tmp/cavex_walker.urdf
```

Expected: exit code 0, and joint count matches 12 legs joints + the fixed sensor-mount joints (camera_joint, lidar_joint, imu if it has one, base_joint) — cross-check against Step 2's actual xacro content.

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/urdf/cavex_walker.urdf.xacro ros2_ws/src/cavex_slam_nav/config/
git commit -m "Add Spot-like walker URDF (CHAMP + gz_ros2_control + 3D lidar/camera/IMU)"
```

---

### Task 5: Launch file to spawn the walker and bridge its sensors

**Files:**
- Create: `ros2_ws/src/cavex_slam_nav/launch/gazebo_walker.launch.py`

**Interfaces:**
- Consumes: `cavex_walker.urdf.xacro` (Task 4).
- Produces: `/lidar/points` (`sensor_msgs/msg/PointCloud2`), `/camera/color/image_raw`, `/camera/color/camera_info`, `/imu`, `/model/cavex_walker/pose` (`geometry_msgs/msg/PoseArray`, the model's true world pose — see Step 3; Task 10 republishes this as the `Odometry`-shaped `/odom_ground_truth` that `ate_evaluator_node.py` actually expects), `/clock`, `/cmd_vel` sink into `champ_base`.

- [ ] **Step 1: Copy `gazebo_sim.launch.py`'s structure**

Read `ros2_ws/src/cavex_slam_nav/launch/gazebo_sim.launch.py` in full first (note: this file currently has local uncommitted edits re-enabling the GUI — read the committed version via `git show HEAD:ros2_ws/src/cavex_slam_nav/launch/gazebo_sim.launch.py` if the working tree version looks different from what this plan expects). Copy its `gz_sim` `IncludeLaunchDescription` block, `robot_state_publisher` node, and `create` spawn node verbatim, changing:
- `urdf_file` to point at `cavex_walker.urdf.xacro`.
- The spawn `-name` to `cavex_walker`.
- Spawn position to somewhere clear in the dry-cave section, e.g. `-x -30 -y 0 -z 0.6` (above ground, matching the walker's standing height from Task 4 Step 1's dimensions).

- [ ] **Step 2: Bridge the new topics**

Extend the `gz_bridge` `parameter_bridge` arguments list (copy the existing one, add):

```python
'/lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
```

(Camera/IMU/clock bridge lines are identical to the existing ones — copy them as-is, just don't duplicate `/lidar/scan` since this robot has no 2D lidar.)

- [ ] **Step 3: Ground truth odometry**

`cavex_walker` has no `OdometryPublisher` plugin (that was `cavex_robot`'s wheeled-body approach; a legged robot's true pose isn't a simple commanded-velocity integral). Add a `<plugin filename="gz-sim-pose-publisher-system" name="gz::sim::systems::PosePublisher">` block to `cavex_walker.urdf.xacro`'s `<gazebo>` section (publishes the model's true world pose — this is what "ground truth" means for this robot, same honesty standard as `cavex_robot`'s `/odom`: it's simulator-internal true state, not a claim about real-hardware ground truth) and bridge it:

```python
'/model/cavex_walker/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
```

Note in a code comment that this differs from `cavex_robot`'s `nav_msgs/Odometry`-shaped ground truth — `ate_evaluator_node.py` expects `Odometry`, so Task 10 needs a small republisher node (`PoseArray` → `Odometry`, position-only, zero velocity fields) rather than assuming this bridges directly into the existing evaluator unchanged.

- [ ] **Step 4: Verify empirically (do not assume `/lidar/points` or the pose topic names above are exactly right — this session's own established finding is that gz-sim topic names from `<topic>` overrides don't follow the expected scoped convention)**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch cavex_slam_nav gazebo_walker.launch.py &
sleep 15
gz topic -l | grep -iE "lidar|points|pose"
ros2 topic list | grep -iE "lidar|points|pose|joint_state"
```

Compare gz-transport's actual topic names against what the bridge arguments in Step 2/3 expect. If they don't match (likely, per this session's track record), fix the bridge arguments to the real names before proceeding, exactly as was done for `cavex_robot`'s camera/lidar topics earlier this session.

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/launch/gazebo_walker.launch.py ros2_ws/src/cavex_slam_nav/urdf/cavex_walker.urdf.xacro
git commit -m "Add gazebo_walker.launch.py: spawn + bridge the legged walker"
```

---

### Task 6: Add Fuel-sourced obstacles to the dry cave section

**Files:**
- Modify: `ros2_ws/src/cavex_slam_nav/worlds/cavex_world.world`

**Interfaces:**
- Produces: real collidable geometry in the dry-cave section (x between -39 and -5, per the existing `dry_cave` model's documented bounds) for Nav2's costmap and `explore_lite`'s frontiers to react to.

- [ ] **Step 1: Pick real Fuel models**

```bash
gz fuel list --owner GoogleResearch --type model 2>&1 | grep -iE "rock|boulder|stone" | head -5
```

If that returns nothing (Fuel's CLI listing can be slow/rate-limited), use `gz sim` model insertion via the GUI's Fuel browser as a one-time lookup, or fall back to `https://app.gazebosim.org/fuel/models` search for "rock" and copy 2-3 real model URIs from there. Do not fabricate a model URI that hasn't been confirmed to exist.

- [ ] **Step 2: Add `<include>` blocks**

For each of 3-4 chosen models, add to `cavex_world.world` inside the dry-cave region (x between -35 and -10, y between -5 and 5, avoiding the walker's spawn point from Task 5):

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/<owner>/models/<model_name></uri>
  <name>obstacle_1</name>
  <pose>-25 2 0.5 0 0 0</pose>
</include>
```

(Replace `<owner>`/`<model_name>` with Step 1's real findings; give each a distinct `-x` position spread through the dry section and a unique `<name>`.)

- [ ] **Step 3: Verify the world still loads**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
timeout 20 gz sim -s -r install/cavex_slam_nav/share/cavex_slam_nav/worlds/cavex_world.world &
sleep 10
gz model -l 2>&1 || gz topic -l | grep -c "^/world"
```

Expected: no SDF parse errors in the launch output, and the obstacle models appear in the scene graph (`gz sim` GUI, or `gz topic -t /world/cavex_world/scene/info -e -n1` shows their names).

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/worlds/cavex_world.world
git commit -m "Add Fuel-sourced obstacle models to the dry cave section"
```

---

### Task 7: RTAB-Map in 3D lidar mode

**Files:**
- Create: `ros2_ws/src/cavex_slam_nav/launch/walker_slam.launch.py`

**Interfaces:**
- Consumes: `/lidar/points` (Task 5), `/camera/color/image_raw`, `/camera/color/camera_info` (Task 5).
- Produces: `map` → `odom` → `base_footprint` TF chain (same convention as the existing `rtabmap_nav.launch.py`), so `slam_pose_publisher.py` (existing, unmodified) can be reused as-is against this robot too — just check its default `base_frame` parameter (`base_footprint`) matches this robot's link naming from Task 4.

- [ ] **Step 1: Copy `rtabmap_nav.launch.py`'s `rtabmap` node block**

Read the existing `rtabmap` Node block in `ros2_ws/src/cavex_slam_nav/launch/rtabmap_nav.launch.py` first. Change its parameters:

```python
'subscribe_depth': False,
'subscribe_rgb': True,
'subscribe_scan': False,
'subscribe_scan_cloud': True,
'frame_id': 'base_link',
'qos_image': 2,
'qos_camera_info': 2,
'qos_scan_cloud': 2,
'Grid/FromDepth': 'false',
'Grid/3D': 'true',  # real 3D occupancy now that we have a real 3D lidar
'Icp/PointToPlane': 'true',
'Icp/VoxelSize': '0.1',
```

Remappings: replace `('scan', '/lidar/scan')` with `('scan_cloud', '/lidar/points')`.

- [ ] **Step 2: Reuse `slam_pose_publisher.py`, `sic_slam_node.py` stays out**

Add `slam_pose_publisher.py` (existing node, unmodified) to this launch file exactly as it appears in `rtabmap_nav.launch.py`. Per the spec's explicit non-goal, do **not** add `sic_slam_node.py` or `ate_evaluator_node.py` with a `/sic_slam/odometry` estimate topic here — Task 10 handles evaluation separately with the ground-truth-vs-RTAB-Map comparison directly (no current-bias-correction layer, since there's no current in this phase).

- [ ] **Step 3: Verify RTAB-Map actually processes 3D data**

Launch Task 5's `gazebo_walker.launch.py`, then this launch file, then:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /lidar/points --window 20
ros2 topic echo /rtabmap/info --once 2>&1 | head -20
```

Expected: `/lidar/points` publishes at roughly the configured `update_rate` (10Hz from Task 4), and RTAB-Map's log output (not just `/rtabmap/info`) shows increasing `WM=` (working memory) counts over time the way the existing 2D setup did (grep the launch log for `rtabmap (` lines, same pattern as every RTAB-Map verification this session).

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/launch/walker_slam.launch.py
git commit -m "Add walker_slam.launch.py: RTAB-Map in 3D lidar/ICP mode"
```

---

### Task 8: Nav2 bringup with a costmap from the 3D lidar

**Files:**
- Create: `ros2_ws/src/cavex_slam_nav/config/walker_nav2_params.yaml`
- Modify: `ros2_ws/src/cavex_slam_nav/launch/walker_slam.launch.py` (add Nav2 bringup)

**Interfaces:**
- Consumes: `map`/`odom` TF (Task 7), RTAB-Map's occupancy grid topic (`/map`, `nav_msgs/msg/OccupancyGrid` — RTAB-Map publishes this natively when `Grid/3D` mode is on, per its documented behavior; verify the exact topic name at Step 3 rather than assume).
- Produces: `/cmd_vel` (consumed by `champ_base`, Task 1), a `local_costmap`/`global_costmap`, and the `NavigateToPose` action server that `explore_lite` (Task 9) sends goals to.

- [ ] **Step 1: Write a minimal Nav2 params file**

Base it on `nav2_bringup`'s own default `nav2_params.yaml` (find it: `find $(ros2 pkg prefix nav2_bringup) -name "nav2_params.yaml"`), copied into `config/walker_nav2_params.yaml`, with these changes:
- `global_costmap`/`local_costmap`: `robot_base_frame: base_link`, `use_sim_time: true`, obstacle layer subscribed to `/map` (or the real RTAB-Map occupancy topic found at Task 7 Step 3's re-check).
- Remove `amcl` and `map_server` entirely from the params — RTAB-Map is the localization+mapping source, not Nav2's own SLAM/AMCL stack. Nav2 only needs `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, and the costmaps.

- [ ] **Step 2: Add the bringup to the launch file**

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
nav2_bringup_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    ),
    launch_arguments={
        'use_sim_time': 'true',
        'params_file': os.path.join(get_package_share_directory('cavex_slam_nav'), 'config', 'walker_nav2_params.yaml'),
    }.items(),
)
```

(`navigation_launch.py`, not `bringup_launch.py` — the latter also launches `slam_toolbox`/`map_server`, which we don't want since RTAB-Map already owns SLAM.)

- [ ] **Step 3: Verify the costmap is real, not empty**

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /global_costmap/costmap --once 2>&1 | grep -A3 "data:" | head -5
```

Expected: non-all-zero data once the walker has moved and RTAB-Map has built some map (may need to manually drive it a bit via `/cmd_vel` first, same as every other manual verification this session, before Nav2 has anything to build a costmap from).

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/config/walker_nav2_params.yaml ros2_ws/src/cavex_slam_nav/launch/walker_slam.launch.py
git commit -m "Add Nav2 bringup (costmap-only, RTAB-Map owns SLAM) for the walker"
```

---

### Task 9: Wire explore_lite against the Nav2 costmap

**Files:**
- Modify: `ros2_ws/src/cavex_slam_nav/launch/walker_slam.launch.py`

**Interfaces:**
- Consumes: `/global_costmap/costmap` (Task 8), `NavigateToPose` action server (Task 8).
- Produces: `explore/frontiers` (`visualization_msgs/msg/MarkerArray`) and autonomous `/cmd_vel` motion with no human-sent goals.

- [ ] **Step 1: Add the explore_lite node**

```python
Node(
    package='explore_lite',
    executable='explore',
    name='explore_node',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'costmap_topic': 'global_costmap/costmap',
        'costmap_updates_topic': 'global_costmap/costmap_updates',
        'visualize': True,
        'planner_frequency': 0.5,
        'progress_timeout': 30.0,
        'robot_base_frame': 'base_link',
    }],
),
```

(Parameter names taken from `explore_lite`'s documented defaults per this session's research — verify against the actual vendored source: `grep -rn "declare_parameter\|get_parameter" ros2_ws/src/m-explore-ros2/*/src/*.cpp` and correct any that don't match before relying on this list.)

- [ ] **Step 2: Verify autonomous movement**

```bash
# with the full stack (Task 5 + 7 + 8 + this) running, and NO manual cmd_vel sent:
source /opt/ros/jazzy/setup.bash
timeout 30 ros2 topic echo /odom_ground_truth 2>&1 | grep "position" | head -2
sleep 30
timeout 3 ros2 topic echo /odom_ground_truth --once 2>&1 | grep -A3 "position"
```

(`/odom_ground_truth` here refers to Task 10's `PoseArray`→`Odometry` republisher — if Task 10 hasn't been implemented yet when testing this task in isolation, echo the raw `/model/cavex_walker/pose` topic instead.)

Expected: position at t=30s differs meaningfully from position at t=0s, with **no** `/cmd_vel` publishes from us in between — proving `explore_lite` is genuinely driving the robot on its own.

- [ ] **Step 3: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/launch/walker_slam.launch.py
git commit -m "Wire explore_lite frontier exploration into the walker stack"
```

---

### Task 10: Ground-truth Odometry republisher + ATE for the walker

**Files:**
- Create: `ros2_ws/src/cavex_slam_nav/cavex_slam_nav/walker_ground_truth_odom.py`
- Create: `ros2_ws/src/cavex_slam_nav/cavex_slam_nav/run_walker_ate_eval.py`
- Modify: `ros2_ws/src/cavex_slam_nav/CMakeLists.txt` (install both)
- Modify: `ros2_ws/src/cavex_slam_nav/launch/walker_slam.launch.py` (add both as nodes)

**Interfaces:**
- Consumes: `/model/cavex_walker/pose` (`geometry_msgs/msg/PoseArray`, Task 5).
- Produces: `/odom_ground_truth` (`nav_msgs/msg/Odometry`) for `ate_evaluator_node.py`'s `ground_truth_topic` parameter; triggers `/cavex/eval/finish_run` after a fixed sim-time budget instead of a fixed scripted path (per the spec's "fixed time budget, not fixed path" methodology for autonomous runs).

- [ ] **Step 1: Write the PoseArray → Odometry republisher**

```python
#!/usr/bin/env python3
"""
walker_ground_truth_odom.py

cavex_walker has no VelocityControl/OdometryPublisher plugin (it's a legged
robot, not a commanded-velocity free body) -- ground truth instead comes
from gz-sim's PosePublisher system on the model, bridged as PoseArray (see
gazebo_walker.launch.py). This republishes the first pose in that array
(the model's own root link) as a plain Odometry message, matching the shape
ate_evaluator_node.py and every other consumer in this package already
expects (same real, no-noise-model ground truth as cavex_robot's /odom --
not a claim about real-hardware ground truth).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry


class WalkerGroundTruthOdom(Node):
    def __init__(self):
        super().__init__('walker_ground_truth_odom')
        self.pub = self.create_publisher(Odometry, '/odom_ground_truth', 10)
        self.create_subscription(PoseArray, '/model/cavex_walker/pose', self._cb, 10)

    def _cb(self, msg: PoseArray):
        if not msg.poses:
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = msg.poses[0]
        self.pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = WalkerGroundTruthOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Write the fixed-time-budget ATE run script**

Base this on the real, existing `drive_fixed_trajectory.py` (read it first) — same sim-time gating pattern via `get_clock().now()`, but instead of publishing `cmd_vel` phases, it just waits for a fixed sim-time budget (exploration is autonomous, driven by `explore_lite`) then triggers `finish_run`:

```python
#!/usr/bin/env python3
"""
run_walker_ate_eval.py

Fixed sim-time-budget ATE evaluation for autonomous exploration runs.
Unlike drive_fixed_trajectory.py (which commands a specific path), this
node sends no cmd_vel at all -- explore_lite is already driving the robot
autonomously. It only gates finish_run on a fixed sim-time budget, so
repeated runs are comparable by time-budget even though the actual path
taken differs run to run (expected, since exploration is autonomous).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class WalkerAteEvalRunner(Node):
    def __init__(self):
        super().__init__('run_walker_ate_eval')
        self.declare_parameter('num_runs', 10)
        self.declare_parameter('budget_sim_s', 60.0)
        self.finish_pub = self.create_publisher(Empty, '/cavex/eval/finish_run', 10)

    def _wait_for_clock(self):
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        self._wait_for_clock()
        n = self.get_parameter('num_runs').value
        budget = self.get_parameter('budget_sim_s').value
        for i in range(1, n + 1):
            self.get_logger().info(f"=== exploration run {i}/{n}: {budget}s sim-time budget ===")
            end_ns = self.get_clock().now().nanoseconds + int(budget * 1e9)
            while rclpy.ok() and self.get_clock().now().nanoseconds < end_ns:
                rclpy.spin_once(self, timeout_sec=0.2)
            self.finish_pub.publish(Empty())
            self.get_logger().info(f"Run {i}: finish_run sent.")
        self.get_logger().info("All runs complete.")


def main(args=None):
    rclpy.init(args=args)
    node = WalkerAteEvalRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Add `ate_evaluator_node.py` to the walker launch, ground truth topic `/odom_ground_truth`, estimate topic RTAB-Map's pose (via `slam_pose_publisher.py`'s existing `/cavex/slam/odom` output — already in the launch file from Task 7)**

```python
Node(
    package='cavex_slam_nav',
    executable='ate_evaluator_node.py',
    name='ate_evaluator_node',
    output='screen',
    parameters=[{
        'use_sim_time': use_sim_time,
        'ground_truth_topic': '/odom_ground_truth',
        'estimate_topic': '/cavex/slam/odom',
    }],
),
Node(
    package='cavex_slam_nav',
    executable='walker_ground_truth_odom.py',
    name='walker_ground_truth_odom',
    output='screen',
    parameters=[{'use_sim_time': use_sim_time}],
),
```

- [ ] **Step 4: Add both new scripts to CMakeLists.txt's `install(PROGRAMS ...)`**

```cmake
  cavex_slam_nav/walker_ground_truth_odom.py
  cavex_slam_nav/run_walker_ate_eval.py
```

- [ ] **Step 5: `chmod +x` both, rebuild, verify a real (not degenerate) ATE run**

```bash
chmod +x ros2_ws/src/cavex_slam_nav/cavex_slam_nav/walker_ground_truth_odom.py \
          ros2_ws/src/cavex_slam_nav/cavex_slam_nav/run_walker_ate_eval.py
cd ros2_ws && colcon build --symlink-install --packages-select cavex_slam_nav
source install/setup.bash
ros2 run cavex_slam_nav run_walker_ate_eval.py --ros-args -p use_sim_time:=true -p num_runs:=1 -p budget_sim_s:=30.0
cat cavex_ate_runs.csv
```

Expected: one new row, `n_samples` > 0, `ate_rmse_m` a small finite positive number (not `0.0` exactly, per this session's own established finding that an exact-zero result on this world usually signals a degenerate measurement window, not genuine perfection — investigate rather than accept at face value if it comes back exactly `0.0`).

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/cavex_slam_nav/walker_ground_truth_odom.py \
        ros2_ws/src/cavex_slam_nav/cavex_slam_nav/run_walker_ate_eval.py \
        ros2_ws/src/cavex_slam_nav/CMakeLists.txt \
        ros2_ws/src/cavex_slam_nav/launch/walker_slam.launch.py
git commit -m "Add ground-truth odom republisher and fixed-time-budget ATE runner for the walker"
```

---

### Task 11: Extend the live dashboard for the walker

**Files:**
- Modify: `ros2_ws/src/cavex_slam_nav/cavex_slam_nav/web_telemetry_bridge.py`
- Modify: `src/components/SICSlamVisualizer.tsx` (or a new small panel component if this grows too large — judge at implementation time per the existing file's size)

**Interfaces:**
- Consumes: `/odom_ground_truth` (Task 10), `explore/frontiers` (Task 9), `/cavex/eval/ate_rmse` (existing).
- Produces: an extended `/api/telemetry` payload consumed by the same polling pattern already proven this session.

- [ ] **Step 1: Add subscriptions to `web_telemetry_bridge.py`**

Read the file first (it currently subscribes to `/odom`, `/cavex/slam/odom`, `/sic_slam/odometry`, `/cavex/eval/ate_rmse`, `/lidar/scan`). Add, following the exact same pattern as the existing callbacks:

```python
self.create_subscription(Odometry, '/odom_ground_truth', self._walker_gt_cb, best_effort)
self.create_subscription(MarkerArray, '/explore/frontiers', self._frontiers_cb, 10)
```

(Import `MarkerArray` from `visualization_msgs.msg`.) Store frontier count (`len(msg.markers)`) in `self._latest['frontier_count']`, not the full marker array — no need to ship full 3D geometry to the browser for a status readout.

- [ ] **Step 2: Add a "Legged Walker" panel to the dashboard**

Follow `SICSlamVisualizer.tsx`'s established live/demo badge pattern exactly (read its `liveTelemetry` polling `useEffect` first, copy it if adding a new component, or extend the existing state shape if adding to the same component). Display: live position, `frontier_count` ("N unexplored regions remaining" or similar), and the walker's ATE RMSE — labeled "Legged Walker (CHAMP)," never "SIC-SLAM," matching this phase's explicit non-goal.

- [ ] **Step 3: Verify end-to-end with the full stack running**

```bash
curl -s http://localhost:3000/api/telemetry | python3 -m json.tool
```

Expected: `frontier_count` present and changing over consecutive polls while exploration is running.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/cavex_slam_nav/web_telemetry_bridge.py src/components/
git commit -m "Extend live telemetry + dashboard for the legged walker phase"
```

---

### Task 12: Full end-to-end verification and README update

**Files:**
- Modify: `README.md`

**Interfaces:** none (this task only verifies and documents; no new code).

- [ ] **Step 1: Clean full-stack launch**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch cavex_slam_nav gazebo_walker.launch.py &
sleep 15
ros2 launch cavex_slam_nav walker_slam.launch.py &
sleep 15
ros2 topic list | grep -iE "lidar|costmap|frontier|odom_ground_truth"
```

Expected: all of `/lidar/points`, `/global_costmap/costmap`, `/explore/frontiers`, `/odom_ground_truth` present and publishing (`ros2 topic hz` each briefly).

- [ ] **Step 2: 10-run ATE evaluation**

```bash
ros2 run cavex_slam_nav run_walker_ate_eval.py --ros-args -p use_sim_time:=true -p num_runs:=10 -p budget_sim_s:=60.0
python3 ros2_ws/src/cavex_slam_nav/cavex_slam_nav/analyze_ate_runs.py ros2_ws/cavex_ate_runs.csv
```

Expected: 10 rows, sane (non-NaN, non-degenerate-zero, `n_samples` roughly consistent run to run per this session's own sim-time-gating finding) results.

- [ ] **Step 3: Confirm autonomous, obstacle-aware exploration visually/empirically**

Cross-check ground-truth position over the run against the Task 6 obstacle positions — confirm the robot's path never overlaps an obstacle's footprint (a `ros2 bag record /odom_ground_truth` during one run, then a quick Python min-distance-to-obstacle-centers check, is a real, concrete way to verify this rather than eyeballing it).

- [ ] **Step 4: Update README**

Add a "Phase 1: Legged Walker" section to `README.md` (read the existing file first, follow its established style/tone) covering: build/launch commands for `gazebo_walker.launch.py` + `walker_slam.launch.py`, the `run_walker_ate_eval.py` usage, and the same honesty caveats already established for the wheeled robot (ground truth is simulator-internal and noiseless; label this robot "Spot-like quadruped (CHAMP)," never "Spot").

- [ ] **Step 5: Final commit and push**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add README.md
git commit -m "Document Phase 1 legged walker build/launch/eval in README"
git push origin main
```
