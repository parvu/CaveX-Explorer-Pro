# CaveX Tracked BlueBoat-like Vehicle (ArduPilot) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A twin-pontoon, BlueBoat-like tracked vehicle — real ArduPilot Rover SITL as its control law, real Gazebo Harmonic continuous-track physics, retractable track assemblies — autonomously explores the dry-cave section of `CaveX-Explorer-Pro`'s Gazebo world via Nav2 frontier exploration, builds a real 3D map (RTAB-Map), and is evaluated with ATE, with live telemetry on the existing dashboard.

**Architecture:** New ROS2 package `cavex_tracked_vehicle` (parallel to `cavex_slam_nav`, which stays untouched). Vendor and build `ardupilot_gazebo` (the Gazebo-side plugin) and ArduPilot itself (Rover firmware SITL binary) plus its native ROS2/DDS bridge (`ardupilot_sitl`, `Micro-XRCE-DDS-Gen`, `micro-ROS-Agent`) from source — none of this is packaged via apt. Build the vehicle as a twin-pontoon hull with two independently retractable continuous-track assemblies, driven by Gazebo Harmonic's own `gz-sim-tracked-vehicle-system`/`track-controller-system` plugins (confirmed installed, real SDF parameters extracted via `strings` on the installed `.so`s during planning — not guessed). ArduPilot's real velocity-control output (its `AP_DDS` `cmd_vel` topic) drives the tracks through two small adapter nodes. Reuse `cavex_slam_nav`'s RTAB-Map/Nav2/`explore_lite`/ATE/dashboard patterns, ported and re-verified against this vehicle rather than assumed to carry over unmodified.

**Tech Stack:** ROS2 Jazzy, Gazebo Harmonic (gz-sim, `gz-sim-tracked-vehicle-system`, `gz-sim-track-controller-system`), ArduPilot Rover SITL + `ardupilot_gazebo` (`ArduPilotPlugin`), ArduPilot's native `AP_DDS`/`micro-ROS-Agent` ROS2 bridge, RTAB-Map (3D/ICP mode), Nav2, `explore_lite` (`m-explore-ros2` — reused from the abandoned branch's vendoring, still valid), existing `web_telemetry_bridge.py`/React dashboard pattern.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-08-05-cavex-tracked-blueboat-ardupilot-design.md` — every task below implements a section of it.
- This replaces the abandoned `cavex-legged-walker-phase1` branch's approach (CHAMP/ros2_control). That branch is left as-is, not touched, not merged.
- No sonar in this phase (no water present). No `sic_slam_node.py` / "SIC-SLAM" labeling (no current to compensate for).
- No claim of an official BlueBoat Gazebo model — always label the vehicle "BlueBoat-like tracked vehicle" in code comments, launch descriptions, and dashboard UI text. No claim of real marine/floating capability.
- Track retraction is manually/topic-commanded only this phase — no water-detection or automatic trigger logic (there's no water simulation to trigger from).
- Build the ROS2 workspace with `colcon build --symlink-install` from `/home/parvu/CaveX-Explorer-Pro/ros2_ws`, sourcing `/opt/ros/jazzy/setup.bash` first, matching every other package in this workspace. `ardupilot_gazebo` and ArduPilot itself are built separately (not colcon packages in the traditional sense for the SITL binary; `ardupilot_sitl`/`ardupilot_dds_tests`/`micro-ROS-Agent` ARE colcon packages and go through the same workspace build).
- Verification in this codebase has consistently meant real `ros2`/`gz` CLI checks (`topic list`, `topic hz`, `topic echo`), not a pytest suite. Follow that established pattern.
- Process hygiene: this environment has repeatedly shown severe flakiness from duplicate/orphaned Gazebo (and now potentially ArduPilot SITL / micro-ROS-Agent) processes surviving across test launches. Before and after every live test in every task below: `ps aux | grep -iE "gz sim|ardupilot|sim_vehicle|micro_ros_agent|MicroXRCEAgent"` and kill any stragglers with explicit `kill -9 <pid>` (not `pkill -f`, confirmed unreliable in this environment).
- Work happens in a new git worktree, isolated from `main` (created via the `superpowers:using-git-worktrees` skill at execution time — not part of this plan's tasks, a pre-step before Task 1 starts).

---

### Task 1: Vendor and build `ardupilot_gazebo`

**Files:**
- Create: `ardupilot_gazebo/` at the repo root (vendored, git-cloned — not a `ros2_ws/src` colcon package; it's a standalone CMake project loaded into Gazebo via `GZ_SIM_SYSTEM_PLUGIN_PATH`, same pattern ArduPilot's own docs use)

**Interfaces:**
- Produces: `ArduPilotPlugin` (filename `ArduPilotPlugin`, real, confirmed via ArduPilot's own repo/docs), loadable from any SDF world/model file once `GZ_SIM_SYSTEM_PLUGIN_PATH` includes this build directory. Later tasks (Task 4, Task 7) reference this plugin by that exact name.

- [ ] **Step 1: Install real, confirmed dependencies (already present in this environment — verify, don't reinstall blindly)**

```bash
dpkg -l | grep -E "libgz-sim8-dev|rapidjson-dev"
```

Expected: both already installed (`libgz-sim8-dev 8.14.0-1~noble`, `rapidjson-dev 1.1.0+dfsg2-7.2` — confirmed present in this environment during planning). If either is missing, install it:

```bash
sudo apt-get install -y libgz-sim8-dev rapidjson-dev
```

- [ ] **Step 2: Clone and build**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j$(nproc)
```

Expected: build completes, producing a `.so` (find its exact name/path: `find /home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/build -iname "*.so"` — should include something like `libArduPilotPlugin.so`; use the real found name in Step 3, don't assume).

- [ ] **Step 3: Verify the plugin loads in Gazebo**

```bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/models:/home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
cd /home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo
timeout 15 gz sim -s -r worlds/iris_runway.sdf 2>&1 | grep -iE "ArduPilotPlugin|error|fail"
```

(`iris_runway.sdf` is the repo's own bundled example world — used only to confirm the plugin *loads without error*, not to test anything about our vehicle.) Expected: no "plugin not found" or load error; if a specific different bundled world file exists at that path, `find worlds -iname "*.sdf"` first and use a real one.

- [ ] **Step 4: Add the two env vars to a repo-local setup snippet so later tasks don't have to rediscover them**

Create `ros2_ws/ardupilot_gazebo_env.sh`:

```bash
#!/bin/bash
export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=/home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/models:/home/parvu/CaveX-Explorer-Pro/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
```

Every later task that launches Gazebo must `source` this file (in addition to `/opt/ros/jazzy/setup.bash`).

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ardupilot_gazebo ros2_ws/ardupilot_gazebo_env.sh
git commit -m "Vendor and build ardupilot_gazebo (ArduPilotPlugin for Gazebo Harmonic)"
```

(`ardupilot_gazebo/build/` will contain build artifacts — check `.gitignore` first; if it doesn't already ignore CMake build directories, add `ardupilot_gazebo/build/` to `.gitignore` before this commit, matching this repo's existing `ros2_ws/install/`/`ros2_ws/build/`-ignoring convention.)

---

### Task 2: Vendor and build ArduPilot Rover SITL + its ROS2/DDS bridge

**Files:**
- Create: `ardupilot/` at the repo root (vendored, git-cloned — the real ArduPilot firmware source, built for SITL)
- Create: `ros2_ws/src/ardupilot_sitl/`, `ros2_ws/src/ardupilot_dds_tests/` (vendored ROS2 packages, from ArduPilot's own `ardupilot_ros`-adjacent repos — real package names confirmed via ArduPilot's own ROS2-with-SITL documentation during planning)
- Create: `ros2_ws/src/micro_ros_agent/` (vendored, `micro-ROS-Agent` wraps `Micro-XRCE-DDS-Agent`, per ArduPilot's own documentation)

**Interfaces:**
- Produces: a runnable ArduPilot Rover SITL binary, a `ros2 launch ardupilot_sitl sitl_dds_udp.launch.py` pattern (real, documented by ArduPilot) that starts SITL + a DDS-to-ROS2 bridge together, exposing `/ap/...`-namespaced ROS2 topics (Task 3 verifies the real topic names empirically — don't assume the exact list here).

- [ ] **Step 1: Clone ArduPilot itself**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
```

(`--recurse-submodules` is required — ArduPilot vendors its own dependencies as git submodules; a shallow non-recursive clone will fail to build.)

- [ ] **Step 2: Install ArduPilot's own prereqs and build Rover for SITL**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

(This script is long-running and installs a large real dependency set — real, documented ArduPilot setup step, not invented. Expect it to take several minutes.) Then, per this session's own research confirming Rover build support (`ardupilot`'s own build docs: "replace copter with plane, rover"):

```bash
source ~/.profile   # install-prereqs-ubuntu.sh appends PATH changes here
./waf configure --board sitl
./waf rover
```

Expected: `./waf rover` finishes with `'rover' finished successfully`, producing a `build/sitl/bin/ardurover` binary. Verify:

```bash
ls -la build/sitl/bin/ardurover
```

- [ ] **Step 3: Vendor ArduPilot's real ROS2/DDS packages**

Per ArduPilot's own documented ROS2-with-SITL setup page (confirmed real during planning — package names `ardupilot_sitl`, `ardupilot_dds_tests` live inside the main `ardupilot` repo's `Tools/ros2` directory, not a separate repo). Check the real location first:

```bash
find /home/parvu/CaveX-Explorer-Pro/ardupilot -iname "ardupilot_sitl" -o -iname "ardupilot_dds_tests" 2>/dev/null
```

If found under `Tools/ros2/`, symlink (not copy — keeps them in sync with the vendored `ardupilot` checkout) into the workspace:

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws/src
ln -s /home/parvu/CaveX-Explorer-Pro/ardupilot/Tools/ros2/ardupilot_sitl .
ln -s /home/parvu/CaveX-Explorer-Pro/ardupilot/Tools/ros2/ardupilot_dds_tests .
```

If the real location differs from `Tools/ros2/` (this session's research established these packages exist and are used via `colcon build --packages-up-to ardupilot_sitl` from within a workspace that includes the `ardupilot` repo itself, but the exact subdirectory wasn't byte-verified during planning), locate them with the `find` command above and adjust the symlink source paths accordingly — do not skip this verification.

- [ ] **Step 4: Vendor `micro-ROS-Agent`**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git micro_ros_agent
```

(Try the `jazzy` branch first; if it doesn't exist, `git ls-remote --heads https://github.com/micro-ROS/micro-ROS-Agent.git` to list real available branches and pick the closest — this repo tracks ROS2 distros by branch name, confirm the real Jazzy-compatible one rather than assuming `jazzy` exists.)

- [ ] **Step 5: Build the ROS2-side packages**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src/ardupilot_sitl src/ardupilot_dds_tests src/micro_ros_agent --ignore-src -r -y
colcon build --symlink-install --packages-up-to ardupilot_sitl ardupilot_dds_tests micro_ros_agent
```

Expected: all packages build. `ardupilot_sitl`'s build step is documented by ArduPilot to internally invoke the ArduPilot `waf` build too (via a CMake/colcon hook) — if this step re-triggers a full ArduPilot rebuild, that's expected and not a bug, just slow.

- [ ] **Step 6: Verify the real launch file exists and starts**

```bash
source install/setup.bash
ros2 pkg prefix ardupilot_sitl
find $(ros2 pkg prefix ardupilot_sitl) -iname "sitl_dds_udp.launch.py"
```

Expected: found. (Exact launch arguments — e.g. how to select `rover` vs `copter` firmware — verified live in Task 3, not assumed here.)

- [ ] **Step 7: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ardupilot ros2_ws/src/ardupilot_sitl ros2_ws/src/ardupilot_dds_tests ros2_ws/src/micro_ros_agent
git commit -m "Vendor and build ArduPilot Rover SITL + its ROS2/DDS bridge (ardupilot_sitl, micro-ROS-Agent)"
```

(Same as Task 1's note: check `.gitignore` covers `ardupilot/build/` and any other real build-artifact directories before committing — these are large; if the vendored `ardupilot` repo's own `.gitignore` doesn't travel with a plain `git clone` the way a submodule would, add `ardupilot/build/` explicitly to this repo's `.gitignore`.)

---

### Task 3: Minimal spin-up-and-verify — de-risk the ArduPilot↔Gazebo-tracks bridge before building anything on top

**Files:**
- Create: `ros2_ws/src/cavex_tracked_vehicle/` (new package — this task creates its skeleton: `package.xml`, `CMakeLists.txt`, matching `cavex_slam_nav`'s real structure, read it first)
- Create: `ros2_ws/src/cavex_tracked_vehicle/worlds/track_test.world` (throwaway test world, NOT the real `cavex_world.world` — this task's only goal is confirming real topic names, not building the real vehicle)
- Create: `ros2_ws/src/cavex_tracked_vehicle/urdf/track_test_rig.sdf` (a minimal two-track rig: one `body_link` box, two simple track links, no hull detail, no sensors, no retraction — throwaway, not reused by later tasks except for the topic names it reveals)

**Interfaces:**
- Produces: no interfaces later tasks call into directly. This task's deliverable is *empirical knowledge* — the real topic names/types on both sides of the ArduPilot↔track-plugin bridge — written into this task's report/commit message for Task 6 (which builds the real bridge nodes) to consume.

This is the spec's own explicitly-called-out highest-risk item: no existing reference config combines ArduPilot's DDS output with Gazebo's track-controller plugin. Do this in isolation, on a throwaway rig, before Task 4-7 build the real vehicle on top of an assumption that might be wrong.

- [ ] **Step 1: Create the package skeleton**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws/src
ros2 pkg create --build-type ament_cmake cavex_tracked_vehicle --dependencies rclcpp rclpy ros_gz_bridge ros_gz_sim geometry_msgs nav_msgs
```

Read `ros2_ws/src/cavex_slam_nav/package.xml` and `CMakeLists.txt` afterward and align `cavex_tracked_vehicle`'s versions/structure to match (e.g. its `install(PROGRAMS ...)` pattern for Python nodes, needed later).

- [ ] **Step 2: Write the throwaway test rig SDF**

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="track_test_rig">
    <pose>0 0 0.3 0 0 0</pose>
    <link name="body_link">
      <inertial><mass>10</mass><inertia><ixx>0.3</ixx><iyy>0.5</iyy><izz>0.5</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="col"><geometry><box><size>0.6 0.4 0.2</size></box></geometry></collision>
      <visual name="vis"><geometry><box><size>0.6 0.4 0.2</size></box></geometry></visual>
    </link>

    <link name="left_track">
      <pose relative_to="body_link">0 0.25 -0.1 0 0 0</pose>
      <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="col"><geometry><box><size>0.5 0.1 0.1</size></box></geometry></collision>
      <visual name="vis"><geometry><box><size>0.5 0.1 0.1</size></box></geometry></visual>
    </link>
    <joint name="left_track_joint" type="fixed">
      <parent>body_link</parent>
      <child>left_track</child>
    </joint>

    <link name="right_track">
      <pose relative_to="body_link">0 -0.25 -0.1 0 0 0</pose>
      <inertial><mass>1</mass><inertia><ixx>0.01</ixx><iyy>0.01</iyy><izz>0.01</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
      <collision name="col"><geometry><box><size>0.5 0.1 0.1</size></box></geometry></collision>
      <visual name="vis"><geometry><box><size>0.5 0.1 0.1</size></box></geometry></visual>
    </link>
    <joint name="right_track_joint" type="fixed">
      <parent>body_link</parent>
      <child>right_track</child>
    </joint>

    <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
      <link>left_track</link>
      <track_orientation>1 0 0</track_orientation>
      <max_velocity>5.0</max_velocity>
    </plugin>
    <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
      <link>right_track</link>
      <track_orientation>1 0 0</track_orientation>
      <max_velocity>5.0</max_velocity>
    </plugin>

    <plugin filename="gz-sim-tracked-vehicle-system" name="gz::sim::systems::TrackedVehicle">
      <body_link>body_link</body_link>
      <left_track>left_track</left_track>
      <right_track>right_track</right_track>
      <tracks_separation>0.5</tracks_separation>
      <max_velocity>5.0</max_velocity>
      <max_acceleration>3.0</max_acceleration>
    </plugin>
  </model>
</sdf>
```

(This SDF's plugin element names/params — `<link>`, `<track_orientation>`, `<max_velocity>`, `<body_link>`, `<left_track>`, `<right_track>`, `<tracks_separation>`, `<max_acceleration>` — are the real parameter strings extracted via `strings` on the installed `libgz-sim8-track-controller-system.so`/`libgz-sim8-tracked-vehicle-system.so` during planning, not guessed from documentation. Their exact XML *nesting* under `<plugin>` wasn't verified byte-for-byte though — if Gazebo logs an SDF parse warning about any unrecognized element at Step 4, that element's real name/location needs re-deriving, e.g. via `gz sdf --check` or trial and error against the log output, before moving on.)

Wrap it in a minimal throwaway world file `track_test.world` (copy `cavex_slam_nav`'s `cavex_world.world` header — physics/light/ground_plane blocks — and `<include>` this model, or embed it directly).

- [ ] **Step 3: Launch just Gazebo with this rig and find the real TrackedVehicle input topic**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
source ardupilot_gazebo_env.sh
timeout 20 gz sim -s -r install/cavex_tracked_vehicle/share/cavex_tracked_vehicle/worlds/track_test.world &
sleep 8
gz topic -l | grep -iE "cmd_vel|track"
```

Expected: some topic the `TrackedVehicle` plugin subscribes to for velocity commands. This session's own `strings` extraction during planning found the literal strings `/cmd_vel` and `OnCmdVel` inside `libgz-sim8-tracked-vehicle-system.so` — verify here whether the real topic is exactly `/cmd_vel`, a model-scoped variant like `/model/track_test_rig/cmd_vel`, or something else. Publish a manual test command and confirm the rig actually moves:

```bash
gz topic -t /cmd_vel -m gz.msgs.Twist -p 'linear: {x: 1.0}' --num 20 --wait 0.05
```

(Or the real topic name found above, if different.) Check the rig's pose changed:

```bash
gz topic -e -t /model/track_test_rig/pose -n 1
```

Record the exact real topic name/type found here — Task 6 depends on it.

- [ ] **Step 4: Launch ArduPilot Rover SITL and find its real DDS `cmd_vel`-equivalent output topic**

In a separate terminal/background process from Step 3 (both running):

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py &
sleep 10
ros2 topic list | grep -iE "^/ap"
```

(Real launch argument for selecting Rover vs the launch file's default vehicle — check `ros2 launch ardupilot_sitl sitl_dds_udp.launch.py --show-args` first; if it takes a `command:=` or `vehicle:=`-style argument for the SITL binary path/type, point it at `ardurover`/Task 2's built binary rather than assuming a default.) Expected: a real `/ap/...`-namespaced topic list. Confirm the real output-velocity topic (this session's research during planning found the string `twist/filtered` in ArduPilot's own `AP_DDS_Topic_Table.h` as `LOCAL_VELOCITY_PUB`'s topic name, and `cmd_vel` as `VELOCITY_CONTROL_SUB`'s — under whatever real namespace prefix `ros2 topic list` shows, likely `/ap/twist/filtered`), via:

```bash
ros2 topic echo /ap/twist/filtered --once
```

(Substitute the real namespaced name found in the `topic list` output above if this exact guess is wrong.)

- [ ] **Step 5: Clean up and record findings**

```bash
ps aux | grep -iE "gz sim|ardurover|MicroXRCEAgent" | grep -v grep
```

Kill everything found (`kill -9 <pid>` for each). Write the real, empirically-confirmed topic names/types from Steps 3-4 into this task's SDD report — Task 6's bridge nodes are written against these exact real names, not the hypotheses in this plan's text.

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/
git commit -m "Add cavex_tracked_vehicle package skeleton + throwaway track/ArduPilot topic-verification rig"
```

---

### Task 4: Compose the twin-pontoon BlueBoat-like hull with retractable track assemblies and sensors

**Files:**
- Create: `ros2_ws/src/cavex_tracked_vehicle/urdf/cavex_tracked_vehicle.urdf.xacro`

**Interfaces:**
- Produces: a spawnable robot description (`robot_description` topic, same pattern as `cavex_robot.urdf.xacro`), with `left_track_retract_joint`/`right_track_retract_joint` (revolute) and `left_track`/`right_track` links positioned below them, `<gz_frame_id>` set on every sensor (this project's established gotcha).

- [ ] **Step 1: Twin-pontoon hull**

Per Blue Robotics' published BlueBoat spec (confirmed during planning: 120 x 93 x 46 cm deployed, 14.5 kg bare vessel, twin catamaran hulls ~93cm apart joined by a crossmember frame, one motor per hull): two elongated hull boxes plus a crossmember deck box.

```xml
<?xml version="1.0"?>
<!--
  BlueBoat-like twin-pontoon hull, tracked-vehicle variant -- NOT an
  official Blue Robotics simulation model (none exists on Gazebo Fuel or
  GitHub, checked during design). Dimensions from Blue Robotics' own
  published BlueBoat spec (120 x 93 x 46 cm deployed, 14.5 kg bare
  vessel, twin LDPE hulls ~93cm apart joined by a crossmember frame, one
  motor per hull for differential thrust) -- reused here as a chassis:
  two pontoon boxes standing in for the real hulls, a crossmember deck
  box joining them (BlueBoat's real flat payload area, where our sensors
  mount), and one retractable continuous-track assembly per pontoon
  (replacing BlueBoat's real per-hull motor/prop with a track drive,
  matching the real per-side differential-thrust layout).
-->
<robot name="cavex_tracked_vehicle" xmlns:xacro="http://ros.org/wiki/xacro">

  <xacro:property name="pontoon_length" value="1.2"/>
  <xacro:property name="pontoon_width" value="0.2"/>
  <xacro:property name="pontoon_height" value="0.3"/>
  <xacro:property name="pontoon_separation" value="0.93"/> <!-- BlueBoat deployed width spec -->
  <xacro:property name="deck_length" value="0.9"/>
  <xacro:property name="deck_width" value="${pontoon_separation - pontoon_width}"/>
  <xacro:property name="deck_height" value="0.08"/>
  <xacro:property name="deck_z" value="0.15"/> <!-- deck sits above the pontoons -->

  <xacro:property name="track_length" value="0.6"/>
  <xacro:property name="track_width" value="0.12"/>
  <xacro:property name="track_height" value="0.15"/>

  <link name="base_link">
    <inertial>
      <mass value="14.5"/> <!-- BlueBoat bare-vessel spec mass -->
      <inertia ixx="0.6" ixy="0" ixz="0" iyy="1.8" iyz="0" izz="1.8"/>
    </inertial>
  </link>

  <!-- Crossmember deck (payload area / sensor mount, matches BlueBoat's real flat deck between hulls) -->
  <link name="deck_link">
    <visual>
      <geometry><box size="${deck_length} ${deck_width} ${deck_height}"/></geometry>
      <material name="deck_grey"><color rgba="0.5 0.5 0.5 1"/></material>
    </visual>
    <collision>
      <geometry><box size="${deck_length} ${deck_width} ${deck_height}"/></geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.05" ixy="0" ixz="0" iyy="0.1" iyz="0" izz="0.1"/>
    </inertial>
  </link>
  <joint name="deck_joint" type="fixed">
    <parent link="base_link"/>
    <child link="deck_link"/>
    <origin xyz="0 0 ${deck_z}" rpy="0 0 0"/>
  </joint>

  <xacro:macro name="pontoon" params="prefix y_sign">
    <link name="${prefix}_pontoon_link">
      <visual>
        <geometry><box size="${pontoon_length} ${pontoon_width} ${pontoon_height}"/></geometry>
        <material name="hull_yellow"><color rgba="1 0.8 0 1"/></material>
      </visual>
      <collision>
        <geometry><box size="${pontoon_length} ${pontoon_width} ${pontoon_height}"/></geometry>
      </collision>
      <inertial>
        <mass value="5.0"/>
        <inertia ixx="0.1" ixy="0" ixz="0" iyy="0.6" iyz="0" izz="0.6"/>
      </inertial>
    </link>
    <joint name="${prefix}_pontoon_joint" type="fixed">
      <parent link="base_link"/>
      <child link="${prefix}_pontoon_link"/>
      <origin xyz="0 ${y_sign * pontoon_separation/2} 0" rpy="0 0 0"/>
    </joint>

    <!-- Retraction joint: hinges the track assembly up against the pontoon.
         Revolute (not prismatic) -- a fold-up motion, matching how real
         retractable-track/wheel mechanisms on small ground vehicles
         typically work (a hinge, not a telescoping slide). 0 rad =
         deployed (track hangs below the pontoon, touching the ground);
         limit upper bound = retracted (track folded flush against the
         pontoon's side). Real limit angle verified against actual
         geometry at Task 5's control-verification step, not assumed
         precisely here -- 1.4 rad (~80 deg) is a reasonable starting
         point for a fold that clears the ground. -->
    <link name="${prefix}_track_link">
      <visual>
        <geometry><box size="${track_length} ${track_width} ${track_height}"/></geometry>
        <material name="track_black"><color rgba="0.1 0.1 0.1 1"/></material>
      </visual>
      <collision>
        <geometry><box size="${track_length} ${track_width} ${track_height}"/></geometry>
      </collision>
      <inertial>
        <mass value="1.5"/>
        <inertia ixx="0.02" ixy="0" ixz="0" iyy="0.05" iyz="0" izz="0.05"/>
      </inertial>
    </link>
    <joint name="${prefix}_track_retract_joint" type="revolute">
      <parent link="${prefix}_pontoon_link"/>
      <child link="${prefix}_track_link"/>
      <origin xyz="0 0 -${pontoon_height/2 + track_height/2}" rpy="0 0 0"/>
      <axis xyz="1 0 0"/>
      <limit lower="0.0" upper="1.4" effort="50" velocity="1.0"/>
    </joint>
  </xacro:macro>

  <xacro:pontoon prefix="left" y_sign="1"/>
  <xacro:pontoon prefix="right" y_sign="-1"/>

  <!-- Sensors mounted on the deck (matches BlueBoat's real flat payload area) -->
  <link name="lidar_link">
    <visual><geometry><cylinder radius="0.05" length="0.08"/></geometry></visual>
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="deck_link"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 ${deck_height/2 + 0.05}" rpy="0 0 0"/>
  </joint>

  <link name="camera_link"/>
  <joint name="camera_joint" type="fixed">
    <parent link="deck_link"/>
    <child link="camera_link"/>
    <origin xyz="${deck_length/2} 0 0" rpy="0 0 0"/>
  </joint>

  <gazebo reference="lidar_link">
    <sensor type="gpu_lidar" name="lidar_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>false</visualize>
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

  <gazebo reference="camera_link">
    <sensor type="camera" name="camera_sensor">
      <update_rate>30.0</update_rate>
      <topic>camera/color</topic>
      <gz_frame_id>camera_link</gz_frame_id>
      <camera>
        <horizontal_fov>1.3962634</horizontal_fov>
        <image><width>800</width><height>800</height><format>R8G8B8</format></image>
        <clip><near>0.02</near><far>300</far></clip>
      </camera>
    </sensor>
  </gazebo>

  <gazebo reference="base_link">
    <sensor type="imu" name="imu_sensor">
      <update_rate>50</update_rate>
      <topic>imu</topic>
      <gz_frame_id>base_link</gz_frame_id>
    </sensor>
  </gazebo>

</robot>
```

(`<gz_frame_id>` set on every sensor per this project's established gotcha: without it, gz-sim stamps `frame_id` with its own scene-graph path instead of the URDF link name, breaking TF lookups against `robot_state_publisher`'s tree.)

- [ ] **Step 2: Verify the xacro parses**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
xacro src/cavex_tracked_vehicle/urdf/cavex_tracked_vehicle.urdf.xacro > /tmp/cavex_tracked_vehicle.urdf
echo "xacro exit code: $?"
grep -c "<joint" /tmp/cavex_tracked_vehicle.urdf
```

Expected: exit code 0, joint count includes `deck_joint`, `left_pontoon_joint`, `right_pontoon_joint`, `left_track_retract_joint`, `right_track_retract_joint`, `lidar_joint`, `camera_joint` (7 total).

- [ ] **Step 3: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/urdf/cavex_tracked_vehicle.urdf.xacro
git commit -m "Add twin-pontoon BlueBoat-like hull with retractable tracks and sensors"
```

---

### Task 5: Wire the real track-drive and retraction-control plugins to the hull

**Files:**
- Modify: `ros2_ws/src/cavex_tracked_vehicle/urdf/cavex_tracked_vehicle.urdf.xacro`
- Create: `ros2_ws/src/cavex_tracked_vehicle/config/cavex_tracked_vehicle_ros2_control.yaml` (retraction-joint controller config)

**Interfaces:**
- Consumes: Task 3's empirically-confirmed real `TrackedVehicle` input topic name/type.
- Produces: real track-belt physics driven by that topic; `left_track_retract_joint`/`right_track_retract_joint` commandable via `ros2_control`'s standard `JointTrajectoryController` (same real, proven pattern this project already used successfully once the correct `gz_ros2_control` gain parameter was found — see the abandoned branch's Task 7 fix history for the exact real parameter name/location, reused here directly rather than rediscovered).

- [ ] **Step 1: Add the `TrackController`/`TrackedVehicle` plugin blocks to the real hull, using Task 3's verified parameter names**

```xml
<gazebo>
  <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
    <link>left_track</link>
    <track_orientation>1 0 0</track_orientation>
    <max_velocity>2.0</max_velocity>
    <max_acceleration>2.0</max_acceleration>
  </plugin>
  <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
    <link>right_track</link>
    <track_orientation>1 0 0</track_orientation>
    <max_velocity>2.0</max_velocity>
    <max_acceleration>2.0</max_acceleration>
  </plugin>
  <plugin filename="gz-sim-tracked-vehicle-system" name="gz::sim::systems::TrackedVehicle">
    <body_link>base_link</body_link>
    <left_track>left_track</left_track>
    <right_track>right_track</right_track>
    <tracks_separation>${pontoon_separation}</tracks_separation>
    <max_velocity>2.0</max_velocity>
    <max_acceleration>2.0</max_acceleration>
  </plugin>
</gazebo>
```

(Same real parameter names verified in Task 3's throwaway rig — `max_velocity`/`max_acceleration` values here are starting points, not final tuning; Task 7's real-motion verification step is where these actually get checked against real behavior, same as every other numeric parameter in this project's history.)

- [ ] **Step 2: Add `ros2_control` for the retraction joints**

The retraction joints need real actuation, same `<ros2_control>` + `gz_ros2_control::GazeboSimROS2ControlPlugin` pattern already proven working in the abandoned branch (once the real `position_proportional_gain` SDF-plugin-level parameter was found — reuse that exact fix here, don't rediscover it from scratch):

```xml
<ros2_control name="GazeboSimSystem" type="system">
  <hardware>
    <plugin>gz_ros2_control/GazeboSimSystem</plugin>
  </hardware>
  <joint name="left_track_retract_joint">
    <command_interface name="position">
      <param name="min">0.0</param>
      <param name="max">1.4</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
  <joint name="right_track_retract_joint">
    <command_interface name="position">
      <param name="min">0.0</param>
      <param name="max">1.4</param>
    </command_interface>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>

<gazebo>
  <plugin filename="gz_ros2_control-system" name="gz_ros2_control::GazeboSimROS2ControlPlugin">
    <parameters>$(find cavex_tracked_vehicle)/config/cavex_tracked_vehicle_ros2_control.yaml</parameters>
    <!-- position_proportional_gain is a GazeboSimSystem-wide SDF <plugin>
         child element, NOT a per-joint <ros2_control>/<param> -- confirmed
         the hard way on the abandoned legged-walker branch (its Task 7
         fix history) by reading gz_ros2_control's real upstream source.
         Reused directly here rather than rediscovering it. -->
    <position_proportional_gain>20.0</position_proportional_gain>
  </plugin>
</gazebo>
```

Write `cavex_tracked_vehicle_ros2_control.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    track_retract_controller:
      type: joint_trajectory_controller/JointTrajectoryController

track_retract_controller:
  ros__parameters:
    joints:
      - left_track_retract_joint
      - right_track_retract_joint
    command_interfaces:
      - position
    state_interfaces:
      - position
      - velocity
    interpolation_method: "none" # continuous streaming commands, not discrete waypoints -- same real gotcha and fix as the abandoned branch's Task 7
```

- [ ] **Step 3: Write a small retraction-command node**

```python
#!/usr/bin/env python3
"""
track_retract_control.py

Commands both track_retract_joints together via track_retract_controller's
JointTrajectory topic, given a simple "deployed"/"retracted" string command.
No automatic trigger logic (e.g. water detection) -- manual/topic-commanded
only this phase, per the design spec's explicit non-goal.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

DEPLOYED = 0.0
RETRACTED = 1.4


class TrackRetractControl(Node):
    def __init__(self):
        super().__init__('track_retract_control')
        self.pub = self.create_publisher(
            JointTrajectory, '/track_retract_controller/joint_trajectory', 10)
        self.create_subscription(String, '/cavex/tracks/command', self._cb, 10)

    def _cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'deployed':
            target = DEPLOYED
        elif cmd == 'retracted':
            target = RETRACTED
        else:
            self.get_logger().warn(f"Unknown track command: {msg.data!r} (expected 'deployed' or 'retracted')")
            return
        traj = JointTrajectory()
        traj.joint_names = ['left_track_retract_joint', 'right_track_retract_joint']
        point = JointTrajectoryPoint()
        point.positions = [target, target]
        point.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [point]
        self.pub.publish(traj)
        self.get_logger().info(f"Tracks commanded {cmd} (joint target {target} rad).")


def main(args=None):
    rclpy.init(args=args)
    node = TrackRetractControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Add it to `CMakeLists.txt`'s `install(PROGRAMS ...)` list (read `cavex_slam_nav/CMakeLists.txt` first for the exact real pattern), `chmod +x` it.

- [ ] **Step 4: Verify both mechanisms independently, in isolation, before wiring ArduPilot in**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select cavex_tracked_vehicle
source install/setup.bash && source ardupilot_gazebo_env.sh
# launch the vehicle standalone (spawn-only, no ArduPilot yet -- write a
# throwaway launch invocation here or reuse Task 3's world with this real
# model swapped in; either is fine, this step doesn't need to survive)
```

Manually drive via the real topic Task 3 found (e.g. `gz topic -t /cmd_vel ...` or the ROS2-bridged equivalent) and confirm real translation. Then command retraction:

```bash
ros2 topic pub --once /cavex/tracks/command std_msgs/msg/String "{data: 'retracted'}"
sleep 3
ros2 topic echo /joint_states --once | grep -A2 "track_retract"
```

Expected: `left_track_retract_joint`/`right_track_retract_joint` positions move toward `1.4`. Repeat with `'deployed'` and confirm they return toward `0.0`. Per the spec's explicit testing requirement: also decide and document here (in this task's report) whether the vehicle can still drive while retracted, or is deliberately prevented from doing so — don't leave this ambiguous.

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/urdf/cavex_tracked_vehicle.urdf.xacro \
        ros2_ws/src/cavex_tracked_vehicle/config/cavex_tracked_vehicle_ros2_control.yaml \
        ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/track_retract_control.py \
        ros2_ws/src/cavex_tracked_vehicle/CMakeLists.txt
git commit -m "Wire real track-drive and retraction-control plugins to the hull"
```

---

### Task 6: `cmd_vel_to_ardupilot` and `track_cmd_vel_bridge` adapter nodes

**Files:**
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_ardupilot.py`
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/track_cmd_vel_bridge.py`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/CMakeLists.txt`

**Interfaces:**
- Consumes: project-standard `/cmd_vel` (`geometry_msgs/msg/Twist`, from Nav2/`explore_lite`, wired in later tasks); ArduPilot's real DDS output topic (Task 3's finding).
- Produces: ArduPilot's real `/ap/cmd_vel` input topic (Task 3's finding, `TwistStamped`); the real `TrackedVehicle` input topic (Task 3's finding).

- [ ] **Step 1: `cmd_vel_to_ardupilot.py`**

```python
#!/usr/bin/env python3
"""
cmd_vel_to_ardupilot.py

Relays the project's standard /cmd_vel (geometry_msgs/Twist, from Nav2 /
explore_lite) into ArduPilot's real AP_DDS cmd_vel input (geometry_msgs/
TwistStamped, topic name confirmed empirically in Task 3 -- this file
assumes /ap/cmd_vel, correct it here if Task 3 found a different real
name). Also arms the vehicle and sets GUIDED mode on the first /cmd_vel
received, via ArduPilot's real service interface (exact service names
verified empirically here, not guessed -- see Step 2 below for how to
find them if this file's assumed names are wrong).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch


class CmdVelToArduPilot(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ardupilot')
        self.pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self._armed = False
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')

    def _ensure_armed_and_guided(self):
        if self._armed:
            return
        if self.mode_client.wait_for_service(timeout_sec=1.0):
            req = ModeSwitch.Request()
            req.mode = 15  # Rover GUIDED mode number -- verify against ArduPilot's real
                            # Rover mode enum (Tools/autotest/pysim/rover.py or
                            # AP_Rover's mode.h ROVER_MODE_GUIDED) before relying
                            # on this; not independently confirmed during planning.
            self.mode_client.call_async(req)
        if self.arm_client.wait_for_service(timeout_sec=1.0):
            req = ArmMotors.Request()
            req.arm = True
            self.arm_client.call_async(req)
        self._armed = True

    def _cb(self, msg: Twist):
        self._ensure_armed_and_guided()
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToArduPilot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: Verify the real `ardupilot_msgs` service names/types before relying on this file as-is**

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py &
sleep 10
ros2 service list | grep -iE "arm|mode"
ros2 interface show ardupilot_msgs/srv/ArmMotors 2>&1
ros2 interface show ardupilot_msgs/srv/ModeSwitch 2>&1
```

If any real name/field differs from what Step 1's code assumes (service names `/ap/arm_motors`/`/ap/mode_switch`, or the `ModeSwitch.Request`'s field name/GUIDED-mode integer value), fix the code in Step 1 to match what's real before proceeding — don't leave a guessed value uncorrected once the real one is known.

- [ ] **Step 3: `track_cmd_vel_bridge.py`**

```python
#!/usr/bin/env python3
"""
track_cmd_vel_bridge.py

Relays ArduPilot's real DDS control-law output (its own computed
velocity command, topic name confirmed empirically in Task 3 -- this
file assumes /ap/twist/filtered, correct it here if Task 3 found a
different real name) into the real TrackedVehicle plugin's input topic
(also confirmed in Task 3 -- this file assumes /cmd_vel maps through
ros_gz_bridge to the gz-transport topic the TrackedVehicle system
subscribes to; if Task 3 found a different real gz-transport topic
name, this node's target changes to a ros_gz_bridge remap instead of a
plain republish, and this file may not be needed as a separate ROS2
node at all -- see Task 3's report for which shape is actually correct).
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist


class TrackCmdVelBridge(Node):
    def __init__(self):
        super().__init__('track_cmd_vel_bridge')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(TwistStamped, '/ap/twist/filtered', self._cb, 10)

    def _cb(self, msg: TwistStamped):
        self.pub.publish(msg.twist)


def main(args=None):
    rclpy.init(args=args)
    node = TrackCmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

(This publishes to the project's own `/cmd_vel` topic name, deliberately reusing it rather than inventing `/track_cmd_vel` — the design spec's original architecture sketch used a separate `/track_cmd_vel` name, but Task 3's `strings` findings suggest `TrackedVehicle` may default to `/cmd_vel` directly, which is simpler and requires no `ros_gz_bridge` remap beyond what's already needed. If Task 3 found the real topic to be genuinely different from `/cmd_vel`, change this publisher's topic name to match — don't silently keep `/cmd_vel` if it's wrong.)

- [ ] **Step 4: Add both to `CMakeLists.txt`, `chmod +x`, build**

```bash
chmod +x ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_ardupilot.py \
          ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/track_cmd_vel_bridge.py
cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select cavex_tracked_vehicle
```

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_ardupilot.py \
        ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/track_cmd_vel_bridge.py \
        ros2_ws/src/cavex_tracked_vehicle/CMakeLists.txt
git commit -m "Add cmd_vel<->ArduPilot<->track adapter nodes"
```

---

### Task 7: Full launch file — spawn the real vehicle, wire everything, verify real motion (go/no-go checkpoint)

**Files:**
- Create: `ros2_ws/src/cavex_tracked_vehicle/launch/gazebo_tracked_vehicle.launch.py`

**Interfaces:**
- Consumes: Task 4's URDF, Task 5's plugins, Task 6's adapter nodes, Task 1-2's ArduPilot/Gazebo binaries.
- Produces: `/lidar/points`, `/camera/color/image_raw`, `/camera/color/camera_info`, `/imu`, `/model/cavex_tracked_vehicle/pose` (ground truth, same `PoseArray` pattern as the abandoned branch), and — the actual deliverable of this task — a vehicle that demonstrably translates in Gazebo under real ArduPilot control.

- [ ] **Step 1: Compose the launch file**

Structure copied from `cavex_slam_nav/launch/gazebo_walker.launch.py` (read it first): `gz_sim` `IncludeLaunchDescription` (this time also sourcing `ardupilot_gazebo_env.sh`'s env vars — either export them before `ros2 launch` runs, or set them as `SetEnvironmentVariable` launch actions inside this file, matching how the two env vars need to reach the `gz sim` subprocess), `robot_state_publisher`, `spawn_entity` (Task 4's URDF, spawned at a real clear point in the dry-cave section, e.g. `-x -30 -y 0 -z 0.3`), `gz_bridge` (lidar/camera/imu/pose, same bridge pattern as the abandoned branch's `gazebo_walker.launch.py`, real topic names re-verified rather than assumed), plus:

```python
ardupilot_sitl_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('ardupilot_sitl'), 'launch', 'sitl_dds_udp.launch.py')
    ),
    # Real launch args verified in Task 3 Step 4 (--show-args) -- fill in
    # whatever real argument selects the Rover/ardurover binary here.
)

cmd_vel_to_ardupilot = Node(
    package='cavex_tracked_vehicle',
    executable='cmd_vel_to_ardupilot.py',
    name='cmd_vel_to_ardupilot',
    output='screen',
    parameters=[{'use_sim_time': True}],
)

track_cmd_vel_bridge = Node(
    package='cavex_tracked_vehicle',
    executable='track_cmd_vel_bridge.py',
    name='track_cmd_vel_bridge',
    output='screen',
    parameters=[{'use_sim_time': True}],
)

track_retract_control = Node(
    package='cavex_tracked_vehicle',
    executable='track_retract_control.py',
    name='track_retract_control',
    output='screen',
    parameters=[{'use_sim_time': True}],
)
```

Plus the `controller_manager` spawners for `joint_state_broadcaster`/`track_retract_controller` (same `OnProcessExit`-sequenced pattern the abandoned branch's `gazebo_walker.launch.py` used, real and proven — copy it directly).

- [ ] **Step 2: Verify empirically — real topic names (this project's established pattern; gz-sim topic names from `<topic>` overrides have repeatedly not matched the naive expected convention)**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash && source ardupilot_gazebo_env.sh
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py &
sleep 20
ps aux | grep -iE "gz sim|ardurover|MicroXRCEAgent" | grep -v grep
ros2 topic list | grep -iE "lidar|camera|imu|pose|ap/|cmd_vel"
```

Fix any bridge/topic-name mismatch found here before Step 3, same as every previous vehicle-integration task in this project's history.

- [ ] **Step 3: The real go/no-go checkpoint — drive the vehicle and confirm real net translation**

```bash
source /opt/ros/jazzy/setup.bash
echo "--- pose before ---"
gz topic -e -t /model/cavex_tracked_vehicle/pose -n 1 2>&1 | grep -A4 'name: "cavex_tracked_vehicle"'
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}" &
PUBPID=$!
sleep 15
kill -9 $PUBPID
echo "--- pose after ---"
gz topic -e -t /model/cavex_tracked_vehicle/pose -n 1 2>&1 | grep -A4 'name: "cavex_tracked_vehicle"'
```

Expected: real, meaningful position delta (this is the exact failure mode — near-zero net translation despite valid-looking commands flowing — that blocked the abandoned legged-walker branch for most of a session; do not declare this task done on partial/ambiguous evidence, get a real, unambiguous delta here before proceeding to any task after this one).

- [ ] **Step 4: Clean up, commit**

```bash
ps aux | grep -iE "gz sim|ardurover|MicroXRCEAgent" | grep -v grep
# kill -9 every PID found
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/launch/gazebo_tracked_vehicle.launch.py
git commit -m "Add full launch file: spawn + bridge the tracked vehicle, real ArduPilot-driven motion verified"
```

---

### Task 8: Add Fuel-sourced obstacles to the dry cave section

**Files:**
- Modify: `ros2_ws/src/cavex_slam_nav/worlds/cavex_world.world` (shared world file — the abandoned branch's Task 6 obstacles never reached `main`; this world file currently has zero `<include>` blocks, confirmed during planning)

**Interfaces:**
- Produces: real collidable geometry in the dry-cave section (x between -39 and -5, per the existing `dry_cave` model's documented bounds) for Nav2's costmap and `explore_lite`'s frontiers to react to.

- [ ] **Step 1: Pick real Fuel models**

```bash
gz fuel list --owner GoogleResearch --type model 2>&1 | grep -iE "rock|boulder|stone" | head -5
```

If empty/rate-limited, use `https://app.gazebosim.org/fuel/models` search for "rock" and copy 2-3 real model URIs from there directly. Do not fabricate a model URI that hasn't been confirmed to exist.

- [ ] **Step 2: Add `<include>` blocks**

```xml
<include>
  <uri>https://fuel.gazebosim.org/1.0/<owner>/models/<model_name></uri>
  <name>obstacle_1</name>
  <pose>-25 2 0.5 0 0 0</pose>
</include>
```

3-4 of these, real owner/model names from Step 1, spread through the dry section (x between -35 and -10, y between -5 and 5), avoiding the vehicle's Task 7 spawn point.

- [ ] **Step 3: Verify the world still loads**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
timeout 20 gz sim -s -r install/cavex_slam_nav/share/cavex_slam_nav/worlds/cavex_world.world &
sleep 10
gz topic -t /world/cavex_world/scene/info -e -n1 2>&1 | grep -iE "obstacle_1|obstacle_2"
```

Expected: no SDF parse errors, obstacle names appear in the scene graph.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/worlds/cavex_world.world
git commit -m "Add Fuel-sourced obstacle models to the dry cave section"
```

---

### Task 9: RTAB-Map in 3D lidar mode

**Files:**
- Create: `ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py`

**Interfaces:**
- Consumes: `/lidar/points`, `/camera/color/image_raw`, `/camera/color/camera_info` (Task 7).
- Produces: `map` → `odom` → `base_link` TF chain.

- [ ] **Step 1: Port the `rtabmap`/`icp_odometry` node blocks from the abandoned branch's approach**

The abandoned `cavex-legged-walker-phase1` branch's `walker_slam.launch.py` (real, working config, reachable via `git show cavex-legged-walker-phase1:ros2_ws/src/cavex_slam_nav/launch/walker_slam.launch.py` from any worktree with that branch fetched) already solved: `icp_odometry` for frame-to-frame odometry (no wheel/track odometry source feeding RTAB-Map directly, same situation here), 3D `Grid/3D`/`Icp/PointToPlane`/`Icp/VoxelSize` params, and the `frame_id='base_footprint'` vs `'base_link'` TF-tree gotcha. This vehicle's URDF (Task 4) uses `base_link` as its root with no separate `base_footprint` — check whether that TF-tree gotcha applies here too (it won't if there's no static `base_footprint`→`base_link` joint in this URDF) before blindly copying `frame_id='base_footprint'` — use `'base_link'` if that's this vehicle's real root frame, verify via `ros2 run tf2_tools view_frames` after Task 7's launch is running.

Port the `icp_odometry` and `rtabmap` node blocks, remapping `scan_cloud` to `/lidar/points`, `rgb/image`/`rgb/camera_info` to this vehicle's real camera topics (Task 7).

- [ ] **Step 2: Reuse `slam_pose_publisher.py`**

Add it (existing node from `cavex_slam_nav`, unmodified — cross-package reuse, add `cavex_slam_nav` as a `<depend>` in `cavex_tracked_vehicle`'s `package.xml` if not already implied by the workspace, or just reference its installed executable by package name in the `Node()` block, same as any other installed package) with a `base_frame` param matching Step 1's real frame_id finding.

- [ ] **Step 3: Verify RTAB-Map processes real 3D data**

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic hz /lidar/points --window 20
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py 2>&1 | grep "rtabmap (" &
sleep 30
# drive the vehicle a bit via /cmd_vel per Task 7's verified command, if not already moving
```

Expected: `rtabmap (N):` lines with `WM=` increasing over time, once the vehicle has real motion (Task 7's checkpoint) feeding it.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py
git commit -m "Add tracked_vehicle_slam.launch.py: RTAB-Map in 3D lidar/ICP mode"
```

---

### Task 10: Nav2 bringup with a costmap from the 3D lidar

**Files:**
- Create: `ros2_ws/src/cavex_tracked_vehicle/config/tracked_vehicle_nav2_params.yaml`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py`

**Interfaces:**
- Consumes: `map`/`odom` TF (Task 9), RTAB-Map's `/map` occupancy grid.
- Produces: `/cmd_vel`, costmaps, `NavigateToPose` action server.

- [ ] **Step 1: Port the abandoned branch's Nav2 params approach**

The abandoned branch already worked out and fixed a real set of gotchas here (reachable via `git show cavex-legged-walker-phase1:ros2_ws/src/cavex_slam_nav/config/walker_nav2_params.yaml`): no `amcl`/`map_server` (RTAB-Map owns SLAM), a `static_layer` on `/map` instead of a LaserScan-based `obstacle_layer` (this vehicle also has no `/scan`, only 3D lidar — same situation), and `navigation_launch.py`'s lifecycle manager hard-coding `collision_monitor`/`docking_server` as managed nodes even when unused (that branch's fix: minimal functionally-inert param blocks for both, stock values, not fabricated — copy that fix's *final*, reviewed version, i.e. after its own fix-round correction of a `cmd_vel_in_topic` mislabeling, not the version before that correction). `robot_base_frame` should be `base_link` (or whatever Task 9 established as this vehicle's real root frame).

- [ ] **Step 2: Add the bringup to the launch file**

```python
nav2_bringup_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
    ),
    launch_arguments={
        'use_sim_time': 'true',
        'params_file': os.path.join(get_package_share_directory('cavex_tracked_vehicle'), 'config', 'tracked_vehicle_nav2_params.yaml'),
    }.items(),
)
```

- [ ] **Step 3: Verify the costmap is real, not empty**

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /global_costmap/costmap --once 2>&1 | grep -A3 "data:" | head -5
```

Expected: non-all-zero data once the vehicle has moved and RTAB-Map has built some map.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/config/tracked_vehicle_nav2_params.yaml \
        ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py
git commit -m "Add Nav2 bringup (costmap-only, RTAB-Map owns SLAM) for the tracked vehicle"
```

---

### Task 11: Wire explore_lite against the Nav2 costmap

**Files:**
- Modify: `ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py`

**Interfaces:**
- Consumes: `/global_costmap/costmap`, `NavigateToPose` (Task 10).
- Produces: `explore/frontiers`, autonomous `/cmd_vel` motion.

- [ ] **Step 1: Vendor `m-explore-ros2` (if not already available from the abandoned branch's checkout — it's a separate git clone, independent of any CHAMP-specific code, safe to reuse verbatim)**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws/src
git clone --depth 1 https://github.com/robo-friends/m-explore-ros2.git
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to explore_lite
```

- [ ] **Step 2: Add the explore_lite node**

```python
Node(
    package='explore_lite',
    executable='explore',
    name='explore_node',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'costmap_topic': 'global_costmap/costmap',
        'costmap_updates_topic': 'global_costmap/costmap_updates',
        'visualize': True,
        'planner_frequency': 0.5,
        'progress_timeout': 30.0,
        'robot_base_frame': 'base_link',
    }],
),
```

(Verify these param names against the vendored source: `grep -rn "declare_parameter\|get_parameter" ros2_ws/src/m-explore-ros2/*/src/*.cpp`, same as the abandoned branch's Task 9 did.)

- [ ] **Step 3: Verify autonomous movement**

```bash
source /opt/ros/jazzy/setup.bash
echo "--- pose t=0 ---"
gz topic -e -t /model/cavex_tracked_vehicle/pose -n 1 2>&1 | grep -A4 'name: "cavex_tracked_vehicle"'
sleep 30
echo "--- pose t=30 ---"
gz topic -e -t /model/cavex_tracked_vehicle/pose -n 1 2>&1 | grep -A4 'name: "cavex_tracked_vehicle"'
```

With no manual `/cmd_vel` published in between. Expected: real position delta, proving `explore_lite` (via ArduPilot, via the real track drive) is genuinely autonomous.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/m-explore-ros2 ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py
git commit -m "Wire explore_lite frontier exploration into the tracked vehicle stack"
```

---

### Task 12: Ground-truth Odometry republisher + ATE for the tracked vehicle

**Files:**
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/tracked_vehicle_ground_truth_odom.py`
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/run_tracked_vehicle_ate_eval.py`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/CMakeLists.txt`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py`

**Interfaces:**
- Consumes: `/model/cavex_tracked_vehicle/pose` (`PoseArray`, Task 7).
- Produces: `/odom_ground_truth` (`nav_msgs/msg/Odometry`) for `ate_evaluator_node.py`.

- [ ] **Step 1: `tracked_vehicle_ground_truth_odom.py`**

Same real, proven pattern as the abandoned branch's `walker_ground_truth_odom.py` (`PoseArray` → `Odometry`, first pose in the array, no velocity fields):

```python
#!/usr/bin/env python3
"""
tracked_vehicle_ground_truth_odom.py

cavex_tracked_vehicle has no VelocityControl/OdometryPublisher plugin --
ground truth comes from gz-sim's PosePublisher system on the model,
bridged as PoseArray (see gazebo_tracked_vehicle.launch.py). Republishes
the first pose as a plain Odometry message for ate_evaluator_node.py.
Same real, no-noise-model ground truth as the wheeled robot's /odom --
not a claim about real-hardware ground truth.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry


class TrackedVehicleGroundTruthOdom(Node):
    def __init__(self):
        super().__init__('tracked_vehicle_ground_truth_odom')
        self.pub = self.create_publisher(Odometry, '/odom_ground_truth', 10)
        self.create_subscription(PoseArray, '/model/cavex_tracked_vehicle/pose', self._cb, 10)

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
    node = TrackedVehicleGroundTruthOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 2: `run_tracked_vehicle_ate_eval.py`**

Same real, proven fixed-sim-time-budget pattern as the abandoned branch's `run_walker_ate_eval.py`:

```python
#!/usr/bin/env python3
"""
run_tracked_vehicle_ate_eval.py

Fixed sim-time-budget ATE evaluation for autonomous exploration runs.
No cmd_vel sent -- explore_lite (via ArduPilot) is already driving the
vehicle. Gates finish_run on a fixed sim-time budget so repeated runs
are comparable by time-budget, even though the actual path differs run
to run (expected, exploration is autonomous).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class TrackedVehicleAteEvalRunner(Node):
    def __init__(self):
        super().__init__('run_tracked_vehicle_ate_eval')
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
    node = TrackedVehicleAteEvalRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Add `ate_evaluator_node.py` (from `cavex_slam_nav`, cross-package reuse) to the launch file**

```python
Node(
    package='cavex_slam_nav',
    executable='ate_evaluator_node.py',
    name='ate_evaluator_node',
    output='screen',
    parameters=[{
        'use_sim_time': True,
        'ground_truth_topic': '/odom_ground_truth',
        'estimate_topic': '/cavex/slam/odom',
    }],
),
Node(
    package='cavex_tracked_vehicle',
    executable='tracked_vehicle_ground_truth_odom.py',
    name='tracked_vehicle_ground_truth_odom',
    output='screen',
    parameters=[{'use_sim_time': True}],
),
```

- [ ] **Step 4: Add both new scripts to `CMakeLists.txt`, `chmod +x`, rebuild, verify a real ATE run**

```bash
chmod +x ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/tracked_vehicle_ground_truth_odom.py \
          ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/run_tracked_vehicle_ate_eval.py
cd ros2_ws && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --packages-select cavex_tracked_vehicle
source install/setup.bash
ros2 run cavex_tracked_vehicle run_tracked_vehicle_ate_eval.py --ros-args -p use_sim_time:=true -p num_runs:=1 -p budget_sim_s:=30.0
cat cavex_ate_runs.csv
```

Expected: one new row, `n_samples` > 0, `ate_rmse_m` a small finite positive number, not exactly `0.0` (per this project's own established finding that exact-zero usually signals a degenerate measurement window).

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/tracked_vehicle_ground_truth_odom.py \
        ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/run_tracked_vehicle_ate_eval.py \
        ros2_ws/src/cavex_tracked_vehicle/CMakeLists.txt \
        ros2_ws/src/cavex_tracked_vehicle/launch/tracked_vehicle_slam.launch.py
git commit -m "Add ground-truth odom republisher and fixed-time-budget ATE runner for the tracked vehicle"
```

---

### Task 13: Extend the live dashboard for the tracked vehicle

**Files:**
- Modify: `ros2_ws/src/cavex_slam_nav/cavex_slam_nav/web_telemetry_bridge.py` (shared telemetry bridge, cross-package — reads this vehicle's real topics too)
- Modify: `src/components/SICSlamVisualizer.tsx` (or a new small panel component, judge at implementation time per file size, same as the abandoned branch's Task 11 did)

**Interfaces:**
- Consumes: `/odom_ground_truth`, `explore/frontiers`, `/cavex/eval/ate_rmse` (Tasks 11-12), `left_track_retract_joint`/`right_track_retract_joint` positions (Task 5, via `/joint_states`).
- Produces: extended `/api/telemetry` payload.

- [ ] **Step 1: Add subscriptions to `web_telemetry_bridge.py`**

Read the file first. Add (same pattern as existing callbacks):

```python
self.create_subscription(Odometry, '/odom_ground_truth', self._tracked_vehicle_gt_cb, best_effort)
self.create_subscription(MarkerArray, '/explore/frontiers', self._frontiers_cb, 10)
self.create_subscription(JointState, '/joint_states', self._track_state_cb, 10)
```

Store frontier count (`len(msg.markers)`), and track-retraction state (deployed/retracted/moving, derived from the two retract-joint positions found in the `JointState` message — near `0.0` = deployed, near `1.4` = retracted, in between = moving) in `self._latest`.

- [ ] **Step 2: Add a "Tracked BlueBoat-like Vehicle" panel**

Follow `SICSlamVisualizer.tsx`'s established live/demo badge pattern (read its `liveTelemetry` polling `useEffect` first). Display: live position, `frontier_count`, ATE RMSE, and track deployment state — labeled "BlueBoat-like tracked vehicle (ArduPilot Rover SITL)," never implying an official model or real marine capability, per the design spec's explicit honesty requirement.

- [ ] **Step 3: Verify end-to-end**

```bash
curl -s http://localhost:3000/api/telemetry | python3 -m json.tool
```

Expected: `frontier_count`, track state present and changing over consecutive polls while exploration is running.

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_slam_nav/cavex_slam_nav/web_telemetry_bridge.py src/components/
git commit -m "Extend live telemetry + dashboard for the tracked BlueBoat-like vehicle"
```

---

### Task 14: Full end-to-end verification and README update

**Files:**
- Modify: `README.md`

**Interfaces:** none (verification + documentation only).

- [ ] **Step 1: Clean full-stack launch**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash && source ardupilot_gazebo_env.sh
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py &
sleep 25
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py &
sleep 15
ros2 topic list | grep -iE "lidar|costmap|frontier|odom_ground_truth|ap/"
```

Expected: all present and publishing.

- [ ] **Step 2: 10-run ATE evaluation**

```bash
ros2 run cavex_tracked_vehicle run_tracked_vehicle_ate_eval.py --ros-args -p use_sim_time:=true -p num_runs:=10 -p budget_sim_s:=60.0
python3 ros2_ws/src/cavex_slam_nav/cavex_slam_nav/analyze_ate_runs.py ros2_ws/cavex_ate_runs.csv
```

Expected: 10 rows, sane (non-NaN, non-degenerate-zero, `n_samples` roughly consistent run to run) results.

- [ ] **Step 3: Confirm autonomous, obstacle-aware exploration + retraction control, both empirically**

Cross-check ground-truth position over a run against Task 8's obstacle positions (min-distance-to-obstacle-centers check, real Python script against a `ros2 bag record /odom_ground_truth` capture — same technique the abandoned branch's Task 12 used). Separately, command track retraction mid-run and confirm (via `/joint_states`) the joints actually move and (per Task 5's documented decision) the vehicle behaves as designed while retracted.

- [ ] **Step 4: Update README**

Add a "Phase 1 (revised): Tracked BlueBoat-like Vehicle" section to `README.md` covering: build/launch commands for `ardupilot_gazebo`/ArduPilot SITL/`gazebo_tracked_vehicle.launch.py`/`tracked_vehicle_slam.launch.py`, `run_tracked_vehicle_ate_eval.py` usage, track retraction control (`ros2 topic pub /cavex/tracks/command std_msgs/msg/String "{data: 'retracted'}"`), and the same honesty caveats already established project-wide (ground truth is simulator-internal and noiseless; label the vehicle "BlueBoat-like tracked vehicle," never claim an official model or real marine capability). Note that the earlier CHAMP legged-walker approach was abandoned (link to its design doc) in favor of this one.

- [ ] **Step 5: Final commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add README.md
git commit -m "Document Phase 1 (revised) tracked BlueBoat-like vehicle build/launch/eval in README"
```

(Real merge to `main` happens via `superpowers:finishing-a-development-branch` after the whole-branch review, not a direct `git push origin main` mid-plan — this deliberately deviates from the abandoned branch's plan, which had this step push directly to `main`, an oversight from before that branch's worktree-isolation decision was made.)
