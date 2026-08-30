# CaveX Explorer Pro

A cave-exploration robot: a ROS 2 Jazzy / Gazebo Harmonic simulation stack
(`ros2_ws/`) with a browser-based 3D viewer and control panel (`web_viewer/`).
Design rationale, verification results, and historical notes live in
`history.txt`, not here — this file is build/run instructions only.

## ROS 2 / Gazebo simulation stack

Requires ROS 2 Jazzy and Gazebo Harmonic, plus `rtabmap_ros`, `nav2_bringup`,
`ros_gz_sim`, `ros_gz_bridge`, `xacro`, `robot_state_publisher`, `tf2_ros`,
`python3-numpy`, and `libgtsam-dev` (see each package's `package.xml`).

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Environment

`gz sim` (both the ROS-launched server and a standalone `gz sim -g` GUI) needs a
few paths that ROS 2's own launch machinery does **not** inject. Put these in
`~/.bashrc` once (assumes the repo is at `$HOME/CaveX-Explorer-Pro`):

```bash
# model:// mesh + world resolution for standalone `gz sim` / `gz service create`
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/CaveX-Explorer-Pro/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/CaveX-Explorer-Pro/ardupilot_gazebo/models:$HOME/CaveX-Explorer-Pro/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/CaveX-Explorer-Pro/ros2_ws/src/cavex_slam_nav/models:$GZ_SIM_RESOURCE_PATH
export GZ_SIM_RESOURCE_PATH=$HOME/CaveX-Explorer-Pro/ros2_ws/src/cavex_tracked_vehicle/models:$GZ_SIM_RESOURCE_PATH
# compiled ManualControl / ActionButtons GUI plugin (must be install/, not src/)
export GZ_GUI_PLUGIN_PATH=$HOME/CaveX-Explorer-Pro/ros2_ws/install/cavex_tracked_vehicle_gui/lib/cavex_tracked_vehicle_gui:$GZ_GUI_PLUGIN_PATH
# micro_ros_agent transitive .so lookup (DT_RUNPATH doesn't propagate)
export LD_LIBRARY_PATH=$HOME/CaveX-Explorer-Pro/ros2_ws/install/micro_ros_msgs/lib:$LD_LIBRARY_PATH
# mavproxy.py + microxrceddsgen on PATH (ArduPilot SITL launch shells out to them by bare name)
export PATH=$HOME/.local/bin:$HOME/CaveX-Explorer-Pro/Micro-XRCE-DDS-Gen/scripts:$PATH
# WSL only: force the real D3D12 GPU for Ogre2 instead of llvmpipe software render
export GALLIUM_DRIVER=d3d12
export MESA_LOADER_DRIVER_OVERRIDE=d3d12
```

An interactive shell picks these up from `~/.bashrc` automatically and every
child process (`ros2 launch`, `gz sim`, `gz sim -g`) inherits them. On a fresh
machine or a non-bash shell, export the block above (or add it to that shell's
rc) before launching anything.

Package quick reference — see `history.txt` (local-only) for full launch
sequences, driving commands, and verification steps for each:

| Package | What |
|---|---|
| `cavex_slam_nav` | Original wheeled robot, RTAB-Map SLAM, ATE eval |
| `cavex_tracked_vehicle` | Tracked BlueBoat + BlueROV2, ArduPilot Rover/Sub SITL |
| `cavex_perception` | RGB-D + lidar instance clustering |
| `cavex_gtsam_slam` | Real GTSAM inertial + sonar-scan-registration factor-graph SLAM (no live sonar source or CurrentFactor on this branch -- see perception branch) |

### Phase 1: tracked BlueBoat + BlueROV2 (dry cave, ArduPilot)

```bash
ros2 launch cavex_tracked_vehicle(_gui) gazebo_tracked_vehicle.launch.py &
sleep 25
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py &
sleep 15
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"
```

Web viewer (browser 3D view + drive controls, at http://localhost:8080) and
RViz — both optional, start alongside the launches above once Gazebo is up:

```bash
cd web_viewer
python3 control_server.py 8080 &
/usr/lib/x86_64-linux-gnu/gz/launch7/gz-launch websocket.gzlaunch &
cd ..

rviz2 -d ros2_ws/src/cavex_tracked_vehicle/rviz/tracked_vehicle_mapping.rviz --ros-args -p use_sim_time:=true &
```

Foxglove (browser RViz-equivalent 3D view: TF, `/map`, costmap, sensors) —
`tracked_vehicle_slam.launch.py` already starts `foxglove_bridge` (port
8765, the apt `ros-jazzy-foxglove-bridge` package). The web client itself
is a separate, persistent Docker container (survives across launches,
only needs starting once):

```bash
sudo dockerd > /tmp/dockerd.log 2>&1 &
git clone --depth 1 https://github.com/flora-suite/flora /tmp/flora
cd /tmp/flora && sudo docker build -t flora .
sudo docker run -d --name flora --restart unless-stopped -p 8766:8080 flora
```

Then open http://localhost:8766 (or click "open Foxglove" in the web
viewer) — pre-connects to the bridge automatically via the page's own
link. On first connect the dashboard is empty; either add panels manually
(same idiom as RViz's own "Add Display"), or import
`web_viewer/foxglove/tracked_vehicle_mapping.json` (Layouts tab → import,
or drag the file onto the window) for a layout mirroring
`tracked_vehicle_mapping.rviz`: 3D panel (TF, `/map`, local costmap, lidar
points, obstacle cloud, SLAM path, explore frontiers) + camera image
panel. This is [Flora](https://github.com/flora-suite/flora),
an actively-maintained open-source fork of Foxglove Studio — not
`app.foxglove.dev` (requires a Foxglove account now) and not a self-hosted
build of the frozen last-open-source Foxglove release (protocol/encoding
mismatch with this apt bridge version, confirmed live).

### Phase 2: BlueROV2 GTSAM-SLAM (flooded section)

Real request, 2026-08-26: `cavex_sonar` and the CurrentFactor subsystem
(and its downstream consumer `cavex_dcs`) were removed from this branch
-- see perception branch for the full sonar/current/DCS version. What's
left here is IMU-only dead reckoning (the node's scan-registration code
is unchanged but has no live sonar feed to register against).

```bash
gz sim -r -v2 src/cavex_slam_nav/worlds/cavex_world.world &
until gz topic -l 2>/dev/null | grep -q "/world/cavex_world/pose/info"; do sleep 1; done
gz service -s /world/cavex_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'sdf_filename: "src/cavex_tracked_vehicle/models/bluerov2/model.sdf", name: "bluerov2", pose: {position: {x: 20, y: 0, z: 7.0}}'
sleep 5
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml &
sleep 10
ros2 run cavex_gtsam_slam gtsam_slam_node &
```

Closed-loop motion demos (real `gz-transport` position control, applied via
`ApplyLinkWrench` -- run after the Phase 2 block above, once the vehicle has
spawned):

```bash
# circling demo: holds a 3m-radius circle around (20,0), depth 7.0m
python3 src/cavex_slam_nav/scripts/circle_demo.py 60

# straight-line demo: slides from (19,0) to (31,0), depth 7.0m
python3 src/cavex_slam_nav/scripts/line_demo.py 60
```

Both take an optional duration in seconds (default 60). Both require the
`gz.transport13`/`gz.msgs10` Python bindings (installed alongside Gazebo
Harmonic) -- no additional ROS nodes needed beyond the spawned vehicle.

### Phase 3: x500 air-pocket SLAM (planned)

Real request 2026-08-27: an air pocket in the cave whose ceiling sits far
above what the tracked boat/ROV can reach or usefully map from the water
or dry floor. For this phase, `cavex_tracked_blueboat` (and its fused
BlueROV2 hull) stays parked -- on dry ground or floating in the water
basin, wherever the run left it -- while `x500` (the PX4 quadcopter
already vendored at `ros2_ws/src/cavex_tracked_vehicle/models/x500/`,
currently only used as decorative static cargo on the boat's own deck,
see `cavex_x500_release` topic) takes off on its own and flies/maps the
air pocket above. Not implemented yet -- x500 has no independent flight
controller, SLAM node, or launch entry of its own today; this section is
the placeholder for that work (PX4 SITL or a direct gz-transport thrust
controller matching Phase 2's own `ApplyLinkWrench` pattern, plus a
lidar/camera-based SLAM node reusing `cavex_gtsam_slam`'s scan-registration
code the way Phase 2 does).

## Known limitations (sim-to-real gap)

**Scan registration is classical, not learned.** `cavex_gtsam_slam`'s
keyframe-to-keyframe registration (`scan_registration.cpp`) is a
plain ICP-style SVD rigid-transform fit — there is no neural network or
learned feature extractor anywhere in this pipeline. Any claim about a
"neural feature extractor" being affected by sonar noise does not apply
to this codebase as implemented.

## Third-party assets

**Cave geometry** — `ros2_ws/src/cavex_slam_nav/models/cave_world/` is
vendored, unmodified, from
[LTU-RAI/gazebo_cave_world](https://github.com/LTU-RAI/gazebo_cave_world)
(MIT license, copy retained alongside the mesh). Cite: Anton Koval,
Christoforos Kanellakis, Emil Vidmark, Jakub Haluska, George Nikolakopoulos,
"A Subterranean Virtual Cave World for Gazebo based on the DARPA SubT
Challenge," arXiv:2004.08452, Control Engineering Group, Luleå University of
Technology.

**Tracked vehicle hull** — `ros2_ws/src/cavex_tracked_vehicle/models/blueboat/`
is vendored from
[markusbuchholz/gazebosim_blueboat_ardupilot_sitl](https://github.com/markusbuchholz/gazebosim_blueboat_ardupilot_sitl)
(a mirror of ArduPilot's own `SITL_Models`, author Rhys Mainwaring, meshes
sourced from Blue Robotics' published CAD). Not an official Blue Robotics or
ArduPilot product release.

**BlueROV2** — `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/` is
vendored, with one documented modification, based on
[clydemcqueen/bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz).
Not an official Blue Robotics product release.

**PX4 x500 quadcopter** — `ros2_ws/src/cavex_tracked_vehicle/models/x500/`
is vendored, unmodified, from
[fuel.gazebosim.org/PX4/models/x500](https://fuel.gazebosim.org/1.0/PX4/models/x500)
(CC BY 4.0, author Benjamin Perseghetti).
