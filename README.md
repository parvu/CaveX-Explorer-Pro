# CaveX Explorer Pro

A cave-exploration robot dashboard: a React/Express web frontend, and a ROS 2
Jazzy / Gazebo Harmonic simulation stack (`ros2_ws/`). Design rationale,
verification results, and historical notes live in `launch.txt`, not here —
this file is build/run instructions only.

## Web frontend

Requires Node.js.

```bash
npm install
```

Optional: copy `.env.example` to `.env` and set `GEMINI_API_KEY` to enable the
AI ROS 2 assistant panel; falls back to an offline template generator without
a key.

```bash
npm run dev      # dev server (Vite + Express) on http://localhost:3000
# or, for a production build:
npm run build
npm start         # http://localhost:3000
```

## ROS 2 / Gazebo simulation stack

Requires ROS 2 Jazzy and Gazebo Harmonic, plus `rtabmap_ros`, `nav2_bringup`,
`ros_gz_sim`, `ros_gz_bridge`, `xacro`, `robot_state_publisher`, `tf2_ros`,
`python3-numpy`, and `libgtsam-dev` (see each package's `package.xml`).

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
source ardupilot_gazebo_env.sh   # ArduPilot SITL env, incl. mavproxy.py on PATH
```

Package quick reference — see `launch.txt` (local-only) for full launch
sequences, driving commands, and verification steps for each:

| Package | What |
|---|---|
| `cavex_slam_nav` | Original wheeled robot, RTAB-Map SLAM, ATE eval |
| `cavex_tracked_vehicle` | Tracked BlueBoat + BlueROV2, ArduPilot Rover/Sub SITL |
| `cavex_sonar` | Simulated BlueROV2 acoustic sonar + ocean current |
| `cavex_perception` | RGB-D + lidar instance clustering |
| `cavex_sic_slam` | Real GTSAM Sonar-Inertial-Current factor-graph SLAM |
| `cavex_dcs` | Drift/Current Suppression controller (feed-forward + PI) |

### Phase 1: tracked BlueBoat + BlueROV2 (dry cave, ArduPilot)

```bash
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py &
sleep 25
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py &
sleep 15
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"
```

### Phase 2: BlueROV2 sonar + current + SIC-SLAM + DCS (flooded section)

```bash
gz sim -s -r -v2 src/cavex_slam_nav/worlds/cavex_world.world &
until gz topic -l 2>/dev/null | grep -q "/world/cavex_world/pose/info"; do sleep 1; done
gz service -s /world/cavex_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'sdf_filename: "src/cavex_tracked_vehicle/models/bluerov2/model.sdf", name: "bluerov2", pose: {position: {x: -88.88, y: -31.4, z: 6.9}}'
sleep 5
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml &
sleep 10
ros2 run cavex_sonar sonar_node --ros-args -p seed:=42 -p frame_id:=bluerov2/sonar &
sleep 3
ros2 run cavex_sic_slam sic_slam_node &
ros2 run cavex_dcs dcs_controller &
```

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
