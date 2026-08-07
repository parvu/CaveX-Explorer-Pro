# CaveX Explorer Pro

A cave-exploration robot dashboard: a React/Express web frontend, and a ROS 2
Jazzy / Gazebo Harmonic simulation stack (`ros2_ws/src/cavex_slam_nav`) with
RTAB-Map SLAM, a real ATE evaluation harness, and a "SIC-SLAM v0" pose-fusion
prototype.

## Web frontend

Requires Node.js.

```bash
npm install
```

Optional: copy `.env.example` to `.env` and set `GEMINI_API_KEY` to enable the
AI ROS 2 assistant panel (`/api/gemini/ros2-assistant`); it falls back to an
offline template generator without a key.

```bash
npm run dev      # dev server (Vite + Express) on http://localhost:3000
# or, for a production build:
npm run build
npm start         # http://localhost:3000
```

While the ROS 2 stack below is running, the SLAM Benchmark panel and the 3D
simulation viewport both poll `/api/telemetry` and show live pose, lidar,
and ATE data instead of the concept-demo numbers. Clicking "Step Waypoint"
in the Nav planner sends a real (x, y) goal to `waypoint_follower.py` via
`/api/waypoint-goal` — z/mode aren't honored (no flight/dive capability in
this sim).

## ROS 2 / Gazebo simulation stack

Requires ROS 2 Jazzy and Gazebo Harmonic, plus `rtabmap_ros`, `nav2_bringup`,
`ros_gz_sim`, `ros_gz_bridge`, `xacro`, `robot_state_publisher`, `tf2_ros`,
and `python3-numpy` (see `ros2_ws/src/cavex_slam_nav/package.xml`).

Build:

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select cavex_slam_nav
source install/setup.bash
```

Launch the simulation (Gazebo runs headless; visualization goes to the web
frontend above, not a desktop GUI):

```bash
ros2 launch cavex_slam_nav gazebo_sim.launch.py
```

In a second terminal, launch RTAB-Map SLAM, SIC-SLAM v0, the ATE evaluator,
and the web telemetry bridge:

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch cavex_slam_nav rtabmap_nav.launch.py
```

Drive the robot (differential-drive style body velocity):

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}}"
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"   # stop
```

Score a run against ground truth (Absolute Trajectory Error, Umeyama-aligned):

```bash
ros2 topic pub --once /cavex/eval/finish_run std_msgs/msg/Empty "{}"
python3 ros2_ws/src/cavex_slam_nav/cavex_slam_nav/analyze_ate_runs.py ros2_ws/cavex_ate_runs.csv
```

Send a real waypoint goal (straight-line P-controller, no obstacle
avoidance -- see `waypoint_follower.py`; it closes the loop on RTAB-Map's
pose, not ground truth, so its own SLAM error isn't hidden from it):

```bash
ros2 topic pub --once /cavex/nav/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -15, y: 1}}}"
# or via the web bridge:
curl -X POST http://localhost:3000/api/waypoint-goal -H "Content-Type: application/json" -d '{"x": -15, "y": 1}'
```

**On "SIC-SLAM"**: `sic_slam_node.py` is a real but minimal prototype (cmd_vel
+ IMU dead reckoning, bias-corrected against RTAB-Map's pose) — not the full
sonar + Invariant-EKF + GTSAM factor-graph system described in the funding
application, which remains a WP2-WP3 deliverable. See the docstrings in
`sic_slam_node.py` and `ate_evaluator_node.py` for the exact scope, and don't
cite its ATE numbers as "SIC-SLAM" results without that caveat.

Latest result (10 runs, meets the OS2 ≥10-run methodology): **ATE RMSE =
0.019 ± 0.004 m**. This simulation's ground truth is noiseless and RTAB-Map's
own input odometry is that same ground truth, so treat this as a best-case/
idealized-loop number, not a real-sensor-noise result — a real caveat for any
report that cites it.

**On "sonar"**: there is no sonar sensor in this simulation (the robot has an
RGB camera, 2D lidar, and IMU only). The frontend's sonar panels and `sonarActive`/
`sonarDepth`/`sonarEchoStrength` fields are concept-demo values, not backed by
any real sensor or ROS2 topic — don't wire them up as if they were.

## Third-party assets

**Cave geometry** — `ros2_ws/src/cavex_slam_nav/models/cave_world/` is
vendored, unmodified, from
[LTU-RAI/gazebo_cave_world](https://github.com/LTU-RAI/gazebo_cave_world)
(MIT license, copy retained alongside the mesh). Cite: Anton Koval,
Christoforos Kanellakis, Emil Vidmark, Jakub Haluska, George Nikolakopoulos,
"A Subterranean Virtual Cave World for Gazebo based on the DARPA SubT
Challenge," arXiv:2004.08452, Control Engineering Group, Luleå University of
Technology. This project re-wrapped the OBJ as a Gazebo Harmonic SDF 1.9
static model; the mesh itself is not our work. The upstream repo's prop
models (backpack, extinguisher, survivor, jersey barrier, tunnel entrance,
AprilTags) are not used in this phase.

**Tracked vehicle hull** — `ros2_ws/src/cavex_tracked_vehicle/models/blueboat/`
is vendored from
[markusbuchholz/gazebosim_blueboat_ardupilot_sitl](https://github.com/markusbuchholz/gazebosim_blueboat_ardupilot_sitl)
(a mirror of ArduPilot's own `SITL_Models`, author Rhys Mainwaring, meshes
sourced from Blue Robotics' published CAD). This is a real, vendored asset,
not an official Blue Robotics or ArduPilot product release — the tracked
variant (`model.sdf.tracked`) is a project-authored modification (motors
removed, track assemblies added) and is labeled "BlueBoat tracked-vehicle
variant" throughout; it makes no marine/floating capability claim and no
Blue Robotics endorsement.
