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

## Phase 1 (revised): Tracked BlueBoat-like Vehicle

The original Phase 1 approach was a CHAMP legged quadruped
(`docs/superpowers/specs/2026-08-04-cavex-legged-walker-phase1-design.md`);
it was abandoned after an unresolved legged-locomotion balance/stance
problem discovered during implementation. This phase replaces it with a
BlueBoat-hulled tracked ground vehicle under ArduPilot Rover (ArduRover)
control, in a separate worktree/branch
(`cavex-tracked-blueboat-ardupilot`) — CHAMP's SLAM/Nav2 bringup for the
legged robot is not reused.

**Build** (from the worktree root):

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
source ardupilot_gazebo_env.sh   # ArduPilot SITL env, incl. mavproxy.py on PATH
```

**Launch** (two terminals, or background both):

```bash
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py   # Gazebo + ArduPilot Rover SITL
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py     # RTAB-Map 3D-lidar SLAM + Nav2
```

Drive it (once ArduPilot arms and sets GUIDED mode, which
`cmd_vel_to_ardupilot.py` does automatically a few seconds after launch):

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
```

**Track retraction control** — the hull's two retractable track assemblies
are commanded via a single string topic:

```bash
ros2 topic pub --once /cavex/tracks/command std_msgs/msg/String "{data: 'retracted'}"
ros2 topic pub --once /cavex/tracks/command std_msgs/msg/String "{data: 'deployed'}"
```

Confirm real joint motion via `/joint_states`
(`left_track_retract_joint`/`right_track_retract_joint`, ~0 rad deployed,
~1.4 rad retracted — live-verified round-trip).

**ATE evaluation** (ground truth vs. RTAB-Map's SLAM estimate):

```bash
ros2 run cavex_tracked_vehicle run_tracked_vehicle_ate_eval.py --ros-args \
  -p use_sim_time:=true -p num_runs:=3 -p budget_sim_s:=15.0
cat cavex_ate_runs.csv
```

This environment's real_time_factor is typically very low (~0.02-0.03, spiking
higher under lighter CPU load) — the eval script does not drive the vehicle
itself; run a manual `/cmd_vel` publish loop alongside it or the run just
measures a stationary vehicle. **Known limitation, not yet resolved**:
`icp_odometry` (RTAB-Map's lidar odometry front end) does not reliably
bootstrap its first keyframe even after well-calibrated bootstrap driving —
symptom is `icp_inliers_ratio` staying at 0.0 and repeated "Registration
failed: No matches available for computing distance quantiles" /
"structural complexity is too low (corridor-like environment)" log lines.
This is the same underlying class of issue as `explore_lite`'s parked
costmap-coverage problem below, not a new bug. No ATE data was successfully
produced in the verification session that added this section; rerun and
check `icp_inliers_ratio` is nonzero (`ros2 topic echo /odom_info --once`)
before relying on a `cavex_ate_runs.csv` result.

**`explore_lite` (autonomous frontier exploration) — known limitation**: it
currently stops itself almost immediately ("No frontiers found, stopping")
because RTAB-Map's published `/global_costmap/costmap` window doesn't track
the robot's real position even though RTAB-Map's internal map graph is
genuinely growing. Root cause not yet isolated (RTAB-Map grid-publishing lag
vs. Nav2 `static_layer` sizing vs. frame-axis mismatch). Don't rely on it for
autonomous verification; drive the vehicle manually via `/cmd_vel` instead —
that's what this section's own verification did, real ground-truth motion
with no autonomous driving in the loop. Obstacle-avoidance verification
(Task 9's four fuel-model obstacles in the dry section) used a
min-distance-to-obstacle-centers check against a recorded `/odom_ground_truth`
bag; the manually-driven verification run never came within collision range
of any obstacle (closest approach ~7.7m), which confirms no collisions but
is a weaker exercise of close-proximity avoidance than a route that
deliberately threads between them.

**Water region** — the flooded chamber for Task 16's BlueROV2/water-boundary
work was re-derived from a live probe-drop survey of the real vendored cave
mesh (not an unverified whole-bounding-box guess): a real, flat, obstacle-free
floor at world `z~=0` was found spanning roughly `x∈[15,65] y∈[2,12]`,
confirmed clear for at least 25m overhead. The water surface sits at `z=2.0`
(a real ~2m partial flood, not a fill to the ceiling), centered at `(40, 7)`.
See `ros2_ws/src/cavex_slam_nav/worlds/cavex_world.world`'s `water_surface`
and `Buoyancy` plugin comments for the full survey data.

**Honesty caveats** (same standard as the rest of this project): this
simulation's ground truth is simulator-internal and noiseless — treat any
ATE/localization numbers as best-case/idealized, not real-sensor-noise
results. The vehicle is labeled "BlueBoat-like tracked vehicle" throughout;
it makes no official Blue Robotics/ArduPilot endorsement claim and no
marine/floating capability claim (tracks only, ArduPilot Rover firmware, not
ArduBoat).

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
