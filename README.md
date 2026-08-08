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

## Phase 1: Tracked BlueBoat-like Vehicle + BlueROV2

Phase 1 uses a BlueBoat-hulled tracked ground vehicle under ArduPilot Rover
(ArduRover) control for the dry cave section, carrying a BlueROV2 as
physical cargo mounted just above its deck. `vehicle_switch_node.py`
watches the tracked vehicle's real ground truth and, on crossing the real
water boundary (x=15, `cave_floor_patch`'s own vertex-confirmed edge),
retracts the tracks and releases the BlueROV2 via Gazebo's real
`gz-sim-detachable-joint-system` plugin (added to `model.sdf.tracked`,
confirmed against the actual upstream reference example at
`/usr/share/gz/gz-sim8/worlds/detachable_joint.sdf`). Live-verified: both
entities spawn together without a physics crash, settle rigidly attached
(constant ~0.4m z offset), and a real detach produces genuine, growing
positional divergence between them within seconds — not inferred from
code alone.

**Known limitations**: the handoff is one-way — `DetachableJoint` has no
usable re-attach for a BlueROV2 that's already drifted away, so returning
to the dry section does not re-dock it or re-deploy the tracks
automatically. `cavex_world.world`'s Buoyancy plugin applies its water
density by world-frame z alone, not scoped to the real water region's x/y
extent, so the carried BlueROV2 technically experiences buoyancy forces
throughout dry-section transport too (absorbed as extra load on the joint
while attached — not visibly wrong, but not physically correct either).
Controlling the BlueROV2 once released still needs ArduSub, which has its
own separate, unresolved limitation — see "BlueROV2 / ArduSub" below;
`vehicle_switch_node.py` does not start or manage ArduSub itself, by
explicit design.

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

**BlueROV2 / ArduSub** (`cmd_vel_to_ardusub.py`, second SITL instance):
running a second ArduPilot instance (ArduSub, for the BlueROV2) alongside
the Rover instance required three real, non-obvious fixes, all now applied
in `ros2_ws/src/cavex_tracked_vehicle/config/dds_udp_instance1.parm`: (1)
`DDS_UDP_PORT`/`micro_ros_agent_ns`/`master:=tcp:127.0.0.1:5770` to avoid
port collisions with the Rover instance; (2) `DDS_USE_NS 1` -- AP_DDS's
own topic/service names (`ap/cmd_vel` etc.) are hardcoded per-firmware
regardless of which `micro_ros_agent` port bridges them, so two
simultaneous instances collide on identical ROS 2 topic names unless this
is enabled (verified topics land under `/ap/v1/...`, not the `/ap/...`
of the Rover instance); (3) BlueROV2's own vendored `ArduPilotPlugin` FDM
port was moved from the default 9002 to 9012 in `models/bluerov2/model.sdf`
because the tracked vehicle's own (unused, inherited) `ArduPilotPlugin`
block already binds 9002 in the same Gazebo process. With these fixes,
DDS connectivity, the real Gazebo FDM/JSON physics link, and GUIDED mode
switching (`/ap/v1/mode_switch`, mode 4) are all live-verified working.
**Known limitation, not yet resolved**: arming (`/ap/v1/arm_motors`)
is consistently rejected (`result=False`) even with `ARMING_CHECK 0`,
`FENCE_ENABLE 0`, and a real disarm-button RC option assigned (ArduSub's
`AP_Arming_Sub::pre_arm_checks` hard-requires one, not gated by
`ARMING_CHECK`) -- the exact rejection reason could not be captured
because MAVLink/SERIAL0 never produced a heartbeat to any client in this
environment (confirmed via both `mavproxy` and a direct `pymavlink`
connection, 30s+ waits), so `AP_Arming`'s own `check_failed()` messages
(routed via MAVLink STATUSTEXT) were never observable. Needs a working
MAVLink console connection to diagnose further.

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

**Water region** — re-derived a second time after a real bug was found: the
tracked vehicle (and everything else) turned out to be resting on
`cavex_world.world`'s flat `ground_plane` at `z=0`, not the real vendored cave
mesh's own collision, which has genuine gaps in floor coverage across large
areas (a probe dropped at the original spawn point fell straight through with
`ground_plane` removed). Re-derived the real floor height (`z=5.9`,
`CAVE_FLOOR_Z`) directly from the mesh's own dense vertex data across the
`x=[-37,65]` corridor this project uses, added a real supplementary
`cave_floor_patch` collision there, and moved `ground_plane` to `z=-200` as a
pure safety net. The flooded chamber sits on that same real floor: water
surface at `z=7.9` (a real ~2m partial flood above the real floor, not a fill
to the ceiling), region `x∈[15,65] y∈[-10,10]`, centered `(40, 0)`. See
`ros2_ws/src/cavex_slam_nav/worlds/cavex_world.world`'s `cave_floor_patch`,
`water_surface`, and `Buoyancy` plugin comments for the full derivation.

**Water-boundary handoff** — end to end: build ArduSub (`cd ardupilot &&
./waf sub`, needs `--enable-DDS` at configure time and a real
`microxrceddsgen` install, see "BlueROV2 / ArduSub" above), launch the
tracked vehicle (which also spawns the carried BlueROV2 and
`vehicle_switch_node.py`), drive to the real water boundary (`x=15`):

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
# real_time_factor here is highly variable (~0.01 to 0.4+) -- the ~50m drive from
# the x=-35 spawn to x=15 can take a long time; to exercise the handoff directly
# instead of waiting, publish its two triggers manually:
ros2 topic pub --once /cavex/tracks/command std_msgs/msg/String "{data: 'retracted'}"
ros2 topic pub --once /cavex/rov_release/detach std_msgs/msg/Empty "{}"
```

Once released, control the BlueROV2 via `cmd_vel_to_ardusub.py`'s manual
teleop input (no autonomous exploration for the BlueROV2 this phase --
explicit non-goal):

```bash
ros2 run cavex_tracked_vehicle cmd_vel_to_ardusub.py
ros2 topic pub -r 10 /cmd_vel_rov geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

Live-verified with the full SLAM/Nav2/`vehicle_switch_node` stack running
together: the handoff produces no new, persistent errors in RTAB-Map, Nav2,
or the tracked vehicle's own control nodes (the pre-existing "waiting for
`map` transform" messages during RTAB-Map's own startup are normal and
predate the handoff, not caused by it) -- confirmed by diffing the stack's
log output from immediately before to after triggering the handoff.

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
