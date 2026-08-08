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

Complete and merged into `main`. Phase 1 uses a BlueBoat-hulled tracked ground vehicle under ArduPilot Rover
(ArduRover) control for the dry cave section, carrying a BlueROV2 as
physical cargo mounted just above its deck. `vehicle_switch_node.py`
watches the tracked vehicle's real ground truth and, on crossing the real
water boundary (x=15, `cave_floor_patch`'s own vertex-confirmed edge),
retracts the tracks and releases the BlueROV2 via Gazebo's real
`gz-sim-detachable-joint-system` plugin (added to `model.sdf.tracked`,
confirmed against the actual upstream reference example at
`/usr/share/gz/gz-sim8/worlds/detachable_joint.sdf`). Live-verified: both
entities spawn together without a physics crash, settle rigidly attached
(constant z offset -- ~0.4m at the mount height first tested, ~0.03m at the
current, deck-flush mount height after a later fix; see "Deployment
mechanism, helipad, and PX4 x500" below), and a real detach produces
genuine, growing positional divergence between them within seconds — not
inferred from code alone.

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

**Build** (from the repo root):

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

**`tracked_vehicle_slam.launch.py` auto-drives the vehicle for the first 300
real seconds of every launch** (undisclosed until this final review):
`bootstrap_nudge` unconditionally publishes `/cmd_vel` (`linear.x=0.3`) for
5 minutes starting 5s after launch, to give `icp_odometry` enough real
parallax to bootstrap at this environment's low real_time_factor (see
that action's own comment in the launch file for the full calibration
history). This means: (a) any manual `/cmd_vel` publish loop started in
that same window competes with it rather than being the only driver, and
(b) "no autonomous driving in the loop" claims elsewhere in this README
about specific verification runs are only true once past that 300s window
-- correct the record on both points if you're relying on either claim
precisely.

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
the Rover instance required three real, non-obvious fixes (corrected in
final review -- an earlier version of this section said all three live in
`dds_udp_instance1.parm`; two are actually launch arguments, since this
branch has no committed launch file that brings the second instance up --
see "Water-boundary handoff" below for the real, manually-run command that
supplies them): (1) `DDS_UDP_PORT` (in
`ros2_ws/src/cavex_tracked_vehicle/config/dds_udp_instance1.parm`) plus the
`micro_ros_agent_ns`/`master:=tcp:127.0.0.1:5770` launch arguments, to avoid
port collisions with the Rover instance; (2) `DDS_USE_NS 1` (also in that
parm file) -- AP_DDS's own topic/service names (`ap/cmd_vel` etc.) are
hardcoded per-firmware regardless of which `micro_ros_agent` port bridges
them, so two simultaneous instances collide on identical ROS 2 topic names
unless this is enabled (verified topics land under `/ap/v1/...`, not the
`/ap/...` of the Rover instance); (3) BlueROV2's own vendored
`ArduPilotPlugin` FDM port was moved from the default 9002 to 9012 in
`models/bluerov2/model.sdf` because the tracked vehicle's own (unused,
inherited) `ArduPilotPlugin` block already binds 9002 in the same Gazebo
process (supplied via the `sim_port_in`/`sim_port_out` launch arguments on
the ArduSub side). With these fixes,
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
with no `explore_lite`/Nav2 autonomy in the loop (the launch file's own
`bootstrap_nudge` auto-drive, see above, was still active for its first
300s as it is on every launch). Obstacle-avoidance verification
(Task 9's four fuel-model obstacles in the dry section) used a
min-distance-to-obstacle-centers check against a recorded `/odom_ground_truth`
bag; the manually-driven verification run never came within collision range
of any obstacle (closest approach ~7.7m), which confirms no collisions but
is a weaker exercise of close-proximity avoidance than a route that
deliberately threads between them. **Stale as of final review**: that
~7.7m figure was measured against the obstacles' original positions
(`x∈[-45,-12]`) and the original spawn (`x=-60`); the floor-collision fix
below moved both the obstacles (`x∈[-30,0]`) and the spawn (`x=-35`), so
that specific number no longer describes a check against the current
world -- re-run the same bag-based check if you need current numbers.

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
`vehicle_switch_node.py`), **and also `tracked_vehicle_slam.launch.py`** --
`vehicle_switch_node.py`'s real trigger watches `/odom_ground_truth`, which
is only published once that second launch file's own
`tracked_vehicle_ground_truth_odom.py` is running (corrected in final
review: an earlier version of this section omitted this second launch
file, under which the automatic trigger can never fire). Then drive to the
real water boundary (`x=15`):

```bash
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py &
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py &
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
# real_time_factor here is highly variable (~0.01 to 0.4+) -- the ~50m drive from
# the x=-35 spawn to x=15 can take a long time; to exercise the handoff directly
# instead of waiting, publish its two triggers manually:
ros2 topic pub --once /cavex/tracks/command std_msgs/msg/String "{data: 'retracted'}"
ros2 topic pub --once /cavex/rov_release/detach std_msgs/msg/Empty "{}"
```

Once released, control the BlueROV2 via `cmd_vel_to_ardusub.py`'s manual
teleop input (no autonomous exploration for the BlueROV2 this phase --
explicit non-goal). This branch does NOT yet commit a launch file that
brings up the second ArduSub SITL instance (corrected in final review --
an earlier version of this section implied `ros2 run
cmd_vel_to_ardusub.py` alone was enough; it needs the second instance
already running underneath it). The real, live-verified bring-up
command, run manually:

```bash
SUB_PARM="install/ardupilot_sitl/share/ardupilot_sitl/config/default_params/sub.parm"
DDS_PARM="install/ardupilot_sitl/share/ardupilot_sitl/config/default_params/dds_udp.parm"
INST1_PARM="src/cavex_tracked_vehicle/config/dds_udp_instance1.parm"
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py command:=ardusub model:=JSON \
  instance:=1 port:=2029 micro_ros_agent_ns:=ap2 master:=tcp:127.0.0.1:5770 \
  sim_port_in:=9013 sim_port_out:=9012 defaults:="$SUB_PARM,$DDS_PARM,$INST1_PARM"
# wait for DDS init (real, patient wait -- 15-25s), then confirm:
ros2 service list | grep v1   # expect /ap/v1/arm_motors, /ap/v1/mode_switch, etc.
ros2 run cavex_tracked_vehicle cmd_vel_to_ardusub.py
ros2 topic pub -r 10 /cmd_vel_rov geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

Live-verified with the full SLAM/Nav2/`vehicle_switch_node` stack running
together: the handoff produces no new, persistent errors in RTAB-Map, Nav2,
or the tracked vehicle's own control nodes (the pre-existing "waiting for
`map` transform" messages during RTAB-Map's own startup are normal and
predate the handoff, not caused by it) -- confirmed by diffing the stack's
log output from immediately before to after triggering the handoff.

**Deployment mechanism, helipad, and PX4 x500** — `model.sdf.tracked` adds a
static davit (deployment crane) fixture near the ROV's mount point (visual
representation only, not actuated/animated — the real deployment mechanism
is the `DetachableJoint` plugin, not this fixture) and a real, marked helipad
at the bow. A second real, vendored model
(`fuel.gazebosim.org/PX4/models/x500`, CC-BY-4.0) is carried on that helipad
via its own `DetachableJoint` (`/cavex/x500_release/detach`), placeholder
scope only — no PX4 SITL/flight-control integration, explicitly future work
("airpocket exploration"). Live-verified: all three carried bodies (tracked
vehicle + BlueROV2 + x500) spawn together without a physics crash across
repeated launches, and a manual x500 detach produced real divergence (it
fell under gravity once released, while the tracked vehicle itself stayed at
its normal rest height). **Known limitation, not fully resolved**: exact
mount alignment (bluerov2's top flush with the deck, x500 sitting precisely
at the helipad surface) is not perfectly reliable — settled offsets varied
between otherwise-identical launches (observed range: a few centimeters to
several tens of centimeters), most likely from real timing variance in when
each `DetachableJoint` actually engages relative to how long its child free-falls
first. Never observed to cause a crash or hull-collision overlap across
multiple extended test launches, but the mounts are best-effort, not exact.

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

**BlueROV2** — `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/` is
vendored, with one documented modification, based on
[clydemcqueen/bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz)'s
real model structure (hull, thrusters, `ArduPilotPlugin` FDM link). The
modification: `model.sdf`'s `ArduPilotPlugin` `fdm_port_in` was changed from
the vendored default 9002 to 9012, to avoid a real port collision with the
tracked vehicle's own (unused, inherited) `ArduPilotPlugin` block in the same
Gazebo process -- see that plugin's own comment. Not an official Blue
Robotics product release; labeled "BlueROV2" as a real vendored simulation
asset, not a claim of hardware-accuracy beyond what that upstream repo
itself provides.

**PX4 x500 quadcopter** — `ros2_ws/src/cavex_tracked_vehicle/models/x500/`
is vendored, unmodified, from
[fuel.gazebosim.org/PX4/models/x500](https://fuel.gazebosim.org/1.0/PX4/models/x500)
(CC BY 4.0, author Benjamin Perseghetti). Model of the NXP HoverGames Drone
development kit (KIT-HGDRONEK66); carried as a placeholder for future
aerial/"airpocket" exploration work (see the Phase 1 section above) — no PX4
SITL or flight-control integration is wired up in this phase.
