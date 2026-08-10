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

A BlueBoat-hulled tracked ground vehicle under ArduPilot Rover (ArduRover)
control handles the dry cave section, carrying a BlueROV2 as physical cargo
mounted just above its deck. The BlueROV2 is rigidly locked to the hull for
the whole dry section via Gazebo's `gz-sim-detachable-joint-system` plugin
(`/cavex/rov_lock/attach`/`/cavex/rov_lock/detach`), not tether tension alone.
`vehicle_switch_node.py` locks it once at startup and releases it only once
both hold: past the water boundary (x=15) and the boat has been afloat
(z > 6.5) for 2s continuously. The motorized tether
(`motorized_tether_control.py`) stays active throughout — the rigid joint
dominates while it holds, and becomes the operative restraint again once
unlocked. Track retraction and tether payout trigger independently, keyed to
the water-boundary crossing.

**Known limitations**: the handoff is one-way — no automatic re-lock/re-dock
once released. `cavex_world.world`'s Buoyancy plugin applies its water
density by world-frame z alone, not scoped to the water region's x/y extent,
so the carried BlueROV2 experiences buoyancy forces during dry-section
transport too (absorbed as extra load on the rigid joint while locked).
Controlling the BlueROV2 once released still needs ArduSub — see "BlueROV2 /
ArduSub" below; `vehicle_switch_node.py` does not start or manage ArduSub
itself.

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

**Memory/CPU**: Gazebo runs headless by default (`gz_args: -r -s`). A
GUI-attached run consumes 7.5GB+ RSS and can push this environment into
CPU-saturation stalls. GPU rendering is on by default for whichever process
attaches a GUI (`ardupilot_gazebo_env.sh` sets `GALLIUM_DRIVER=d3d12`/
`MESA_LOADER_DRIVER_OVERRIDE=d3d12` for WSL2 GPU passthrough). Under the full
stack (GPU render + RTAB-Map + Nav2 together), CPU can still saturate hard
enough that Nav2's lifecycle bond heartbeats (4s timeout) get missed,
crashing `collision_monitor`/`waypoint_follower` and aborting Nav2 bringup —
SLAM/TF/rviz are unaffected. Retry via `ros2 service call
/lifecycle_manager_navigation/manage_nodes
nav2_msgs/srv/ManageLifecycleNodes "{command: 0}"` once load has settled.

**GPU compute**: RTAB-Map/OpenCV in this environment have zero CUDA/GPU
compute support (`rtabmap --version` shows `With CudaSift: false`;
`cv2.cuda.getCudaEnabledDeviceCount()` returns 0). An NVIDIA GTX 1050 Ti is
visible via WSL CUDA passthrough but sits idle — only the CUDA compute libs
are present, not the graphics/EGL/Vulkan driver stack Gazebo's rendering
needs. Enabling real GPU-accelerated SLAM would need a full from-source
rebuild of PCL/OpenCV/libpointmatcher/rtabmap with CUDA flags.

Visualize with the saved rviz2 config (TF, `/map`, `/lidar/points`,
ground-truth + SLAM odometry paths, `/explore/frontiers`) and/or attach
Gazebo's own GUI on demand in follow mode (model name is
`cavex_tracked_blueboat`, confirm with `gz model --list` if unsure):

```bash
ros2 run rviz2 rviz2 -d src/cavex_tracked_vehicle/rviz/tracked_vehicle_mapping.rviz &
gz sim -g &   # attaches to the already-running headless server
gz service -s /gui/follow --reqtype gz.msgs.StringMsg --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'data: "cavex_tracked_blueboat"'
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
~1.4 rad retracted).

**`tracked_vehicle_slam.launch.py` auto-drives the vehicle for the first 300
real seconds of every launch**: `bootstrap_nudge` unconditionally publishes
`/cmd_vel` (`linear.x=0.3`) for 5 minutes starting 5s after launch, to give
`icp_odometry` enough real parallax to bootstrap at this environment's low
real_time_factor. Any manual `/cmd_vel` publish loop started in that same
window competes with it rather than being the only driver.

**ATE evaluation** (ground truth vs. RTAB-Map's SLAM estimate):

```bash
ros2 run cavex_tracked_vehicle run_tracked_vehicle_ate_eval.py --ros-args \
  -p use_sim_time:=true -p num_runs:=3 -p budget_sim_s:=15.0
cat cavex_ate_runs.csv
```

This environment's real_time_factor is typically very low (~0.02-0.03,
spiking higher under lighter CPU load) — the eval script does not drive the
vehicle itself; run a manual `/cmd_vel` publish loop alongside it or the run
just measures a stationary vehicle. **Known limitation, not yet resolved**:
`icp_odometry` (RTAB-Map's lidar odometry front end) does not reliably
bootstrap its first keyframe even after well-calibrated bootstrap driving —
symptom is `icp_inliers_ratio` staying at 0.0 and repeated "Registration
failed" / "structural complexity is too low (corridor-like environment)" log
lines. Check `icp_inliers_ratio` is nonzero (`ros2 topic echo /odom_info
--once`) before relying on a `cavex_ate_runs.csv` result.

**Navigation performance**: this environment (WSL2) commonly runs at
CPU/scheduling oversubscription — `nproc` may report far fewer cores than
the combined demand of Gazebo, RTAB-Map, `icp_odometry`, Nav2, and ArduPilot
SITL running together, which shows up as elevated RTAB-Map `Conversion` time
and general navigation sluggishness independent of any single ROS parameter.
If navigation feels slow, check `uptime`/`nproc` before tuning sensor or
SLAM parameters.

**BlueROV2 / ArduSub** (`cmd_vel_to_ardusub.py`, second SITL instance):
running a second ArduPilot instance (ArduSub, for the BlueROV2) alongside the
Rover instance needs: (1) `DDS_UDP_PORT` (in
`ros2_ws/src/cavex_tracked_vehicle/config/dds_udp_instance1.parm`) plus the
`micro_ros_agent_ns`/`master:=tcp:127.0.0.1:5770` launch arguments, to avoid
port collisions with the Rover instance; (2) `DDS_USE_NS 1` (also in that
parm file) — AP_DDS's own topic/service names are hardcoded per-firmware, so
two simultaneous instances collide on identical ROS 2 topic names unless this
is enabled (topics land under `/ap/v1/...` instead of `/ap/...`); (3)
BlueROV2's own vendored `ArduPilotPlugin` FDM port is 9012 (not the default
9002), because the tracked vehicle's own `ArduPilotPlugin` block already
binds 9002 in the same Gazebo process. With these in place, DDS connectivity,
the Gazebo FDM/JSON physics link, and GUIDED mode switching
(`/ap/v1/mode_switch`, mode 4) all work.

**Known limitation, not yet resolved**: arming (`/ap/v1/arm_motors`) is
consistently rejected even with `ARMING_CHECK 0`, `FENCE_ENABLE 0`, and a
disarm-button RC option assigned (ArduSub's `AP_Arming_Sub::pre_arm_checks`
hard-requires one, not gated by `ARMING_CHECK`) — the exact rejection reason
requires a working MAVLink console connection to diagnose (MAVLink/SERIAL0
has not produced a heartbeat to any client in this environment).

**`explore_lite`** (autonomous frontier exploration) starts after a delay
(`TimerAction`, 320s) to give RTAB-Map time to publish a stable `map` frame
first. Nav2's `global_costmap` inflation is tuned low (`0.35`, just above
`robot_radius=0.3`) so frontier search has real free-cell-adjacent-to-unknown
cells to find; raising it breaks frontier detection. `BackUp` recovery
distance is `0.60m` (`config/tracked_vehicle_nav_to_pose_bt.xml`), sized for
this vehicle's ~1.19m hull. RTAB-Map's `map`->`odom` TF publishes at 10Hz
(`Rtabmap/DetectionRate`, matching the lidar's own update rate) to avoid
"could not transform ... to map" flicker.

**Dead-end handling** (`dead_end_backtrack_node.py`, launched as part of
`tracked_vehicle_slam.launch.py`) — `explore_lite` itself has no built-in
dead-end mitigation (its frontier blacklist only avoids a goal Nav2 already
aborted, no physical escape behavior). This node is the real mitigation:
1. **Trigger — closed corridor only, not staleness.** A costmap-blocked wall
   within 1m ahead with no lateral opening at the current position either.
   No reactive "no progress for N seconds" fallback — a stall that isn't a
   genuinely closed corridor is left to Nav2's own progress checker and
   recovery behaviors.
2. **Retreat until there's real room to rotate** — adaptive, not a fixed
   distance. Backs straight up (the same path it entered on) checking the
   costmap every 0.2m for a real, obstruction-free circle around the
   vehicle (0.45m radius, matching `robot_radius=0.3` plus margin), capped
   at 12m before giving up. The only reverse driving anywhere in this node.
3. **Full 360° survey** — always completes the whole sweep (no early stop
   at the first candidate), scoring every ~15° direction's real clear
   distance, then turns to face whichever direction had the best (widest)
   opening. Direction of rotation (CW/CCW) is chosen once at the start,
   toward whichever side has more real clearance — this only affects sweep
   order, not coverage, since the full 360° is always completed. Hands
   control back to Nav2/explore_lite once done.
4. **Backtrack only if the full sweep finds nothing**: turn to face back
   along the recorded trail of waypoints, then drive forward along it (no
   reverse driving here either), checking the costmap periodically for a
   lateral opening, capped at 12m.

Core grid-math logic (`find_lateral_opening`, `ray_is_clear`,
`clearance_on_side`, `ray_clear_distance`, `can_rotate_freely`) is
pure-function and covered by a synthetic self-check
(`dead_end_backtrack_node.py --self-check`).

**Obstacle avoidance** — Nav2's `collision_monitor` is wired to this
vehicle's real 3D lidar (`/lidar/points`, height-filtered to exclude ground
hits) with an `ObstacleBubble` circular slowdown polygon (0.8m radius from
`base_link`, i.e. ~0.5m clearance beyond the 0.3m footprint) plus a
footprint-based `approach` polygon for imminent-contact braking. This is
independent of the local costmap's own inflation-based avoidance used by the
MPPI controller during normal path following (`inflation_radius=1.8`,
`cost_scaling_factor=1.5`, tuned for corridor centering — `CostCritic`
weight 10.0 kept above `PathAlignCritic`'s 8.0 so centering wins over
blindly hugging the global plan, which itself has no centering incentive
since `global_costmap` inflation stays low to protect frontier detection).

**Cave scaled 2x** (`ros2_ws/src/cavex_slam_nav/models/cave_world/model.sdf`)
— the vendored mesh's `<collision>`/`<visual>` geometry carries a real
`<scale>2 2 2</scale>` (the only place scale actually works in this
sdformat version — a `<scale>` on the world file's `<include>` is silently
ignored). Scaling happens around the mesh's own local origin, not around
any fixed world position, so the real corridor moved: the vehicle now
spawns around `x=-88.78 y=-31.4` (was `x=-35 y=0`), with a second
supplementary floor patch (`cave_floor_patch_scaled`) covering that area
since the original patch's coverage doesn't reach it. Every environment
detection range that used to be tuned for the un-scaled cave was doubled
alongside it: local costmap rolling-window size, lidar max range (60m),
RTAB-Map's `Grid/RangeMax`, and the dead-end node's own lookahead/survey/
retreat/backtrack distances. Vehicle-size parameters (`robot_radius`,
`ObstacleBubble`, the dead-end node's rotation-clearance radius) were
deliberately **not** doubled — the vehicle itself didn't get bigger, only
the cave did.

**ArduPilot heading drift** (fixed in `cmd_vel_to_ardupilot.py` and
`track_cmd_vel_bridge.py`) — this vehicle has no GPS/navsat sensor (removed
as orphaned; the original vendored `navsat_link`/sensor was never wired
into `ArduPilotPlugin` or bridged to ROS at all), so ArduPilot's own EKF
runs on pure IMU dead-reckoning and drifts over distance with nothing to
correct it — measured live at ~91° of heading drift from Gazebo's real
ground truth after a modest drive. That drift corrupted driving direction
on both sides of the ArduPilot round-trip: `cmd_vel_to_ardupilot.py` now
rotates outgoing `/cmd_vel` from body-frame to world-frame using Gazebo's
own ground-truth heading (read live via gz-transport,
`/world/<world>/pose/info`) before sending it to `/ap/cmd_vel` with
`frame_id=map` (bypassing ArduPilot's own drifted-heading conversion for
commands sent as `frame_id=base_link`); `track_cmd_vel_bridge.py` does the
same in reverse for `/ap/twist/filtered`'s reported world-frame velocity
(itself `ahrs.get_velocity_NED()` converted to ROS ENU, despite the
message's `frame_id` claiming `base_link`) before handing it to Gazebo's
`TrackedVehicle` plugin. Both self-check (`--self-check`), including a
round-trip check between the two files' rotation functions.

**Note on the water region below**: its `x=15`/`x=-35`-spawn coordinates
predate the cave 2x scale above and were not re-derived against it — the
water surface, `cave_floor_patch`, and their boundary are separate, fixed
SDF models untouched by the mesh scale, while the vehicle's own spawn moved
to a different part of the now-larger cave. Whether the dry-section
corridor still actually connects the new spawn to this water region needs
live re-verification before trusting the water-boundary handoff instructions
below as-is.

**BlueROV2 spawn reliability** — the primary spawn chain
(`spawn_x500_cargo -> spawn_entity -> spawn_bluerov2`, sequenced via
`OnProcessExit`) is backed by an independent safety net,
`spawn_bluerov2_retry.py`, which polls the world's pose stream for the boat
to exist before attempting anything, does nothing if `bluerov2` already
exists, and otherwise retries the create service up to 5 times. Gazebo's
create service no-ops harmlessly on a name collision rather than
double-spawning, so running both paths together is safe even if they
overlap.

**Water region** — the tracked vehicle rests on a supplementary
`cave_floor_patch` collision surface (`z=5.9`, `CAVE_FLOOR_Z`) added because
the vendored cave mesh's own collision has gaps in floor coverage; the flat
`ground_plane` sits far below (`z=-200`) purely as a safety net. The flooded
chamber's water surface is at `z=7.9` (a partial flood above the real floor,
not a fill to the ceiling), region `x∈[15,65] y∈[-10,10]`, centered `(40,
0)`. See `ros2_ws/src/cavex_slam_nav/worlds/cavex_world.world`'s
`cave_floor_patch`, `water_surface`, and `Buoyancy` plugin comments for the
full derivation.

**Water-boundary handoff** — end to end: build ArduSub (`cd ardupilot &&
./waf sub`, needs `--enable-DDS` at configure time and a
`microxrceddsgen` install, see "BlueROV2 / ArduSub" above), launch the
tracked vehicle (which also spawns the carried BlueROV2 and
`vehicle_switch_node.py`) **and also `tracked_vehicle_slam.launch.py`** —
`vehicle_switch_node.py`'s trigger watches `/odom_ground_truth`, which is
only published once that second launch file's
`tracked_vehicle_ground_truth_odom.py` is running. Then drive to the water
boundary (`x=15`):

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
teleop input (no autonomous exploration for the BlueROV2 this phase). This
branch does not commit a launch file that brings up the second ArduSub SITL
instance; bring it up manually:

```bash
SUB_PARM="install/ardupilot_sitl/share/ardupilot_sitl/config/default_params/sub.parm"
DDS_PARM="install/ardupilot_sitl/share/ardupilot_sitl/config/default_params/dds_udp.parm"
INST1_PARM="src/cavex_tracked_vehicle/config/dds_udp_instance1.parm"
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py command:=ardusub model:=JSON \
  instance:=1 port:=2029 micro_ros_agent_ns:=ap2 master:=tcp:127.0.0.1:5770 \
  sim_port_in:=9013 sim_port_out:=9012 defaults:="$SUB_PARM,$DDS_PARM,$INST1_PARM"
# wait for DDS init (15-25s), then confirm:
ros2 service list | grep v1   # expect /ap/v1/arm_motors, /ap/v1/mode_switch, etc.
ros2 run cavex_tracked_vehicle cmd_vel_to_ardusub.py
ros2 topic pub -r 10 /cmd_vel_rov geometry_msgs/msg/Twist "{linear: {x: 0.3}}"
```

**Deployment mechanism, helipad, and PX4 x500** — `model.sdf.tracked` adds a
static davit (deployment crane) fixture near the ROV's mount point (visual
only — the real deployment mechanism is the `DetachableJoint` plugin) and a
marked helipad at the bow. A second vendored model
(`fuel.gazebosim.org/PX4/models/x500`, CC-BY-4.0) is carried on that helipad
via its own `DetachableJoint` (`/cavex/x500_release/detach`), placeholder
scope only — no PX4 SITL/flight-control integration ("airpocket exploration"
is future work). **Known limitation, not fully resolved**: exact mount
alignment (BlueROV2 flush with the deck, x500 at the helipad surface) is not
perfectly reliable — settled offsets vary between otherwise-identical
launches (a few centimeters to several tens of centimeters), most likely
from timing variance in when each `DetachableJoint` engages relative to how
long its child free-falls first.

**Honesty caveats**: this simulation's ground truth is simulator-internal and
noiseless — treat any ATE/localization numbers as best-case/idealized, not
real-sensor-noise results. The vehicle is labeled "BlueBoat-like tracked
vehicle" throughout; it makes no official Blue Robotics/ArduPilot
endorsement claim and no marine/floating capability claim (tracks only,
ArduPilot Rover firmware, not ArduBoat).

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
sourced from Blue Robotics' published CAD). This is a vendored asset, not an
official Blue Robotics or ArduPilot product release — the tracked variant
(`model.sdf.tracked`) is a project-authored modification (motors removed,
track assemblies added) and is labeled "BlueBoat tracked-vehicle variant"
throughout; it makes no marine/floating capability claim and no Blue
Robotics endorsement.

**BlueROV2** — `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/` is
vendored, with one documented modification, based on
[clydemcqueen/bluerov2_gz](https://github.com/clydemcqueen/bluerov2_gz)'s
model structure (hull, thrusters, `ArduPilotPlugin` FDM link). The
modification: `model.sdf`'s `ArduPilotPlugin` `fdm_port_in` was changed from
the vendored default 9002 to 9012, to avoid a port collision with the
tracked vehicle's own `ArduPilotPlugin` block in the same Gazebo process.
Not an official Blue Robotics product release; labeled "BlueROV2" as a
vendored simulation asset, not a claim of hardware-accuracy beyond what that
upstream repo itself provides.

**PX4 x500 quadcopter** — `ros2_ws/src/cavex_tracked_vehicle/models/x500/`
is vendored, unmodified, from
[fuel.gazebosim.org/PX4/models/x500](https://fuel.gazebosim.org/1.0/PX4/models/x500)
(CC BY 4.0, author Benjamin Perseghetti). Model of the NXP HoverGames Drone
development kit (KIT-HGDRONEK66); carried as a placeholder for future
aerial/"airpocket" exploration work (see the Phase 1 section above) — no PX4
SITL or flight-control integration is wired up in this phase.
