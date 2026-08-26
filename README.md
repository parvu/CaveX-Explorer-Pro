# CaveX Explorer Pro

A cave-exploration robot dashboard: a React/Express web frontend, and a ROS 2
Jazzy / Gazebo Harmonic simulation stack (`ros2_ws/`). Design rationale,
verification results, and historical notes live in `history.txt`, not here —
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

Package quick reference — see `history.txt` (local-only) for full launch
sequences, driving commands, and verification steps for each:

| Package | What |
|---|---|
| `cavex_slam_nav` | Original wheeled robot, RTAB-Map SLAM, ATE eval |
| `cavex_tracked_vehicle` | Tracked BlueBoat + BlueROV2, ArduPilot Rover/Sub SITL |
| `cavex_sonar` | Simulated BlueROV2 acoustic sonar + ocean current |
| `cavex_perception` | RGB-D + lidar instance clustering |
| `cavex_gtsam_slam` | Real GTSAM Sonar-Inertial-Current factor-graph SLAM |
| `cavex_dcs` | Drift/Current Suppression controller (feed-forward + PI) |

### Phase 1: tracked BlueBoat + BlueROV2 (dry cave, ArduPilot)

```bash
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py &
sleep 25
ros2 launch cavex_tracked_vehicle tracked_vehicle_slam.launch.py &
sleep 15
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}}"
```

### Phase 2: BlueROV2 sonar + current + GTSAM-SLAM + DCS (flooded section)

```bash
gz sim -r -v2 src/cavex_slam_nav/worlds/cavex_world.world &
until gz topic -l 2>/dev/null | grep -q "/world/cavex_world/pose/info"; do sleep 1; done
gz service -s /world/cavex_world/create --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean --timeout 10000 \
  --req 'sdf_filename: "src/cavex_tracked_vehicle/models/bluerov2/model.sdf", name: "bluerov2", pose: {position: {x: 20, y: 0, z: 7.0}}'
sleep 5
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml &
sleep 10
ros2 run cavex_sonar sonar_node --ros-args -p seed:=42 -p frame_id:=bluerov2/sonar &
sleep 3
ros2 run cavex_gtsam_slam gtsam_slam_node &
ros2 run cavex_dcs dcs_controller &
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

### ATE ablation test harness (10/25-run current + turbidity sweeps)

`src/cavex_slam_nav/scripts/` has two self-contained scripts that spawn
the vehicle, run a closed-loop excitation trajectory, and compute
Absolute Trajectory Error against Gazebo ground truth, repeating until
`n_target` valid (non-discarded) runs complete. Each attempt is a fully
fresh process restart (gz sim, bridge, sonar, gtsam_slam_node); a run is
discarded and retried (not counted) on node-stacking or an implausible
solution jump, so the reported n is always n genuinely valid runs. Output
(per-run logs + `ate_Nx_<label>_results.csv`) goes to `$ATE_OUT_DIR`
(default `/tmp/cavex_ate_results`).

```bash
# Zero-current baseline: with-CurrentFactor vs without, 10 or 25 runs/leg
src/cavex_slam_nav/scripts/run_ate.sh with    10 baseline_with
src/cavex_slam_nav/scripts/run_ate.sh without 10 baseline_without
src/cavex_slam_nav/scripts/run_ate.sh with    25 baseline_with25
src/cavex_slam_nav/scripts/run_ate.sh without 25 baseline_without25

# Real current + turbidity: <with|without> <n> <label> [current_vx m/s] [absorption_db_per_m] [dcs]
#   current_vx: ocean current speed (default 2.0 m/s)
#   absorption_db_per_m: turbidity proxy -- higher = murkier water, shorter
#     sonar range (default 0.4 = no turbidity; 3.0+ = heavy turbidity)
#   trailing "dcs" enables Dynamic Covariance Scaling (robust M-estimator)
src/cavex_slam_nav/scripts/run_ate_current.sh without 10 current_test 2.0 0.4
src/cavex_slam_nav/scripts/run_ate_current.sh without 10 current_test_dcs 2.0 0.4 dcs
src/cavex_slam_nav/scripts/run_ate_current.sh with    25 heavy_turbidity25 2.0 3.0
```

## Known limitations (sim-to-real gap)

**The acoustic sonar model is not calibrated against real hardware.**
`cavex_sonar`'s transmission-loss, backscatter, and speckle parameters
(`sonar_acoustics.hpp`'s `AcousticParams`) are physically motivated
(spherical spreading + absorption, Lambertian backscatter, Rayleigh
speckle) but their specific values were chosen to be plausible for a
~1 MHz short-range imaging sonar in fresh water, not fit to any real
sensor's measured response.

**No multi-path/reverberation modeling.** Every simulated beam is a
single direct ray-cast (one range, one echo level) — there is no
secondary-reflection or reverberation physics. In a confined, high-clutter
underwater space this is a real simplification: a real sonar in that kind
of environment sees genuine multi-path returns that a single-bounce model
cannot produce, and that could degrade real scan registration in ways this
simulation cannot currently exercise or catch.

**Partially addressed:** `sonar_node`'s `clutter_probability` parameter
(default `0.0`, off) injects spurious short-range false detections on
any beam that would otherwise report a real dropout (including a beam
with zero valid rays at all, the most realistic case) — a first-order
model of turbidity-driven false returns (suspended-particle scattering),
not true multi-path physics. Clutter range also drifts with the real
current (`current_drift_range_m` in `AcousticParams`, driven by
`sonar_node`'s subscription to `/ocean_current`): beams looking upstream
get closer clutter over time, downstream beams get farther, both
saturating at `clutter_max_range_m`/`min_range_m`. `sonar_node` also
populates `LaserScan`'s standard `time_increment`/`scan_time` fields
(`scan_duration_s` parameter, default 1.0s) — informational only, since
the underlying `gpu_lidar` source samples every ray at one simulated
instant, so there's no real per-beam motion distortion in the data to
correct for; this just gives a consumer the field a real sensor driver
would provide.

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
