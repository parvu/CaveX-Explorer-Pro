# Phase 1 (revised): Tracked BlueBoat-like Vehicle, ArduPilot-Controlled

## Context

This replaces the earlier Phase 1 design (`2026-08-04-cavex-legged-walker-phase1-design.md`, implemented on the `cavex-legged-walker-phase1` worktree branch). That branch built a CHAMP-driven Spot-like quadruped through Tasks 1-8, but implementation uncovered a genuine, unresolved legged-locomotion balance/stance problem: the walker has no working balance control and tips over on open ground (it only appeared to stand because it was spawned wedged against a world collision box). Chasing this further requires specialist CHAMP inverse-kinematics debugging, out of proportion to what this phase needs. That branch is left as-is (not deleted, not merged) in case anything on it (the RTAB-Map 3D lidar SLAM wiring, the Nav2 costmap-only bringup pattern) is worth reusing later.

This revised design keeps everything about the old Phase 1 scope that isn't about the vehicle body/locomotion/control: the dry-cave section of `cavex_world.world`, RTAB-Map 3D lidar SLAM, Nav2 frontier exploration via `explore_lite`, ATE evaluation, and the live dashboard. It replaces the vehicle itself: instead of a legged quadruped driven by CHAMP/ros2_control, this is a tracked ground vehicle built on a BlueBoat-like hull, driven by real continuous tracks, controlled by a real ArduPilot Rover SITL instance.

No real, official BlueBoat Gazebo/SDF model exists anywhere (checked Gazebo Fuel and GitHub) — same situation as the earlier "no real Spot model" constraint. The hull is built from primitives sized to Blue Robotics' own published spec, labeled "BlueBoat-like," never a claim of running an official model. BlueBoat is a real twin-pontoon catamaran (two 120cm LDPE hulls, ~93cm apart, joined by a crossmember frame, one motor per hull for differential thrust) — reused here as: two elongated hull boxes standing in for the pontoons, a crossmember deck box joining them, and one retractable track assembly mounted under each pontoon (one per side, matching the real per-hull motor layout).

## Goals

- A tracked ground vehicle (BlueBoat-like twin-pontoon hull + two continuous tracks) that can actually move and stay upright in the dry-cave section, unlike the abandoned legged walker.
- Each track assembly can retract (fold up against its pontoon) via a real, independently commandable joint — the mechanical capability only. What triggers retraction (e.g. water detection) is Phase 2's concern (see non-goals); this phase builds and verifies the joint itself, commandable manually/by a topic, not by any water-sensing logic that doesn't exist yet.
- A more detailed vehicle model than the abandoned walker's simple boxes-and-cylinders: real twin-pontoon hull proportions, distinguishable motor/track-drive housings, and sensor mounts placed the way BlueBoat's real payload area (the flat deck between pontoons) is described in its own spec — still primitive-geometry, not textured meshes, but composed to actually read as the vehicle it's modeling rather than an abstract box.
- Real ArduPilot Rover SITL in the loop as the vehicle's control law and safety envelope (arming, GUIDED mode, skid-steer mixing) — not a stub, not bypassed.
- Real continuous-track physics via Gazebo Harmonic's own `gz-sim-tracked-vehicle-system`/`track-controller-system` plugins (already installed in this environment) — not wheels styled to look like tracks.
- Nav2 frontier-exploration-driven autonomous mapping of the dry-cave section, same as the original Phase 1 goal, now actually achievable because the vehicle can move.
- Real 3D RTAB-Map SLAM (reused from the abandoned branch's approach) and ATE evaluation against simulator ground truth.
- Live 3D React dashboard extended for this vehicle, same honesty conventions as the rest of the project (labeled "BlueBoat-like tracked vehicle," ArduPilot's real involvement, no invented sensors).

## Explicit non-goals (this phase)

- No flooded-section / water physics / buoyancy simulation — the tracked vehicle stays on the dry-cave floor. (Real BlueBoat buoyancy/hull-in-water physics is a distinct, later concern if this vehicle is ever extended toward the flooded section — not attempted here.)
- No water-detection/auto-retraction logic — the retraction joint is built and independently testable this phase, but nothing in this phase decides *when* to retract based on terrain/water. That decision logic belongs to Phase 2, once there's actually water to detect.
- No BlueROV2 tether, no PX4 flying drone — unchanged from the original Phase 2/3 split, still out of scope.
- No claim that the hull is an official Blue Robotics simulation model — it's dimensionally BlueBoat-like, built from primitives.
- No claim that this vehicle floats or is amphibious in this phase — "BlueBoat-like" describes the hull shape/dimensions reused as a chassis, not real marine capability; it's fitted with tracks and driven on dry ground only.

## Architecture

New ROS2 package `cavex_tracked_vehicle`, parallel to the existing `cavex_slam_nav` (which is untouched — it still serves the original wheeled `cavex_robot`). Reuses `cavex_world.world` (same dry-cave section, same Fuel obstacles from the abandoned branch's Task 6 if that world file is still current on `main`).

```
Nav2 / explore_lite
      |  /cmd_vel (geometry_msgs/Twist)
      v
cmd_vel_to_ardupilot adapter node (new)
      |  /ap/cmd_vel (TwistStamped, GUIDED mode, arms on first goal)
      v
ArduPilot Rover SITL (real process, skid-steer frame)
      |  AP_DDS "cmd_vel" topic (TwistStamped, ArduPilot's own control-law output)
      v
track_cmd_vel bridge (remap/republish, new — thin)
      |  /track_cmd_vel (gz.msgs.Twist, via ros_gz_bridge)
      v
gz-sim track-controller-system -> gz-sim-tracked-vehicle-system (real Gazebo Harmonic plugins)
      |
      v
Real track-belt physics on the BlueBoat-like hull -> real motion
      |
      v
lidar/camera/IMU sensors -> RTAB-Map 3D SLAM -> /map, map->odom->base_link TF
      |
      v
Nav2 costmap (closes the loop back to the top)
```

ATE evaluation taps `/odom_ground_truth` (gz-sim PosePublisher on the vehicle model, same pattern as the abandoned branch's Task 10 design — simulator-internal, noiseless ground truth, not a claim about real-hardware sensing) against RTAB-Map's estimate, same harness (`ate_evaluator_node.py`, `ate_metrics.py`, `cavex_ate_runs.csv`) already proven on the wheeled robot.

## Components

1. **`cavex_tracked_vehicle.urdf.xacro`** — twin-pontoon BlueBoat-like hull (two elongated hull boxes + a crossmember deck box joining them, dimensions from Blue Robotics' published spec) plus two track assemblies (`left_track`, `right_track`), each mounted under its own pontoon via a **retraction joint** (`left_track_retract_joint`/`right_track_retract_joint`, revolute, hinging the track assembly up against the pontoon — a real, independently ros2_control-or-equivalent-commandable joint, not baked into the track plugin itself) and carrying the real geometry a `gz-sim-tracked-vehicle-system` element expects below that joint (verify the exact SDF schema — link names, contact surface parameters — empirically against the plugin before assuming, same "verify don't guess" discipline used throughout this project). Same sensor set as the abandoned walker branch: 3D lidar (`gpu_lidar`), camera, IMU, all with `<gz_frame_id>` set (a real gotcha this project hit before), mounted on the deck box between the pontoons (matching BlueBoat's real flat payload area).

2. **Track retraction control** — the retraction joints are actuated joints, driven the same way any other Gazebo Harmonic joint in this project has been (a small `gz-sim-joint-position-controller-system` or equivalent, commanded via a topic/service — exact real mechanism to be verified against the installed plugin, not assumed). Exposed as a simple manual command this phase (e.g. a topic accepting "deployed"/"retracted"), with no automatic trigger — see non-goals.

3. **ArduPilot SITL bring-up** — real `ardupilot_sitl` ROS2 package, `sitl_dds_udp.launch.py` pattern (per ArduPilot's own documented ROS2-with-Gazebo setup), Rover firmware, skid-steer frame config (`SERVO1_FUNCTION`/`SERVO3_FUNCTION` = throttle-left/throttle-right, ArduPilot's real, documented skid-steer parameters). Runs as a real external process, not embedded in a ROS2 node.

4. **`micro-ROS-Agent`** — the real DDS bridge ArduPilot's documentation specifies, translating ArduPilot's internal DDS topics (`/ap/...` namespace) to standard ROS2 topics.

5. **`cmd_vel_to_ardupilot.py`** (new node) — subscribes to the project's standard `/cmd_vel` (from Nav2/explore_lite, unchanged from the rest of this project), republishes as ArduPilot's real `/ap/cmd_vel` (`TwistStamped`, confirmed real topic via ArduPilot's own `AP_DDS_Topic_Table.h`). Also responsible for arming the vehicle and setting GUIDED mode via ArduPilot's real service/topic interface (exact service names to be verified empirically at implementation time, not guessed here).

6. **`track_cmd_vel_bridge`** (new, thin) — subscribes to ArduPilot's own DDS `cmd_vel` output topic (ArduPilot's control-law output, i.e. `twist/filtered` or the equivalent real output topic — verify exact name empirically), republishes to `/track_cmd_vel` for the `gz-sim-track-controller-system` to consume. This is the one genuinely new, unproven piece of wiring in this design (see Risks) — no existing reference config combines ArduPilot's DDS output with the track-controller plugin, so this bridge's exact shape depends on what's empirically verified in the first implementation task.

7. **Ground-truth odometry republisher** — same `PoseArray`→`Odometry` pattern as the abandoned branch's design (gz-sim `PosePublisher` system, `publish_model_pose=true`, bridged as `geometry_msgs/PoseArray`, republished as `nav_msgs/Odometry` for ATE).

8. **RTAB-Map 3D SLAM, Nav2 bringup (costmap-only, no AMCL/map_server), explore_lite** — same real config approach the abandoned branch already worked out and verified (icp_odometry frame-to-frame ICP since there's no wheel/track odometry source feeding RTAB-Map directly; costmap-only Nav2 params since RTAB-Map owns SLAM). Ported into the new package rather than assumed to carry over unmodified — frame IDs, topic remaps, and the `navigation_launch.py` lifecycle-manager quirk (it hard-codes `collision_monitor`/`docking_server` as managed nodes even when unused) all need to be re-verified against this vehicle's actual TF tree and sensor topics.

9. **Dashboard extension** — same live-telemetry-polling pattern already proven (`web_telemetry_bridge.py` → `/api/telemetry` → React polling), extended with this vehicle's position/heading, frontier count, ATE RMSE, and an honest "BlueBoat-like tracked vehicle, ArduPilot Rover SITL" label (never implying an official model or claiming buoyancy/marine capability).

## Data flow

Described in the architecture diagram above. In short: Nav2/explore_lite goals flow through ArduPilot (a real control loop, not a pass-through) before reaching the tracks, and sensor data flows back up through RTAB-Map to close the SLAM/costmap loop — the same two-way structure the wheeled robot and the abandoned walker both used, with ArduPilot inserted as a real intermediary on the outbound side.

## Error handling / risks

- **The ArduPilot-DDS-output → track-controller-input bridge is genuinely new and unproven** — no existing reference (ArduPilot's own `r1_rover` example drives wheels via raw PWM→`ApplyJointForce`, not via the track plugin's Twist interface). The first implementation task must be a minimal spin-up-and-verify step: launch SITL + Gazebo + the track plugins together, confirm real topics/types on both sides, and only then build the bridge node — before committing to any larger wiring. If this bridge turns out not to work cleanly, the fallback is the PWM→`ApplyJointForce` pattern from `r1_rover`, mirrored for two track-drive joints instead of four wheel joints (documented as the explicitly-considered, deliberately-not-chosen alternative during brainstorming — worth revisiting if the DDS path proves unworkable rather than silently abandoning "real tracks").
- **Three real external processes now in the loop** (Gazebo, ArduPilot SITL, micro-ROS-Agent) instead of CHAMP's single control node — more startup-ordering and process-hygiene risk. This project has already hit severe flakiness from orphaned/duplicate Gazebo processes surviving across test launches; the same explicit-PID-kill discipline (not `pkill -f`, which has been unreliable in this environment) applies here, more so.
- **Track-controller SDF schema is not yet verified against the plugin's real expectations** (link naming, contact/friction parameters) — flagged rather than guessed; first real task should read the plugin's actual parameter parsing (via `strings` on the installed `.so`, or its SDF `<param_v>`/config schema, same technique used to find the real `gz_ros2_control` gain parameter earlier in this project) before writing the URDF/SDF.

## Testing / verification

- Same "verify empirically, never assume a topic/type/parameter name" discipline established throughout this project.
- Minimum bar before declaring the vehicle "working": drive it via a manually-armed, manually-commanded `/ap/cmd_vel` and observe real net translation in Gazebo (the exact failure mode that blocked the abandoned legged walker) — this is the first concrete go/no-go checkpoint, before any SLAM/Nav2/exploration work is layered on top.
- Retraction joints verified independently: command deployed→retracted→deployed and confirm real joint-state transitions in Gazebo, and confirm the vehicle still drives correctly in both positions (or is deliberately prevented from driving while retracted, if that's the more realistic behavior — decide and document at implementation time, don't leave it ambiguous).
- ATE evaluation reused unmodified in methodology (≥10 runs, sim-time-gated, curated for buffer-contamination the way this project has done twice already) once the vehicle can move and RTAB-Map has real trajectory data to score.

## Out-of-scope future phases (unchanged from the original design)

- **Phase 2**: flooded-section physics, real BlueBoat buoyancy, tethered BlueROV2, sonar, water-current modeling (with real, not fabricated, SLAM+current-estimation techniques — never "SIC-SLAM"/"CurrentFactor" unless genuinely real and cited).
- **Phase 3**: air-shaft section, detachable PX4-controlled flying drone.
