# Phase 1 (revised again): Tracked BlueBoat + Water-Triggered BlueROV2, ArduPilot-Controlled

## Context

This replaces the earlier Phase 1 design (`2026-08-04-cavex-legged-walker-phase1-design.md`, implemented on the `cavex-legged-walker-phase1` worktree branch). That branch built a CHAMP-driven Spot-like quadruped through Tasks 1-8, but implementation uncovered a genuine, unresolved legged-locomotion balance/stance problem: the walker has no working balance control and tips over on open ground (it only appeared to stand because it was spawned wedged against a world collision box). Chasing this further requires specialist CHAMP inverse-kinematics debugging, out of proportion to what this phase needs. That branch is left as-is (not deleted, not merged) in case anything on it (the RTAB-Map 3D lidar SLAM wiring, the Nav2 costmap-only bringup pattern) is worth reusing later.

This revised design keeps everything about the old Phase 1 scope that isn't about the vehicle body/locomotion/control: the dry-cave section of `cavex_world.world`, RTAB-Map 3D lidar SLAM, Nav2 frontier exploration via `explore_lite`, ATE evaluation, and the live dashboard. It replaces the vehicle itself: instead of a legged quadruped driven by CHAMP/ros2_control, this is a tracked ground vehicle built on a BlueBoat hull, driven by real continuous tracks, controlled by a real ArduPilot Rover SITL instance.

**Second revision (this pass):** the original version of this doc claimed no real, official BlueBoat Gazebo/SDF model exists. That was wrong in the way that mattered — ArduPilot's own `SITL_Models` repo (referenced via `markusbuchholz/gazebosim_blueboat_ardupilot_sitl`, the mirror the human partner pointed at) ships a real `blueboat` Gazebo model: author Rhys Mainwaring, meshes sourced directly from Blue Robotics' own published CAD files (`cad.bluerobotics.com/BLUEBOAT_120_BR-101447_RevA_PUB.zip`), real per-hull `Thruster`/IMU/NavSat plugins. It's still not a Blue Robotics-published asset, but it's a real, ArduPilot-maintained model, not primitives approximating a spec sheet. This phase now vendors that model directly and adds the retractable-track assemblies onto it, rather than building a boxy hull from scratch.

**Third revision (this pass, cave geometry):** `cavex_world.world`'s "cave" was never a cave — it was two hand-authored placeholder boxes (`dry_cave`, a 34×12×2 box; `flooded_cave_water`, a 34×12×0.1 slab) standing in as zone markers. This revision replaces them with a real cave: the static Wavefront OBJ mesh from `LTU-RAI/gazebo_cave_world` (MIT licensed), re-wrapped as a Gazebo Harmonic SDF 1.9 static model. Only the geometry changes — the vehicle/control/SLAM architecture below is untouched; the dry-section, transition and water-section coordinates are simply re-anchored to the real mesh's bounds. See Components 15 and the provenance note under Error handling / risks.

This revision also folds in what was previously deferred to "Phase 2": a flooded section of the cave with real buoyancy physics, and a second vehicle — a BlueROV2 (`clydemcqueen/bluerov2_gz`, real Gazebo Harmonic `Buoyancy`/`Hydrodynamics`/`Thruster` plugins, driven by a second, separate ArduSub SITL instance) — that the sim switches to automatically when the tracked vehicle crosses into the water region, and switches back from on exit. This is a genuinely large scope increase over the original tracked-only design; see Goals/Architecture below for what it actually requires.

## Goals

- **Real cave geometry**: the dry and flooded sections are named sub-regions of one real, vendored, textured cave mesh (`LTU-RAI/gazebo_cave_world`, MIT, cited), not the hand-authored placeholder boxes `cavex_world.world` shipped with.
- A tracked ground vehicle (real vendored `blueboat` hull + two continuous tracks) that can actually move and stay upright in the dry-cave section, unlike the abandoned legged walker.
- Each track assembly can retract (fold up against the hull) via a real, independently commandable joint.
- A real, detailed vehicle model: the vendored `blueboat` Gazebo model (real Collada meshes: hull, crosstubes, hatches, motor housings) with the retractable-track assemblies added onto it as new links/joints, replacing its two real thruster/prop links (the tracked variant doesn't need BlueBoat's real props — see Components).
- Real ArduPilot Rover SITL in the loop as the tracked vehicle's control law and safety envelope (arming, GUIDED mode, skid-steer mixing) — not a stub, not bypassed.
- Real continuous-track physics via Gazebo Harmonic's own `gz-sim-tracked-vehicle-system`/`track-controller-system` plugins (already installed in this environment) — not wheels styled to look like tracks.
- **A flooded section of the cave world with real buoyancy physics** (Gazebo Harmonic's own `Buoyancy`/`Hydrodynamics` plugins, same ones `bluerov2_gz` uses), separate from the dry-cave section the tracked vehicle explores.
- **A real BlueROV2** (vendored `clydemcqueen/bluerov2_gz` model, real `Buoyancy`/`Hydrodynamics`/`Thruster` plugins), controlled by a second, separate ArduSub SITL instance (real ArduPilot firmware, `-v ArduSub -f vectored`).
- **A water-boundary trigger**: when the tracked vehicle's pose crosses into the water region, retract its tracks, park/despawn it, and spawn+arm the BlueROV2 at that position; reverse the handoff on the way back out. This is the one piece of "automatic" logic in the whole design — deliberately scoped to just the vehicle switch, not a general water-detection sensor model.
- Nav2 frontier-exploration-driven autonomous mapping of the dry-cave section (tracked vehicle), same as the original Phase 1 goal, now actually achievable because the vehicle can move.
- Real 3D RTAB-Map SLAM (reused from the abandoned branch's approach) and ATE evaluation against simulator ground truth, for the tracked vehicle's dry-cave run.
- Live 3D React dashboard extended for both vehicles, same honesty conventions as the rest of the project (labeled "BlueBoat tracked-vehicle variant" / "BlueROV2 (ArduSub SITL)," ArduPilot's real involvement, no invented sensors, current vehicle-in-control clearly indicated).

## Explicit non-goals (this phase)

- No PX4 flying drone, no air-shaft section — unchanged, still Phase 3.
- No full Nav2/RTAB-Map autonomy loop underwater for the BlueROV2 — the water-triggered handoff gets the BlueROV2 spawned, armed, and drivable (manual `/cmd_vel`-equivalent teleop via ArduSub GUIDED, same adapter-node pattern as the tracked vehicle), but building a second full SLAM/exploration stack for an underwater vehicle is out of scope this pass. Verification for the BlueROV2 side stops at "it's real, it's armed, it moves correctly under buoyancy."
- No sonar, no water-current modeling, no "SIC-SLAM"/current-compensation claims — unchanged from the original non-goal, still real, cited techniques only, still not attempted.
- No claim that the tracked hull is an official Blue Robotics *simulation* product — it's the real ArduPilot-maintained `blueboat` model (see Context), used as-is plus our own added track assemblies; it is not something Blue Robotics ships or endorses.
- No adoption of `gazebo_cave_world`'s prop models — the upstream repo also ships backpack, extinguisher, survivor, jersey_barrier, tunnel_entrance and AprilTag models (its DARPA SubT artifact set). This phase vendors the cave geometry only; the props are explicitly out of scope. Obstacles in the dry section stay the Fuel-sourced rocks already planned, not SubT artifacts.
- No claim that the cave mesh is this project's work — it is vendored, unmodified, MIT-licensed third-party geometry (see the provenance note under Error handling / risks).
- No claim that the tracked vehicle itself floats or is amphibious — it never enters the water region under its own control; the water-boundary trigger despawns/parks it and hands off to the BlueROV2 instead. "BlueBoat" here names the real vendored hull model, not a claim about this vehicle's real-world capability.

## Architecture

New ROS2 package `cavex_tracked_vehicle`, parallel to the existing `cavex_slam_nav` (which is untouched — it still serves the original wheeled `cavex_robot`). Reuses `cavex_world.world`, whose placeholder `dry_cave`/`flooded_cave_water` boxes are replaced by the real vendored LTU-RAI cave mesh (Component 15), extended with Fuel-sourced obstacles and a flooded region.

The mesh is placed so its ~150 m long axis lies along world X, it is centred on the origin in X/Y, and its lowest vertex sits at `z = 0`, giving world-frame bounds `x ∈ [-75, +75]`, `y ∈ [-51, +51]`, `z ∈ [0, +33]`. Everything else's coordinates derive from that: the **dry section** the tracked vehicle explores is `x ∈ [-70, 0]`, `y ∈ [-45, +45]`; a **transition passage** occupies `x ∈ [0, +10]`; the **water section** is `x ∈ [+10, +70]`, `y ∈ [-45, +45]` with its surface and buoyancy interface at `z = 4.0`. The water-boundary trigger fires at `x = +10`. These are sub-regions of one continuous real cave, not separate boxes — the two "sections" are named volumes within the mesh, and the tunnel's actual walkable floor inside them is confirmed empirically (drop a probe body, see where it rests) rather than assumed flat.

**Tracked-vehicle control path (dry cave, unchanged from the first revision):**

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
Real track-belt physics on the vendored blueboat hull -> real motion
      |
      v
lidar/camera/IMU sensors -> RTAB-Map 3D SLAM -> /map, map->odom->base_link TF
      |
      v
Nav2 costmap (closes the loop back to the top)
```

**Water-boundary handoff (new this revision):**

```
Tracked vehicle's ground-truth pose (existing /odom_ground_truth tap)
      v
vehicle_switch_node (new): pose.z crosses the water-region boundary?
      |
      +-- entering water: retract tracks, freeze/despawn tracked vehicle,
      |   spawn "bluerov2" at that pose, launch its ArduSub SITL instance,
      |   arm it in GUIDED/DEPTH_HOLD
      |
      +-- exiting water (BlueROV2 crosses back out): disarm/despawn
          BlueROV2, respawn tracked vehicle at that pose, deploy tracks
```

**BlueROV2 control path (water region, new this revision):**

```
teleop / manual /cmd_vel_rov (geometry_msgs/Twist)
      v
cmd_vel_to_ardusub adapter node (new, same pattern as cmd_vel_to_ardupilot)
      |  /ap2/cmd_vel (TwistStamped, second AP_DDS instance, distinct ROS2 namespace)
      v
ArduSub SITL (real, second process, vectored frame)
      |
      v
gz-sim Thruster plugins (real, from the vendored bluerov2 model) -> Buoyancy/Hydrodynamics
      |
      v
Real underwater motion under buoyancy
```

Two ArduPilot SITL instances run concurrently but never control the same vehicle at once — the water-boundary handoff (de)spawns the two Gazebo models so only one is ever present/armed. Each SITL instance uses a distinct UDP port pair and a distinct `micro-ROS-Agent`/DDS namespace (real ArduPilot support — `--instance` on `sim_vehicle.py`, and per-instance ports are ArduPilot's own documented multi-vehicle mechanism) to avoid port collisions.

ATE evaluation taps `/odom_ground_truth` (gz-sim PosePublisher on the vehicle model, same pattern as the abandoned branch's Task 10 design — simulator-internal, noiseless ground truth, not a claim about real-hardware sensing) against RTAB-Map's estimate, same harness (`ate_evaluator_node.py`, `ate_metrics.py`, `cavex_ate_runs.csv`) already proven on the wheeled robot. This is scored for the tracked vehicle's dry-cave run only, per the non-goals above (no full SLAM stack for the BlueROV2 this phase).

## Components

1. **Vendored `blueboat` model + `cavex_tracked_vehicle.urdf.xacro` overlay** — the real `blueboat` SDF/meshes vendored from ArduPilot's `SITL_Models` repo (`base_link` with the real hull/crosstube/hatch/frame Collada meshes, `imu_link`, `navsat_link`), with its two real `motor_port_link`/`motor_stbd_link`/prop assemblies **removed** (a tracked vehicle doesn't drive props) and replaced with two new track assemblies (`left_track`, `right_track`) mounted where the motors were, each behind a **retraction joint** (`left_track_retract_joint`/`right_track_retract_joint`, revolute, hinging the track up against the hull — real, independently ros2_control-commandable). Same sensor set as the abandoned walker branch: 3D lidar (`gpu_lidar`), camera, all with `<gz_frame_id>` set (a real gotcha this project hit before) — the vendored model's own real `imu_link`/`navsat_link` are reused directly rather than re-added from scratch.

2. **Track retraction control** — the retraction joints are actuated joints, driven the same way any other Gazebo Harmonic joint in this project has been (`ros2_control`/`JointTrajectoryController`, the exact real pattern already proven on the abandoned walker branch). Exposed as a simple manual command this phase (e.g. a topic accepting "deployed"/"retracted"), plus the water-boundary trigger node (Component 10) issuing the same command automatically at the water crossing.

3. **ArduPilot Rover SITL bring-up** — real `ardupilot_sitl` ROS2 package, `sitl_dds_udp.launch.py` pattern (per ArduPilot's own documented ROS2-with-Gazebo setup), Rover firmware, skid-steer frame config (`SERVO1_FUNCTION`/`SERVO3_FUNCTION` = throttle-left/throttle-right, ArduPilot's real, documented skid-steer parameters). Runs as a real external process, not embedded in a ROS2 node.

4. **`micro-ROS-Agent`** — the real DDS bridge ArduPilot's documentation specifies, translating ArduPilot's internal DDS topics (`/ap/...` namespace) to standard ROS2 topics.

5. **`cmd_vel_to_ardupilot.py`** (new node) — subscribes to the project's standard `/cmd_vel` (from Nav2/explore_lite, unchanged from the rest of this project), republishes as ArduPilot's real `/ap/cmd_vel` (`TwistStamped`, confirmed real topic via ArduPilot's own `AP_DDS_Topic_Table.h`). Also responsible for arming the vehicle and setting GUIDED mode via ArduPilot's real service/topic interface (exact service names to be verified empirically at implementation time, not guessed here).

6. **`track_cmd_vel_bridge`** (new, thin) — subscribes to ArduPilot's own DDS `cmd_vel` output topic (ArduPilot's control-law output, i.e. `twist/filtered` or the equivalent real output topic — verify exact name empirically), republishes to `/track_cmd_vel` for the `gz-sim-track-controller-system` to consume. This is the one genuinely new, unproven piece of wiring in the tracked-vehicle side of this design (see Risks) — no existing reference config combines ArduPilot's DDS output with the track-controller plugin, so this bridge's exact shape depends on what's empirically verified in the first implementation task.

7. **Ground-truth odometry republisher** — same `PoseArray`→`Odometry` pattern as the abandoned branch's design (gz-sim `PosePublisher` system, `publish_model_pose=true`, bridged as `geometry_msgs/PoseArray`, republished as `nav_msgs/Odometry` for ATE).

8. **RTAB-Map 3D SLAM, Nav2 bringup (costmap-only, no AMCL/map_server), explore_lite** — same real config approach the abandoned branch already worked out and verified (icp_odometry frame-to-frame ICP since there's no wheel/track odometry source feeding RTAB-Map directly; costmap-only Nav2 params since RTAB-Map owns SLAM). Ported into the new package rather than assumed to carry over unmodified — frame IDs, topic remaps, and the `navigation_launch.py` lifecycle-manager quirk (it hard-codes `collision_monitor`/`docking_server` as managed nodes even when unused) all need to be re-verified against this vehicle's actual TF tree and sensor topics.

9. **Dashboard extension** — same live-telemetry-polling pattern already proven (`web_telemetry_bridge.py` → `/api/telemetry` → React polling), extended with both vehicles' position/heading, frontier count, ATE RMSE (tracked vehicle only), which vehicle currently holds control, and honest labels ("BlueBoat tracked-vehicle variant, ArduPilot Rover SITL" / "BlueROV2, ArduSub SITL" — never implying an official Blue Robotics product or unverified capability).

10. **Water region + buoyancy** (new) — a flooded sub-volume of `cavex_world.world` (a real water-surface plane at `z = 4.0` spanning `x ∈ [+10, +70]`, `y ∈ [-45, +45]`, plus a `Buoyancy` plugin region with its interface at the same `z`, the same real Gazebo Harmonic plugin `bluerov2_gz` uses), occupying the far half of the real cave mesh, reached from the dry section through the `x ∈ [0, +10]` transition passage the tracked vehicle can approach but not enter under its own power (see non-goals).

11. **Vendored `bluerov2` model** (new) — the real `bluerov2` SDF/meshes from `clydemcqueen/bluerov2_gz`, used as-is (its real `Buoyancy`/`Hydrodynamics`/`Thruster` plugins are exactly what a floating/diving vehicle needs, no modification required the way the tracked-vehicle overlay needed one).

12. **Second ArduPilot SITL instance (ArduSub)** — real ArduPilot firmware built for `ArduSub` (`./waf sub`, alongside Task 2's already-built `ardurover`), run with `-f vectored` (BlueROV2 base config, matching the vendored model), on a distinct instance/port set from the Rover SITL so both can run concurrently without colliding.

13. **`cmd_vel_to_ardusub.py`** (new node) — same shape as Component 5, aimed at the second SITL instance's `/ap2/cmd_vel`-equivalent namespace, real ArduSub GUIDED/DEPTH_HOLD mode switching (real ArduSub mode enum, verified empirically at implementation time, not guessed).

14. **`vehicle_switch_node.py`** (new) — subscribes to the tracked vehicle's ground-truth pose (Component 7) and the BlueROV2's, watches for crossing the water region's real geometric boundary (Component 10), and on each crossing: commands track retraction (Component 2), despawns/parks the outgoing vehicle (real `gz service` model-removal call, or a "parked" pose far outside both operating areas if removal proves unreliable — decided empirically at implementation time), and spawns+arms the incoming one at the crossing pose. This is the one place in the whole design where "automatic" logic exists — deliberately narrow (a single boundary-crossing state machine), not a general perception-based water-detection sensor.

15. **Vendored `cave_world` mesh** (new) — the real cave geometry, replacing the hand-authored placeholder boxes. A static Wavefront OBJ (`cave_world.obj`, ~24 MB, plus its `.mtl` and PNG textures) from `LTU-RAI/gazebo_cave_world`, real external dimensions 150 m (length) × 102 m (width) × 33 m (height). Upstream it is a ROS1/Gazebo-Classic package, so it is not directly usable: this phase extracts the mesh + materials + textures and re-wraps them in a Gazebo Harmonic SDF 1.9 `<model>` with `<static>true</static>` and `<mesh><uri>` pointing at the OBJ. gz-sim/ogre2 loads OBJ meshes directly, but this project has not done it before, so the implementation task includes a real load-and-collide verification (no parse/mesh-not-found errors, and a dropped probe body comes to rest on the mesh floor rather than falling through). The same OBJ serves as both `<visual>` and `<collision>` geometry — the model is static, so ODE only ever evaluates static-trimesh-vs-vehicle contacts, and the collision surface then matches exactly what the lidar sees, which matters because RTAB-Map's map is scored against ground truth. Real-time factor is measured and recorded at that verification step rather than assumed. **Vendored third-party work, MIT licensed — see the provenance note below.**

## Data flow

Described in the architecture diagram above. In short: Nav2/explore_lite goals flow through ArduPilot (a real control loop, not a pass-through) before reaching the tracks, and sensor data flows back up through RTAB-Map to close the SLAM/costmap loop — the same two-way structure the wheeled robot and the abandoned walker both used, with ArduPilot inserted as a real intermediary on the outbound side.

## Error handling / risks

- **The ArduPilot-DDS-output → track-controller-input bridge is genuinely new and unproven** — no existing reference (ArduPilot's own `r1_rover` example drives wheels via raw PWM→`ApplyJointForce`, not via the track plugin's Twist interface). The first implementation task must be a minimal spin-up-and-verify step: launch SITL + Gazebo + the track plugins together, confirm real topics/types on both sides, and only then build the bridge node — before committing to any larger wiring. If this bridge turns out not to work cleanly, the fallback is the PWM→`ApplyJointForce` pattern from `r1_rover`, mirrored for two track-drive joints instead of four wheel joints (documented as the explicitly-considered, deliberately-not-chosen alternative during brainstorming — worth revisiting if the DDS path proves unworkable rather than silently abandoning "real tracks").
- **Two concurrent SITL instances is new territory for this project** — port/namespace collision is the main real risk (mitigated by ArduPilot's own documented `--instance`/port-offset mechanism, verified empirically rather than assumed). The `vehicle_switch_node` must never have both vehicles armed/live at once; if the handoff logic has any ordering bug, the safe failure mode is "both parked, nothing moving," never "both armed."
- **Model removal/respawn mid-simulation is not something this project has done before** — Gazebo's real `gz service` model-removal call is the intended mechanism, but if it proves unreliable (leaves orphaned links, breaks TF), the documented fallback is parking the outgoing vehicle at a fixed off-map pose instead of true removal — decide and document which one actually works at implementation time, don't assume removal succeeds cleanly.
- **Four real external processes now in the loop** (Gazebo, two ArduPilot SITL instances, micro-ROS-Agent(s)) instead of CHAMP's single control node — significant startup-ordering and process-hygiene risk. This project has already hit severe flakiness from orphaned/duplicate Gazebo processes surviving across test launches; the same explicit-PID-kill discipline (not `pkill -f`, which has been unreliable in this environment) applies here, more so.
- **Track-controller SDF schema is not yet verified against the plugin's real expectations** (link naming, contact/friction parameters) — flagged rather than guessed; first real task should read the plugin's actual parameter parsing (via `strings` on the installed `.so`, or its SDF `<param_v>`/config schema, same technique used to find the real `gz_ros2_control` gain parameter earlier in this project) before writing the URDF/SDF.
- **The cave mesh is vendored third-party work and must be attributed in the repo, not just in these docs** — same provenance discipline this design already applies to the `blueboat` hull (ArduPilot's `SITL_Models`, author Rhys Mainwaring, not a Blue Robotics product) and the `bluerov2` model (`clydemcqueen/bluerov2_gz`). The cave geometry is `LTU-RAI/gazebo_cave_world`, MIT licensed, and must be cited as: Anton Koval, Christoforos Kanellakis, Emil Vidmark, Jakub Haluska, George Nikolakopoulos, "A Subterranean Virtual Cave World for Gazebo based on the DARPA SubT Challenge," arXiv:2004.08452, Control Engineering Group, Luleå University of Technology. The upstream `LICENSE` travels with the vendored mesh, the citation lands in `model.config`, in a `README.md` third-party-assets section, and in the vendoring commit message. Nothing in this project may present this cave as original geometry — the failure mode here is not a bug, it's a licensing and honesty failure, and it is the reason this is a listed risk rather than an implementation detail.
- **A 24 MB collision trimesh is the heaviest geometry this project has loaded** — the deliberate choice is to accept it (same OBJ for visual and collision, static model). If the measured real-time factor makes 10-run ATE evaluation impractical, that is a real finding to record and act on then, with real measured numbers; it is not a reason to pre-emptively substitute a simplified collision hull that would no longer match what the lidar sees.
- **Removing the vendored `blueboat` model's real motor/prop links and grafting on track assemblies changes its mass/inertia distribution** — the vendored model's `base_link` inertial values were tuned for the real motorized BlueBoat; re-verify (or re-derive) inertia after the track assemblies are added, don't leave the original values in place unexamined.

## Testing / verification

- Same "verify empirically, never assume a topic/type/parameter name" discipline established throughout this project.
- The cave mesh is verified before anything is placed relative to it: `gz sdf --check` passes, the world loads with no mesh-not-found/parse errors (an unresolved `model://` URI is the expected failure), `cave_world` appears in the scene graph while `dry_cave`/`flooded_cave_water` do not, and a dropped probe body comes to rest on the mesh floor — proving the mesh really collides rather than only renders. The real-time factor is recorded at the same time.
- Minimum bar before declaring the tracked vehicle "working": drive it via a manually-armed, manually-commanded `/ap/cmd_vel` and observe real net translation in Gazebo (the exact failure mode that blocked the abandoned legged walker) — this is the first concrete go/no-go checkpoint, before any SLAM/Nav2/exploration work is layered on top.
- Retraction joints verified independently: command deployed→retracted→deployed and confirm real joint-state transitions in Gazebo, and confirm the vehicle still drives correctly in both positions (or is deliberately prevented from driving while retracted, if that's the more realistic behavior — decide and document at implementation time, don't leave it ambiguous).
- BlueROV2 verified independently before wiring the handoff: spawn it standalone in the water region, arm it, drive it via manual `/cmd_vel`-equivalent, confirm real buoyancy (it floats/holds depth rather than sinking or rocketing to the surface) and real thruster-driven translation.
- Water-boundary handoff verified as its own go/no-go checkpoint: drive the tracked vehicle to the boundary, confirm tracks retract and the BlueROV2 appears armed and controllable at the right pose; drive the BlueROV2 back out, confirm the tracked vehicle reappears and redeploys tracks. Confirm the two vehicles are never simultaneously armed.
- ATE evaluation reused unmodified in methodology (≥10 runs, sim-time-gated, curated for buffer-contamination the way this project has done twice already) for the tracked vehicle's dry-cave run.

## Out-of-scope future phases

- **Phase 2**: full Nav2/RTAB-Map autonomy for the BlueROV2 underwater, sonar, water-current modeling (with real, not fabricated, SLAM+current-estimation techniques — never "SIC-SLAM"/"CurrentFactor" unless genuinely real and cited), a tether between the two vehicles if ever needed.
- **Phase 3**: air-shaft section, detachable PX4-controlled flying drone.
