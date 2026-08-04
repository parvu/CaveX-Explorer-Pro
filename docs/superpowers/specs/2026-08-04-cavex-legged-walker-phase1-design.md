# CaveX Hybrid Robot — Phase 1: Legged Walker in the Dry Cave

## Context

The long-term ask (from the user, this session) is a single "hybrid robot"
that walks (Spot-like legs), sails (BlueBoat), dives (tethered BlueROV2),
and flies (detachable PX4 X500 drone), autonomously mapping a partially
flooded cave (dry section, flooded section, air shaft) in 3D, using vision +
3D lidar + sonar as appropriate, running "SIC-SLAM with CurrentFactor, or
better," with a live 3D React dashboard and real ATE measurement.

That request bundles at least seven independently-substantial subsystems.
A literal arXiv search this session confirmed "SIC-SLAM" and "CurrentFactor"
are not published, citable algorithms — they're this project's own internal
name (from `ate_evaluator_node.py`'s docstring, referencing a funding
application) for a not-yet-built system. Real, relevant published cave/
underwater SLAM work exists (SVIn2, Sunfish, CavePI, AQUA-SLAM, Water-DSLAM)
but none of it is literally "SIC-SLAM" — building it means designing a real
system informed by that literature, not implementing a specific paper.

Given the scope, this spec covers **Phase 1 only**: a legged, Spot-like
walker, autonomously exploring the existing dry-cave section of
`CaveX-Explorer-Pro`'s world, in 3D, with real SLAM, real obstacle
avoidance, and the existing live dashboard/ATE harness extended to match.
Later phases (flooded section + BlueBoat/BlueROV2 + sonar + CurrentFactor,
air shaft + PX4 drone) are out of scope here and get their own spec each,
once Phase 1 is proven to actually work end-to-end.

This builds on the existing, working `cavex_slam_nav` ROS2 package and the
CaveX-Explorer-Pro React dashboard (both already real and tested this
session) rather than a new workspace.

## Goals

- A quadruped robot with Spot-like proportions walks through the dry cave
  section under Nav2's control (not scripted waypoints).
- The robot builds a real 3D map (RTAB-Map, 3D-lidar mode) of the dry cave.
- Exploration is genuinely self-guided: a frontier-exploration node picks
  unexplored-space goals; Nav2's local planner avoids obstacles using a
  costmap built from the real 3D lidar scan.
- The live 3D web dashboard shows the walker's real pose and a live-updating
  occupancy/point-cloud summary while it explores.
- ATE is measured the same honest way as the existing wheeled-robot harness:
  real Umeyama-aligned trajectory error against ground truth, over a fixed
  sim-time exploration budget per run (not a fixed path — paths vary because
  exploration is autonomous).

## Explicit non-goals for Phase 1

- No sonar (no water present in this phase — a sonar reading would have
  nothing real to sense).
- No "SIC-SLAM" / "CurrentFactor" labeling or wiring. `sic_slam_node.py`
  stays as-is, unused by this phase's launch files, until the flooded-phase
  spec gives it a real current to compensate for.
- No BlueBoat, BlueROV2, or PX4 drone integration.
- No literal Boston Dynamics Spot model (proprietary, unavailable for
  simulation) — a CHAMP-generated quadruped with Spot-like body/leg
  proportions instead. Any report or dashboard label describing this robot
  should say "Spot-like quadruped (CHAMP)," not "Spot," to avoid implying
  we're running Boston Dynamics' actual robot or software.

## Architecture

```
cavex_slam_nav/                      (existing package, extended)
  urdf/cavex_walker.urdf.xacro       NEW: CHAMP-generated quadruped +
                                      3D lidar + camera + IMU (same sensor
                                      pattern as cavex_robot.urdf.xacro)
  worlds/cavex_world.world           MODIFIED: add Fuel-sourced rock/
                                      obstacle models to the dry-cave section
  launch/
    gazebo_walker.launch.py          NEW: spawns cavex_walker instead of
                                      cavex_robot, same gz-bridge pattern
    walker_nav.launch.py             NEW: RTAB-Map (3D mode) + Nav2 bringup
                                      + frontier explorer + CHAMP controller
                                      + ate_evaluator_node.py + telemetry
                                      bridge (reused, no changes needed)
  cavex_slam_nav/
    (no new Python nodes expected — CHAMP and Nav2/frontier-exploration are
    existing packages, launched and configured, not written from scratch)
```

CHAMP's controller subscribes to `/cmd_vel` exactly like the current
`VelocityControl` plugin does, so Nav2's local planner output needs no
changes to target this robot instead of the wheeled one.

## Components

**Quadruped body & gait (CHAMP).** Use CHAMP's config generator
(`champ_setup_assistant` or equivalent) to produce a URDF with Spot-like
proportions (long body, 4 legs, hip/thigh/knee per leg) and CHAMP's gait
controller package to drive it from `cmd_vel`. This is the single largest
open technical risk in this phase (see Risks) — CHAMP + Gazebo Harmonic
compatibility hasn't been verified by us yet, unlike PX4 which we've proven
this session.

**3D lidar.** Reuse the existing `gpu_lidar` sensor type (already proven
working for the 2D case) reconfigured with real vertical FOV/multiple
vertical samples for a genuine 3D spin — no new plugin.

**SLAM (RTAB-Map, 3D mode).** Switch `rtabmap`'s launch parameters from the
current `subscribe_scan` (2D) + RGB mode to 3D lidar mode
(`subscribe_scan_cloud`, ICP odometry from the 3D point cloud). This is a
real, supported RTAB-Map configuration, not a new algorithm.

**Autonomy (Nav2 + frontier exploration).** Real `nav2_bringup`
(costmap_2d/3d, planner server, controller server — finally using the
dependency that's been declared-but-unused all session) plus a frontier-
exploration node. Exact package TBD at plan time — needs a build-time check
of what's actually available/working for ROS2 Jazzy (candidates:
`explore_lite`, or a small custom `nav2_simple_commander`-based frontier
picker if nothing ports cleanly). This is flagged as a checkpoint, not a
promise of a specific package.

**World.** Add a handful of Fuel-sourced rock/stalagmite models to the dry
section (currently an empty box) so obstacle avoidance has something real
to avoid and frontier exploration has real unexplored geometry to find.

**Dashboard.** Reuse `web_telemetry_bridge.py` → `/api/telemetry` →
`SICSlamVisualizer.tsx`/`GazeboSimViewport.tsx` live-data pattern. Extend
the telemetry payload with a lidar-derived summary (e.g. point count / a
coarse occupancy snapshot) and frontier-exploration status (current goal,
% of reachable space explored). No new bridge architecture — same
polling-over-HTTP approach already proven this session.

**ATE.** Reuse `ate_evaluator_node.py` and the `drive_fixed_trajectory.py`
sim-time-gating pattern, but change what "fixed" means: instead of a fixed
scripted path, each run gets a fixed sim-time exploration budget (e.g. 60s),
then `finish_run` is triggered automatically. Estimate topic becomes
RTAB-Map's real 3D pose (or an un-augmented dead-reckoning baseline —
TBD at plan time whether a Phase-1-appropriate second estimate is worth
adding, given there's no CurrentFactor to showcase yet).

## Data flow

```
3D lidar + camera + IMU  →  RTAB-Map (3D/ICP)  →  map→odom→base_footprint TF
                                    ↓
                          frontier explorer (Nav2 costmap)
                                    ↓
                              Nav2 controller → /cmd_vel
                                    ↓
                          CHAMP gait controller → leg joint commands
                                    ↓
                              Gazebo physics (legs walk)
                                    ↓
                    ground_truth /odom (unchanged, from gz OdometryPublisher)
                                    ↓
                    ate_evaluator_node.py (ground truth vs RTAB-Map pose)
                                    ↓
                    web_telemetry_bridge.py → /api/telemetry → dashboard
```

## Error handling / risks

- **CHAMP + Gazebo Harmonic compatibility (highest risk).** Unproven by us.
  If CHAMP's Gazebo plugins target Classic Gazebo only, the fallback is
  either a `ros2_control`-based custom gait controller (more work, still
  real) or, if that's also infeasible in the available time, downgrading
  Phase 1 to a simpler multi-legged kinematic walker without CHAMP
  specifically. This gets resolved at plan/implementation time, not
  guessed here.
- **3D lidar performance.** 3D ICP odometry is heavier than the current 2D
  mode; may need a reduced point count or slower detection rate to stay
  real-time in this environment.
- **Frontier-exploration package availability on Jazzy.** Checked at plan
  time; a small custom frontier picker is an acceptable fallback if nothing
  ports cleanly, since the core requirement (robot picks its own goals) is
  simple enough to implement directly against Nav2's costmap if needed.

## Testing / verification

- Build must succeed (`colcon build`).
- Launch both stacks headless, confirm via `ros2 topic list`/`hz` that
  lidar, RTAB-Map's 3D map topic, and Nav2's costmap are all actually
  publishing (the established failure mode all session has been silent
  QoS/topic mismatches — same checks apply here).
- Let frontier exploration run for a fixed sim-time budget, confirm the
  robot actually moves autonomously (ground-truth pose changes without any
  manual `cmd_vel` being sent) and RTAB-Map's map grows.
- Run the ATE harness for >=10 runs, confirm results are sane
  (non-degenerate, not NaN/inf, comparable across runs given the fixed
  time-budget methodology already validated for the wheeled robot).
- Manually verify the dashboard shows live pose + exploration status while
  a run is in progress.

## Out of scope / future phases (not designed here)

- Phase 2: flooded section, BlueBoat + tethered BlueROV2, real sonar,
  ArduPilot SITL integration, tether physics.
- Phase 3: air shaft, detachable PX4 X500 drone (leverages this session's
  existing PX4 SITL + Gazebo Harmonic experience from the `ros2_pinn_sim`
  project).
- A real "SIC-SLAM with CurrentFactor" design — only meaningful once there's
  an actual water current to compensate for (Phase 2+), informed by the
  real published literature found this session (SVIn2, Sunfish, CavePI,
  AQUA-SLAM, Water-DSLAM) rather than invented from the name alone.
