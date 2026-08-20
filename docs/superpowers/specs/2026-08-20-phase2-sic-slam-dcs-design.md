# Phase 2: Sonar + IMU + Dynamics SIC-SLAM and Drift/Current Suppression

**Date:** 2026-08-20
**Status:** Design approved, implementation not started
**Scope:** The flooded (partially submerged) cave section, BlueROV2 only. The
dry-cave tracked-vehicle phase is untouched.

## Summary

This delivers the real **Sonar-Inertial-Current Graph-SLAM (SIC-SLAM)** named
in the funding application (WP2-WP3) — sonar + IMU + vehicle dynamics fused in
a GTSAM factor graph — and the **DCS (Drift/Current Suppression)** controller
that consumes its current estimate to hold the ROV on track against water flow.

It replaces the "WP2-WP3 deliverable" caveat currently recorded in the top-level
README and in `cavex_slam_nav/cavex_slam_nav/sic_slam_node.py`'s docstring.

## Goals

1. A real acoustic sonar sensor for the BlueROV2 in Gazebo Harmonic.
2. A real water current in the simulation, from upstream physics, not faked.
3. A real GTSAM factor graph fusing sonar, IMU preintegration, and a BlueROV2
   dynamics model, jointly estimating pose, velocity, IMU bias, **and water
   current**.
4. A DCS controller that uses the current estimate to reduce cross-track error.
5. An honest, measured A/B result: cross-track RMSE with DCS on vs. off.

## Non-goals

- No DVL. `gz::sensors::DopplerVelocityLog` exists in `gz-sensors8` but
  `gz-sim8`'s sensors system never references it, so `<sensor type="dvl">` will
  not instantiate from SDF without a custom host system plugin, and
  `ros_gz_bridge` has no DVL mapping. Current is instead made observable
  through the dynamics residual (see `CurrentFactor`). A DVL factor is a clean
  future extension but is not built here.
- No change to the ArduSub control path. DCS is inserted as a filter *upstream*
  of the existing `cmd_vel_to_ardusub.py`, which is not modified.
- No autonomous frontier exploration underwater. This delivers localization,
  mapping, and disturbance rejection; goal selection stays manual/waypoint.
- No claim that the sonar model is validated against a real sonar. It is a
  physically-motivated model, not a calibrated one. See "Honesty conventions".

## Environment (verified 2026-08-20)

| Item | State |
|---|---|
| Gazebo Sim | 8.14.0 (Harmonic) |
| ROS 2 | Jazzy |
| rtabmap | installed, 13 packages |
| `ros_gz_bridge` | installed, `/opt/ros/jazzy` |
| `ardupilot_msgs` | built in `ros2_ws/install` |
| GTSAM | **not installed**; apt candidate `libgtsam-dev 4.2.0+dfsg-1build1` |
| Sonar sensor in `gz-sensors8` | **does not exist** — hence Component 1 |
| Ocean current in `Hydrodynamics` | **exists** — `default_current`, `lookup_current_x/y/z`, topic `/ocean_current` |

### BlueROV2 model ground truth

From `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf` and
`configs.yaml`:

- mass 10.0 kg, `fluid_density` 998.0
- **Added mass: all zero.** `xDotU..nDotR` = 0
- **Linear damping: all zero.** `xU..nR` = 0
- **Quadratic drag only:** `xUabsU=-33.732`, `yVabsV=-54.16`, `zWabsW=-73.225`,
  `kPabsP=mQabsQ=nRabsR=-3.992`
- Thrusters: `thrust_coefficient` ±0.02 (T3/T4/T6 negative to balance torque),
  `propeller_diameter` 0.1, `velocity_control` true
- Thruster positions (m, body frame): T1 (0.14, -0.092, 0), T2 (0.14, 0.092, 0),
  T3 (-0.14, -0.092, 0), T4 (-0.14, 0.092, 0), T5 (0, -0.109, 0.077),
  T6 (0, 0.109, 0.077). T1-T4 are vectored horizontal at ±45° yaw; T5/T6 vertical.
- IMU: `imu_sensor` on `base_link`, 1000 Hz, posed roll-180° into the ArduPilot
  body frame (x-forward, y-right, z-down).

**This matters more than it looks.** The dynamics factor must model *exactly*
the hydrodynamics the simulator applies — pure quadratic drag, no added mass, no
linear damping. Modelling added-mass terms the simulator does not have would
inject a systematic residual that the graph would absorb into the current node,
producing a confidently wrong current estimate. Any future change to the SDF
hydrodynamics block must be mirrored in the dynamics model, and this coupling
should be called out in both files.

## Architecture

Four components, in dependency order.

```
                        /ocean_current (gz topic)
                                 |
                                 v
   [Hydrodynamics plugin: real quadratic drag + current] --> BlueROV2 motion
                                 |
        +------------------------+------------------------+
        |                        |                        |
        v                        v                        v
  sonar plugin (C1)         imu_sensor              thruster cmds
        |                        |                        |
   sensor_msgs/LaserScan    sensor_msgs/Imu        thrust setpoints
        |                        |                        |
        +------------> cavex_sic_slam (C3) <--------------+
                        GTSAM iSAM2 factor graph
                                 |
              +------------------+------------------+
              |                                     |
   /sic_slam/odometry, /sic_slam/map      /sic_slam/current_estimate
                                                    |
                                                    v
  /cmd_vel_rov_desired --> [ dcs_controller (C4) ] --> /cmd_vel_rov
                                                            |
                                                            v
                                          cmd_vel_to_ardusub.py (UNMODIFIED)
                                                            |
                                                            v
                                                    ArduSub SITL, GUIDED
```

### Component 1 — `cavex_sonar_gazebo`: acoustic sonar system plugin

A new C++ `gz-sim8` system plugin implementing a mechanically-scanning
profiling sonar, because Gazebo Harmonic ships no sonar sensor type.

**Hosting decision (revised 2026-08-20, during planning).** Gazebo Harmonic
cannot readily host a custom *rendering* sensor: `gz-sim8`'s `Sensors` system
only instantiates sensor types it already knows, which is the same root cause
that rules out the DVL above. Rather than fight render-thread integration, the
acoustic model is built as a **host-agnostic C++ library with no Gazebo and no
ROS dependencies**, and a dense, real `gpu_lidar` declared in the BlueROV2 SDF
serves purely as the **ray engine**.

This keeps the acoustic physics real and fully unit-testable off-simulator, and
it is emphatically *not* the "bare gpu_lidar stand-in" option that was rejected
— the beam-forming and acoustic layers below are all still built. Because the
library is host-agnostic, it can later be relocated inside a gz-sim system
plugin without touching its physics or its tests, should that ever be worth the
render-thread work.

**Range acquisition.** A dense horizontal-fan `gpu_lidar` on the BlueROV2
supplies geometrically correct first-return ranges off the cave mesh at
simulation rate, without writing a ray tracer. Several adjacent dense rays are
integrated per sonar beam (see "Beam spread"), and surface normals for the
incidence term are estimated from adjacent range returns within the dense scan.

**Acoustic layer**, applied per beam on top of the raw range:

1. **Beam spread** — each "beam" integrates several `GpuRays` samples across the
   main-lobe solid angle rather than taking a single infinitely-thin ray, so
   the return smears near grazing incidence the way a real beam does.
2. **Transmission loss** — spherical spreading (`20·log10(r)`) plus
   frequency-dependent absorption (`α·r`, α configurable, default for a
   ~1 MHz imaging sonar in fresh water).
3. **Angle-dependent backscatter** — return strength falls off with the
   incidence angle between beam and surface normal (Lambertian-style), which is
   what makes a smooth tunnel wall a weak, ambiguous target and is precisely the
   effect that drives the degeneracy risk below.
4. **Rayleigh speckle** — multiplicative Rayleigh-distributed noise on echo
   amplitude, the standard first-order model for coherent acoustic imaging.
5. **Detection threshold and dropout** — beams whose modelled echo falls below
   threshold report no return, rather than reporting a confident wrong range.

**Output.** `gz.msgs.LaserScan`, which carries an `intensities` field alongside
`ranges`. This is deliberate: it bridges to `sensor_msgs/LaserScan` through
stock `ros_gz_bridge` with **zero custom bridge code and zero custom messages**,
and echo strength rides along in `intensities` for the registration front-end to
weight by.

**Parameters:** fan aperture, beam count, angular resolution, min/max range,
centre frequency, absorption coefficient, noise seed, update rate.

Installed so its `.so` lands on `GZ_SIM_SYSTEM_PLUGIN_PATH`, following the
pattern the existing `ardupilot_gazebo` plugin already establishes in this repo.

### Component 2 — Ocean current (configuration, no new plugin)

The stock `Hydrodynamics` plugin already supports ocean current. No new physics
code is written.

- `<default_current>` in the BlueROV2's hydrodynamics block sets a baseline flow.
- `current_field_node.py` (small, Python) publishes time-varying current to the
  `/ocean_current` Gazebo topic to script repeatable disturbance profiles:
  constant, step, sinusoidal, and a spatially-keyed profile that ramps flow up
  in the narrow parts of the tunnel.
- The commanded profile is also published on a ROS topic
  `/cavex/current_ground_truth` purely for evaluation, and must never be
  subscribed to by SIC-SLAM or DCS. This mirrors the discipline already applied
  to ground-truth pose elsewhere in this project.

### Component 3 — `cavex_sic_slam`: the real SIC-SLAM

C++ ROS 2 node, GTSAM 4.2, iSAM2 incremental optimization over keyframes.

**Variables per keyframe `i`:**

| Symbol | Type | Meaning |
|---|---|---|
| `X(i)` | `gtsam::Pose3` | body pose in map frame |
| `V(i)` | `gtsam::Vector3` | body velocity in map frame |
| `B(i)` | `gtsam::imuBias::ConstantBias` | accel + gyro bias |
| `C(i)` | `gtsam::Vector3` | **water current velocity, map frame** |

**Factors:**

1. **`CombinedImuFactor`** between consecutive keyframes, from GTSAM's
   `PreintegratedCombinedMeasurements` over `/imu`. Standard, well-tested GTSAM;
   handles bias evolution internally. Note the IMU is mounted roll-180° into the
   ArduPilot frame — the preintegration must use the correct body-to-sensor
   extrinsic, not identity.
2. **Sonar odometry `BetweenFactor<Pose3>`** — scan-to-scan registration of
   consecutive sonar fans.
3. **Sonar loop-closure `BetweenFactor<Pose3>`** — scan-to-submap matching
   against earlier keyframes, gated by a match-quality threshold and validated
   before insertion.
4. **`CurrentFactor` (custom)** — the observability mechanism, and the reason
   this system is "SIC" and not just "SI". A ternary factor on `(X(i), V(i), C(i))`:

   ```
   residual = V(i) - [ R(X(i)) · v_body_predicted(thrust, drag) + C(i) ]
   ```

   where `v_body_predicted` comes from the BlueROV2 dynamics model: thrust
   allocation over the six thrusters → body wrench → the quasi-steady velocity
   at which the modelled **quadratic** drag balances that wrench. Because the
   dynamics model predicts the vehicle's velocity *through the water*, and the
   graph's `V(i)` is velocity *over ground* (constrained by sonar and IMU), the
   difference between them is exactly the water current — which is what makes
   `C(i)` observable at all. Analytic Jacobians, verified against
   `gtsam::numericalDerivative` in unit tests.
5. **Current random walk `BetweenFactor<Vector3>`** on `(C(i), C(i+1))` with
   zero mean and a tuned process noise — encodes "current varies slowly", which
   is what stops the current node from absorbing fast-changing residuals that
   actually come from registration error.
6. **Priors** on the first keyframe's pose, velocity, bias, and current.

**Front-end.** Keyframe selection on distance/rotation/time thresholds. Sonar
registration weights returns by the `intensities` field, so weak grazing
returns count for less — the honest use of the acoustic model from Component 1.

**Outputs:** `/sic_slam/odometry` (`nav_msgs/Odometry`),
`/sic_slam/current_estimate` (`geometry_msgs/TwistWithCovarianceStamped`,
covariance taken from the graph's marginal on `C`), `/sic_slam/map`
(accumulated sonar point cloud), and `/sic_slam/keyframes` for visualization.

### Component 4 — `dcs_controller`: Drift/Current Suppression

A pure filter node in the command chain. It subscribes to
`/cmd_vel_rov_desired` and publishes `/cmd_vel_rov`, which
`cmd_vel_to_ardusub.py` already consumes. **`cmd_vel_to_ardusub.py` is not
modified**, and with DCS not running the chain behaves exactly as it does today.

**Feed-forward.** Rotate the estimated current into the body frame and subtract
it from the commanded velocity, so the commanded through-water velocity produces
the desired over-ground velocity.

**Feedback.** A PI on cross-track error between the SIC-SLAM pose and the active
planned track, catching what feed-forward misses (current estimate error,
dynamics model error, unmodelled effects).

**Gating on estimate quality.** Feed-forward is scaled by confidence derived
from the marginal covariance on `C`. Before the graph has observed enough
motion to separate current from drift, the feed-forward term is near zero and
DCS degrades gracefully to plain feedback rather than confidently steering into
a bad estimate.

**Saturation awareness.** The compensated command is clamped to real thruster
authority. When the required compensation exceeds what the vehicle can produce,
the node publishes a warning on `/dcs/status` and reports the shortfall rather
than silently under-delivering.

## Data flow and frames

- Gazebo body frame is x-forward, y-left, z-up; the ArduPilot/IMU frame is
  x-forward, y-right, z-down (the `<pose>` roll-180° on `imu_sensor`).
  Every transform between them is applied explicitly and tested, not assumed.
  Frame confusion here would appear as a plausible-looking but wrong current
  estimate, which is the hardest class of bug to notice in this system.
- SIC-SLAM publishes in the `map` frame with `base_link` as child, matching the
  convention `slam_pose_publisher.py` already uses on the dry-cave side.

## Error handling

- **No sonar returns** (open water, all beams below threshold): the graph
  continues on IMU + dynamics alone; keyframes are still created; covariance is
  allowed to grow honestly rather than being clamped.
- **Registration failure / low match quality:** the factor is rejected, not
  inserted with inflated noise. A rejected loop closure is logged with its score.
- **iSAM2 indeterminate linear system:** caught, logged with the offending
  keyframe, and the update is skipped rather than crashing the node. This is a
  real and expected failure mode in degenerate corridor geometry.
- **Current estimate diverging** beyond a physical plausibility bound: clamped
  for the purposes of DCS feed-forward, flagged on `/dcs/status`, and reported
  in the evaluation output rather than hidden.
- **DCS with stale SIC-SLAM input:** if `/sic_slam/current_estimate` goes stale
  past a timeout, feed-forward decays to zero and the node warns.

## Testing

**Unit (gtest / pytest):**
- Thrust allocation: known thruster commands → expected body wrench, including
  the torque-balance sign convention on T3/T4/T6.
- Quadratic drag model: known wrench → expected quasi-steady velocity.
- `CurrentFactor` Jacobians vs. `gtsam::numericalDerivative` — non-negotiable,
  this is where subtle sign and frame errors hide.
- Frame transforms: Gazebo body ↔ ArduPilot body ↔ map, round-trip identity.
- DCS feed-forward: known current + desired velocity → expected command;
  saturation clamps; confidence gating; stale-input decay.
- Sonar acoustic model: transmission loss and incidence falloff monotonicity;
  threshold produces dropouts not wrong ranges; fixed seed → reproducible noise.

**Integration (sim):**
- Sonar plugin standalone: fan returns match known cave geometry within
  tolerance; dropout behaviour at grazing incidence is present, not absent.
- Current estimate convergence: inject a known constant current via
  `/ocean_current`, verify `/sic_slam/current_estimate` converges to it, and
  report steady-state error and convergence time.
- Time-varying current: step and sinusoidal profiles, report tracking lag.
- **Headline A/B:** fly the same planned track with DCS off, then DCS on, under
  the same current profile and noise seed. Report cross-track RMSE for both.
  This single number is the deliverable's justification.
- ATE for SIC-SLAM pose, reusing the existing `ate_evaluator_node.py` harness
  and its ≥10-run methodology so the number is comparable to the dry-cave result.

**All evaluation uses ground truth only as a scorer, never as an input** — the
same discipline the existing ATE harness follows.

## Risks

**1. Featureless-tunnel degeneracy (highest risk).** Sonar registration along a
smooth, straight cave tunnel is under-constrained in the along-track direction —
the classic degenerate-corridor problem. The danger specific to this design is
that unobservable along-track drift gets absorbed into the current node,
yielding a confident, wrong current estimate that DCS then acts on, actively
steering the vehicle off track. This must be **measured, not assumed**:
the convergence test above is the detector, and if it fires, the mitigations in
order of preference are (a) tighter current random-walk process noise, (b) an
explicit degeneracy check on the registration Hessian's conditioning that
down-weights or rejects along-track constraints, (c) a prior pinning current
magnitude. This risk is the reason the DCS confidence gate exists.

**2. GTSAM 4.2 apt + Jazzy ABI.** `libgtsam-dev 4.2.0+dfsg-1build1` must link
cleanly against ROS 2 Jazzy's C++ ABI and Boost version. **Verified first, as a
throwaway spike, before any other work begins** — if it fails, the fallback is
building GTSAM from source pinned to a known-good tag, which changes build
instructions but not the design.

**3. Dynamics/SDF coupling.** As noted above, the dynamics model mirrors the SDF
hydrodynamics block exactly. Divergence between them silently corrupts the
current estimate. Mitigated by a unit test that reads the SDF values and asserts
the model's coefficients match.

**4. Real-time factor.** The sonar plugin's GPU ray work plus iSAM2 updates may
depress RTF. Measured and recorded, as the cave-mesh work already did.

## Naming: resolving the SIC-SLAM collision

This repo currently has **two** unrelated things called `sic_slam`, neither of
which is SIC-SLAM:

- `cavex_slam_nav/.../sic_slam_node.py` — a cmd_vel + IMU complementary filter
  bias-corrected against RTAB-Map, for the dry-cave wheeled robot. Its own
  docstring says it is not the real system.
- `cavex_perception/src/sic_slam_node.cpp` — RGB-D + lidar instance clustering,
  a completely different thing that happens to share the name.

Adding a third would be indefensible. Therefore the real system built here takes
the name — package `cavex_sic_slam`, node `sic_slam_node` — and as a final,
**separable** stage the two incumbents are renamed to what they actually are
(`pose_fusion_node.py` and an accurate name for the perception node), with their
README sections and launch files updated.

This stage is sequenced last and is independently revertible precisely because
it touches working Phase 1 code. If it looks risky at the time, it can be
dropped without affecting anything else in this design.

## Honesty conventions

This project has an established and enforced practice of not overclaiming, and
this work is the single most tempting place in the repo to break it.

- The sonar is a **physically-motivated model, not a calibrated one**, and has
  not been validated against real sonar data. The README's current "On sonar"
  note is updated to say the sensor is now real within the simulation while
  stating plainly that it is not validated against hardware.
- SIC-SLAM results are simulation results. The cave mesh is static, the water
  current is one we inject ourselves, and the acoustic model is our own. None of
  that makes the numbers fake, but all of it must be stated wherever they are cited.
- The existing README caveat that the full system "remains a WP2-WP3
  deliverable" is replaced with an accurate description of what was built and
  what was not — specifically that there is no DVL and no hardware validation.
- The A/B cross-track result must be reported with the current profile, seed,
  and run count that produced it.

## Implementation order

0. **Spike:** GTSAM 4.2 + Jazzy ABI. Throwaway, answers a yes/no question.
1. Sonar plugin (Component 1) + standalone verification.
2. Ocean current configuration and `current_field_node.py` (Component 2).
3. SIC-SLAM factor graph (Component 3), built inside-out: IMU-only graph
   first, then sonar factors, then `CurrentFactor` last.
4. DCS controller (Component 4).
5. Evaluation harness and the A/B result.
6. Naming cleanup (separable, droppable).
