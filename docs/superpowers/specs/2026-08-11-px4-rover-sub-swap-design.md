# PX4 SITL replacing ArduPilot for the tracked rover and BlueROV2 sub — Design

**Status:** approved by user, implementation deliberately deferred. This spec exists so the
work can resume later without re-deriving the architecture.

**Branch:** `px4-rover-sitl` (off `main`). Do not implement on `main`.

## Goal

Replace ArduRover (tracked vehicle) and ArduSub (BlueROV2) with PX4 SITL equivalents, on
the `px4-rover-sitl` branch, without touching `main`'s ArduPilot-based stack.

## Background / why this is non-trivial

This project's *existing* ArduPilot integration does **not** use ArduPilot's own SDF
motor/FDM plugin (`ArduPilotPlugin`) to actually drive the vehicles. Per
`track_cmd_vel_bridge.py`'s own comments and `launch.txt`/`README.md`'s documented history:
actuation flows entirely through a DDS velocity round-trip —
`cmd_vel_to_ardupilot.py` sends `/cmd_vel` to ArduPilot via `/ap/cmd_vel` (AP_DDS velocity
control), ArduPilot runs its GUIDED-mode control/EKF logic and reports back via
`/ap/twist/filtered`, and `track_cmd_vel_bridge.py` converts that into gz-transport commands
for this project's own custom `TrackedVehicle`/thruster gz-sim plugins.
`ArduPilotPlugin`'s FDM/servo output is present in the SDF for compatibility only and is
never actually wired into the real control path.

This matters because it means the SDF/actuation layer doesn't need to change at all — only
the flight-stack (ArduPilot → PX4) needs replacing, reusing the exact same "bridge node
converts ROS2 cmd_vel ↔ flight-stack DDS setpoint/feedback" shape already proven here.

**Investigated and ruled out**: PX4's SIH mode (fully internal physics, no Gazebo needed —
the PX4 equivalent of how ArduPilot's own SIM_Rover backend already runs disconnected from
Gazebo in this project) only ships an **Ackermann**-steering rover variant
(`10045_sihsim_rover_ackermann`) and **no UUV/sub variant at all**. Ackermann kinematics
don't match this vehicle's skid-steer tracks, and there's no sub option, so SIH is not
usable for either vehicle — ruled out, do not re-investigate this path without new
information.

## Architecture

PX4 runs as a pure velocity-setpoint/state-estimator black box, mirroring the existing
ArduPilot integration's shape exactly:

1. **State estimate without needing PX4's own Gazebo sensor wiring.** Rather than making
   PX4's `gz_bridge` module understand this project's non-PX4-shaped Gazebo models (which
   would require re-modeling sensors to match PX4's r1_rover/bluerov2_heavy reference
   layouts), feed PX4's EKF2 external localization via `/fmu/in/vehicle_visual_odometry`,
   sourced from this project's existing ground-truth publisher
   (`tracked_vehicle_ground_truth_odom.py` for the rover; an equivalent for the sub, likely
   adapted from the same pattern). This is the key trick that avoids needing PX4's actuator/
   sensor topics to match our Gazebo models at all.

2. **Command path — rover.** New node `cmd_vel_to_px4_rover.py` converts `/cmd_vel`
   (`geometry_msgs/Twist`) into PX4's `RoverSpeedSetpoint` (linear speed) +
   `RoverRateSetpoint` (yaw rate), publishing to `/fmu/in/rover_speed_setpoint` and
   `/fmu/in/rover_rate_setpoint`. **Both topics are already present in this PX4-Autopilot
   checkout's default `src/modules/uxrce_dds_client/dds_topics.yaml`** — confirmed by direct
   inspection, no PX4 firmware config edits or rebuild needed for this part. Also handles
   arming (`VehicleCommand` ARM) and mode switching.

3. **Command path — sub.** New node `cmd_vel_to_px4_sub.py` converts `/cmd_vel_rov` into
   PX4's `manual_control_input` message (also already present in `dds_topics.yaml` by
   default) to drive `uuv_att_control`'s manual-control input path. Chosen over
   `trajectory_setpoint6dof` (which `uuv_pos_control` reads) specifically because
   `trajectory_setpoint6dof` is **not** in the default `dds_topics.yaml` and would require
   editing PX4's own DDS config and rebuilding — `manual_control_input` needs neither.

4. **Feedback path — both.** New nodes `px4_rover_twist_bridge.py` /
   `px4_sub_twist_bridge.py` read PX4's `/fmu/out/vehicle_odometry` (already bridged) and
   convert to the gz-transport command topics this project's existing `TrackedVehicle`/
   BlueROV2 thruster plugins already expect — the exact same mechanism
   `track_cmd_vel_bridge.py` uses today, only the upstream source topic changes.

5. **Launch integration.** Two separate PX4 SITL instances (rover + sub, distinct ports/
   namespaces, mirroring this project's existing dual-ArduPilot-instance pattern for
   rover+sub) replace the `ardurover` process launch in `gazebo_tracked_vehicle.launch.py`
   and the `ros2 launch ardupilot_sitl sitl_dds_udp.launch.py command:=ardusub` call.

6. **New dependency: `px4_msgs`.** Needs a colcon package matching this exact
   `PX4-Autopilot` checkout's message definitions field-for-field (a version mismatch
   silently breaks DDS (de)serialization) — generate/vendor from this same checkout's
   `msg/` definitions, do not pull an arbitrary prebuilt `px4_msgs` release without
   verifying it matches.

## Open risk, not resolved by this spec

The exact PX4 flight-mode/arming sequence `rover_differential` and `uuv_att_control`
require to actually **accept** these setpoints (mode name, whether an offboard-style
`OffboardControlMode` heartbeat is mandatory the way it is for multicopter offboard control)
could not be fully determined by reading PX4 source alone during this design pass. This is a
genuine unknown to resolve via live SITL iteration during implementation — the same way this
project's own ArduPilot arm/mode/DDS bugs were only found by testing, not by reading code
(see `launch.txt`'s extensive "FIXED" history for precedent). Implementation should budget
real iteration time for this, not assume the mode/arm sequence works on the first attempt.

## Explicitly out of scope for this pass

- Any change to `main`'s ArduPilot-based stack.
- Re-architecting the Gazebo actuation plugins (ruled out during brainstorming — see
  "Architecture" above, decision was to reuse existing plugins and swap only the
  flight-stack).
- PX4's native actuator-topic-driven model (the `gz-sim-joint-controller-system` /
  `command/motor_speed` pattern PX4's own reference r1_rover/bluerov2_heavy models use) —
  considered and explicitly rejected in favor of the DDS-setpoint-passthrough approach above.
