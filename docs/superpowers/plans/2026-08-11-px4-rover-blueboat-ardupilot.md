# PX4 SITL replacing ArduPilot for the tracked rover and BlueROV2 sub Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Status when written: implementation deliberately deferred by the user ("delay this, leave
it for later"). This plan exists purely for future reference — do not start executing it
without the user explicitly asking to resume this work.**

**Goal:** Replace ArduRover (tracked vehicle) and ArduSub (BlueROV2) with PX4 SITL
equivalents on the `px4-rover-sitl` branch, reusing this project's existing Gazebo
actuation plugins unchanged.

**Architecture:** PX4 runs as a pure velocity-setpoint/state-estimator black box, mirroring
the existing ArduPilot integration's shape: new ROS2 bridge nodes convert `/cmd_vel` into
PX4 DDS setpoints (rover: `RoverSpeedSetpoint`/`RoverRateSetpoint`; sub:
`manual_control_input`), and convert PX4's `/fmu/out/vehicle_odometry` back into the
gz-transport commands this project's existing `TrackedVehicle`/BlueROV2 thruster plugins
already expect. PX4's EKF2 gets its state from this project's own ground-truth publishers
via `/fmu/in/vehicle_visual_odometry`, sidestepping the need for PX4's `gz_bridge` module to
understand our non-PX4-shaped Gazebo models at all.

**Tech Stack:** ROS2 Jazzy, PX4-Autopilot SITL (`PX4-Autopilot/`, gitignored, not vendored),
`px4_msgs` (new colcon package, generated from this exact PX4-Autopilot checkout),
Micro-XRCE-DDS (same protocol/agent family already used for ArduPilot's AP_DDS in this
project), gz-transport13 (`gz.transport13`/`gz.msgs10`, same as the existing ArduPilot
bridge nodes).

## Global Constraints

- Spec source: `docs/superpowers/specs/2026-08-11-px4-rover-sub-swap-design.md`
- Implement on branch `px4-rover-sitl` only. Never touch `main`'s ArduPilot-based stack.
- Do not modify this project's existing Gazebo actuation plugins (`TrackedVehicle`,
  BlueROV2 thruster plugin) — reuse them exactly as-is; only the upstream flight-stack
  changes.
- Do not use PX4's SIH simulator backend — investigated and ruled out (Ackermann-only
  rover, no UUV variant at all; see spec's "Background" section).
- Do not attempt to make PX4's `gz_bridge` module understand this project's Gazebo models'
  sensor/actuator topics — use the external-visual-odometry EKF2 fusion path instead (see
  Architecture above).
- `px4_msgs` must match this exact `PX4-Autopilot` checkout's `msg/` definitions
  field-for-field — a version mismatch silently breaks DDS (de)serialization. Never pull an
  arbitrary prebuilt `px4_msgs` release without verifying the match.
- The exact PX4 flight-mode/arming sequence `rover_differential` and `uuv_att_control`
  require to accept setpoints is NOT known ahead of time and must be determined via live
  SITL testing during implementation (Tasks 2 and 8 are dedicated discovery tasks for this
  — do not skip them or assume an untested mode/arm sequence works).

---

## File Structure

- Create: `ros2_ws/src/px4_msgs/` — generated colcon package, PX4 ROS2 message definitions
  matching this checkout.
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_px4_rover.py`
  — `/cmd_vel` → PX4 rover DDS setpoints + arm/mode handling (rover equivalent of
  `cmd_vel_to_ardupilot.py`).
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/px4_visual_odom_bridge.py`
  — shared by both vehicles: republishes a `nav_msgs/Odometry` ground-truth topic into PX4's
  `/fmu/in/vehicle_visual_odometry` (parametrized by vehicle namespace/topic, one instance
  per vehicle at launch time).
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/px4_rover_twist_bridge.py`
  — PX4 `/fmu/out/vehicle_odometry` → `/track_cmd_vel` (rover equivalent of
  `track_cmd_vel_bridge.py`).
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_px4_sub.py` —
  `/cmd_vel_rov` → PX4 `manual_control_input` (sub equivalent of `cmd_vel_to_ardusub.py`).
- Create:
  `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/bluerov2_ground_truth_odom.py` —
  BlueROV2 ground-truth `nav_msgs/Odometry` publisher (mirrors
  `tracked_vehicle_ground_truth_odom.py`, filtered for model name `bluerov2`; does not exist
  yet).
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/px4_sub_twist_bridge.py`
  — PX4 sub instance `/fmu/out/vehicle_odometry` → BlueROV2 thruster command topic.
- Modify: `ros2_ws/src/cavex_tracked_vehicle/launch/gazebo_tracked_vehicle.launch.py` —
  replace the `ardupilot_sitl_launch` (`command:=ardurover`) block with a PX4 rover instance
  launch.
- Modify: `ros2_ws/src/cavex_tracked_vehicle/package.xml` — add `px4_msgs` exec_depend.
- No changes needed to `ros2_ws/src/cavex_tracked_vehicle/models/*/model.sdf*` or any
  gz-sim plugin config (Global Constraint above).

---

## Task 1: Generate `px4_msgs` matching this PX4-Autopilot checkout

**Files:**
- Create: `ros2_ws/src/px4_msgs/` (full colcon package — `package.xml`, `CMakeLists.txt`,
  `msg/*.msg`)

**Interfaces:**
- Produces: ROS2 message types `px4_msgs::msg::RoverSpeedSetpoint`,
  `px4_msgs::msg::RoverRateSetpoint`, `px4_msgs::msg::VehicleOdometry`,
  `px4_msgs::msg::VehicleVisualOdometry` (or `VehicleOdometry` used for both in/out — confirm
  in Step 3), `px4_msgs::msg::VehicleCommand`, `px4_msgs::msg::OffboardControlMode`,
  `px4_msgs::msg::ManualControlSetpoint` — consumed by every later task.

- [ ] **Step 1: Clone the official px4_msgs repo alongside PX4-Autopilot**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git clone https://github.com/PX4/px4_msgs.git /tmp/px4_msgs_upstream
```

- [ ] **Step 2: Diff the upstream px4_msgs `.msg` files against this checkout's real `msg/` definitions**

```bash
cd /home/parvu/CaveX-Explorer-Pro
for f in RoverSpeedSetpoint RoverRateSetpoint VehicleOdometry VehicleCommand \
         OffboardControlMode ManualControlSetpoint; do
  echo "=== $f ==="
  diff "PX4-Autopilot/msg/${f}.msg" "/tmp/px4_msgs_upstream/msg/${f}.msg" 2>&1 \
    || diff "PX4-Autopilot/msg/versioned/${f}.msg" "/tmp/px4_msgs_upstream/msg/${f}.msg" 2>&1
done
```

Expected: no diff output (fields match exactly), or a diff limited to comments/whitespace
only. If any field, type, or ordering differs, do NOT use the upstream clone as-is — fall
back to Step 3's generation path instead, since a field mismatch here causes silent DDS
(de)serialization corruption at runtime (wrong field interpreted as wrong type), not a
build-time error.

- [ ] **Step 3: If Step 2 found real mismatches, generate px4_msgs directly from this checkout**

PX4 ships its own uORB-to-ROS2-message generation tooling. From this checkout:

```bash
cd /home/parvu/CaveX-Explorer-Pro/PX4-Autopilot
find . -iname "*generate*msg*" -o -iname "*px4_msgs*" 2>/dev/null | grep -v build
```

Follow whatever generation script that search turns up (this varies by PX4 version — read
its own `--help`/docstring rather than guessing flags). The output must be a valid ROS2
`.msg`-file colcon package at `ros2_ws/src/px4_msgs/`.

- [ ] **Step 4: If Step 2 matched cleanly, copy the upstream clone into the workspace instead**

```bash
cd /home/parvu/CaveX-Explorer-Pro
cp -r /tmp/px4_msgs_upstream ros2_ws/src/px4_msgs
rm -rf ros2_ws/src/px4_msgs/.git
rm -rf /tmp/px4_msgs_upstream
```

- [ ] **Step 5: Build and verify**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs --symlink-install
source install/setup.bash
ros2 interface show px4_msgs/msg/RoverSpeedSetpoint
ros2 interface show px4_msgs/msg/VehicleOdometry
```

Expected: both commands print real field lists (not "package not found" or "unknown type").

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/px4_msgs/
git commit -m "Add px4_msgs package matching this PX4-Autopilot checkout"
```

---

## Task 2: Discovery — standalone PX4 rover SITL, real arm/mode/setpoint acceptance sequence

**Files:** none (diagnostic-only task; findings get written into Task 3's node and this
plan's own follow-up notes)

**Interfaces:**
- Produces: the real, empirically-confirmed sequence of `VehicleCommand`s and mode value(s)
  needed for `rover_differential` to actually move in response to `RoverSpeedSetpoint`/
  `RoverRateSetpoint` — consumed by Task 3's `_ensure_ready()` implementation.

- [ ] **Step 1: Launch a standalone PX4 rover instance (its own reference model, not our Gazebo world) with a live DDS agent**

```bash
cd /home/parvu/CaveX-Explorer-Pro/PX4-Autopilot
source /opt/ros/jazzy/setup.bash
make px4_sitl gz_r1_rover
```

(This uses PX4's own reference `r1_rover` Gazebo model, not this project's tracked vehicle —
deliberately, to isolate "does PX4's rover control loop accept DDS setpoints at all" from
"does our own external-odometry EKF2 fusion work," which Task 8 area covers separately once
this task's findings are in hand.)

- [ ] **Step 2: In a second terminal, start the DDS agent PX4's client connects to and confirm the bridge is live**

```bash
source /opt/ros/jazzy/setup.bash
MicroXRCEAgent udp4 -p 8888   # or the project's own micro_ros_agent if it can serve PX4's
                              # client too -- confirm by checking which port/transport this
                              # checkout's uxrce_dds_client module defaults to:
grep -rn "UXRCE_DDS_PRT\|UXRCE_DDS_AG_IP" \
  /home/parvu/CaveX-Explorer-Pro/PX4-Autopilot/src/modules/uxrce_dds_client/
ros2 topic list | grep fmu
```

Expected: `/fmu/out/...` and `/fmu/in/...` topics appear once PX4 connects to the agent.

- [ ] **Step 3: Confirm rover_differential is actually running and what its idle status is**

```bash
ros2 topic echo /fmu/out/vehicle_odometry --once
ros2 topic echo /fmu/out/vehicle_status --once 2>&1 | grep -iE "arming_state|nav_state"
```

Record the real field names/enum values printed (do not assume PX4's documented enum names
without checking — this varies slightly by PX4 release).

- [ ] **Step 4: Publish an ARM VehicleCommand and observe the real result**

```bash
ros2 topic pub --once /fmu/in/vehicle_command px4_msgs/msg/VehicleCommand \
  "{command: 400, param1: 1.0, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"
ros2 topic echo /fmu/out/vehicle_command_ack --once
ros2 topic echo /fmu/out/vehicle_status --once 2>&1 | grep -iE "arming_state"
```

(`command: 400` = `VEHICLE_CMD_COMPONENT_ARM_DISARM`, `param1: 1.0` = arm — confirm this
against `msg/VehicleCommand.msg`'s own comments in this checkout before running, in case the
enum value differs from what's written here.) Record the real `result` field from the ack —
if it's a rejection, the ack's `result` value tells you which precondition failed (e.g.
"DENIED", "TEMPORARILY_REJECTED") — do not just retry blindly, inspect why.

- [ ] **Step 5: Publish a RoverSpeedSetpoint and observe whether the rover actually reports nonzero velocity**

```bash
ros2 topic pub -r 10 /fmu/in/rover_speed_setpoint px4_msgs/msg/RoverSpeedSetpoint \
  "{speed_body_x: 0.5}" &
PUBPID=$!
sleep 5
ros2 topic echo /fmu/out/vehicle_odometry --once
kill -9 $PUBPID
```

Expected: nonzero velocity in the echoed `vehicle_odometry`. If it's still zero, check
`/fmu/out/vehicle_status`'s `nav_state` — PX4 rover setpoints are very likely gated behind a
specific mode (commonly an "Auto"-family or offboard-equivalent mode for direct setpoint
acceptance in recent PX4 rover firmware) that must be explicitly requested via another
`VehicleCommand` (`command: 176` = `VEHICLE_CMD_DO_SET_MODE`) before setpoints are honored —
if so, find the real mode value by reading `msg/VehicleStatus.msg`'s `nav_state` enum
comments in this checkout, request it the same way Step 4 requested arming, and repeat this
step.

- [ ] **Step 6: Write up the real, confirmed sequence**

Append the real findings (exact `VehicleCommand` values that worked, in what order, any
`OffboardControlMode` heartbeat requirement discovered) as a comment block at the top of
Task 3's `cmd_vel_to_px4_rover.py` before writing its `_ensure_ready()` method — Task 3's
Step 3 code below is a structural skeleton only; replace its `# TODO: mode/arm sequence
from Task 2's findings` marker with the real sequence found here.

- [ ] **Step 7: Kill the standalone PX4 instance and agent before moving on**

```bash
ps aux | grep -E "bin/px4|MicroXRCEAgent" | grep -v grep
# kill -9 every PID found; confirm none survive
```

---

## Task 3: `cmd_vel_to_px4_rover.py` — /cmd_vel → PX4 rover DDS setpoints

**Files:**
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_px4_rover.py`
- Modify: `ros2_ws/src/cavex_tracked_vehicle/setup.py` (or `CMakeLists.txt`'s
  `ament_python_install_package` — check which this package already uses) to install the new
  script as a console entry point, same as the existing `cmd_vel_to_ardupilot.py` entry.

**Interfaces:**
- Consumes: `geometry_msgs/msg/Twist` on `/cmd_vel`; `px4_msgs/msg/VehicleCommand`,
  `px4_msgs/msg/RoverSpeedSetpoint`, `px4_msgs/msg/RoverRateSetpoint` (Task 1).
- Produces: publishes `/fmu/in/vehicle_command`, `/fmu/in/rover_speed_setpoint`,
  `/fmu/in/rover_rate_setpoint` — no other task subscribes to these directly (PX4 does).

- [ ] **Step 1: Write the pure conversion function + its self-check first**

`/cmd_vel`'s `linear.x` (m/s forward) maps directly to `RoverSpeedSetpoint.speed_body_x`
(confirm the exact field name against Task 1's generated/copied `RoverSpeedSetpoint.msg` —
this plan's guess may not match); `angular.z` (rad/s yaw rate) maps directly to
`RoverRateSetpoint`'s yaw-rate field. Unlike the ArduPilot bridge, this is NOT a
body-to-world rotation — PX4's rover setpoints are body-frame speed + body-frame yaw rate
directly, so no ground-truth-heading lookup is needed here at all (simpler than
`cmd_vel_to_ardupilot.py`). Confirm this assumption against the real `.msg` field
comments before relying on it.

```python
def cmd_vel_to_rover_setpoints(linear_x: float, angular_z: float):
    """Pure passthrough -- PX4 rover setpoints are body-frame already, no
    rotation needed (unlike ArduPilot's AP_DDS body_link path). Returns
    (speed_body_x, yaw_rate) as a plain tuple for the self-check below."""
    return linear_x, angular_z


def _self_check():
    speed, yaw_rate = cmd_vel_to_rover_setpoints(0.5, -0.2)
    assert speed == 0.5 and yaw_rate == -0.2, (speed, yaw_rate)
    print("cmd_vel_to_px4_rover self-check: OK")
```

- [ ] **Step 2: Run the self-check to confirm it passes (it's trivial by construction, but confirms the module imports cleanly)**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
python3 src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_px4_rover.py --self-check
```

Expected: `cmd_vel_to_px4_rover self-check: OK`

- [ ] **Step 3: Write the full node**

```python
#!/usr/bin/env python3
"""
cmd_vel_to_px4_rover.py

Relays /cmd_vel into PX4's DDS rover setpoint topics
(/fmu/in/rover_speed_setpoint, /fmu/in/rover_rate_setpoint), and arms +
sets whatever mode PX4's rover_differential module requires to honor
external setpoints on the first /cmd_vel received.

# TODO: mode/arm sequence from Task 2's findings -- replace this comment
# with the real, empirically-confirmed VehicleCommand sequence (command
# values, params, ordering, any OffboardControlMode heartbeat requirement)
# found by Task 2. Do not ship this node with an untested guessed sequence.
"""
import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleCommand, RoverSpeedSetpoint, RoverRateSetpoint

ARM_RETRY_MIN_INTERVAL_S = 2.0
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
# TODO (Task 2): confirm this is the real mode command needed, and its
# real param1 value for whatever mode rover_differential requires.
VEHICLE_CMD_DO_SET_MODE = 176


def cmd_vel_to_rover_setpoints(linear_x: float, angular_z: float):
    """Pure passthrough -- PX4 rover setpoints are body-frame already, no
    rotation needed (unlike ArduPilot's AP_DDS body_link path)."""
    return linear_x, angular_z


class CmdVelToPX4Rover(RclpyNode):
    def __init__(self):
        super().__init__('cmd_vel_to_px4_rover')
        self.speed_pub = self.create_publisher(
            RoverSpeedSetpoint, '/fmu/in/rover_speed_setpoint', 10)
        self.rate_pub = self.create_publisher(
            RoverRateSetpoint, '/fmu/in/rover_rate_setpoint', 10)
        self.cmd_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)

        self._armed = False
        self._mode_set = False
        self._last_arm_attempt = None
        self._last_mode_attempt = None
        self.get_logger().info(
            "cmd_vel_to_px4_rover ready: relaying /cmd_vel -> PX4 rover "
            "DDS setpoints; will arm + set mode on first /cmd_vel "
            "(retried until confirmed via /fmu/out/vehicle_status).")

    def _send_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.command = command
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

    def _ensure_ready(self):
        if self._armed and self._mode_set:
            return
        now = self.get_clock().now()
        min_interval = Duration(seconds=ARM_RETRY_MIN_INTERVAL_S)
        if (not self._mode_set and (self._last_mode_attempt is None
                or now - self._last_mode_attempt >= min_interval)):
            self._last_mode_attempt = now
            # TODO (Task 2): real param1 mode value goes here.
            self._send_vehicle_command(VEHICLE_CMD_DO_SET_MODE, param1=0.0)
        if (not self._armed and (self._last_arm_attempt is None
                or now - self._last_arm_attempt >= min_interval)):
            self._last_arm_attempt = now
            self._send_vehicle_command(
                VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        # TODO (Task 2): subscribe /fmu/out/vehicle_status and set
        # self._armed/self._mode_set from its real fields instead of this
        # placeholder -- do not ship without confirming arming/mode from
        # real feedback, the same way cmd_vel_to_ardupilot.py confirms via
        # its service response's success field, not by assumption.

    def _cb(self, msg: Twist):
        self._ensure_ready()
        speed, yaw_rate = cmd_vel_to_rover_setpoints(
            msg.linear.x, msg.angular.z)
        speed_msg = RoverSpeedSetpoint()
        speed_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        speed_msg.speed_body_x = speed
        self.speed_pub.publish(speed_msg)
        rate_msg = RoverRateSetpoint()
        rate_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        rate_msg.yaw_rate_setpoint = yaw_rate
        self.rate_pub.publish(rate_msg)


def _self_check():
    speed, yaw_rate = cmd_vel_to_rover_setpoints(0.5, -0.2)
    assert speed == 0.5 and yaw_rate == -0.2, (speed, yaw_rate)
    print("cmd_vel_to_px4_rover self-check: OK")


def main(args=None):
    import sys
    if args is None:
        args = sys.argv[1:]
    if '--self-check' in args:
        _self_check()
        return
    rclpy.init(args=args)
    node = CmdVelToPX4Rover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 4: Register the console entry point**

Check whether `cavex_tracked_vehicle` uses `setup.py` (`entry_points`/`console_scripts`) or
pure `CMakeLists.txt`/`ament_python_install_package` for its existing nodes (e.g. how
`cmd_vel_to_ardupilot` is registered) and add `cmd_vel_to_px4_rover` the same way, in the
same file.

- [ ] **Step 5: Build and run the self-check for real (not just the standalone module import)**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select cavex_tracked_vehicle --symlink-install
source install/setup.bash
ros2 run cavex_tracked_vehicle cmd_vel_to_px4_rover.py --self-check
```

Expected: `cmd_vel_to_px4_rover self-check: OK`

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_px4_rover.py \
        ros2_ws/src/cavex_tracked_vehicle/setup.py
git commit -m "Add cmd_vel_to_px4_rover.py: /cmd_vel -> PX4 rover DDS setpoints"
```

---

## Task 4: `px4_visual_odom_bridge.py` — ground-truth Odometry → PX4 external vision fusion

**Files:**
- Create:
  `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/px4_visual_odom_bridge.py`

**Interfaces:**
- Consumes: `nav_msgs/msg/Odometry` on a configurable input topic (ROS2 parameter
  `input_odom_topic`, defaulting to `/odom_ground_truth` — the tracked vehicle's existing
  ground-truth publisher from `tracked_vehicle_ground_truth_odom.py`).
- Produces: `px4_msgs/msg/VehicleOdometry` (confirm the exact message name Task 1's
  generated package uses for the *input*-direction external-vision topic — PX4's own
  `dds_topics.yaml` naming for `/fmu/in/vehicle_visual_odometry` may map to a
  `VehicleOdometry`-typed message reused for both directions, or a distinct
  `VehicleVisualOdometry` type; check `dds_topics.yaml`'s `type:` field for that specific
  topic entry directly, do not assume) on `/fmu/in/vehicle_visual_odometry` (or a
  vehicle-specific namespaced variant when running two PX4 instances — see Task 6).

- [ ] **Step 1: Confirm the real message type for `/fmu/in/vehicle_visual_odometry`**

```bash
grep -B1 -A1 "vehicle_visual_odometry" \
  /home/parvu/CaveX-Explorer-Pro/PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

If this topic isn't present in the default config at all (unlike the rover setpoint topics,
this wasn't confirmed present during the design brainstorm — verify now, don't assume), it
needs to be added to `dds_topics.yaml` and PX4 rebuilt (breaks the "no PX4 firmware config
changes needed" property Tasks 2/3 relied on for the rover setpoint path — flag this to
the user if it happens, since it changes the plan's risk profile).

- [ ] **Step 2: Write the node**

```python
#!/usr/bin/env python3
"""
px4_visual_odom_bridge.py

Republishes a ground-truth nav_msgs/Odometry topic (this project's own,
e.g. tracked_vehicle_ground_truth_odom.py's /odom_ground_truth) into PX4's
external-vision EKF2 fusion input, so PX4's state estimator has a valid
pose/velocity without needing PX4's own gz_bridge module to understand this
project's non-PX4-shaped Gazebo sensor topics. One instance per PX4
vehicle instance, parametrized by which ground-truth topic to read and
which PX4 instance's /fmu/in/... namespace to publish into.
"""
import rclpy
from rclpy.node import Node as RclpyNode
from nav_msgs.msg import Odometry
# TODO (Step 1): import the real confirmed message type, e.g.:
# from px4_msgs.msg import VehicleOdometry as PX4VehicleOdometry


class PX4VisualOdomBridge(RclpyNode):
    def __init__(self):
        super().__init__('px4_visual_odom_bridge')
        self.declare_parameter('input_odom_topic', '/odom_ground_truth')
        self.declare_parameter('output_topic', '/fmu/in/vehicle_visual_odometry')
        in_topic = self.get_parameter('input_odom_topic').value
        out_topic = self.get_parameter('output_topic').value
        # TODO (Step 1): real message type here.
        self.pub = self.create_publisher(Odometry, out_topic, 10)
        self.create_subscription(Odometry, in_topic, self._cb, 10)
        self.get_logger().info(
            f"px4_visual_odom_bridge ready: {in_topic} -> {out_topic}")

    def _cb(self, msg: Odometry):
        # TODO (Step 1): construct the real PX4VehicleOdometry message,
        # mapping msg.pose.pose.position/orientation into its real field
        # names (check the .msg file directly -- PX4's VehicleOdometry
        # typically uses a flat q[4]/position[3] array layout, NOT nested
        # geometry_msgs-style fields, so this is not a 1:1 field copy).
        out = self.pub.msg_type()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PX4VisualOdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

- [ ] **Step 3: Build**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --packages-select cavex_tracked_vehicle --symlink-install
```

Expected: clean build (this file has real TODOs from Step 1's findings — resolve them
before this step, not after; "no placeholders" applies to what ships, Step 1's discovery
happens first).

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/px4_visual_odom_bridge.py
git commit -m "Add px4_visual_odom_bridge.py: ground-truth Odometry -> PX4 EKF2 external vision"
```

---

## Task 5: `px4_rover_twist_bridge.py` — PX4 rover odometry → /track_cmd_vel

**Files:**
- Create:
  `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/px4_rover_twist_bridge.py`

**Interfaces:**
- Consumes: `px4_msgs/msg/VehicleOdometry` on `/fmu/out/vehicle_odometry` (Task 1).
- Produces: `geometry_msgs/msg/Twist` on `/track_cmd_vel` — same output topic
  `track_cmd_vel_bridge.py` already produces (this node fully replaces it on the
  `px4-rover-sitl` branch; do not run both simultaneously).

- [ ] **Step 1: Confirm `VehicleOdometry`'s real velocity field layout**

```bash
ros2 interface show px4_msgs/msg/VehicleOdometry
```

PX4's `VehicleOdometry` reports velocity in a frame indicated by its own `velocity_frame`
field (commonly NED or the vehicle's local frame, NOT a fixed ENU/body convention like
ArduPilot's `/ap/twist/filtered`) — read the message's own field comments and
`velocity_frame` enum before assuming a fixed conversion; this determines whether this
node needs a rotation step at all (unlike the ArduPilot bridge, which always rotates
world→body using ground-truth yaw).

- [ ] **Step 2: Write the node**, following `track_cmd_vel_bridge.py`'s structure
  (subscribe PX4 odometry, convert velocity into `/track_cmd_vel`'s expected body-frame
  Forward/Left/yaw-rate convention per Step 1's real findings, publish). Reuse
  `track_cmd_vel_bridge.py`'s `world_to_body()` pure function directly (import it, don't
  duplicate) if Step 1 confirms the velocity is genuinely world-frame; skip the rotation
  entirely if it's already body-frame.

- [ ] **Step 3: Build**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --packages-select cavex_tracked_vehicle --symlink-install
```

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/px4_rover_twist_bridge.py
git commit -m "Add px4_rover_twist_bridge.py: PX4 rover odometry -> /track_cmd_vel"
```

---

## Task 6: Rover launch integration

**Files:**
- Modify: `ros2_ws/src/cavex_tracked_vehicle/launch/gazebo_tracked_vehicle.launch.py:310-325`
  (the `ardupilot_sitl_launch` `IncludeLaunchDescription` block and its preceding comment)
- Modify: `ros2_ws/src/cavex_tracked_vehicle/package.xml` (add `<exec_depend>px4_msgs</exec_depend>`)

**Interfaces:**
- Consumes: `cmd_vel_to_px4_rover.py` (Task 3), `px4_visual_odom_bridge.py` (Task 4),
  `px4_rover_twist_bridge.py` (Task 5) — all three get `Node(...)` entries in this launch
  file.
- Produces: a running PX4 rover SITL instance + all three bridge nodes, wired the same way
  `ardupilot_sitl_launch` + `cmd_vel_to_ardupilot` + `track_cmd_vel_bridge` are today.

- [ ] **Step 1: Read the current `ardupilot_sitl_launch` block and its surrounding context**

```bash
sed -n '305,330p' /home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_tracked_vehicle/launch/gazebo_tracked_vehicle.launch.py
```

- [ ] **Step 2: Replace it with an `ExecuteProcess` launching the built PX4 rover binary**

```python
from launch.actions import ExecuteProcess

px4_rover_env = os.environ.copy()
px4_rover_env['PX4_SIM_MODEL'] = 'none'  # not connecting to Gazebo via gz_bridge at all --
                                          # see Global Constraints; confirm 'none' is a real
                                          # accepted value in this checkout before relying on
                                          # it (Task 2's own Step 1 used gz_r1_rover only to
                                          # isolate the DDS-acceptance question -- for the
                                          # real integration here we deliberately do NOT want
                                          # gz_bridge trying to connect to our Gazebo world).
px4_rover_env['PX4_SIMULATOR'] = 'none'  # same rationale -- confirm this disables gz_bridge
                                          # entirely rather than erroring; if it errors,
                                          # investigate PX4's own '--help'/module list for
                                          # a genuinely no-op simulator backend instead of
                                          # guessing further flags here.
px4_rover_process = ExecuteProcess(
    cmd=['/home/parvu/CaveX-Explorer-Pro/PX4-Autopilot/build/px4_sitl_default/bin/px4',
         '-i', '0'],
    env=px4_rover_env,
    output='screen',
)
```

**Note:** this step's `PX4_SIM_MODEL=none`/`PX4_SIMULATOR=none` values are a plan-time
best-guess at "run PX4 without Gazebo," not confirmed — verify PX4 actually starts cleanly
and reaches a state where `rover_differential`/EKF2 run at all under this configuration
before trusting it; if PX4 refuses to start without a real simulator backend, this may need
`PX4_SIMULATOR=gz` pointed at a lightweight/empty placeholder Gazebo world instead (adds a
second, otherwise-unused Gazebo instance just to satisfy PX4's startup requirements) — an
open question this task's own live testing must resolve, not assume away.

- [ ] **Step 3: Add the three bridge nodes to the launch description**

```python
cmd_vel_to_px4_rover_node = Node(
    package='cavex_tracked_vehicle',
    executable='cmd_vel_to_px4_rover.py',
    name='cmd_vel_to_px4_rover',
    output='screen',
)
px4_visual_odom_bridge_node = Node(
    package='cavex_tracked_vehicle',
    executable='px4_visual_odom_bridge.py',
    name='px4_visual_odom_bridge',
    output='screen',
    parameters=[{'input_odom_topic': '/odom_ground_truth',
                 'output_topic': '/fmu/in/vehicle_visual_odometry'}],
)
px4_rover_twist_bridge_node = Node(
    package='cavex_tracked_vehicle',
    executable='px4_rover_twist_bridge.py',
    name='px4_rover_twist_bridge',
    output='screen',
)
```

Add `px4_rover_process`, `cmd_vel_to_px4_rover_node`, `px4_visual_odom_bridge_node`,
`px4_rover_twist_bridge_node` to the file's `LaunchDescription([...])` list, in place of
`ardupilot_sitl_launch`.

**Note:** `px4_visual_odom_bridge_node` depends on `/odom_ground_truth`, which
`tracked_vehicle_ground_truth_odom.py` publishes — confirm that node is launched by THIS
file already (it may currently be launched from `tracked_vehicle_slam.launch.py` instead;
check before assuming it's available at this launch's startup) and adjust which launch file
starts `px4_visual_odom_bridge_node` accordingly if not.

- [ ] **Step 4: Add the `px4_msgs` dependency**

```xml
<exec_depend>px4_msgs</exec_depend>
```

in `ros2_ws/src/cavex_tracked_vehicle/package.xml`, alongside the existing
`<exec_depend>cavex_perception</exec_depend>`.

- [ ] **Step 5: Build**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --packages-select cavex_tracked_vehicle --symlink-install
```

- [ ] **Step 6: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/launch/gazebo_tracked_vehicle.launch.py \
        ros2_ws/src/cavex_tracked_vehicle/package.xml
git commit -m "Launch PX4 rover SITL + bridge nodes in place of ardurover"
```

---

## Task 7: Live end-to-end verification — rover

**Files:** none (verification-only task)

**Interfaces:** none produced; validates Tasks 1-6 together.

- [ ] **Step 1: Full launch**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
source /opt/ros/jazzy/setup.bash && source install/setup.bash
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py > /tmp/px4_rover_test.log 2>&1 &
sleep 30
grep -iE "error|traceback|exception" /tmp/px4_rover_test.log | grep -v "^--"
```

Expected: no errors attributable to the new PX4 nodes (an `ArduPilotPlugin`-related warning,
if any leftover SDF reference still exists, is a separate pre-existing cosmetic issue, not a
new failure from this work — but there should be none of those either, since Global
Constraints says the SDF is unchanged).

- [ ] **Step 2: Confirm the PX4 process and all three bridge nodes are alive**

```bash
ros2 node list | grep -iE "cmd_vel_to_px4_rover|px4_visual_odom_bridge|px4_rover_twist_bridge"
ps aux | grep "bin/px4" | grep -v grep
```

- [ ] **Step 3: Drive it**

```bash
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.4}, angular: {z: 0.0}}" &
DRIVEPID=$!
sleep 15
kill -9 $DRIVEPID
gz topic -t /world/cavex_world/pose/info -e -n 1 | grep -A4 'name: "cavex_tracked_blueboat"'
```

Expected: real position change from wherever the vehicle started, confirming the full round
trip (`/cmd_vel` → PX4 → `/fmu/out/vehicle_odometry` → `/track_cmd_vel` → gz-transport →
`TrackedVehicle` plugin) actually moves the vehicle.

- [ ] **Step 4: Kill everything launched**

```bash
ps aux | grep -iE "gz sim|bin/px4|cmd_vel_to_px4|px4_visual_odom|px4_rover_twist|MicroXRCEAgent|micro_ros_agent" | grep -v grep
# kill -9 every PID found; confirm none survive
```

---

## Task 8: Discovery — standalone PX4 sub SITL, real `uuv_att_control` manual-input acceptance

Same shape as Task 2, but for the sub side. Uses PX4's own reference
`60002_gz_uuv_bluerov2_heavy` airframe/model to isolate "does `uuv_att_control` respond to
`manual_control_input` at all" from the external-odometry integration question, exactly as
Task 2 did for the rover.

**Files:** none (diagnostic-only task)

**Interfaces:**
- Produces: the real, empirically-confirmed sequence needed for `uuv_att_control` to accept
  `manual_control_input` and drive real thrust — consumed by Task 9.

- [ ] **Step 1: Launch a standalone PX4 sub instance**

```bash
cd /home/parvu/CaveX-Explorer-Pro/PX4-Autopilot
make px4_sitl gz_uuv_bluerov2_heavy
```

- [ ] **Step 2: Start the DDS agent, confirm bridge is live** (same as Task 2 Step 2, on a
  different agent port if Task 2's instance might still be running — use a distinct port to
  test both independently, or ensure Task 2's instance is fully torn down first per its own
  Step 7).

- [ ] **Step 3: Arm it** (same `VehicleCommand` pattern as Task 2 Step 4 — confirm the real
  ack, do not assume the same values necessarily apply to this different vehicle type
  without checking).

- [ ] **Step 4: Publish a `ManualControlSetpoint`-equivalent input and observe thrust/attitude response**

```bash
ros2 interface show px4_msgs/msg/ManualControlSetpoint
```

Read the real field names first (this plan does not assume them — UUV manual control
typically maps forward/lateral/vertical/yaw onto whatever this message's roll/pitch/
throttle/yaw-style fields are, but confirm against the actual `.msg` file, since a wrong
field mapping here would silently produce nonsensical thrust). Then:

```bash
ros2 topic pub -r 10 /fmu/in/manual_control_input px4_msgs/msg/ManualControlSetpoint \
  "{<real fields from above, e.g. throttle: 0.3>}" &
PUBPID=$!
sleep 5
ros2 topic echo /fmu/out/vehicle_odometry --once
kill -9 $PUBPID
```

- [ ] **Step 5: Write up the real, confirmed sequence** — same as Task 2 Step 6, for Task 9's
  `_ensure_ready()` and manual-control field mapping.

- [ ] **Step 6: Kill the standalone instance and agent**

---

## Task 9: `cmd_vel_to_px4_sub.py` — /cmd_vel_rov → PX4 manual_control_input

**Files:**
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_px4_sub.py`

**Interfaces:**
- Consumes: `geometry_msgs/msg/Twist` on `/cmd_vel_rov` (same input topic
  `cmd_vel_to_ardusub.py` already reads); `px4_msgs/msg/ManualControlSetpoint`,
  `px4_msgs/msg/VehicleCommand` (Task 1).
- Produces: `/fmu/in/manual_control_input`, `/fmu/in/vehicle_command` — on the sub PX4
  instance's own namespaced topics (see Task 11 for the dual-instance namespacing pattern,
  mirroring how this project already runs two simultaneous ArduPilot instances today).

- [ ] **Step 1: Read `cmd_vel_to_ardusub.py` fully first**, to match its existing
  `/cmd_vel_rov` input contract and any project-specific conventions (deadzone, max speed
  clamping, etc.) it already established:

```bash
cat /home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_ardusub.py
```

- [ ] **Step 2: Write the node**, following `cmd_vel_to_ardupilot.py`'s arm/mode-retry
  structure (Task 3's pattern), but publishing `ManualControlSetpoint` per Task 8's
  confirmed field mapping instead of rover setpoints, matching whatever
  linear.x/y/z + angular.z → forward/lateral/vertical/yaw convention
  `cmd_vel_to_ardusub.py` already uses for consistency with the existing manual-teleop
  contract.

- [ ] **Step 3: Build**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --packages-select cavex_tracked_vehicle --symlink-install
```

- [ ] **Step 4: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/cmd_vel_to_px4_sub.py
git commit -m "Add cmd_vel_to_px4_sub.py: /cmd_vel_rov -> PX4 manual_control_input"
```

---

## Task 10: `bluerov2_ground_truth_odom.py` + `px4_sub_twist_bridge.py`

**Files:**
- Create:
  `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/bluerov2_ground_truth_odom.py`
  — copy `tracked_vehicle_ground_truth_odom.py` exactly, changing only
  `VEHICLE_MODEL_NAME = 'cavex_tracked_blueboat'` to `VEHICLE_MODEL_NAME = 'bluerov2'` and
  the output topic to `/odom_ground_truth_bluerov2` (must differ from the tracked vehicle's
  `/odom_ground_truth` — both are running simultaneously).
- Create: `ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/px4_sub_twist_bridge.py`
  — mirrors Task 5's `px4_rover_twist_bridge.py`, but publishes to whatever gz-transport
  topic the BlueROV2's own thruster plugin expects (check
  `ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf` for its real command topic
  name — do not assume it matches the tracked vehicle's `/model/.../cmd_vel` pattern
  exactly).

**Interfaces:**
- Consumes: `bluerov2_ground_truth_odom.py` produces `nav_msgs/msg/Odometry` on
  `/odom_ground_truth_bluerov2`, consumed by a second `px4_visual_odom_bridge.py` instance
  (Task 4, parametrized via its `input_odom_topic`/`output_topic` ROS2 parameters — no code
  change needed, just a second `Node(...)` launch entry with different parameter values).
- Produces: `px4_sub_twist_bridge.py` subscribes the sub PX4 instance's own
  `/fmu/out/vehicle_odometry` (namespaced — see Task 11) and publishes the BlueROV2's real
  thruster command topic (found via the model.sdf check above).

- [ ] **Step 1: Copy and adapt the ground-truth node**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle
cp tracked_vehicle_ground_truth_odom.py bluerov2_ground_truth_odom.py
```

Edit the copy: change `VEHICLE_MODEL_NAME` to `'bluerov2'`, the publisher topic to
`/odom_ground_truth_bluerov2`, and the node name to `'bluerov2_ground_truth_odom'`.

- [ ] **Step 2: Find the BlueROV2's real thruster command topic**

```bash
grep -B3 -A3 "cmd_vel\|thrust\|<topic>" \
  /home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_tracked_vehicle/models/bluerov2/model.sdf
```

- [ ] **Step 3: Write `px4_sub_twist_bridge.py`** following Task 5's structure, publishing
  to whatever topic Step 2 found (via the same `ros_gz_bridge` `parameter_bridge`
  mechanism `track_cmd_vel_bridge.py` relies on — check
  `config/gazebo_tracked_vehicle_bridge.yaml` for whether a bridge entry for this BlueROV2
  topic already exists from the ArduSub integration, or needs a new one added).

- [ ] **Step 4: Build**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --packages-select cavex_tracked_vehicle --symlink-install
```

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/bluerov2_ground_truth_odom.py \
        ros2_ws/src/cavex_tracked_vehicle/cavex_tracked_vehicle/px4_sub_twist_bridge.py \
        ros2_ws/src/cavex_tracked_vehicle/config/gazebo_tracked_vehicle_bridge.yaml
git commit -m "Add BlueROV2 ground-truth publisher and PX4 sub twist bridge"
```

---

## Task 11: Sub launch integration (dual PX4 instance namespacing)

**Files:**
- Modify: whichever launch file currently includes
  `ardupilot_sitl/launch/sitl_dds_udp.launch.py` with `command:=ardusub` (search for it —
  likely `gazebo_tracked_vehicle.launch.py` alongside the rover's own ArduPilot launch, per
  this project's existing dual-instance pattern documented in `launch.txt` section 8b).

**Interfaces:**
- Consumes: `cmd_vel_to_px4_sub.py` (Task 9), `bluerov2_ground_truth_odom.py` +
  `px4_sub_twist_bridge.py` (Task 10), a second `px4_visual_odom_bridge.py` instance
  (Task 4).
- Produces: a second, independently-addressable running PX4 SITL instance for the BlueROV2.

- [ ] **Step 1: Find the current ArduSub launch block**

```bash
grep -rn "ardusub\|sitl_dds_udp" /home/parvu/CaveX-Explorer-Pro/ros2_ws/src/cavex_tracked_vehicle/launch/*.py
```

- [ ] **Step 2: Add a second `ExecuteProcess` for the PX4 sub instance, using PX4's own
  multi-instance flag (`-i 1`, distinct from the rover's `-i 0` in Task 6) so both DDS
  clients can run simultaneously without port collision** — mirroring this project's
  existing real port-separation pattern for its two simultaneous ArduPilot instances (see
  `launch.txt` section 8b's `port:=2029`/`sim_port_in:=9013`/`sim_port_out:=9012` — find
  PX4's own equivalent instance-separation mechanism, likely a `-i` flag offsetting default
  ports by a fixed stride per instance; confirm the real port PX4 uses for instance 1 via
  `ros2 topic list` after launch, don't assume).

- [ ] **Step 3: Add the four sub-side nodes to the launch description**: `cmd_vel_to_px4_sub`
  (Task 9), `bluerov2_ground_truth_odom` (Task 10), a second `px4_visual_odom_bridge`
  instance parametrized for the BlueROV2's own topics, `px4_sub_twist_bridge` (Task 10) — all
  namespaced/parametrized to talk to the sub PX4 instance's own `/fmu/...` topics, not the
  rover's (PX4's multi-instance DDS setup typically namespaces `/fmu/...` under a per-instance
  prefix — confirm the real topic names via `ros2 topic list` with both instances running
  before wiring this up, do not assume they're identically-named and rely on node-level
  remapping alone without checking for an actual collision).

- [ ] **Step 4: Build**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
colcon build --packages-select cavex_tracked_vehicle --symlink-install
```

- [ ] **Step 5: Commit**

```bash
cd /home/parvu/CaveX-Explorer-Pro
git add ros2_ws/src/cavex_tracked_vehicle/launch/
git commit -m "Launch second PX4 SITL instance for BlueROV2 in place of ArduSub"
```

---

## Task 12: Live end-to-end verification — sub

**Files:** none (verification-only task)

- [ ] **Step 1: Full launch with both PX4 instances**

```bash
cd /home/parvu/CaveX-Explorer-Pro/ros2_ws
ros2 launch cavex_tracked_vehicle gazebo_tracked_vehicle.launch.py > /tmp/px4_both_test.log 2>&1 &
sleep 40
grep -iE "error|traceback|exception" /tmp/px4_both_test.log | grep -v "^--"
```

- [ ] **Step 2: Confirm both PX4 instances and all sub-side nodes are alive**

```bash
ps aux | grep "bin/px4" | grep -v grep   # expect 2 processes
ros2 node list | grep -iE "cmd_vel_to_px4_sub|bluerov2_ground_truth_odom|px4_sub_twist_bridge"
```

- [ ] **Step 3: Drive the BlueROV2**

```bash
ros2 topic pub -r 10 /cmd_vel_rov geometry_msgs/msg/Twist "{linear: {x: 0.3}}" &
PUBPID=$!
sleep 15
kill -9 $PUBPID
gz topic -t /world/cavex_world/pose/info -e -n 1 | grep -A5 'name: "bluerov2"'
```

Expected: real position change, confirming the full sub-side round trip works.

- [ ] **Step 4: Kill everything launched**

```bash
ps aux | grep -iE "gz sim|bin/px4|cmd_vel_to_px4|px4_visual_odom|px4_.*_twist_bridge|bluerov2_ground_truth|MicroXRCEAgent|micro_ros_agent" | grep -v grep
# kill -9 every PID found; confirm none survive
```

- [ ] **Step 5: Update `launch.txt`/`README.md`** on the `px4-rover-sitl` branch once both
  vehicles are confirmed working end-to-end, following this project's own established
  documentation convention (see the SIC-SLAM and ArduPilot history entries in `launch.txt`
  for the expected level of detail: what changed, what was live-verified, what real bugs
  were found and fixed along the way).
