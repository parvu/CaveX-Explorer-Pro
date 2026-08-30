#!/usr/bin/env python3
"""
vehicle_switch_node.py

Watches the tracked vehicle's real ground-truth pose (/odom_ground_truth,
Task 13's real gz-transport-sourced Odometry -- see
tracked_vehicle_ground_truth_odom.py) and drives the tracks<->props
locomotion-mode state machine, publishing it on /cavex/locomotion_mode
(std_msgs/String: "tracks"/"retracting"/"props"/"deploying") for
boat_thruster_control.py, manual_gui_bridge.py, and cmd_vel_gz_bridge.py
to gate off of.

Real request, 2026-08-26 ("make bluerov2 static in reference with
blueboat not the world"): the ROV lock/unlock and motorized-tether
logic this node used to also own were removed. bluerov2 is now a fixed
child link of the boat's own model (model.sdf.tracked's bluerov2_link)
instead of a separately spawned entity held on by a DetachableJoint +
tether -- there is no longer a separate ROV to lock, unlock, release,
or tether. See perception branch for the full, functional,
independently-swimming, tethered BlueROV2.

Real request 2026-08-27: this used to just fire a single "retracted"/
"deployed" command on a fixed WATER_BOUNDARY_X crossing (see git history),
with track_retract_control.py, boat_thruster_control.py, and
manual_gui_bridge.py/cmd_vel_gz_bridge.py each *independently* re-deriving
their own x/z thresholds to decide when to actually act -- the exact
duplication that caused two stale-constant bugs the same day (both files
fell out of sync with a water-surface-height change). Replaced with a
single real state machine, owned here, that's the one source of truth:

  tracks --[buoyant]--> retracting --[2s]--> props --[<1m from shore]--> deploying --[2s]--> tracks

"Buoyant" is z >= FLOAT_Z_MIN (boat_buoyancy_control.py's lift has it up
near its target float height, not still resting on the cave floor).
"Close to shore" is real distance to the nearest dry-floor collision
geometry (cave_floor_patch/_scaled/_bridge), NOT a fixed X
coordinate -- since the dry-floor footprint is an irregular multi-patch
shape (see cavex_world.world), not a simple x-threshold. Both
transitions send the existing /cavex/tracks/command String
track_retract_control.py already consumes to actually move the retract
joints; the 2s hold in "retracting"/"deploying" matches that node's own
JointTrajectory duration (track motors stay live through "retracting" so
the vehicle keeps driving while the tracks lift, thrusters stay live
through "deploying" so it keeps driving while they redeploy) -- track
motors and props are never both active, both idle, at the same instant
past that transition, by construction of this state machine.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Real bug found live 2026-08-27, same session: a Z-only "buoyant" check
# fired the instant the vehicle spawned, on DRY LAND at x=-88.78 -- the
# boat's real RESTING height on dry ground (~6.65, hull clearance above
# the 5.9 floor) turned out HIGHER than its floating TARGET_FLOAT_Z in the
# new shallow basin (6.07, boat_buoyancy_control.py), so no single ">="
# threshold can tell "resting on dry ground" from "floating" by height
# alone. "Buoyant" now also requires being inside the water region's own
# real footprint (WATER_BOX below, matching the boundary walls actually
# built in cavex_world.world) -- not an arbitrary coordinate, the same
# real known geometry the shore-distance check below already uses.
# 2026-08-28: water surface raised 6.0 -> 7.0 (boat_buoyancy_control.py
# TARGET_FLOAT_Z now 7.06), so this "is it actually floating" gate follows.
# 2026-08-30: water surface LOWERED 6.0 -> 5.65 so the cave-floor shore
# (z~5.98) is a dry beach. The vehicle now RESTS HIGHER on land (base_link
# ~6.36) than it FLOATS (target ~5.77), so the test flips: "buoyant" = has
# descended into the water, z <= FLOAT_Z_MAX, while inside the water box.
FLOAT_Z_MAX = 6.20  # float line ~6.10 (surface 5.98 + 0.12); land-rest ~6.36 sits above this
# Real request 2026-08-27: water east edge trimmed x=70 -> x=40 (see
# cavex_world.world's water_surface / *_boundary_wall / basin_floor). This
# box must stay matched to those walls.
WATER_BOX = (-0.3, 40.0, -12.0, 25.0)  # x0, x1, y0, y1
# 2026-08-29: x0 0.0 -> -0.3 to match boat_buoyancy_control.py's
# WATER_BOUNDARY_X -- tracks retract as soon as the vehicle reaches the
# wet foot of water_entry_ramp (x~-0.22), not 0.3 m later.


def _in_water_box(x, y):
    x0, x1, y0, y1 = WATER_BOX
    return x0 <= x <= x1 and y0 <= y <= y1

# Real request 2026-08-27: "allow tracks deploy when close under 1m from
# shore" -- real collision distance to the nearest dry-floor geometry,
# not a fixed X coordinate.
#
# Real bug fixed 2026-08-27: these AABBs previously claimed the SUBMERGED
# floor as "shore" -- so a floating boat flip-flopped tracks<->props
# forever. "Shore" is only where the drivable dry floor is near the
# surface. 2026-08-28: cave_floor_patch flat top now ends at x=-10;
# cave_entry_ramp (x[-10,5], z 5.9->3.0) carries the descent into the
# basin. "Shore" = the ramp's shallow top, x ~ -10..-7 near z=5.9.
# 2026-08-29: the cave_floor_patch* box MODELS were deleted from
# cavex_world.world (floor collision is now the cavex_floor_seal sheet baked into
# cave_world_holed.obj). These AABBs are unchanged and still correct -- just
# world-frame rectangles marking the drivable dry floor for the shore-distance
# check. Nothing here ever queried the models by name; the names are region labels.
SHORE_DISTANCE_M = 1.0
# 2026-08-30: the dry cave floor / seal now runs all the way east to the
# water_entry_ramp top at x=-1 (there is no separate cave_entry_ramp any
# more), so 'dry_approach' extends to -1, and 'ramp_shore' covers the ramp
# top + shallows so a boat driving IN from the water trips the props->tracks
# handoff there instead of only at x=-10 (which it can never float to --
# the submerged cave-floor lip at z~5.98 stops the hull at ~x=-0.3).
DRY_BOXES = [
    # name, x0, x1, y0, y1, z0, z1
    ('dry_approach', -40.0,  0.0, -12.0, 12.0, 4.9, 6.1),
    ('shore_edge', -1.0, 1.5, -13.0, 13.0, 4.5, 6.3),  # right at the graded-ramp lip; x=4 caused a deploy/retract flip-flop
    ('spawn_area', -120.0, -60.0, -45.9, -16.9, 4.9, 5.9),
    ('spawn_approach_link', -62.0, -38.0, -46.0, 12.0, 4.9, 5.9),
]

# Matches track_retract_control.py's own JointTrajectory duration (2s) --
# how long "retracting"/"deploying" holds before the mode machine
# considers the joint move complete. No joint-state feedback subscription
# exists (or is needed) for this; reusing the same fixed duration that
# node already commands is the simplest correct source of truth.
TRANSITION_DURATION_S = 2.0


def _aabb_distance(px, py, pz, box):
    _, x0, x1, y0, y1, z0, z1 = box
    dx = max(x0 - px, 0.0, px - x1)
    dy = max(y0 - py, 0.0, py - y1)
    dz = max(z0 - pz, 0.0, pz - z1)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def _distance_to_shore(x, y, z):
    return min(_aabb_distance(x, y, z, box) for box in DRY_BOXES)


class VehicleSwitchNode(Node):
    def __init__(self):
        super().__init__('vehicle_switch_node')
        self._mode = 'tracks'
        self._transition_deadline = None
        # Manual override: once an operator flips the Track up/down switch
        # (web viewer -> /cavex/track_cmd -> manual_gui_bridge ->
        # /cavex/tracks/command), the automatic odom-driven state machine
        # stands down for the rest of the run and the switch owns the
        # locomotion mode directly: tracks down -> 'tracks' (track motors),
        # tracks up -> 'props' (boat thrusters). Real request 2026-08-29,
        # for hand-testing the dry<->water transition.
        self._manual = False
        self._last_self_cmd = self.get_clock().now()

        self.track_cmd_pub = self.create_publisher(String, '/cavex/tracks/command', 10)
        self.mode_pub = self.create_publisher(String, '/cavex/locomotion_mode', 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self.create_subscription(String, '/cavex/tracks/command', self._track_cmd_cb, 10)
        # 2026-08-29: _publish_mode used to fire only on state transitions +
        # once at startup, on a plain (non-latched) publisher -- so
        # skid_steer / boat_thruster / boat_buoyancy that connect after that
        # single publish never learn the mode, fall back to their defaults,
        # and then MULTIPLE of them publish wrenches to the shared
        # /world/cavex_world/wrench every step, racing (measured: tracked
        # forward drive stuck at ~20% of commanded). Republish at 5 Hz so a
        # late/restarted subscriber always picks it up within 0.2 s.
        self.create_timer(0.2, self._publish_mode)

        self.get_logger().info(
            f"vehicle_switch_node ready: tracks -> retracting (buoyant, "
            f"z<={FLOAT_Z_MAX}) -> props ({TRANSITION_DURATION_S}s) -> "
            f"deploying (<{SHORE_DISTANCE_M}m from shore) -> tracks "
            f"({TRANSITION_DURATION_S}s). Manual Track switch overrides.")
        self._publish_mode()

    def _publish_mode(self):
        self.mode_pub.publish(String(data=self._mode))

    def _send_track_cmd(self, cmd):
        self._last_self_cmd = self.get_clock().now()
        self.track_cmd_pub.publish(String(data=cmd))

    def _track_cmd_cb(self, msg: String):
        # Ignore the echo of our own state-machine commands (published via
        # _send_track_cmd); anything else on this topic is the operator.
        dt = (self.get_clock().now() - self._last_self_cmd).nanoseconds * 1e-9
        if dt < 1.0:
            return
        if msg.data == 'retracted':
            target = 'props'
        elif msg.data == 'deployed':
            target = 'tracks'
        else:
            return
        if not self._manual:
            self.get_logger().info("Manual Track switch -- automatic mode machine standing down.")
        self._manual = True
        self._transition_deadline = None
        if self._mode != target:
            self._mode = target
            self.get_logger().info(f"Manual switch -> {target}.")
            self._publish_mode()

    def _odom_cb(self, msg: Odometry):
        if self._manual:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        now = self.get_clock().now()

        if self._mode == 'tracks':
            # Only retract once actually OUT over water -- not while still within
            # a track-length of shore, or the machine ping-pongs
            # tracks<->retracting<->deploying on the grade and the vehicle never
            # gets sustained track authority to climb out.
            if (z <= FLOAT_Z_MAX and _in_water_box(x, y)
                    and _distance_to_shore(x, y, z) >= SHORE_DISTANCE_M):
                self.get_logger().info(
                    f"Buoyant at z={z:.2f} -- retracting tracks.")
                self._send_track_cmd('retracted')
                self._mode = 'retracting'
                self._transition_deadline = now + rclpy.duration.Duration(
                    seconds=TRANSITION_DURATION_S)
                self._publish_mode()

        elif self._mode == 'retracting':
            if now >= self._transition_deadline:
                self.get_logger().info("Tracks retracted -- switching to props.")
                self._mode = 'props'
                self._publish_mode()

        elif self._mode == 'props':
            dist = _distance_to_shore(x, y, z)
            if dist < SHORE_DISTANCE_M:
                self.get_logger().info(
                    f"{dist:.2f}m from shore -- redeploying tracks.")
                self._send_track_cmd('deployed')
                self._mode = 'deploying'
                self._transition_deadline = now + rclpy.duration.Duration(
                    seconds=TRANSITION_DURATION_S)
                self._publish_mode()

        elif self._mode == 'deploying':
            if now >= self._transition_deadline:
                self.get_logger().info("Tracks deployed -- switching to tracks.")
                self._mode = 'tracks'
                self._publish_mode()


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSwitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
