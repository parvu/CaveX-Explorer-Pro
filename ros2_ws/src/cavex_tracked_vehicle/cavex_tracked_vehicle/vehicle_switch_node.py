#!/usr/bin/env python3
"""
vehicle_switch_node.py

Watches the tracked vehicle's real ground-truth pose (/odom_ground_truth,
Task 13's real gz-transport-sourced Odometry -- see
tracked_vehicle_ground_truth_odom.py) and manages the dry-section/water
handoff in two parts:

1. ROV lock/unlock (real request): the BlueROV2 is RIGIDLY LOCKED to the
   hull (model.sdf.tracked's bluerov2 DetachableJoint plugin, real
   attach_topic/detach_topic -- confirmed via `strings`) for the whole dry
   section, not just held by tether tension. Locked once at startup (a
   short retry burst, since bluerov2 spawns AFTER the boat -- see the
   spawn-order comment in gazebo_tracked_vehicle.launch.py -- so there's
   no Configure()-time auto-attach the way x500 gets; the joint has to be
   established explicitly once bluerov2 actually exists). Released ONLY
   once BOTH real conditions hold: past the water boundary AND the boat
   is genuinely afloat (not just past an x threshold) -- see
   AFLOAT_Z_THRESHOLD below for how "afloat" is determined. One-way,
   matching the explicit request ("release it only after...") -- no
   re-lock modeled.

2. Track retraction + tether payout: unchanged from before, still keyed
   to the water-boundary x crossing alone (independent of the lock/afloat
   condition -- the tracks don't care whether the ROV has been released
   yet).

The tether (motorized_tether_control.py) stays active throughout,
including while locked -- the rigid joint dominates while it holds (tether
force is negligible against a hard constraint), and becomes the operative
restraint again once unlocked, the same way a real ROV stays connected by
its tether/umbilical even once released to operate independently.

Scope, explicitly narrower than the original plan: no ArduSub SITL
launch/control is started or required here -- that's the same real,
separately-documented arm-rejection limitation already found live this
session (see README.md's "BlueROV2 / ArduSub" section), and is out of
scope for the mechanical handoff this node owns. Bringing ArduSub up (or
arming/controlling the BlueROV2) is left as a separate step for whoever
operates it.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64, String

# The real, live-verified x edge of the flooded chamber -- NOT the plan's
# original guess of 10.0. Derived from cave_floor_patch's own real,
# vertex-confirmed floor coverage (cavex_world.world), the same real
# geometry the water_surface region (x [15,65]) was re-anchored to. See
# that file's own comments for the full derivation.
WATER_BOUNDARY_X = 15.0

# Matches motorized_tether_control.py's own MIN_PAYOUT_LENGTH -- the real
# geometric floor (see that module's own derivation comment), not an
# arbitrary "docked" number. Setting this any lower would just get
# silently clamped up to MIN_PAYOUT_LENGTH anyway.
TETHER_LENGTH_DOCKED = 0.55
TETHER_LENGTH_DEPLOYED = 8.0

# "Fully afloat" threshold -- a real, empirically-derived value, not
# guessed. Live-tested this session (gz service /world/cavex_world/
# set_pose, teleporting the boat around the water region and watching
# where it actually settles): this modified hull (heavier than the real
# BlueBoat -- motors removed but tracks/frame/tether hardware added, see
# base_link's own inertia comment) turns out close to NEUTRALLY buoyant,
# not clearly floating up to a surface waterline the way a real boat
# would -- it stayed stable wherever placed (z=6.6 stayed at 6.6, z=6.05
# near the floor stayed at 6.05, both with zero drift after settling).
# So "afloat" can't be detected as "has the boat's z stabilized at some
# known floating height" -- there isn't one. Instead: z clearly above
# BOTH observed grounded readings (dry-section resting height ~6.41, and
# the near-floor-in-water reading ~6.06 from this same test) means it is
# NOT resting on the bottom, which is the real, honest thing "afloat"
# can mean for this particular vehicle. 6.5 gives real margin above both.
AFLOAT_Z_THRESHOLD = 6.5

# How long z has to stay above AFLOAT_Z_THRESHOLD, continuously, before
# trusting it (rather than a single noisy sample) -- real settling after
# crossing the boundary takes some real time, not instant.
AFLOAT_STABLE_DURATION_S = 2.0

# How many times (at ~1s spacing, driven by /odom_ground_truth's own real
# publish rate) to retry the initial lock at startup. bluerov2 typically
# finishes spawning within a few seconds of the whole launch starting;
# this is a generous, empirically-reasonable margin, not a precise
# handshake (DetachableJoint's own real attach mechanism has no
# "child now exists" acknowledgment this node could wait on instead).
INITIAL_LOCK_RETRIES = 15


class VehicleSwitchNode(Node):
    def __init__(self):
        super().__init__('vehicle_switch_node')
        self._in_water = False
        self._rov_locked = False
        self._rov_released = False
        self._afloat_since = None
        self._lock_retries_left = INITIAL_LOCK_RETRIES

        self.track_cmd_pub = self.create_publisher(String, '/cavex/tracks/command', 10)
        self.tether_length_pub = self.create_publisher(Float64, '/cavex/tether/payout_length_cmd', 10)
        self.rov_lock_pub = self.create_publisher(Empty, '/cavex/rov_lock/attach', 10)
        self.rov_unlock_pub = self.create_publisher(Empty, '/cavex/rov_lock/detach', 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)

        self.create_timer(1.0, self._initial_lock_retry)
        self.get_logger().info(
            f"vehicle_switch_node ready: locking the ROV to the hull, will "
            f"release it once x >= {WATER_BOUNDARY_X} AND the boat has been "
            f"afloat (z > {AFLOAT_Z_THRESHOLD}) for "
            f"{AFLOAT_STABLE_DURATION_S}s; will retract tracks + pay the "
            f"tether out at x >= {WATER_BOUNDARY_X} independently (and "
            f"reverse tracks/tether on the way back out).")

    def _initial_lock_retry(self):
        if self._rov_locked or self._lock_retries_left <= 0:
            return
        self.rov_lock_pub.publish(Empty())
        self._lock_retries_left -= 1
        if self._lock_retries_left == 0:
            self._rov_locked = True
            self.get_logger().info(
                "Initial ROV lock retry burst finished -- assuming locked "
                "(no attach acknowledgment exists to confirm this directly).")

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        z = msg.pose.pose.position.z
        now = self.get_clock().now()

        now_in_water = x >= WATER_BOUNDARY_X
        if now_in_water != self._in_water:
            self._in_water = now_in_water
            if now_in_water:
                self.get_logger().info(
                    f"Crossing into water at x={x:.2f} -- retracting tracks and "
                    f"paying the tether out to {TETHER_LENGTH_DEPLOYED}m.")
                self.track_cmd_pub.publish(String(data='retracted'))
                self.tether_length_pub.publish(Float64(data=TETHER_LENGTH_DEPLOYED))
            else:
                self.get_logger().info(
                    f"Crossing back out of water at x={x:.2f} -- reeling the "
                    f"tether in to {TETHER_LENGTH_DOCKED}m and redeploying tracks.")
                self.tether_length_pub.publish(Float64(data=TETHER_LENGTH_DOCKED))
                self.track_cmd_pub.publish(String(data='deployed'))

        if self._rov_released or not self._in_water:
            self._afloat_since = None
            return

        if z <= AFLOAT_Z_THRESHOLD:
            self._afloat_since = None
            return
        if self._afloat_since is None:
            self._afloat_since = now
            return
        if (now - self._afloat_since).nanoseconds * 1e-9 < AFLOAT_STABLE_DURATION_S:
            return

        self.get_logger().info(
            f"Boat afloat (z={z:.2f} for {AFLOAT_STABLE_DURATION_S}s) and past "
            f"the water boundary (x={x:.2f}) -- unlocking the ROV.")
        self.rov_unlock_pub.publish(Empty())
        self._rov_released = True


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSwitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
