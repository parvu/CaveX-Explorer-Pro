#!/usr/bin/env python3
"""
boat_buoyancy_control.py

Real request 2026-08-26: gz-sim-buoyancy-system (the world-level Buoyancy
plugin in cavex_world.world) grades fluid density purely by world-frame Z
height (<above_depth>), with no concept of the water region's real X
boundary (WATER_BOUNDARY_X=15.0, same constant vehicle_switch_node.py
uses for track retract/deploy). Since the dry cave floor sits at roughly
the same Z band as the flooded floor, the built-in plugin was lifting the
boat even on dry land -- there is no way to region-gate it from the SDF
config alone. It's disabled for this vehicle (removed from the world
plugin's <enable> list) and replaced by this node, which applies an
equivalent force manually via gz-sim-apply-link-wrench-system (already a
world plugin, real proven pattern from cavex_slam_nav/scripts/
circle_demo.py) -- and only while x > WATER_BOUNDARY_X.

The other real bug this replaces: the built-in Buoyancy system computes
its lift through the hull collision mesh's own volume centroid, which
isn't guaranteed to sit above the vehicle's real center of mass (bluerov2's
fused hull and 6 thruster props add real off-axis mass -- see
model.sdf.tracked's bluerov2_link) -- with the centroid below or level
with the CoM there's no real righting moment, so it settled upside-down.
This node instead applies its lift through Wrench.force_offset, a fixed
point straight up from base_link's own origin in the LINK's frame. As the
hull rolls or pitches, that point (rigid to the body) swings away from
world-vertical while the lift force itself stays world-vertical, so
gz-sim's own force x offset torque computation becomes a real righting
moment back to upright -- the same physical mechanism as a boat's
metacenter sitting above its center of gravity, just picked directly
instead of derived from hull geometry.
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from gz.transport13 import Node as GzNode
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity

WATER_BOUNDARY_X = 15.0

# Sum of every <mass> in model.sdf.tracked (base_link + track/strut/mount
# links + bluerov2_link + tether_anchor_link) = 47.1 kg.
VEHICLE_MASS_KG = 47.1
GRAVITY = 9.81
WEIGHT_N = VEHICLE_MASS_KG * GRAVITY

# Real request 2026-08-26: was 8.15 (settled 8.02-8.13, i.e. 12-23cm of deck
# clearance) -- real request narrowed the target range to 6-12cm clearance
# (deck z 7.96-8.02, surface at 7.9). hull_collision's own real bounding box
# (see model.sdf.tracked's lidar_link mount-height comment) has its top at
# local z=0, so base_link's own Z is essentially deck height. This relationship
# turned out non-linear/noisy near the surface (a naive proportional guess
# from 8.15's own settle offset undershot badly at first try) -- 7.97 is
# empirically interpolated from live measurements at several target values
# and confirmed live: settles mostly in the 4-11cm band.
TARGET_FLOAT_Z = 7.97

# Real request 2026-08-26: KZ bumped from 400 -- a P-only height controller
# always leaves a steady-state error proportional to whatever's fighting it
# (here, real lift lost to tilt whenever the boat heels, since force.z is a
# fixed direction, not one that re-aims itself upright as the hull rolls/
# pitches). Live-confirmed settling ~0.3m short of target at KZ=400 while
# heeled; a stiffer gain shrinks that residual error instead of chasing the
# exact tilt-loss number.
KZ = 700.0    # N per meter of Z error
DZ = 350.0    # N per (m/s) of Z velocity -- keeps ~critical damping at the
              # higher KZ (c_crit = 2*sqrt(KZ*m) ~= 363)
MAX_LIFT_N = 1200.0  # clamp, headroom for the higher KZ

# Real request 2026-08-26: passive righting via Wrench.force_offset alone was
# a real live tradeoff -- 0.4 kept roll tight (~+-2 deg) but let pitch
# oscillate hard; halving it to 0.2 calmed pitch but let roll heel over
# ~45 deg (too weak to hold upright at all). Kept the stronger, roll-proven
# 0.4 and added real ACTIVE damping torque below (opposing measured roll/
# pitch rate, not just the offset's passive spring-like restoring force) --
# that's what a passive offset alone structurally can't provide, and is what
# was actually missing to calm the oscillation without weakening the static
# righting stiffness.
BUOY_OFFSET_Z = 0.4
ANGULAR_DAMPING = 40.0  # N*m per (rad/s) of roll/pitch rate

# Real request 2026-08-26: "too back heavy" -- live-confirmed a genuine, real
# mass-imbalance trim (~-7deg pitch, stable, not oscillating), not noise: the
# stern motor/prop links (model.sdf.tracked's motor_port_link/motor_stbd_link,
# x=-0.488) plus tether_anchor_link (x=-0.2) and bluerov2_link (x=-0.10) all
# sit aft of base_link's own origin, unbalanced by the (now hull-centered,
# x=0.1) helipad/x500 cargo. ANGULAR_DAMPING only opposes RATE, so it can calm
# oscillation but can't correct a steady offset -- a real P term on the angle
# itself is what's actually missing to drive the resting trim toward level.
LEVEL_KP = 550.0  # N*m per radian of roll/pitch angle


def roll_pitch_from_quat(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


def wrap_angle_diff(a, b):
    """a - b, wrapped to [-pi, pi]. Real bug found live: a plain subtraction
    across the atan2 branch cut (e.g. roll going from +170deg to -170deg,
    really a +20deg step) produced a huge bogus rate, which the angular
    damping torque below then amplified into real instability instead of
    damping it."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi


class BoatBuoyancyControl(Node):
    def __init__(self, gz_pub):
        super().__init__('boat_buoyancy_control')
        self.gz_pub = gz_pub
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self._prev = None  # (t, z, roll, pitch)
        self.get_logger().info(
            f"boat_buoyancy_control ready: applying lift only while "
            f"x > {WATER_BOUNDARY_X} (target float Z={TARGET_FLOAT_Z}).")

    def _odom_cb(self, msg: Odometry):
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        roll, pitch = roll_pitch_from_quat(q.x, q.y, q.z, q.w)

        if x <= WATER_BOUNDARY_X:
            self._prev = None
            self._publish_wrench(0.0, 0.0, 0.0, 0.0)
            return

        vz = roll_rate = pitch_rate = 0.0
        if self._prev is not None:
            dt = t - self._prev[0]
            if dt > 1e-3:
                vz = (z - self._prev[1]) / dt
                roll_rate = wrap_angle_diff(roll, self._prev[2]) / dt
                pitch_rate = wrap_angle_diff(pitch, self._prev[3]) / dt
        self._prev = (t, z, roll, pitch)

        lift = WEIGHT_N + KZ * (TARGET_FLOAT_Z - z) - DZ * vz
        lift = max(0.0, min(MAX_LIFT_N, lift))
        torque_x = -LEVEL_KP * roll - ANGULAR_DAMPING * roll_rate
        torque_y = -LEVEL_KP * pitch - ANGULAR_DAMPING * pitch_rate
        self._publish_wrench(lift, BUOY_OFFSET_Z, torque_x, torque_y)

    def _publish_wrench(self, force_z, offset_z, torque_x, torque_y):
        w = EntityWrench()
        w.entity.name = 'cavex_tracked_blueboat::base_link'
        w.entity.type = Entity.LINK
        w.wrench.force.z = force_z
        w.wrench.force_offset.z = offset_z
        w.wrench.torque.x = torque_x
        w.wrench.torque.y = torque_y
        self.gz_pub.publish(w)


def main(args=None):
    rclpy.init(args=args)
    gz_node = GzNode()
    gz_pub = gz_node.advertise('/world/cavex_world/wrench', EntityWrench)
    node = BoatBuoyancyControl(gz_pub)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
