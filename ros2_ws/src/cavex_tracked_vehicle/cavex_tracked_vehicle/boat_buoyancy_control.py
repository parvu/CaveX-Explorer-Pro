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

# Real request 2026-08-26: "float above water", not partially submerged --
# was 7.2 (a bit under the surface). This is a direct PD height target, not
# real Archimedes displacement, so there's nothing physically stopping it
# from sitting above the water surface (world Z=7.9, cave floor at Z=5.9);
# 8.15 puts the hull deck clearly above the surface with the keel still
# down in it.
TARGET_FLOAT_Z = 8.15

KZ = 400.0    # N per meter of Z error
DZ = 250.0    # N per (m/s) of Z velocity -- near-critical damping for
              # VEHICLE_MASS_KG against KZ (c_crit = 2*sqrt(KZ*m) ~= 275)
MAX_LIFT_N = 900.0  # clamp, ~2x weight

# How far above base_link's own origin (in its own body frame) the lift is
# applied. Bigger = stronger righting torque per degree of heel, but also
# a stronger pendulum-style overshoot if too large; picked to give a
# roughly boat-like righting stiffness against the hull's real roll
# inertia (dominated by the two ~1.5kg tracks at their own ~0.35-0.4m
# lateral offset) -- tune live if it over/under-corrects.
BUOY_OFFSET_Z = 0.4


class BoatBuoyancyControl(Node):
    def __init__(self, gz_pub):
        super().__init__('boat_buoyancy_control')
        self.gz_pub = gz_pub
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self._prev = None  # (t, z)
        self.get_logger().info(
            f"boat_buoyancy_control ready: applying lift only while "
            f"x > {WATER_BOUNDARY_X} (target float Z={TARGET_FLOAT_Z}).")

    def _odom_cb(self, msg: Odometry):
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        z = msg.pose.pose.position.z

        if x <= WATER_BOUNDARY_X:
            self._prev = None
            self._publish_wrench(0.0, 0.0)
            return

        vz = 0.0
        if self._prev is not None:
            dt = t - self._prev[0]
            if dt > 1e-3:
                vz = (z - self._prev[1]) / dt
        self._prev = (t, z)

        lift = WEIGHT_N + KZ * (TARGET_FLOAT_Z - z) - DZ * vz
        lift = max(0.0, min(MAX_LIFT_N, lift))
        self._publish_wrench(lift, BUOY_OFFSET_Z)

    def _publish_wrench(self, force_z, offset_z):
        w = EntityWrench()
        w.entity.name = 'cavex_tracked_blueboat::base_link'
        w.entity.type = Entity.LINK
        w.wrench.force.z = force_z
        w.wrench.force_offset.z = offset_z
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
