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
from std_msgs.msg import String

from gz.transport13 import Node as GzNode
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity

# Real request 2026-08-27: basin redesign moved this to match
# vehicle_switch_node.py's own WATER_BOUNDARY_X (15.0 -> 5.0) -- see that
# file's own comment and cavex_world.world's entry_ramp/basin_floor for the
# full story (a real basin bottom now exists under x[5,15] too, so the
# earlier void concern that kept this at 15.0 no longer applies physics-
# wise).
# 2026-08-29: 0.0 -> -0.3 -- water_entry_ramp's wet foot is at x~-0.22,
# z~5.78; buoyancy must engage right there so the vehicle is supported the
# instant it noses off the ramp instead of free-falling ~2.8 m to
# basin_floor (z=3.0) and getting popped back up. Matches vehicle_switch_
# node.py's WATER_BOX x0.
WATER_BOUNDARY_X = -0.3

# Sum of every <mass> in model.sdf.tracked. Was 47.5 kg over 15 links
# (base_link 32.6 + 2x motor 0.2 + 2x track 1.5 + 2x strut 0.15 + 2x
# retract_mount 0.1 + imu/lidar 0.1 + camera 0.05 + tether_anchor 0.2 +
# bluerov2_link 10.05 + helipad 0.5). 2026-08-28: x500 fused in as a
# fixed decor child link (+2.064 kg) -> 49.56 kg.
VEHICLE_MASS_KG = 49.56
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
#
# Target base_link Z while floating. Derived, not guessed:
#   water surface z = 6.0 (cavex_world.world water_surface pose)
#   hull_collision bbox top sits at local z=0 (base_link Z == hull-top Z)
#   real request 2026-08-28: hull top exactly 6cm proud of the water.
WATER_SURFACE_Z = 6.0   # cavex_world.world water_surface (lowered 7.0 -> 6.0 to the cave-floor level, 2026-08-29)
TARGET_FREEBOARD = 0.12  # 2026-08-29: doubled 0.06 -> 0.12 (real request -- more
                         # deck clearance so drive-induced pitch doesn't dip the
                         # pontoons under)
TARGET_FLOAT_Z = WATER_SURFACE_Z + TARGET_FREEBOARD   # 6.12

# Root cause of the earlier "won't settle / floats too high" was NOT this
# law: (1) the world gz-sim-buoyancy-system plugin was silently lifting the
# boat -- removed (see cavex_world.world); (2) the water_ceiling collision
# box was catching the mast/x500 -- raised; (3) test-only: gz set_pose on
# the boat leaves the DetachableJoint-attached x500 behind, and the
# stretched joint's constraint forces pin base_link (a -3000N test wrench
# could not move it) -- teleport x500 with the boat, or drive in.
#
# feedforward WEIGHT_N (measured 47.5kg) + stiff P (KZ) + D (DZ) gets
# within ~3cm; a tightly-bounded integral removes the residual (mostly the
# ~20N of x500 weight transmitted through the helipad joint) so it settles
# exactly at TARGET_FLOAT_Z. The clamp (|I| <= KI_CLAMP) caps the integral
# term at +-KI*KI_CLAMP = +-36N -- enough for that offset, far too small to
# run away even if some larger disturbance appears.
KZ = 700.0    # N per meter of Z error
KI = 200.0    # N per (meter*second) of accumulated Z error
KI_CLAMP = 0.50  # |integral| bound -> KI term bounded to +-100 N
DZ = 430.0    # N per (m/s) of Z velocity -- a bit over critical
              # (c_crit = 2*sqrt(KZ*m) ~= 365); 2026-08-29 raised from 350
              # to stop the ~0.3 m hull dip during a drive burst
MAX_LIFT_N = 800.0  # headroom over the ~486N hover point, not a rocket

# Real request 2026-08-27: "add water drag". Without it the model has
# almost no resistance -- a 4s thrust burst coasted the boat ~30m. Applied
# as a body-frame-agnostic (world XY) linear + quadratic force opposing
# translation, plus linear yaw-rate damping, only while in the water box
# (this callback already returns early on dry land). Quadratic term
# dominates at drive speed; the small linear term gives a clean stop at
# low speed. Tuned so a 0.6 m/s drive coasts ~2-3m.
# ponytail: naive isotropic drag, no added-mass / Fossen cross terms --
# a real hydrodynamics plugin if this ever needs proper maneuvering fidelity.
DRAG_LIN_XY = 15.0    # N per (m/s)
DRAG_QUAD_XY = 47.0   # N per (m/s)^2
DRAG_YAW = 140.0      # N*m per (rad/s) of yaw rate -- raised 60 -> 140
                      # 2026-08-29: straight drive was curving off course;
                      # stiffer yaw-rate damping plus the port thruster
                      # coefficient-sign fix (model.sdf.tracked) hold it
                      # straight.

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
ANGULAR_DAMPING = 60.0  # N*m per (rad/s) of roll/pitch rate (was 40 --
                       # raised with LEVEL_KP to hold pitch under drive)

# Real request 2026-08-26: "too back heavy" -- live-confirmed a genuine, real
# mass-imbalance trim (~-7deg pitch, stable, not oscillating), not noise: the
# stern motor/prop links (model.sdf.tracked's motor_port_link/motor_stbd_link,
# x=-0.488) plus tether_anchor_link (x=-0.2) and bluerov2_link (x=-0.10) all
# sit aft of base_link's own origin, unbalanced by the (now hull-centered,
# x=0.1) helipad/x500 cargo. ANGULAR_DAMPING only opposes RATE, so it can calm
# oscillation but can't correct a steady offset -- a real P term on the angle
# itself is what's actually missing to drive the resting trim toward level.
# When locomotion mode leaves the water set (-> 'tracks'), don't cut the
# ~500 N of lift dead -- the hull would drop nose-first onto the still-
# deploying tracks / down the entry ramp (the "abnormal pitch-down on the
# water->land transition", 2026-08-29). Ramp lift + righting + drag to zero
# over this window instead; it roughly matches track_retract_control's 2 s
# deploy trajectory.
MODE_FADE_S = 2.0

LEVEL_KP = 850.0  # N*m per radian of roll/pitch angle (was 550 -- drive
                  # thrust sits ~0.1 m below the CoM and pitches the hull;
                  # stiffer P holds trim near level under fwd/rev)


def roll_pitch_from_quat(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    return roll, pitch


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


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
        # Locomotion mode gate. Buoyancy is for water: 'retracting' (tracks
        # lifting, still afloat), 'props', 'deploying' (tracks lowering near
        # shore). In 'tracks' the vehicle is driving on land under
        # skid_steer_control -- keep buoyancy OFF or the two fight over
        # /world/.../wrench and the hull pitches / crabs the moment it
        # climbs out of the water (real bug, 2026-08-29). skid_steer and
        # boat_thruster already gate on this same topic; this node was the
        # odd one out, using a raw x>5 check that stays true on the shore.
        # Default 'props' (buoyancy-ON): if no /cavex/locomotion_mode
        # message ever arrives, a boat in the water region should float,
        # not silently fade out. The real system always publishes the mode.
        self._mode = 'props'
        self._fade = 1.0        # 0..1 buoyancy authority; ramps down over
        self._fade_t = None     # MODE_FADE_S when mode -> 'tracks'
        self.create_subscription(String, '/cavex/locomotion_mode', self._mode_cb, 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self._prev = None  # (t, x, y, z, roll, pitch, yaw)
        self._z_i = 0.0    # bounded Z-error integral (see KI / KI_CLAMP)
        # gz-sim ApplyLinkWrench applies a plain /world/.../wrench message
        # for exactly ONE physics step, and the /wrench/persistent topic
        # ACCUMULATES a new entry per message (never dedupes by entity ->
        # republishing at any rate makes the force run away). So the odom
        # callback only COMPUTES the wrench; this ~physics-rate timer
        # re-publishes the current one every step, giving a continuous
        # force without accumulation.
        self._wrenches = ()
        self.create_timer(1.0 / 250.0, self._republish)
        self.get_logger().info(
            f"boat_buoyancy_control ready: applying lift only while "
            f"x > {WATER_BOUNDARY_X} (target float Z={TARGET_FLOAT_Z:.2f}, "
            f"P+I+D lift + drag, 250Hz re-publish).")

    def _mode_cb(self, msg: String):
        self._mode = msg.data

    def _republish(self):
        for w in self._wrenches:
            self.gz_pub.publish(w)

    def _odom_cb(self, msg: Odometry):
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        roll, pitch = roll_pitch_from_quat(q.x, q.y, q.z, q.w)
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        # Hard cut only once genuinely out of the water region. On land
        # don't publish a zero wrench -- the topic is non-persistent (each
        # msg = one physics step) and shared with skid_steer, so a 250 Hz
        # stream of zeros would land on ~half the steps and halve its real
        # force/torque (measured live 2026-08-28: yaw stuck ~0.36 vs 1.25).
        if x <= WATER_BOUNDARY_X:
            self._prev = None
            self._z_i = 0.0
            self._fade = 0.0
            self._fade_t = None
            self._wrenches = ()
            return

        # BIDIRECTIONAL fade of buoyancy authority over MODE_FADE_S, so the
        # water<->land handoff with skid_steer isn't a force step. mode
        # 'tracks' -> ramp _fade toward 0 (hull settles onto the tracks
        # instead of dropping nose-first); mode props/retracting/deploying
        # -> ramp _fade toward 1 (lift eases in instead of the hull
        # shooting up off the ramp). The "unreal pitch" on both crossings
        # was this being a hard 0<->1 step in one direction.
        target = 0.0 if self._mode == 'tracks' else 1.0
        dt_fade = t - self._fade_t if self._fade_t is not None else 0.0
        self._fade_t = t
        step = min(0.2, dt_fade / MODE_FADE_S) if dt_fade > 0.0 else 0.0
        if self._fade < target:
            self._fade = min(target, self._fade + step)
        elif self._fade > target:
            self._fade = max(target, self._fade - step)

        if target == 0.0 and self._fade <= 0.0:
            self._prev = None
            self._z_i = 0.0
            self._wrenches = ()
            return

        vx = vy = vz = roll_rate = pitch_rate = yaw_rate = 0.0
        dt = 0.0
        if self._prev is not None:
            dt = t - self._prev[0]
            if dt > 1e-3:
                vx = (x - self._prev[1]) / dt
                vy = (y - self._prev[2]) / dt
                vz = (z - self._prev[3]) / dt
                roll_rate = wrap_angle_diff(roll, self._prev[4]) / dt
                pitch_rate = wrap_angle_diff(pitch, self._prev[5]) / dt
                yaw_rate = wrap_angle_diff(yaw, self._prev[6]) / dt
        self._prev = (t, x, y, z, roll, pitch, yaw)

        z_err = TARGET_FLOAT_Z - z
        if dt > 1e-3:
            self._z_i = max(-KI_CLAMP, min(KI_CLAMP, self._z_i + z_err * dt))
        lift_raw = WEIGHT_N + KZ * z_err + KI * self._z_i - DZ * vz
        lift = self._fade * max(0.0, min(MAX_LIFT_N, lift_raw))

        torque_x = self._fade * (-LEVEL_KP * roll - ANGULAR_DAMPING * roll_rate)
        torque_y = self._fade * (-LEVEL_KP * pitch - ANGULAR_DAMPING * pitch_rate)

        drag_x = self._fade * -(DRAG_LIN_XY * vx + DRAG_QUAD_XY * abs(vx) * vx)
        drag_y = self._fade * -(DRAG_LIN_XY * vy + DRAG_QUAD_XY * abs(vy) * vy)
        torque_z = self._fade * -DRAG_YAW * yaw_rate

        # TWO wrench messages, not one. The lift + roll/pitch righting act at
        # force_offset.z = BUOY_OFFSET_Z (0.4, above the CoM) -- that offset
        # is what makes the vertical lift a righting spring. But drag is
        # HORIZONTAL, and at that same +0.4 offset a drive-speed drag force
        # (~40 N at 0.8 m/s) becomes a ~24 N*m bow-down (fwd) / stern-down
        # (rev) pitch couple that buries a pontoon -- the "almost sinking
        # under drive" report, 2026-08-29. Drag goes in its own message at
        # offset 0 (through the CoM) so it only decelerates, never pitches.
        # gz's ApplyLinkWrench sums all messages a link receives in a step.
        self._publish_wrenches(
            lift=lift, torque_x=torque_x, torque_y=torque_y,
            drag_x=drag_x, drag_y=drag_y, torque_z=torque_z)

    def _publish_wrenches(self, lift, torque_x, torque_y,
                          drag_x, drag_y, torque_z):
        w_lift = EntityWrench()
        w_lift.entity.name = 'cavex_tracked_blueboat::base_link'
        w_lift.entity.type = Entity.LINK
        w_lift.wrench.force.z = lift
        w_lift.wrench.force_offset.z = BUOY_OFFSET_Z
        w_lift.wrench.torque.x = torque_x
        w_lift.wrench.torque.y = torque_y

        w_drag = EntityWrench()
        w_drag.entity.name = 'cavex_tracked_blueboat::base_link'
        w_drag.entity.type = Entity.LINK
        w_drag.wrench.force.x = drag_x
        w_drag.wrench.force.y = drag_y
        w_drag.wrench.torque.z = torque_z   # yaw drag, offset-independent

        self._wrenches = (w_lift, w_drag)  # both applied every step by _republish()


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
