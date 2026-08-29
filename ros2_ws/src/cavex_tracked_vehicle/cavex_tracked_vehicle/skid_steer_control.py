#!/usr/bin/env python3
"""
skid_steer_control.py

Land locomotion for cavex_tracked_blueboat. Replaces the gz-sim
TrackedVehicle / TrackController plugin drive: those systems steer by
setting track-link velocities and relying on anisotropic surface
friction, a path that works under dartsim but produces NO motion under
bullet-featherstone (confirmed live 2026-08-28 -- track_cmd_vel stayed
silent, the vehicle sat still). The world switched to bullet-featherstone
for real mesh collision + RTF; this node is the trade-off fix.

It is a body-frame speed / yaw-rate controller: reads the shared
/model/cavex_tracked_blueboat/cmd_vel (gz-transport Twist, same topic
manual_gui_bridge.py / cmd_vel_gz_bridge.py publish and
boat_thruster_control.py also taps) and drives base_link with a
force + torque via /world/cavex_world/wrench, only while
/cavex/locomotion_mode is 'tracks' or 'retracting' (mirrors
boat_thruster_control.py's own ('props','deploying') gate, so exactly one
of the two ever drives at a time).

Stability (real bug fixed 2026-08-28): the first version applied the drive
force at base_link's ORIGIN, which sits ~0.17m above the CoG and ~0.45m
above the ground-contact plane. Every accel/decel force then made a
pitch couple against the ground reaction -> braking dropped the nose,
forward-after-a-stop flipped the vehicle. It also had no lateral-velocity
control, so a turn just crabbed sideways on the (deliberately near-zero)
track/hull friction. Fixes, mirroring boat_buoyancy_control.py's own
approach:
  - traction forces are applied at the contact plane (force_offset.z =
    -CONTACT_Z) so a horizontal force makes ~no pitch couple;
  - a lateral-velocity damping force emulates track across-axis grip;
  - a pitch/roll righting P-torque + rate damping keeps it upright on
    land (buoyancy provides this in water; land had nothing).

Wrench mechanics match boat_buoyancy_control.py: the plain /world/.../wrench
topic applies a message for ONE physics step, so the control tick only
COMPUTES the wrench and a ~physics-rate timer re-publishes it (the
/persistent topic would ACCUMULATE per message and run away).
"""
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity

ACTIVE_MODES = ('tracks', 'retracting')

# Forward: P on speed error. Softened from 800 -- 800 made a stop a ~560N
# reverse slam.
KP_V = 450.0            # N per (m/s) of forward-speed error
MAX_FORCE_N = 700.0     # forward force clamp

# Lateral: damp the body-across velocity so the vehicle doesn't crab on the
# (near-zero by design) track friction. FADED as a turn is commanded -- a
# pivot legitimately swings the body's fore/aft ends sideways and full
# damping there acts as a yaw brake (measured live 2026-08-28) -- but NOT
# to zero: fully off, the vehicle skidded sideways into walls while
# manoeuvring (2026-08-29). LAT_FADE_FLOOR keeps ~half the grip through a
# hard turn; FF_YAW is strong enough to pivot against it.
K_LAT = 450.0           # N per (m/s) of body-lateral velocity -- back down
MAX_LAT_N = 350.0       # (from 1100/700): the track collision is now
W_LAT_CUTOFF = 0.6      # anisotropic (mu2=1.4 across-track), so real
LAT_FADE_FLOOR = 0.5    # friction does most of the anti-crab work now;
                        # this just trims residual drift.

# Yaw: feedforward + P-on-rate trim. Two 0.6m track boxes 0.7m apart make
# pivot resistance high and STIFF (stiction-like, not viscous) -- measured
# live 2026-08-28 pivoting in place: ~970 N*m gave 0.66 rad/s but ~1250
# N*m gave 1.78 (a ~1000 N*m breakaway threshold, steep above it). FF_YAW
# is sized to clear that threshold decisively for a full turn command;
# a real pivot then runs a bit faster than commanded, which a manual
# driver modulates. It's a drag calibration -- re-measure and retune if
# track mu, box length/spacing or vehicle mass change. KP_W trims.
FF_YAW = 1100.0
KP_W = 650.0           # N*m per (rad/s) of yaw-rate error (380 -> 650:
                      # stiffer braking so releasing a turn doesn't coast
                      # ~50 deg past; at steady turn the error ~0 so it
                      # doesn't fight the FF term)
MAX_TORQUE_NM = 1900.0

# Keep it upright on land (mirror of boat_buoyancy_control.py LEVEL_KP /
# ANGULAR_DAMPING). Only acts on small tilts -- a deliberate big flip
# isn't fought.
LEVEL_KP = 700.0        # N*m per rad of roll/pitch
LEVEL_KD = 120.0        # N*m per (rad/s) of roll/pitch rate
LEVEL_MAX = 600.0       # righting torque clamp per axis
LEVEL_DEADBAND = 0.9    # rad (~52deg); past this the vehicle is on its side/back, stop fighting

# Apply traction forces this far below base_link's origin -- the track
# contact plane. base_link rests ~0.43m above the floor on the tracks;
# CoG is 0.168m below origin, so this is ~0.28m below CoG -> a horizontal
# force gives a tiny, stable nose-up on accel / nose-down on brake, like a
# real vehicle, instead of the destabilising couple the origin gave.
CONTACT_Z = -0.45

LINK = 'cavex_tracked_blueboat::base_link'

_lock = threading.Lock()
_state = {"mode": "tracks", "cmd_v": 0.0, "cmd_w": 0.0}


def rpy_from_quat(x, y, z, w):
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def _cmd_vel_cb(msg: Twist):
    with _lock:
        _state["cmd_v"] = msg.linear.x
        _state["cmd_w"] = msg.angular.z


class SkidSteerControl(Node):
    def __init__(self, gz_pub):
        super().__init__('skid_steer_control')
        self.gz_pub = gz_pub
        self._wrench = None
        self._prev = None  # (t, x, y, roll, pitch, yaw)
        self.create_subscription(String, '/cavex/locomotion_mode', self._mode_cb, 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self.create_timer(1.0 / 250.0, self._republish)
        self.get_logger().info(
            f"skid_steer_control ready: cmd_vel -> base_link wrench "
            f"(contact-plane force, lateral damping, upright hold) "
            f"only while /cavex/locomotion_mode in {ACTIVE_MODES}.")

    def _mode_cb(self, msg: String):
        with _lock:
            _state["mode"] = msg.data

    def _republish(self):
        if self._wrench is not None:
            self.gz_pub.publish(self._wrench)

    def _odom_cb(self, msg: Odometry):
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        roll, pitch, yaw = rpy_from_quat(q.x, q.y, q.z, q.w)

        with _lock:
            mode = _state["mode"]
            cmd_v, cmd_w = _state["cmd_v"], _state["cmd_w"]

        if mode not in ACTIVE_MODES:
            self._prev = None
            # Go SILENT, don't publish a zero wrench -- the /world/.../wrench
            # topic is non-persistent (one msg = one step) and shared with
            # boat_buoyancy_control; a zero stream here would land on ~half
            # the steps and halve the buoyancy node's real lift/righting.
            self._wrench = None
            return

        vx = vy = roll_rate = pitch_rate = yaw_rate = 0.0
        if self._prev is not None:
            dt = t - self._prev[0]
            if dt > 1e-3:
                vx = (x - self._prev[1]) / dt
                vy = (y - self._prev[2]) / dt
                roll_rate = wrap(roll - self._prev[3]) / dt
                pitch_rate = wrap(pitch - self._prev[4]) / dt
                yaw_rate = wrap(yaw - self._prev[5]) / dt
        self._prev = (t, x, y, roll, pitch, yaw)

        c, s = math.cos(yaw), math.sin(yaw)
        fwd = vx * c + vy * s          # body forward speed
        lat = -vx * s + vy * c         # body lateral speed (+left)

        f_fwd = clamp(KP_V * (cmd_v - fwd), -MAX_FORCE_N, MAX_FORCE_N)
        lat_fade = max(LAT_FADE_FLOOR, 1.0 - abs(cmd_w) / W_LAT_CUTOFF)
        f_lat = lat_fade * clamp(-K_LAT * lat, -MAX_LAT_N, MAX_LAT_N)
        # body (fwd, lat) -> world
        fx = f_fwd * c - f_lat * s
        fy = f_fwd * s + f_lat * c

        tz = FF_YAW * cmd_w + KP_W * (cmd_w - yaw_rate)
        # Stop-brake: releasing a turn leaves the FF term at 0, so a fast
        # pivot coasted ~55 deg past the stop (2026-08-29). When no turn is
        # commanded, add an FF-scale counter-torque against any residual
        # yaw rate to arrest it quickly.
        if abs(cmd_w) < 0.1 and abs(yaw_rate) > 0.03:
            tz -= FF_YAW * clamp(yaw_rate / 0.25, -1.0, 1.0)
        tz = clamp(tz, -MAX_TORQUE_NM, MAX_TORQUE_NM)

        # upright hold -- skip once well past level (vehicle already on its side)
        if abs(roll) < LEVEL_DEADBAND and abs(pitch) < LEVEL_DEADBAND:
            tx = clamp(-LEVEL_KP * roll - LEVEL_KD * roll_rate, -LEVEL_MAX, LEVEL_MAX)
            ty = clamp(-LEVEL_KP * pitch - LEVEL_KD * pitch_rate, -LEVEL_MAX, LEVEL_MAX)
        else:
            tx = ty = 0.0

        self._set_wrench(fx, fy, tx, ty, tz)

    def _set_wrench(self, fx, fy, tx, ty, tz):
        w = EntityWrench()
        w.entity.name = LINK
        w.entity.type = Entity.LINK
        w.wrench.force.x = fx
        w.wrench.force.y = fy
        w.wrench.force_offset.z = CONTACT_Z   # traction acts at the contact plane
        w.wrench.torque.x = tx
        w.wrench.torque.y = ty
        w.wrench.torque.z = tz
        self._wrench = w  # applied every step by _republish()


def main(args=None):
    rclpy.init(args=args)
    gz_node = GzNode()
    gz_node.subscribe(Twist, '/model/cavex_tracked_blueboat/cmd_vel', _cmd_vel_cb)
    gz_pub = gz_node.advertise('/world/cavex_world/wrench', EntityWrench)
    node = SkidSteerControl(gz_pub)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
