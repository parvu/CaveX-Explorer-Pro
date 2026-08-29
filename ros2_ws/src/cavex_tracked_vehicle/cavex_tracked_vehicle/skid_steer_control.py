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
KP_V = 900.0            # N per (m/s) of forward-speed error -- high enough
                       # that a 0.8 m/s command reaches ~0.6 m/s against
                       # the isotropic track friction (mu 0.35)
MAX_FORCE_N = 1050.0    # forward force clamp -- headroom for a ramp climb

# Lateral: damp the body-across velocity so the vehicle doesn't crab. This
# is the PRIMARY anti-sideslip mechanism -- the track collision friction is
# isotropic (fdir1 doesn't work under bullet-featherstone), so it can't
# resist lateral sliding more than forward driving. FADED as a turn is
# commanded (a pivot legitimately swings the body's ends sideways and full
# damping acts as a yaw brake), but never below LAT_FADE_FLOOR -- fully
# off, the vehicle skidded into walls while manoeuvring.
K_LAT = 1000.0          # N per (m/s) of body-lateral velocity
MAX_LAT_N = 700.0
W_LAT_CUTOFF = 0.6      # rad/s of commanded yaw over which the fade runs
LAT_FADE_FLOOR = 0.5    # lateral damping never fades below this fraction

# Yaw: feedforward + P-on-rate trim. Two 0.6m track boxes 0.7m apart make
# pivot resistance high and STIFF (stiction-like, not viscous) -- measured
# live 2026-08-28 pivoting in place: ~970 N*m gave 0.66 rad/s but ~1250
# N*m gave 1.78 (a ~1000 N*m breakaway threshold, steep above it). FF_YAW
# is sized to clear that threshold decisively for a full turn command;
# a real pivot then runs a bit faster than commanded, which a manual
# driver modulates. It's a drag calibration -- re-measure and retune if
# track mu, box length/spacing or vehicle mass change. KP_W trims.
FF_YAW = 1300.0        # 1100 -> 1300: track mu 0.25 -> 0.35 raised the
                      # pivot-resistance breakaway
KP_W = 300.0          # N*m per (rad/s) of yaw-rate error -- 700 slammed
                      # the clamp on filtered-rate noise; the stop-brake
                      # below handles arresting a real coast
MAX_TORQUE_NM = 2800.0

# Keep it upright on land. ROLL only -- a tracked vehicle is meant to
# PITCH freely to follow terrain (ramps, obstacles). Holding pitch to
# level fought the ~22 deg entry ramp: it needed full throttle to climb,
# rode nose-high, then the stored fight released as an erratic slide once
# it crested (report 2026-08-29). Pitch now only gets rate damping (stops
# a flip-fast oscillation), not an angle spring.
LEVEL_KP = 700.0        # N*m per rad of ROLL angle
LEVEL_KD = 120.0        # N*m per (rad/s) of roll/pitch rate
LEVEL_MAX = 600.0       # righting torque clamp per axis
LEVEL_DEADBAND = 0.9    # rad (~52deg); past this the vehicle is on its side/back, stop fighting

# Slope feed-forward: climbing a pitch of theta needs ~WEIGHT*sin(theta)
# just to hold station. Add it directly so the speed P-term isn't forced
# to saturate on the ramp (and so descents get an equal brake instead of
# running away). WEIGHT ~ full-vehicle mass * g.
GRAV_FF_N = 490.0

# Apply the drive force THROUGH the CoM (base_link inertial z = -0.168), so
# a horizontal force makes zero pitch couple regardless of heading or
# slope. -0.45 (0.28 m below the CoM) was stable on flat ground where the
# planted tracks countered the couple, but on the entry ramp -- where the
# tracks bridge the edge and lose ground reaction -- a forward force at
# that low point pitched the nose down and the slippery bow dug in and
# tumbled, while reverse (nose-up couple) climbed fine. Zero couple = no
# forward/reverse asymmetry. (2026-08-29)
CONTACT_Z = -0.168

# EMA weight for the finite-differenced velocity/rate estimates (0..1;
# lower = smoother, more lag). ~0.3 kills the force-slamming feedback
# without adding meaningful control lag at this odom rate.
VEL_LPF_ALPHA = 0.3

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
        self._vx = self._vy = self._yaw_rate = 0.0  # LPF state
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
            self._vx = self._vy = self._yaw_rate = 0.0
            # Go SILENT, don't publish a zero wrench -- the /world/.../wrench
            # topic is non-persistent (one msg = one step) and shared with
            # boat_buoyancy_control; a zero stream here would land on ~half
            # the steps and halve the buoyancy node's real lift/righting.
            self._wrench = None
            return

        rvx = rvy = roll_rate = pitch_rate = ryaw_rate = 0.0
        if self._prev is not None:
            dt = t - self._prev[0]
            if dt > 1e-3:
                rvx = (x - self._prev[1]) / dt
                rvy = (y - self._prev[2]) / dt
                roll_rate = wrap(roll - self._prev[3]) / dt
                pitch_rate = wrap(pitch - self._prev[4]) / dt
                ryaw_rate = wrap(yaw - self._prev[5]) / dt
        self._prev = (t, x, y, roll, pitch, yaw)

        # Low-pass the finite-differenced rates. Raw, they're noisy enough
        # (odom is ~10-50 Hz, position/quat jitter) that KP_W and the
        # stop-brake slammed +-2800 N*m at the noise, which actually spun
        # the hull -> feedback loop -> "erratic" land drive (2026-08-29).
        a = VEL_LPF_ALPHA
        vx = self._vx = a * rvx + (1 - a) * self._vx
        vy = self._vy = a * rvy + (1 - a) * self._vy
        yaw_rate = self._yaw_rate = a * ryaw_rate + (1 - a) * self._yaw_rate

        c, s = math.cos(yaw), math.sin(yaw)
        fwd = vx * c + vy * s          # body forward speed
        lat = -vx * s + vy * c         # body lateral speed (+left)

        # pitch > 0 = nose up = climbing -> forward assist; < 0 = descending
        # -> brake. Keeps the P-term off its rails on the entry ramp.
        grav_ff = GRAV_FF_N * math.sin(pitch)
        f_fwd = clamp(KP_V * (cmd_v - fwd) + grav_ff, -MAX_FORCE_N, MAX_FORCE_N)
        lat_fade = max(LAT_FADE_FLOOR, 1.0 - abs(cmd_w) / W_LAT_CUTOFF)
        f_lat = lat_fade * clamp(-K_LAT * lat, -MAX_LAT_N, MAX_LAT_N)
        # body (fwd, lat) -> world
        fx = f_fwd * c - f_lat * s
        fy = f_fwd * s + f_lat * c

        tz = FF_YAW * cmd_w + KP_W * (cmd_w - yaw_rate)
        # Stop-brake: releasing a turn leaves the FF term at 0, so a fast
        # pivot coasts past the stop. Only engage for a REAL residual spin
        # (> 0.4 rad/s on the filtered rate -- below that it's noise), and
        # scale it in gently so it doesn't slam the clamp.
        if abs(cmd_w) < 0.1 and abs(yaw_rate) > 0.4:
            tz -= 0.7 * FF_YAW * clamp((yaw_rate - math.copysign(0.4, yaw_rate)) / 0.6,
                                       -1.0, 1.0)
        tz = clamp(tz, -MAX_TORQUE_NM, MAX_TORQUE_NM)

        # Upright hold -- roll only (angle spring + rate damping). Pitch
        # gets rate damping alone so the hull can follow ramp/terrain
        # slope without the controller fighting it. Skip entirely once
        # well past level (already on its side/back).
        if abs(roll) < LEVEL_DEADBAND and abs(pitch) < LEVEL_DEADBAND:
            tx = clamp(-LEVEL_KP * roll - LEVEL_KD * roll_rate, -LEVEL_MAX, LEVEL_MAX)
            ty = clamp(-LEVEL_KD * pitch_rate, -LEVEL_MAX, LEVEL_MAX)
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
