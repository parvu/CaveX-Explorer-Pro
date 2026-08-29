#!/usr/bin/env python3
"""
skid_steer_control.py

Land locomotion for cavex_tracked_blueboat. The gz-sim TrackedVehicle /
TrackController plugins steer by setting track-link velocities and relying
on a per-step contact-surface-motion callback that only dartsim implements;
under bullet-featherstone (this world's engine, for real mesh collision +
RTF) they produce NO motion. This node is the trade-off replacement.

It is a BODY-FRAME VELOCITY SERVO on base_link, active only while
/cavex/locomotion_mode is 'tracks' or 'retracting' (exact complement of
boat_thruster_control.py's ('props','deploying') gate, so exactly one
drives at a time). Each odom tick it computes the wrench that drives the
measured body velocity toward the target:

    target: forward = cmd_vel.linear.x,  lateral = 0,  yaw rate = cmd_vel.angular.z

Per axis the wrench is  breakaway_feedforward * sat(err) + inertia/tau * err,
clamped. The feedforward carries the Coulomb friction (the tracks on the
floor -- ~mu 0.55, so ~260 N to slide, ~1000 N*m to pivot); the servo term
trims the residual and sets the response time. Driving LATERAL velocity to
zero (not just damping it) is the structural anti-crab -- during a turn the
lateral servo is relaxed so the pivot can swing.

Because it's body-frame, a ramp is handled for free: body-forward on a
nose-up hull points up-slope, and the forward servo simply pushes harder
as gravity slows the climb. Roll is actively held level (tip-over guard);
pitch is only rate-damped so the hull follows terrain.

Wrench mechanics: the plain /world/<world>/wrench topic applies a message
for exactly ONE physics step (the /persistent topic accumulates and runs
away), so the odom callback only COMPUTES the wrench and a ~physics-rate
timer re-publishes it. Forces act through the CoM (force_offset.z) so a
horizontal drive force makes no pitch couple on any slope.
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
LINK = 'cavex_tracked_blueboat::base_link'

# --- vehicle ---
MASS = 49.6            # kg, full vehicle (matches boat_buoyancy_control)
IZZ = 10.0             # kg*m^2, yaw inertia about the CoM (base_link ~6.1
                       # + davit/helipad/x500 children via parallel axis)
# Force application points (base_link frame). The FORWARD drive force acts
# at the track contact patch (CONTACT_Z, ~0.42 m down) so it and the
# ground friction reaction are collinear -> no fwd/rev pitch couple. The
# LATERAL anti-crab force acts through the CoM (COM_X, COM_Z_TRUE) -> no
# yaw couple (at the contact patch its 0.05 m x-arm from the CoM slowly
# spun the vehicle at rest, ~50 deg during a settle -- looked like a crab)
# and no roll couple. Sent as two summed wrench messages.
COM_X = -0.05
COM_Z_TRUE = -0.168
CONTACT_Z = -0.42

# --- Coulomb friction feedforward (breakaway) ---
# The track boxes on the floor patch, combined mu ~0.55. Slide force
# ~ mu * m * g; pivot torque is larger (two 0.6 m boxes 0.7 m apart).
FF_FWD = 360.0         # N to break the tracks loose fore/aft
FF_LAT = 320.0         # N sideways (a touch stiffer -- anti-crab)
FF_YAW = 1050.0        # N*m to break a pivot loose
# err below which the FF fades linearly to 0. Must sit ABOVE the filtered
# finite-difference noise floor, or a phantom rate triggers the full FF
# and kicks a real transient (a ~20 deg uncommanded yaw at drive start).
FF_DEADBAND_V = 0.12   # m/s
FF_DEADBAND_W = 0.25   # rad/s

# --- servo (reach target velocity in ~tau seconds) ---
TAU_FWD = 0.30         # s
TAU_LAT = 0.25         # s  (driving crab to zero -- fairly quick)
TAU_LAT_TURN = 1.20    # s  (relaxed while a turn is commanded, so the
                       #     pivot can swing the body's ends sideways)
TAU_YAW = 0.35         # s
W_TURN_RELAX = 0.4     # rad/s of |cmd_w| over which TAU_LAT -> TAU_LAT_TURN

# Heading hold: the yaw servo only regulates yaw RATE, so a drift slower
# than the odom noise floor is invisible and the heading wanders (seen:
# ~180 deg over a couple minutes idle). When no turn is commanded, capture
# the heading and spring back to it. Released the moment |cmd_w| rises.
YAW_HOLD_KP = 500.0    # N*m per rad of heading error
CMD_W_HOLD = 0.05      # rad/s -- below this, hold heading; above, rate servo

# --- target clamps ---
# The spd+ button (manual_gui_bridge) scales the command up to 2.4 m/s;
# on the rugged cave-mesh floor that speed launches the vehicle off floor
# relief and it tumbles. Cap the servo target to what the tracks can hold
# on rough ground regardless of what's commanded.
CMD_V_MAX = 1.1        # m/s
CMD_W_MAX = 1.6        # rad/s

# --- force/torque clamps ---
FMAX_FWD = 800.0       # N  (lowered with CMD_V_MAX -- less to slam with)
FMAX_LAT = 800.0       # N
TMAX_YAW = 1900.0      # N*m

# --- slope feedforward ---
# forward assist / brake = GRAV_FF_N * sin(pitch_slow), where pitch_slow
# is a heavily low-passed pitch. Using the RAW pitch here let bumps pump a
# fore/aft force oscillation that drove a pitch oscillation (unreal pitch,
# worse in reverse). GRAV_FF_N is ~15% over m*g so a parked vehicle on the
# entry ramp holds station instead of creeping downhill.
GRAV_FF_N = 570.0
PITCH_SLOW_ALPHA = 0.03  # EMA weight for the slope estimate (~1 s settle)

# --- upright hold ---
ROLL_KP = 800.0        # N*m per rad of roll
ROLL_KD = 220.0        # N*m per (rad/s) of roll rate
# Pitch: RATE DAMPING ONLY, no angle spring. The drive force acts at the
# contact patch (CONTACT_Z), so driving induces almost no pitch couple
# (measured +-0.2 deg on flat); gravity + contact settle the hull onto
# whatever the terrain is (both tracks planted, pitch = ramp angle), and
# the rate term just kills oscillation. Any angle spring here either
# teetered the hull off a ramp (toward level) or held it at a wrong pitch
# (toward the slow estimate).
PITCH_KD = 300.0       # N*m per (rad/s) of pitch rate
LEVEL_MAX = 800.0
LEVEL_DEADBAND = 1.3   # rad (~75 deg); keep fighting further before giving up

# --- odom-rate finite-difference filter ---
VEL_LPF_ALPHA = 0.30   # EMA weight (lower = smoother, more lag)

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
        self._wrenches = ()                      # (drive, lateral) EntityWrench, summed by gz
        self._prev = None                       # (t, x, y, roll, pitch, yaw)
        self._vx = self._vy = self._wz = 0.0    # filtered world vx/vy, yaw rate
        self._pitch_slow = 0.0                   # heavily LP'd pitch for slope FF
        self._yaw_hold = None                    # captured heading when not turning
        self.create_subscription(String, '/cavex/locomotion_mode', self._mode_cb, 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self.create_timer(1.0 / 250.0, self._republish)
        self.get_logger().info(
            f"skid_steer_control ready: body-frame velocity servo on base_link "
            f"(fwd = cmd_vel.x, lateral -> 0, yaw = cmd_vel.z) while "
            f"/cavex/locomotion_mode in {ACTIVE_MODES}.")

    def _mode_cb(self, msg: String):
        with _lock:
            _state["mode"] = msg.data

    def _republish(self):
        for w in self._wrenches:
            self.gz_pub.publish(w)

    def _odom_cb(self, msg: Odometry):
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        roll, pitch, yaw = rpy_from_quat(q.x, q.y, q.z, q.w)

        with _lock:
            mode = _state["mode"]
            cmd_v = clamp(_state["cmd_v"], -CMD_V_MAX, CMD_V_MAX)
            cmd_w = clamp(_state["cmd_w"], -CMD_W_MAX, CMD_W_MAX)

        if mode not in ACTIVE_MODES:
            # Go silent (don't stream zero wrenches -- the topic is
            # non-persistent and shared with boat_buoyancy_control).
            self._prev = None
            self._vx = self._vy = self._wz = 0.0
            self._pitch_slow = 0.0
            self._yaw_hold = None
            self._wrenches = ()
            return

        # --- measured velocities: finite-difference + EMA (odom is noisy) ---
        rvx = rvy = roll_rate = pitch_rate = rwz = 0.0
        if self._prev is not None:
            dt = t - self._prev[0]
            if dt > 1e-3:
                rvx = (x - self._prev[1]) / dt
                rvy = (y - self._prev[2]) / dt
                roll_rate = wrap(roll - self._prev[3]) / dt
                pitch_rate = wrap(pitch - self._prev[4]) / dt
                rwz = wrap(yaw - self._prev[5]) / dt
        self._prev = (t, x, y, roll, pitch, yaw)
        a = VEL_LPF_ALPHA
        self._vx = a * rvx + (1 - a) * self._vx
        self._vy = a * rvy + (1 - a) * self._vy
        self._wz = a * rwz + (1 - a) * self._wz
        pa = PITCH_SLOW_ALPHA
        self._pitch_slow = pa * pitch + (1 - pa) * self._pitch_slow

        c, s = math.cos(yaw), math.sin(yaw)
        fwd = self._vx * c + self._vy * s      # body forward speed
        lat = -self._vx * s + self._vy * c     # body lateral speed (+left)

        # --- forward servo (target = cmd_v) ---
        grav_hold = GRAV_FF_N * math.sin(self._pitch_slow)
        if abs(cmd_v) < 0.03 and abs(fwd) < 0.06:
            # PARKED: hold against gravity only. The Coulomb FF term reacts
            # hard to any tiny residual velocity, and on a slope that's a
            # stick-slip limit cycle -- the "wobble at stop on the ramp".
            f_fwd = clamp(grav_hold, -FMAX_FWD, FMAX_FWD)
        else:
            e_fwd = cmd_v - fwd
            f_fwd = clamp(_ff(e_fwd, FF_FWD, FF_DEADBAND_V)
                          + MASS / TAU_FWD * e_fwd + grav_hold,
                          -FMAX_FWD, FMAX_FWD)

        # --- lateral servo (target = 0; relaxed while turning) ---
        tau_lat = TAU_LAT + (TAU_LAT_TURN - TAU_LAT) * min(1.0, abs(cmd_w) / W_TURN_RELAX)
        turn_frac = min(1.0, abs(cmd_w) / W_TURN_RELAX)
        e_lat = 0.0 - lat
        f_lat = ((1.0 - 0.7 * turn_frac) * _ff(e_lat, FF_LAT, FF_DEADBAND_V)
                 + MASS / tau_lat * e_lat)
        f_lat = clamp(f_lat, -FMAX_LAT, FMAX_LAT)

        # --- yaw servo (target = cmd_w) + heading hold when not turning ---
        e_wz = cmd_w - self._wz
        tz = _ff(e_wz, FF_YAW, FF_DEADBAND_W) + IZZ / TAU_YAW * e_wz
        if abs(cmd_w) < CMD_W_HOLD:
            if self._yaw_hold is None:
                self._yaw_hold = yaw
            tz += -YAW_HOLD_KP * wrap(yaw - self._yaw_hold)
        else:
            self._yaw_hold = None
        tz = clamp(tz, -TMAX_YAW, TMAX_YAW)

        # --- upright hold: roll spring + roll/pitch rate damping ---
        if abs(roll) < LEVEL_DEADBAND and abs(pitch) < LEVEL_DEADBAND:
            tx = clamp(-ROLL_KP * roll - ROLL_KD * roll_rate, -LEVEL_MAX, LEVEL_MAX)
            ty = clamp(-PITCH_KD * pitch_rate, -LEVEL_MAX, LEVEL_MAX)
        else:
            tx = ty = 0.0

        # Two wrench messages (gz sums them). Forward drive force at the
        # contact patch; lateral force + all torques through the CoM.
        w_drive = EntityWrench()
        w_drive.entity.name = LINK
        w_drive.entity.type = Entity.LINK
        w_drive.wrench.force.x = f_fwd * c
        w_drive.wrench.force.y = f_fwd * s
        w_drive.wrench.force_offset.x = COM_X
        w_drive.wrench.force_offset.z = CONTACT_Z
        w_drive.wrench.torque.x = tx
        w_drive.wrench.torque.y = ty
        w_drive.wrench.torque.z = tz

        w_lat = EntityWrench()
        w_lat.entity.name = LINK
        w_lat.entity.type = Entity.LINK
        w_lat.wrench.force.x = -f_lat * s
        w_lat.wrench.force.y = f_lat * c
        w_lat.wrench.force_offset.x = COM_X
        w_lat.wrench.force_offset.z = COM_Z_TRUE

        self._wrenches = (w_drive, w_lat)


def _ff(err, mag, deadband):
    """Coulomb-friction feedforward: +-mag in the direction of err, faded
    linearly to 0 as |err| drops below deadband (no limit-cycle chatter
    at the target)."""
    if abs(err) <= 1e-9:
        return 0.0
    return mag * clamp(err / deadband, -1.0, 1.0)


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
