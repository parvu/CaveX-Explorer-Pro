#!/usr/bin/env python3
"""Closed-loop excitation for the ATE ablation harness, replacing BOTH
the old raw open-loop thrust pulse AND ate_excitation.py's ApplyLinkWrench
approach.

Real bug found and fixed here (2026-08-22): gtsam_slam_node's CurrentFactor
predicts through-water velocity from `thrust_n_`, which is populated ONLY
by subscribing to /bluerov2/thrusterN/cmd_thrust (bridged from the real
Gazebo thruster-joint topics). ate_excitation.py drove the vehicle via
ApplyLinkWrench (/world/cavex_world/wrench) directly, which never touches
those topics -- so thrust_n_ stayed all-zero for every run, making
CurrentFactor's residual the maximally-degenerate `V - C` (v_pred_=0)
instead of its intended `V - v_pred(thrust) - C`. This script computes a
desired body-frame force via the same PD position control, then allocates
it across the real 4 horizontal + 2 vertical thrusters (using the EXACT
same geometry CurrentFactor's own dynamics_model.cpp uses,
defaultBlueRov2Geometry: kD=cos(45deg) horizontal thrusters, thrusters 5/6
pure -Z), and publishes real per-thruster gz.msgs.Double commands. This is
self-consistent with what CurrentFactor actually models, and matches how
the vehicle is driven in every real deployment scenario.

Minimum-norm thruster allocation, derived analytically (not numerically
inverted at runtime): for A=[[-kD,-kD,kD,kD],[-kD,kD,-kD,kD]] mapping
[t1,t2,t3,t4]->[Fx,Fy] in body frame, A@A^T = 2*I, so the pseudo-inverse
is 0.5*A^T:
    t1 = -0.5*kD*(Fx+Fy)   t2 = 0.5*kD*(Fy-Fx)
    t3 =  0.5*kD*(Fx-Fy)   t4 = 0.5*kD*(Fx+Fy)
Thrusters 5,6 (direction (0,0,-1) each) split Fz evenly: t5=t6=-Fz/2.

Because real thrusters at real offset positions produce real torque, the
allocation above was verified by hand to be torque-free on all 3 axes
(t1..t4 chosen so -t1+t2+t3-t4=0, t5=t6 so no roll). The desired
world-frame force is transformed into the vehicle's CURRENT body frame
every tick (not assumed fixed at identity) before allocation, so thrust
always pushes toward the target regardless of any accumulated rotation.

Yaw control added, then REMOVED again, 2026-08-22 (real finding). Added
for a real GUI-watching request (the vehicle visibly moved without
turning to face its direction of travel); the underlying heading fix
itself (yaw_from_quat() already returns true heading, model.sdf's
base_link is x-forward/y-left/z-up, no offset needed) is correct and
still used in circle_demo.py/line_demo.py. But live-tested here at full
gain (KYAW=1.5, MAX_TORQUE=3.0) and at 10x-reduced gain (0.15/0.3): BOTH
corrupted `/gtsam_slam/odometry` into incoherent noise from the very first
sample -- ground truth stayed smooth in both cases (confirmed via direct
raw-sample inspection and a live odometry-topic watch), so this is real
yaw-torque-vs-sonar-registration interference, not a coincidence, and not
fixed by simply turning the gain down. Since this script's actual job is
SLAM accuracy testing, not visual demo, yaw control is OFF here
(KYAW=0.0, MAX_TORQUE=0.0 below) -- the "moving sideways" cosmetic issue
only matters for GUI-watching, and this script's real trajectory
correctness does not depend on the vehicle's visual heading. The
differential-thrust mechanism below is kept (harmless at torque_z=0) in
case yaw control is revisited with a real, tested fix for the sonar
interaction rather than just a smaller gain.
"""
import math
import sys
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.double_pb2 import Double

# CENTER/RADIUS reworked 2026-08-22 (real request, live GUI observation):
# spawning at RADIUS=5's t=0 point (29,6) put the vehicle too close to a
# real wall -- it hit it. Spawn reverted to (24,6,7.0) (the harness's
# original point), and RADIUS shrunk to 2.5 so (24,6) is STILL exactly
# the circle's own t=0 point (CENTER=(21.5,6)+RADIUS=2.5 -> (24,6)) --
# keeps the "start where the target already is" jerk fix, just with a
# smaller, safer radius: west extent 21.5-2.5=19 (5.5m clear of the
# x=14.5 wall, vs radius=5's 4m), east extent (the spawn/t=0 point) 24
# (11m clear of the open x=35 edge).
CENTER = (21.5, 6.0)
TARGET_Z = 7.0
RADIUS = 2.5
PERIOD_S = 90.0
K = 90.0
KZ = 20.0
# Was 400N -- real request 2026-08-22: that's well beyond what this
# vehicle could physically produce. A real T200 thruster maxes out around
# 40-50N; with 4 horizontal thrusters at 45deg, the real physical ceiling
# on combined horizontal force is ~4*40*cos(45deg)=113N. 400N let the
# spawn-transient (see harness: now spawns at the circle's own t=0 point,
# not the center, to remove the other jerk source) produce an
# unrealistic ~40 m/s^2 acceleration burst whenever error was large.
MAX_FORCE = 113.0
CONTROL_PERIOD_S = 0.1
DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 45.0

KD_GEOM = math.sqrt(2.0) / 2.0  # matches dynamics_model.cpp's kD exactly
# Thruster offset from centerline (model.sdf: |y|=0.092), used to convert
# a desired yaw torque into the (-k,+k,+k,-k) differential -- see module
# docstring for the by-hand torque derivation.
TORQUE_ARM = 0.232  # = 0.14 + 0.092, matches dynamics_model.cpp's thruster positions
KYAW = 0.0
MAX_TORQUE = 0.0

_latest_pose = {"pos": None, "quat": None}


def _pose_cb(msg: Pose_V):
    for pose in msg.pose:
        if pose.name == "bluerov2":
            _latest_pose["pos"] = (pose.position.x, pose.position.y, pose.position.z)
            q = pose.orientation
            _latest_pose["quat"] = (q.x, q.y, q.z, q.w)


def quat_to_rot_matrix(x, y, z, w):
    """3x3 rotation matrix (body->world), row-major as a flat 9-tuple."""
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy),
        2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),
        2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy),
    )


def world_to_body(R, fx, fy, fz):
    """F_body = R^T @ F_world (R rotates body->world, so its transpose
    rotates world->body for an orthonormal rotation matrix)."""
    return (
        R[0] * fx + R[3] * fy + R[6] * fz,
        R[1] * fx + R[4] * fy + R[7] * fz,
        R[2] * fx + R[5] * fy + R[8] * fz,
    )


def allocate_thrust(fx_body, fy_body, fz_body, torque_z=0.0):
    t1 = -0.5 * KD_GEOM * (fx_body + fy_body)
    t2 = 0.5 * KD_GEOM * (fy_body - fx_body)
    t3 = 0.5 * KD_GEOM * (fx_body - fy_body)
    t4 = 0.5 * KD_GEOM * (fx_body + fy_body)
    t5 = -fz_body / 2.0
    t6 = -fz_body / 2.0
    k = torque_z / (4.0 * KD_GEOM * TORQUE_ARM)
    t1, t2, t3, t4 = t1 - k, t2 + k, t3 + k, t4 - k
    return [t1, t2, t3, t4, t5, t6]


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    node = GzNode()
    node.subscribe(Pose_V, "/world/cavex_world/pose/info", _pose_cb)
    pubs = [
        node.advertise(f"/model/bluerov2/joint/thruster{i}_joint/cmd_thrust", Double)
        for i in range(1, 7)
    ]

    for _ in range(100):
        if _latest_pose["pos"] is not None and _latest_pose["quat"] is not None:
            break
        time.sleep(0.1)

    omega = 2 * math.pi / PERIOD_S
    t0 = time.time()
    prev_xy = None
    while time.time() - t0 < DURATION_S:
        t = time.time() - t0
        tx = CENTER[0] + RADIUS * math.cos(omega * t)
        ty = CENTER[1] + RADIUS * math.sin(omega * t)

        pos, quat = _latest_pose["pos"], _latest_pose["quat"]
        if pos is None or quat is None:
            time.sleep(CONTROL_PERIOD_S)
            continue
        px, py, pz = pos
        fx, fy, fz = K * (tx - px), K * (ty - py), KZ * (TARGET_Z - pz)
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        if mag > MAX_FORCE:
            scale = MAX_FORCE / mag
            fx, fy, fz = fx * scale, fy * scale, fz * scale

        current_yaw = yaw_from_quat(*quat)
        dx = dy = 0.0
        if prev_xy is not None:
            dx, dy = px - prev_xy[0], py - prev_xy[1]
            if math.hypot(dx, dy) > 0.02:
                desired_yaw = math.atan2(dy, dx)
            else:
                desired_yaw = current_yaw
        else:
            desired_yaw = current_yaw
        prev_xy = (px, py)
        torque_z = KYAW * wrap_angle(desired_yaw - current_yaw)
        torque_z = max(-MAX_TORQUE, min(MAX_TORQUE, torque_z))

        R = quat_to_rot_matrix(*quat)
        fx_b, fy_b, fz_b = world_to_body(R, fx, fy, fz)
        thrusts = allocate_thrust(fx_b, fy_b, fz_b, torque_z)

        for pub, val in zip(pubs, thrusts):
            msg = Double()
            msg.data = val
            pub.publish(msg)

        time.sleep(CONTROL_PERIOD_S)


if __name__ == "__main__":
    main()
