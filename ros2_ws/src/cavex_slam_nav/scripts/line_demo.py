#!/usr/bin/env python3
"""Straight-line version of circle_demo.py -- same proven controller
(real gz-transport pose subscription, 3D spring-force position control,
yaw toward real measured velocity direction with the confirmed +90deg
offset), just a linearly-moving target instead of a circular one. Target
slides from START to END over LINE_DURATION_S, then holds at END.
"""
import math
import sys
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity

# Real incident: with a large initial position error (vehicle far from
# target at start), a pure-proportional controller (no damping) builds up
# real momentum toward the target with nothing opposing velocity itself,
# and can overshoot straight through it -- this actually hit the
# shoreline_ramp's solid collision (x<15) after a large-error run pulled
# it past x=15 at speed. Real fix: added KD (damping proportional to
# measured velocity) below, turning this into a proper PD controller that
# can't build up unbounded momentum. Tightened margins too, as a second,
# independent line of defense -- not a substitute for the damping fix.
START = (19.0, 0.0)
END = (31.0, 0.0)
TARGET_Z = 7.0
LINE_DURATION_S = 40.0   # time to slide from START to END (~0.3 m/s)
K = 40.0
KD = 30.0           # N per (m/s) of velocity -- opposes speed, prevents overshoot
KZ = 20.0
MAX_FORCE = 150.0
KYAW = 1.5
MAX_TORQUE = 3.0

# Was +pi/2 (looked right on the circle, but "moving straight but sideways"
# on a straight line -- circular motion can mask a wrong-signed offset in a
# way a dead-straight path can't). Trying the opposite sign.
YAW_OFFSET = 0.0  # model.sdf documents base_link as x-forward, y-left, z-up --
# yaw_from_quat() already returns true heading with no offset needed. The
# earlier +-pi/2 guesses were wrong (verified against the SDF's own
# comment, 2026-08-22) -- this WAS the "moving sideways" bug.
CONTROL_PERIOD_S = 0.1
DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0

_latest_pose = {"pos": None, "quat": None}


def _pose_cb(msg: Pose_V):
    for pose in msg.pose:
        if pose.name == "bluerov2":
            _latest_pose["pos"] = (pose.position.x, pose.position.y, pose.position.z)
            q = pose.orientation
            _latest_pose["quat"] = (q.x, q.y, q.z, q.w)


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_angle(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def apply_wrench(pub, fx, fy, fz, tz):
    w = EntityWrench()
    w.entity.name = "bluerov2::base_link"
    w.entity.type = Entity.LINK
    w.wrench.force.x = fx
    w.wrench.force.y = fy
    w.wrench.force.z = fz
    w.wrench.torque.z = tz
    pub.publish(w)


def main():
    node = GzNode()
    node.subscribe(Pose_V, "/world/cavex_world/pose/info", _pose_cb)
    pub = node.advertise("/world/cavex_world/wrench", EntityWrench)

    print("Waiting for first pose...")
    for i in range(200):
        if _latest_pose["pos"] is not None:
            print(f"Got first pose after {i * 0.1:.1f}s: {_latest_pose['pos']}")
            break
        time.sleep(0.1)
    else:
        print("No pose received after 20s -- aborting.")
        return

    t0 = time.time()
    n = 0
    prev_xy = None
    prev_t = None
    vx, vy = 0.0, 0.0  # real measured velocity, for damping
    while time.time() - t0 < DURATION_S:
        t = time.time() - t0
        frac = min(1.0, t / LINE_DURATION_S)
        tx = START[0] + (END[0] - START[0]) * frac
        ty = START[1] + (END[1] - START[1]) * frac

        pos = _latest_pose["pos"]
        quat = _latest_pose["quat"]
        if pos is None or quat is None:
            time.sleep(CONTROL_PERIOD_S)
            continue
        px, py, pz = pos
        now = time.time()

        dx, dy = 0.0, 0.0
        if prev_xy is not None and prev_t is not None:
            dt = now - prev_t
            if dt > 1e-3:
                dx, dy = px - prev_xy[0], py - prev_xy[1]
                vx, vy = dx / dt, dy / dt

        ex, ey, ez = tx - px, ty - py, TARGET_Z - pz
        dist = math.hypot(ex, ey)
        # PD, not P-only: KD*velocity opposes real measured speed, so a
        # large position error can't build up unbounded momentum and
        # overshoot through the target (the real cause of hitting the ramp).
        fx = K * ex - KD * vx
        fy = K * ey - KD * vy
        fz = KZ * ez
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        if mag > MAX_FORCE:
            scale = MAX_FORCE / mag
            fx, fy, fz = fx * scale, fy * scale, fz * scale

        current_yaw = yaw_from_quat(*quat)
        if math.hypot(dx, dy) > 0.02:
            desired_yaw = math.atan2(dy, dx) + YAW_OFFSET
        else:
            desired_yaw = current_yaw
        prev_xy = (px, py)
        prev_t = now

        yaw_err = wrap_angle(desired_yaw - current_yaw)
        tz = KYAW * yaw_err
        tz = max(-MAX_TORQUE, min(MAX_TORQUE, tz))

        apply_wrench(pub, fx, fy, fz, tz)

        n += 1
        if n % 20 == 0:
            print(f"t={t:5.1f}s  pos=({px:6.2f},{py:6.2f},{pz:5.2f})  "
                  f"target=({tx:6.2f},{ty:6.2f},{TARGET_Z:5.2f})  err={dist:5.2f}m  "
                  f"F=({fx:5.1f},{fy:5.1f},{fz:5.1f})  yaw={math.degrees(current_yaw):6.1f}deg "
                  f"desired={math.degrees(desired_yaw):6.1f}deg  T={tz:4.2f}")

        time.sleep(CONTROL_PERIOD_S)


if __name__ == "__main__":
    main()
