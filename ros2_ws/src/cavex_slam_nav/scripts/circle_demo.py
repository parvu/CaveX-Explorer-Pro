#!/usr/bin/env python3
"""Closed-loop circling demo: real gz-transport Python bindings (same
gz.transport13 pattern already proven in motorized_tether_control.py), not
subprocess+regex -- the earlier version blocked on `gz topic -e -n 1` per
loop iteration, which sometimes took 5+ seconds to return a message, and an
untrapped subprocess.TimeoutExpired killed the whole script after ~1
iteration, leaving the vehicle with zero corrective force for the rest of
the run (root cause of "still not moving, still on the floor" -- the
controller wasn't running, not a physics problem). A background subscription
keeps the latest pose cached; the control loop reads it locally with no
blocking call and no subprocess per tick, and publishes force the same way.
"""
import math
import sys
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity

CENTER = (20.0, 0.0)  # spawn position (x, y)
TARGET_Z = 7.0        # hold depth
RADIUS = 3.0
# Was PERIOD_S=15 (1.26 m/s target speed): real logged data showed
# tracking error staying pinned at ~3-3.6m for the ENTIRE 118s run, never
# converging -- the vehicle was chasing a target rotating faster than it
# could keep up with, so it traced a much smaller, different shape than
# the intended circle, not slowly catching up. Slowed down so the
# controller can actually converge.
PERIOD_S = 45.0     # tangential speed = 2*pi*RADIUS/PERIOD_S ~= 0.42 m/s
K = 40.0            # N per meter of XY position error
# Was XY-only: with zero Z-correction, ANY residual buoyancy imbalance
# (however small -- exact neutral isn't achievable by picking a mass to
# 2 decimal places against a fluid-density model) accumulates unopposed
# over a 90s run and eventually settles it back on the floor, same
# friction-pinning problem as before. Real fix is controlling depth too,
# not chasing an exact-zero mass that doesn't reliably exist.
KZ = 20.0           # N per meter of Z position error
MAX_FORCE = 150.0   # N, clamp (applies to combined XYZ magnitude)
# Real request after watching it live: the vehicle was turning correctly
# (yaw genuinely tracked the desired heading) but visually looked like it
# was facing sideways to its own direction of travel, not forward along
# it -- consistent with the model's rendered "front" not being aligned
# with local +X, the axis yaw_from_quat() actually measures. Offset
# applied to the DESIRED heading so the correction is isolated to "what
# heading counts as facing forward," not the yaw math itself.
YAW_OFFSET = 0.0  # model.sdf documents base_link as x-forward, y-left, z-up --
# yaw_from_quat() already returns true heading with no offset needed. The
# earlier +-pi/2 guesses were wrong (verified against the SDF's own
# comment, 2026-08-22) -- root cause of "moving sideways" on line_demo.py.
# Yaw: point along the direction of travel (the circle's own tangent
# direction at the current target angle -- the vehicle's real velocity
# isn't directly available from Pose_V, only position, but since the
# controller is always chasing a smoothly rotating target, the target's
# own tangent direction is a good proxy for "the direction it's moving").
# izz=0.269 (model.sdf) is small, so even modest torque gives large
# angular acceleration -- kept KYAW/MAX_TORQUE small accordingly.
KYAW = 1.5          # N*m per radian of yaw error
MAX_TORQUE = 3.0    # N*m, clamp
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

    # Wait for the first real pose before starting the control loop. The
    # earlier 5s wait was too short -- this topic's real publish interval
    # turned out longer than that at least once, and the abort-with-zero-
    # force left the vehicle sinking unforced the whole run (same
    # floor-settle symptom as the buoyancy bug, different cause). 20s is
    # generous margin against that.
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
    omega = 2 * math.pi / PERIOD_S
    n = 0
    prev_xy = None  # for computing REAL velocity direction, not assumed
    while time.time() - t0 < DURATION_S:
        t = time.time() - t0
        tx = CENTER[0] + RADIUS * math.cos(omega * t)
        ty = CENTER[1] + RADIUS * math.sin(omega * t)

        pos = _latest_pose["pos"]
        quat = _latest_pose["quat"]
        if pos is None or quat is None:
            time.sleep(CONTROL_PERIOD_S)
            continue
        px, py, pz = pos
        ex, ey, ez = tx - px, ty - py, TARGET_Z - pz
        dist = math.hypot(ex, ey)
        fx, fy, fz = K * ex, K * ey, KZ * ez
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        if mag > MAX_FORCE:
            scale = MAX_FORCE / mag
            fx, fy, fz = fx * scale, fy * scale, fz * scale

        # Was: assumed the target's own circular tangent ~= the vehicle's
        # real direction of travel. Real logged data showed the vehicle
        # never actually converging onto the target circle (tracking error
        # stayed ~3-3.6m the whole run), so its real motion direction
        # diverged heavily from that assumption -- yaw was chasing a
        # heading the vehicle wasn't actually moving toward. Real fix:
        # compute desired heading from the vehicle's OWN measured
        # displacement between consecutive samples (real velocity
        # direction), only when it's moved enough for that direction to be
        # meaningful (avoids yaw noise/undefined atan2 near-zero-motion).
        current_yaw = yaw_from_quat(*quat)
        if prev_xy is not None:
            dx, dy = px - prev_xy[0], py - prev_xy[1]
            if math.hypot(dx, dy) > 0.02:  # ~2cm/tick real-motion threshold
                desired_yaw = math.atan2(dy, dx) + YAW_OFFSET
            else:
                desired_yaw = current_yaw  # not moving enough to have a heading
        else:
            desired_yaw = current_yaw
        prev_xy = (px, py)

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
