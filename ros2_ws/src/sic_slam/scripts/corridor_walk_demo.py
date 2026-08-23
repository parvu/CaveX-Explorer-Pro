#!/usr/bin/env python3
"""Drives bluerov2 straight down sic_slam_tank.world's 2m-gap corridor
(walls at x=[0,10], y=+-1) using a moving-waypoint PD position controller,
then reports PASS/FAIL: did it clear the corridor without hitting a wall
(|y| too large) or sinking to the tank floor (z too low)?

Thrust allocation and body-frame PD control are copied from
cavex_slam_nav/scripts/ate_thrust_excitation.py (real, already-verified
minimum-norm allocator for this exact BlueROV2 T200x6 geometry -- see that
script's own docstring for the by-hand derivation). Adapted here for a
straight-line moving waypoint (spawn -> END_X) instead of a circle, and
against this package's own world/pose topic (sic_slam_tank, not
cavex_world).
"""
import math
import sys
import time

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.double_pb2 import Double

START_X, TARGET_Y, TARGET_Z = 0.0, 0.0, -2.0  # matches sim_launch.py's spawn
END_X = 12.0          # 2m clear past the far wall edge (walls span x=[0,10])
SPEED_MPS = 0.25       # waypoint advance rate -- no jerk, starts at spawn
K, KZ = 90.0, 20.0     # same gains as ate_thrust_excitation.py (same vehicle)
MAX_FORCE = 113.0      # same physical ceiling (4 T200s @ 45deg, ~40-50N each)
CONTROL_PERIOD_S = 0.1
DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else (END_X - START_X) / SPEED_MPS + 10.0

# Wall geometry (sic_slam_tank.world): wall_left/right at y=+-1, thickness
# 0.3 -> inner faces at |y|=0.85. Fail margin leaves 0.1m before contact.
WALL_Y_LIMIT = 0.75
# tank_floor at z=-5 (world/sic_slam_tank.world); fail margin 0.5m above it.
FLOOR_Z_LIMIT = -4.5

KD_GEOM = math.sqrt(2.0) / 2.0

_latest_pose = {"pos": None, "quat": None}


def _pose_cb(msg: Pose_V):
    for pose in msg.pose:
        if pose.name == "bluerov2":
            _latest_pose["pos"] = (pose.position.x, pose.position.y, pose.position.z)
            q = pose.orientation
            _latest_pose["quat"] = (q.x, q.y, q.z, q.w)


def quat_to_rot_matrix(x, y, z, w):
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return (
        1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy),
        2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx),
        2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy),
    )


def world_to_body(R, fx, fy, fz):
    return (
        R[0] * fx + R[3] * fy + R[6] * fz,
        R[1] * fx + R[4] * fy + R[7] * fz,
        R[2] * fx + R[5] * fy + R[8] * fz,
    )


def allocate_thrust(fx_body, fy_body, fz_body):
    t1 = -0.5 * KD_GEOM * (fx_body + fy_body)
    t2 = 0.5 * KD_GEOM * (fy_body - fx_body)
    t3 = 0.5 * KD_GEOM * (fx_body - fy_body)
    t4 = 0.5 * KD_GEOM * (fx_body + fy_body)
    t5 = -fz_body / 2.0
    t6 = -fz_body / 2.0
    return [t1, t2, t3, t4, t5, t6]


def main():
    node = GzNode()
    node.subscribe(Pose_V, "/world/sic_slam_tank/pose/info", _pose_cb)
    pubs = [
        node.advertise(f"/model/bluerov2/joint/thruster{i}_joint/cmd_thrust", Double)
        for i in range(1, 7)
    ]

    for _ in range(100):
        if _latest_pose["pos"] is not None:
            break
        time.sleep(0.1)
    else:
        print("FAIL: never received a pose sample -- is the sim running?")
        sys.exit(1)

    t0 = time.time()
    min_y_clearance = WALL_Y_LIMIT
    min_z = TARGET_Z
    reached_end = False
    while time.time() - t0 < DURATION_S:
        t = time.time() - t0
        tx = min(START_X + SPEED_MPS * t, END_X)

        pos, quat = _latest_pose["pos"], _latest_pose["quat"]
        if pos is None or quat is None:
            time.sleep(CONTROL_PERIOD_S)
            continue
        px, py, pz = pos
        min_y_clearance = min(min_y_clearance, WALL_Y_LIMIT - abs(py))
        min_z = min(min_z, pz)

        if abs(py) > WALL_Y_LIMIT:
            print(f"FAIL: hit a wall -- y={py:.3f} exceeds +-{WALL_Y_LIMIT} at x={px:.3f}")
            sys.exit(1)
        if pz < FLOOR_Z_LIMIT:
            print(f"FAIL: sank to the floor -- z={pz:.3f} below {FLOOR_Z_LIMIT} at x={px:.3f}")
            sys.exit(1)

        if px >= END_X - 0.1:
            reached_end = True
            break

        fx, fy, fz = K * (tx - px), K * (TARGET_Y - py), KZ * (TARGET_Z - pz)
        mag = math.sqrt(fx * fx + fy * fy + fz * fz)
        if mag > MAX_FORCE:
            scale = MAX_FORCE / mag
            fx, fy, fz = fx * scale, fy * scale, fz * scale

        R = quat_to_rot_matrix(*quat)
        fx_b, fy_b, fz_b = world_to_body(R, fx, fy, fz)
        thrusts = allocate_thrust(fx_b, fy_b, fz_b)

        for pub, val in zip(pubs, thrusts):
            msg = Double()
            msg.data = val
            pub.publish(msg)

        time.sleep(CONTROL_PERIOD_S)

    for pub in pubs:
        pub.publish(Double(data=0.0))

    if not reached_end:
        print(f"FAIL: timed out at x={_latest_pose['pos'][0]:.3f}, never reached x={END_X}")
        sys.exit(1)

    print(f"PASS: cleared the corridor (x=0 -> {END_X}) in {time.time()-t0:.1f}s, "
          f"min wall clearance {min_y_clearance:.3f}m, min z {min_z:.3f} "
          f"(floor at -5)")


if __name__ == "__main__":
    main()
