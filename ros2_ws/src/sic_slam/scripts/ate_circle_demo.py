#!/usr/bin/env python3
"""Circular-trajectory ATE run for sic_slam_cave_water.world -- the real cave
mesh's flooded water section (vendored from cavex_world.world, main branch),
not the small enclosed tank. Spawns at the water region's own center
(25, 0) and drives a closed circle around it, radius chosen with generous
margin from the validated navigable bounds (x[15,35], y[-12,12] -- see
sic_slam_cave_water.world's own comments for how those were derived) so the
loop never approaches a wall: worst-case distance from center to the x=15
shoreline-ramp boundary is 10m, to the y=+-12 real-cave-floor-coverage edge
is 12m; RADIUS=6.0 below leaves >=4m clearance on every side.

Same Umeyama-alignment ATE methodology as ate_baseline_demo.py (reused
directly from cavex_slam_nav.ate_metrics, not duplicated) -- see that
script's own docstring for why a baseline number here isn't a tracking-
accuracy claim, just what training/CurrentFactor should be measured
against.
"""
import math
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.double_pb2 import Double

from cavex_slam_nav.ate_metrics import compute_ate

# Water region center, per sic_slam_cave_water.world's water_surface/
# cave_floor_patch geometry (x[15,35], y[-12,12], vendored from
# cavex_world.world). TARGET_Z=6.9 is mid-depth: 1m below the water
# surface (z=7.9) and 1m above the real-mesh-derived floor (z=5.9).
CENTER_X, CENTER_Y, TARGET_Z = 25.0, 0.0, 6.9
RADIUS = 6.0
SPEED_MPS = 0.25
OMEGA = SPEED_MPS / RADIUS
K, KZ = 90.0, 20.0
MAX_FORCE = 113.0
CONTROL_PERIOD_S = 0.1
DURATION_S = float(sys.argv[1]) if len(sys.argv) > 1 else 80.0

KD_GEOM = math.sqrt(2.0) / 2.0

_latest_gt = {"pos": None, "quat": None}
_latest_est = {"pos": None}


def _pose_cb(msg: Pose_V):
    for pose in msg.pose:
        if pose.name == "bluerov2":
            _latest_gt["pos"] = (pose.position.x, pose.position.y, pose.position.z)
            q = pose.orientation
            _latest_gt["quat"] = (q.x, q.y, q.z, q.w)


class OdometrySubscriber(Node):
    def __init__(self):
        super().__init__('ate_circle_gt_est_recorder')
        self.create_subscription(PoseStamped, '/sic_slam/odometry', self._est_cb, 10)

    def _est_cb(self, msg: PoseStamped):
        _latest_est["pos"] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)


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
    rclpy.init()
    ros_node = OdometrySubscriber()
    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    spin_thread.start()

    gz_node = GzNode()
    gz_node.subscribe(Pose_V, "/world/sic_slam_cave_water/pose/info", _pose_cb)
    pubs = [
        gz_node.advertise(f"/model/bluerov2/joint/thruster{i}_joint/cmd_thrust", Double)
        for i in range(1, 7)
    ]

    for _ in range(100):
        if _latest_gt["pos"] is not None:
            break
        time.sleep(0.1)
    else:
        print("FAIL: never received a ground-truth pose sample -- is the sim running?")
        sys.exit(1)

    gt_samples, est_samples = [], []
    t0 = time.time()
    while time.time() - t0 < DURATION_S:
        t = time.time() - t0
        theta = OMEGA * t
        tx = CENTER_X + RADIUS * math.cos(theta)
        ty = CENTER_Y + RADIUS * math.sin(theta)

        pos, quat = _latest_gt["pos"], _latest_gt["quat"]
        if pos is None or quat is None:
            time.sleep(CONTROL_PERIOD_S)
            continue
        px, py, pz = pos

        if _latest_est["pos"] is not None:
            gt_samples.append(pos)
            est_samples.append(_latest_est["pos"])

        fx, fy, fz = K * (tx - px), K * (ty - py), KZ * (TARGET_Z - pz)
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

    rclpy.shutdown()

    if len(gt_samples) < 3:
        print(f"FAIL: only {len(gt_samples)} matched (ground-truth, estimate) samples -- "
              "is sic_slam_graph_backend publishing /sic_slam/odometry?")
        sys.exit(1)

    result = compute_ate(np.array(est_samples), np.array(gt_samples), align=True)
    print(f"BASELINE ATE: n={result['n_samples']} samples, "
          f"rmse={result['rmse']:.3f} m, mean={result['mean']:.3f} m, "
          f"median={result['median']:.3f} m, std={result['std']:.3f} m, "
          f"max={result['max']:.3f} m")


if __name__ == "__main__":
    main()
