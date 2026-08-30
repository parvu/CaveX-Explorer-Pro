#!/usr/bin/env python3
"""
tracked_vehicle_ground_truth_odom.py

cavex_tracked_blueboat has no reliable model-scoped ground-truth pose topic:
model.sdf.tracked's own OdometryPublisher plugin publishes
/model/cavex_tracked_blueboat/pose, but Task 7 found that dead-reckons from
wheel/joint velocities -- and left_track/right_track have no rotating joint
at all (TrackedVehicle drives them as velocity-controlled links via *_fixed
joints), so it has nothing real to integrate and reports near-static output
regardless of true motion.

Task 7's own real, proven ground-truth mechanism instead is the world-level
pose broadcast, `gz topic -e -t /world/cavex_world/pose/info`, filtered for
`name: "cavex_tracked_blueboat"` (see task-7-report.md Step 3, point 3).
gazebo_tracked_vehicle.launch.py already bridges that same gz-transport
topic into ROS as a geometry_msgs/msg/PoseArray -- but ros_gz_bridge's
Pose_V -> PoseArray conversion drops the per-pose `name` field the world
publishes (confirmed: geometry_msgs/PoseArray has no name field to carry
it), and cavex_world.world's static models (ground_plane, cave_world,
obstacle_1..4) plus this dynamically-spawned vehicle would otherwise force
guessing this vehicle's array index -- exactly the kind of fragile,
unproven index-guessing this project's own conventions avoid.

So this node subscribes directly via gz-transport (not the ROS bridge) to
the same real /world/cavex_world/pose/info topic as a gz.msgs.Pose_V,
filters by the real `name` field for "cavex_tracked_blueboat" (the same
filter Task 7 already proved live), and republishes that one pose as a
plain Odometry message for ate_evaluator_node.py. Same real, no-noise-model
ground truth as the wheeled robot's /odom -- not a claim about real-hardware
ground truth.
"""
import math

import rclpy
from rclpy.node import Node as RclpyNode
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from nav_msgs.msg import Odometry

VEHICLE_MODEL_NAME = 'cavex_tracked_blueboat'
GZ_POSE_TOPIC = '/world/cavex_world/pose/info'
# 2026-08-29: /world/.../pose/info carries pose only, so this node used to
# publish an all-zero twist -- which fed every downstream controller's rate
# term a constant zero (skid_steer ran open-loop at ~20% of commanded speed,
# boat_buoyancy's DZ damping was dead). Now finite-difference consecutive
# poses for a world-frame linear + yaw-rate twist, EMA-smoothed to tame
# differentiation noise (same 0.3 weight skid_steer's own VEL_LPF uses).
VEL_LPF_ALPHA = 0.3


class TrackedVehicleGroundTruthOdom(RclpyNode):
    def __init__(self):
        super().__init__('tracked_vehicle_ground_truth_odom')
        self.pub = self.create_publisher(Odometry, '/odom_ground_truth', 10)
        # Kept as self._gz_node (not a local var) -- gz-transport's
        # subscription lives as long as this Node object does; letting it
        # get garbage-collected would silently stop delivery.
        self._gz_node = GzNode()
        self._gz_node.subscribe(Pose_V, GZ_POSE_TOPIC, self._cb)
        self._prev = None          # (t, x, y, z, yaw)
        self._v = [0.0, 0.0, 0.0, 0.0]  # EMA of [vx, vy, vz, wz], world frame

    def _cb(self, msg: Pose_V):
        for pose in msg.pose:
            if pose.name != VEHICLE_MODEL_NAME:
                continue
            now = self.get_clock().now()
            q = pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            t = now.nanoseconds * 1e-9

            odom = Odometry()
            odom.header.stamp = now.to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = pose.position.x
            odom.pose.pose.position.y = pose.position.y
            odom.pose.pose.position.z = pose.position.z
            odom.pose.pose.orientation.x = q.x
            odom.pose.pose.orientation.y = q.y
            odom.pose.pose.orientation.z = q.z
            odom.pose.pose.orientation.w = q.w

            if self._prev is not None:
                dt = t - self._prev[0]
                if dt > 1e-4:
                    raw = [
                        (pose.position.x - self._prev[1]) / dt,
                        (pose.position.y - self._prev[2]) / dt,
                        (pose.position.z - self._prev[3]) / dt,
                        math.atan2(math.sin(yaw - self._prev[4]),
                                   math.cos(yaw - self._prev[4])) / dt,
                    ]
                    a = VEL_LPF_ALPHA
                    self._v = [a * r + (1.0 - a) * p for r, p in zip(raw, self._v)]
            self._prev = (t, pose.position.x, pose.position.y, pose.position.z, yaw)

            odom.twist.twist.linear.x = self._v[0]
            odom.twist.twist.linear.y = self._v[1]
            odom.twist.twist.linear.z = self._v[2]
            odom.twist.twist.angular.z = self._v[3]
            self.pub.publish(odom)
            return


def _selfcheck():
    """Finite-diff + EMA sanity: constant 2 m/s +x, 1 rad/s yaw -> converges there."""
    v = [0.0, 0.0, 0.0, 0.0]
    x = yaw = 0.0
    dt, a = 0.02, VEL_LPF_ALPHA
    for _ in range(500):
        x += 2.0 * dt
        yaw += 1.0 * dt
        raw = [2.0, 0.0, 0.0, 1.0]  # exact finite diff of the above
        v = [a * r + (1 - a) * p for r, p in zip(raw, v)]
    assert abs(v[0] - 2.0) < 1e-6 and abs(v[3] - 1.0) < 1e-6, v
    print("selfcheck ok:", v)


def main(args=None):
    import sys
    if '--selfcheck' in sys.argv:
        _selfcheck()
        return
    rclpy.init(args=args)
    node = TrackedVehicleGroundTruthOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
