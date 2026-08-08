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
import rclpy
from rclpy.node import Node as RclpyNode
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from nav_msgs.msg import Odometry

VEHICLE_MODEL_NAME = 'cavex_tracked_blueboat'
GZ_POSE_TOPIC = '/world/cavex_world/pose/info'


class TrackedVehicleGroundTruthOdom(RclpyNode):
    def __init__(self):
        super().__init__('tracked_vehicle_ground_truth_odom')
        self.pub = self.create_publisher(Odometry, '/odom_ground_truth', 10)
        # Kept as self._gz_node (not a local var) -- gz-transport's
        # subscription lives as long as this Node object does; letting it
        # get garbage-collected would silently stop delivery.
        self._gz_node = GzNode()
        self._gz_node.subscribe(Pose_V, GZ_POSE_TOPIC, self._cb)

    def _cb(self, msg: Pose_V):
        for pose in msg.pose:
            if pose.name != VEHICLE_MODEL_NAME:
                continue
            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.pose.pose.position.x = pose.position.x
            odom.pose.pose.position.y = pose.position.y
            odom.pose.pose.position.z = pose.position.z
            odom.pose.pose.orientation.x = pose.orientation.x
            odom.pose.pose.orientation.y = pose.orientation.y
            odom.pose.pose.orientation.z = pose.orientation.z
            odom.pose.pose.orientation.w = pose.orientation.w
            self.pub.publish(odom)
            return


def main(args=None):
    rclpy.init(args=args)
    node = TrackedVehicleGroundTruthOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
