#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Imu
import numpy as np
import gtsam
from gtsam import Point3


class GtsamISAM2Optimizer:
    """Incremental Point3 dead-reckoning graph, corrected by landmark priors,
    solved with real GTSAM ISAM2 (ros-jazzy-gtsam / python3-gtsam 4.2.0).

    The latent water-current vector is not itself a GTSAM variable (that
    would need a custom CurrentFactor, out of scope here) -- it stays a
    plain running estimate driven by the landmark residual, same role it
    played in the previous hand-rolled version.
    """

    def __init__(self):
        self.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.step = 0
        self.pose = np.zeros(3)
        self.estimated_current = np.array([0.1, -0.05, 0.02])

        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.05]))
        self.landmark_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.2]))
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3, 1e-3, 1e-3]))

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(gtsam.PriorFactorPoint3(self._key(0), Point3(0.0, 0.0, 0.0), prior_noise))
        values.insert(self._key(0), Point3(0.0, 0.0, 0.0))
        self.isam.update(graph, values)

    @staticmethod
    def _key(i):
        return gtsam.symbol('x', i)

    def update_graph(self, imu_delta, landmark_vector=None):
        prev_key = self._key(self.step)
        self.step += 1
        cur_key = self._key(self.step)

        raw_displacement = imu_delta + self.estimated_current * 0.2
        predicted = self.pose + raw_displacement

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(gtsam.BetweenFactorPoint3(
            prev_key, cur_key, Point3(*raw_displacement), self.odom_noise))
        values.insert(cur_key, Point3(*predicted))

        if landmark_vector is not None:
            graph.add(gtsam.PriorFactorPoint3(
                cur_key, Point3(*landmark_vector), self.landmark_noise))

        self.isam.update(graph, values)
        self.pose = self.isam.calculateEstimate().atPoint3(cur_key)

        if landmark_vector is not None:
            residual_error = landmark_vector - predicted
            self.estimated_current += residual_error * 0.05

        return self.pose, self.estimated_current


class SicSlamGraphBackendNode(Node):
    def __init__(self):
        super().__init__('sic_slam_graph_backend')

        self.optimizer = GtsamISAM2Optimizer()
        self.last_imu_reading = np.zeros(3)

        self.landmark_sub = self.create_subscription(
            PointStamped,
            '/sic_slam/predicted_landmarks',
            self.landmark_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            50
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/sic_slam/odometry',
            10
        )

        self.get_logger().info('SIC-SLAM GTSAM ISAM2 Factor-Graph Estimation Backend Online.')

    def imu_callback(self, msg):
        self.last_imu_reading = np.array([
            msg.linear_acceleration.x * 0.04,
            msg.linear_acceleration.y * 0.04,
            msg.linear_acceleration.z * 0.04
        ])

        optimized_pose, _ = self.optimizer.update_graph(self.last_imu_reading, landmark_vector=None)
        self.publish_corrected_pose(optimized_pose)

    def landmark_callback(self, msg):
        landmark_vector = np.array([msg.point.x, msg.point.y, msg.point.z])

        optimized_pose, current_field = self.optimizer.update_graph(self.last_imu_reading, landmark_vector)

        self.get_logger().info(
            f'[ISAM2 Update] Optimized Pose: {optimized_pose} | Latent Current Vector: {current_field}'
        )
        self.publish_corrected_pose(optimized_pose)

    def publish_corrected_pose(self, pose_array):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(pose_array[0])
        msg.pose.position.y = float(pose_array[1])
        msg.pose.position.z = float(pose_array[2])
        self.pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SicSlamGraphBackendNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down factor graph node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
