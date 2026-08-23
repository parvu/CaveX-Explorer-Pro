#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import numpy as np
import gtsam
from gtsam import Point3

from sic_slam.current_factor import make_current_factor

# Real bug found and fixed 2026-08-23 (8-leg/25-run ATE ablation matrix,
# real request): CurrentFactor used to be given this HARDCODED dt for
# every graph step, regardless of which callback actually triggered it or
# how much real time had passed. Measured against real log timestamps
# from a completed ablation run: 100% of steps had real dt more than 0.02s
# off from this constant (median real dt 0.10s -- 2.5x this value; min
# 0.003s, max 0.40s). CurrentFactor's residual is
# (X_cur-X_prev) - (v_pred+C)*dt -- a wrong-by-up-to-10x dt means it
# asserts a physically wrong displacement almost every step, forcing a
# spurious correction into C (and, through it, into the shared pose
# estimate). This is why the with-CurrentFactor "nocurrent/clear" leg of
# that matrix came out clearly worse than without (0.535m +/- 0.163 vs
# 0.489m +/- 0.020) despite being the cleanest, easiest condition.
# Kept only as a fallback for the very first graph step, where there is
# no previous real timestamp yet to measure from.
FALLBACK_DT_S = 0.04
MIN_DT_S = 0.001   # guards a possible near-zero dt on a duplicate/racing callback
MAX_DT_S = 0.5     # guards a possible large gap (e.g. right after node startup)


class GtsamISAM2Optimizer:
    """Incremental Point3 dead-reckoning graph, corrected by landmark priors,
    solved with real GTSAM ISAM2 (ros-jazzy-gtsam / python3-gtsam 4.2.0).

    The latent water-current vector IS a real GTSAM variable now (2026-08-23,
    real request) -- current_factor.py's CurrentFactor connects each
    consecutive position pair to a single shared current key, using
    current_dynamics.py's thruster/drag physics (ported from
    cavex_sic_slam's real dynamics_model.hpp/current_factor.hpp) to predict
    through-water displacement; the residual against actual ground
    displacement is what makes the current observable. Replaces the earlier
    plain running estimate (current += landmark_residual * 0.05) entirely --
    that was a heuristic outside the graph, this is solved by ISAM2 like
    every other variable.
    """

    def __init__(self, enable_current_factor=True):
        # Matches cavex_sic_slam's own convention (sic_slam_node.cpp): when
        # disabled, there is no C(i) variable, no CurrentFactor, no current
        # modeling at all -- not "CurrentFactor present but unconstrained".
        self.enable_current_factor = enable_current_factor

        self.isam = gtsam.ISAM2(gtsam.ISAM2Params())
        self.step = 0
        self.pose = np.zeros(3)

        self.odom_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.05, 0.05, 0.05]))
        self.landmark_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([0.2, 0.2, 0.2]))
        prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([1e-3, 1e-3, 1e-3]))

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(gtsam.PriorFactorPoint3(self._key(0), Point3(0.0, 0.0, 0.0), prior_noise))
        values.insert(self._key(0), Point3(0.0, 0.0, 0.0))

        self.estimated_current = np.zeros(3)
        if self.enable_current_factor:
            self.current_key = gtsam.symbol('c', 0)
            # Loose, not tight: unlike the pose prior above (which anchors a
            # known spawn point), the initial current guess is genuinely
            # unknown -- a tight prior here would fight every CurrentFactor
            # residual instead of letting ISAM2 actually solve for it.
            self.current_prior_noise = gtsam.noiseModel.Diagonal.Sigmas(np.array([5.0, 5.0, 5.0]))
            # Matches cavex_sic_slam's own CurrentFactor noise exactly
            # (sic_slam_node.cpp: gtsam::noiseModel::Isotropic::Sigma(3, 0.15)).
            self.current_factor_noise = gtsam.noiseModel.Isotropic.Sigma(3, 0.15)
            graph.add(gtsam.PriorFactorVector(self.current_key, np.zeros(3), self.current_prior_noise))
            values.insert(self.current_key, np.zeros(3))

        self.isam.update(graph, values)

    @staticmethod
    def _key(i):
        return gtsam.symbol('x', i)

    def update_graph(self, imu_delta, thrust_n, dt=FALLBACK_DT_S, landmark_vector=None):
        prev_key = self._key(self.step)
        self.step += 1
        cur_key = self._key(self.step)

        # Real bug found and fixed 2026-08-23: this used to fold
        # self.estimated_current back into the BetweenFactor's own fixed
        # MEASUREMENT (raw_displacement = imu_delta + estimated_current *
        # 0.2), which was fine for the old heuristic (current only ever
        # driven by landmark residuals, an independent source) but becomes
        # a real positive-feedback loop now that CurrentFactor also feeds
        # off this same BetweenFactor: current estimate grows ->
        # raw_displacement grows -> BetweenFactor pins X_cur-X_prev to that
        # larger value -> CurrentFactor's residual pulls current higher
        # still. Live-verified runaway (current_x: 11 -> 461 within ~1s of
        # sim time, at rest with zero thrust). Same "information
        # double-counting" failure class as the five failed continuous-
        # current-field feedback attempts documented in history_main.md.
        # Fix: the odometry measurement is IMU-only now (matches real IMU
        # preintegration, which never incorporates a current estimate);
        # current correction happens ONLY through CurrentFactor's own
        # separate residual against this same displacement.
        predicted = self.pose + imu_delta

        graph = gtsam.NonlinearFactorGraph()
        values = gtsam.Values()
        graph.add(gtsam.BetweenFactorPoint3(
            prev_key, cur_key, Point3(*imu_delta), self.odom_noise))
        values.insert(cur_key, Point3(*predicted))
        if self.enable_current_factor:
            graph.add(make_current_factor(
                prev_key, cur_key, self.current_key, thrust_n, dt, self.current_factor_noise))

        if landmark_vector is not None:
            graph.add(gtsam.PriorFactorPoint3(
                cur_key, Point3(*landmark_vector), self.landmark_noise))

        self.isam.update(graph, values)
        estimate = self.isam.calculateEstimate()
        self.pose = estimate.atPoint3(cur_key)
        if self.enable_current_factor:
            self.estimated_current = estimate.atVector(self.current_key)

        return self.pose, self.estimated_current


class SicSlamGraphBackendNode(Node):
    def __init__(self):
        super().__init__('sic_slam_graph_backend')

        self.declare_parameter('enable_current_factor', True)
        enable_current_factor = self.get_parameter('enable_current_factor').get_parameter_value().bool_value
        self.optimizer = GtsamISAM2Optimizer(enable_current_factor=enable_current_factor)
        self.last_imu_reading = np.zeros(3)
        self.last_thrust = [0.0] * 6
        self.last_step_time = None

        for i in range(6):
            self.create_subscription(
                Float64, f'/bluerov2/thruster{i + 1}/cmd_thrust',
                self._make_thrust_cb(i), 10)

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

        self.get_logger().info(
            'SIC-SLAM GTSAM ISAM2 Factor-Graph Estimation Backend Online '
            '(with real CurrentFactor).')

    def _make_thrust_cb(self, idx):
        def cb(msg):
            self.last_thrust[idx] = msg.data
        return cb

    def _measure_dt(self):
        """Real elapsed time since the last graph step, not a hardcoded
        constant -- see FALLBACK_DT_S's own comment for why this matters."""
        now = self.get_clock().now()
        if self.last_step_time is None:
            dt = FALLBACK_DT_S
        else:
            dt = (now - self.last_step_time).nanoseconds * 1e-9
            dt = min(max(dt, MIN_DT_S), MAX_DT_S)
        self.last_step_time = now
        return dt

    def imu_callback(self, msg):
        self.last_imu_reading = np.array([
            msg.linear_acceleration.x * 0.04,
            msg.linear_acceleration.y * 0.04,
            msg.linear_acceleration.z * 0.04
        ])

        optimized_pose, _ = self.optimizer.update_graph(
            self.last_imu_reading, self.last_thrust, self._measure_dt(), landmark_vector=None)
        self.publish_corrected_pose(optimized_pose)

    def landmark_callback(self, msg):
        landmark_vector = np.array([msg.point.x, msg.point.y, msg.point.z])

        optimized_pose, current_field = self.optimizer.update_graph(
            self.last_imu_reading, self.last_thrust, self._measure_dt(), landmark_vector)

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
