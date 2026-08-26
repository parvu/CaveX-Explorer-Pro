#!/usr/bin/env python3
"""
ate_evaluator_node.py

ROS2 node providing simulated ATE (Absolute Trajectory Error) evaluation
for CAVE-SLAM / gtsam_slam, matching the OS2 objective's stated methodology
(Funding Application, Section B.2.1): "Absolute Trajectory Error (ATE)
below 0.5 m in simulation and below 1.5 m in basin testing, each measured
against ground truth over a minimum of 10 runs, reported as mean +/-
standard deviation."

This node subscribes to two Odometry streams:
  - /cavex/ground_truth/odom : ground-truth pose, published by the Gazebo
    simulation (or, for basin testing, derived from the acoustic-beacon
    ground-truth grid described in Section B.2.3 / T4.3).
  - /cavex/slam/odom          : gtsam_slam's estimated pose, published by the
    perception/SLAM pipeline (WP2-WP3).

It buffers timestamp-matched pairs, and on receiving a `/cavex/eval/finish_run`
trigger (std_msgs/Empty) -- e.g. published at the end of a simulated
trajectory -- computes ATE for that run using the same Umeyama-alignment
methodology as ate_metrics.py (tested standalone; see that file's __main__
self-test), appends the result to a CSV log, and publishes a summary.

Running `analyze_ate_runs.py` afterwards on the accumulated CSV produces
the mean +/- std across runs required by OS2.

NOTE ON HONESTY: this node provides the *evaluation harness* -- it computes
ATE correctly given two pose streams. It does not itself implement gtsam_slam
or the CurrentFactor; those remain WP2-WP3 deliverables. Do not present the
existence of this evaluator as evidence that gtsam_slam itself has been run
against it -- that distinction matters when this repository is cited in the
Funding Application (Section B.2.2, ref. [9]).
"""

import csv
import os
from collections import deque

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, Float64

from cavex_slam_nav.ate_metrics import compute_ate


class ATEEvaluatorNode(Node):
    def __init__(self):
        super().__init__('ate_evaluator_node')

        self.declare_parameter('ground_truth_topic', '/cavex/ground_truth/odom')
        self.declare_parameter('estimate_topic', '/cavex/slam/odom')
        self.declare_parameter('sync_tolerance_sec', 0.05)
        self.declare_parameter('output_csv', 'cavex_ate_runs.csv')

        gt_topic = self.get_parameter('ground_truth_topic').value
        est_topic = self.get_parameter('estimate_topic').value
        self.sync_tol = self.get_parameter('sync_tolerance_sec').value
        self.output_csv = self.get_parameter('output_csv').value

        self._gt_buffer = deque(maxlen=20000)   # (t_sec, x, y, z)
        self._est_buffer = deque(maxlen=20000)  # (t_sec, x, y, z)

        # Best-effort so this matches either a best-effort publisher (e.g.
        # ros_gz_bridge-sourced ground truth) or a reliable one -- a
        # best-effort subscriber is DDS-compatible with both, while a
        # reliable subscriber (rclpy's default) silently never connects to
        # a best-effort publisher. Confirmed the hard way against rtabmap
        # itself before applying the same fix here.
        odom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
        )
        self.gt_sub = self.create_subscription(Odometry, gt_topic, self._gt_cb, odom_qos)
        self.est_sub = self.create_subscription(Odometry, est_topic, self._est_cb, odom_qos)
        self.finish_sub = self.create_subscription(Empty, '/cavex/eval/finish_run', self._finish_run_cb, 1)

        self.ate_pub = self.create_publisher(Float64, '/cavex/eval/ate_rmse', 10)

        self._run_index = self._count_existing_runs()
        self.get_logger().info(
            f"ATE evaluator started. GT topic: {gt_topic}, estimate topic: {est_topic}. "
            f"Publish an empty message to /cavex/eval/finish_run to score a run. "
            f"Existing runs in {self.output_csv}: {self._run_index}"
        )

    def _count_existing_runs(self):
        if not os.path.exists(self.output_csv):
            return 0
        with open(self.output_csv, 'r') as f:
            return max(0, sum(1 for _ in f) - 1)  # minus header

    def _odom_to_xyz_t(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        return t, p.x, p.y, p.z

    def _gt_cb(self, msg):
        self._gt_buffer.append(self._odom_to_xyz_t(msg))

    def _est_cb(self, msg):
        self._est_buffer.append(self._odom_to_xyz_t(msg))

    def _match_trajectories(self):
        """Greedy nearest-timestamp matching between the two buffers."""
        if len(self._gt_buffer) < 3 or len(self._est_buffer) < 3:
            return None, None

        gt = np.array(self._gt_buffer)   # (N, 4): t, x, y, z
        est = np.array(self._est_buffer)

        gt_matched, est_matched = [], []
        for t_e, x, y, z in est:
            idx = np.argmin(np.abs(gt[:, 0] - t_e))
            if abs(gt[idx, 0] - t_e) <= self.sync_tol:
                gt_matched.append(gt[idx, 1:4])
                est_matched.append([x, y, z])

        if len(gt_matched) < 3:
            return None, None
        return np.array(est_matched), np.array(gt_matched)

    def _finish_run_cb(self, _msg):
        est, gt = self._match_trajectories()
        if est is None:
            self.get_logger().warn(
                "Not enough time-synchronized ground-truth/estimate pairs "
                f"(tolerance={self.sync_tol}s) to compute ATE for this run."
            )
            return

        result = compute_ate(est, gt, align=True)
        self._run_index += 1

        self.get_logger().info(
            f"Run {self._run_index}: ATE RMSE = {result['rmse']:.4f} m "
            f"(mean={result['mean']:.4f}, std={result['std']:.4f}, "
            f"n_samples={result['n_samples']})"
        )

        self.ate_pub.publish(Float64(data=result['rmse']))
        self._append_csv_row(self._run_index, result)

        self._gt_buffer.clear()
        self._est_buffer.clear()

    def _append_csv_row(self, run_index, result):
        write_header = not os.path.exists(self.output_csv)
        with open(self.output_csv, 'a', newline='') as f:
            writer = csv.writer(f)
            if write_header:
                writer.writerow(['run', 'ate_rmse_m', 'ate_mean_m', 'ate_std_m',
                                  'ate_median_m', 'ate_max_m', 'n_samples'])
            writer.writerow([run_index, result['rmse'], result['mean'], result['std'],
                              result['median'], result['max'], result['n_samples']])


def main(args=None):
    rclpy.init(args=args)
    node = ATEEvaluatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
