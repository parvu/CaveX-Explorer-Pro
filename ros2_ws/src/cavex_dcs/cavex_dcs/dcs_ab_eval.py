#!/usr/bin/env python3
# Copyright 2026 CaveX Explorer Pro
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
dcs_ab_eval.

Measures cross-track velocity RMSE (actual body velocity vs. the commanded
heading -- the same "instantaneous track" convention dcs_math.py's
cross_track_error() uses, consistent with this project's other manual-teleop
phases having no persistent waypoint path to score against) with DCS on vs.
off, appending one CSV row per run to cavex_dcs_ab_runs.csv.

Also logs the mean commanded current from /cavex/current_ground_truth
alongside the RMSE, purely as run-context for interpreting the result (e.g.
"this RMSE was measured under a 0.3 m/s current") -- it is READ ONLY for
that logging purpose and never feeds the RMSE computation or any control
decision, matching this project's scorer-only discipline for ground truth.
"""
import csv
import math
import os
import sys

from cavex_dcs.dcs_math import cross_track_error, rotate_world_to_body
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class DcsAbEvalRunner(Node):
    """Drive a fixed heading for budget_sim_s, log cross-track RMSE."""

    def __init__(self, dcs_enabled: bool, budget_sim_s: float, heading_vx: float):
        super().__init__('dcs_ab_eval_runner')
        self._dcs_enabled = dcs_enabled
        self._budget_sim_s = budget_sim_s
        self._heading_vx = heading_vx
        self._errors = []
        self._yaw = 0.0
        self._start_time = None
        self._ground_truth_currents = []

        cmd_topic = '/cmd_vel_rov_desired' if dcs_enabled else '/cmd_vel_rov'
        self._cmd_pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(Odometry, '/gtsam_slam/odometry', self._odom_cb, 10)
        self.create_subscription(
            Vector3Stamped, '/cavex/current_ground_truth', self._current_gt_cb, 10)
        self._timer = self.create_timer(0.1, self._tick)

    def _current_gt_cb(self, msg: Vector3Stamped):
        # Logging context only -- never used in the RMSE computation.
        self._ground_truth_currents.append(msg.vector.x)

    def mean_ground_truth_current(self):
        if not self._ground_truth_currents:
            return float('nan')
        return sum(self._ground_truth_currents) / len(self._ground_truth_currents)

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)
        body_vx, body_vy = rotate_world_to_body(
            msg.twist.twist.linear.x, msg.twist.twist.linear.y, self._yaw)
        err = cross_track_error(body_vx, body_vy, self._heading_vx, 0.0)
        self._errors.append(err)

    def _tick(self):
        now = self.get_clock().now()
        if self._start_time is None:
            self._start_time = now
        elapsed = (now - self._start_time).nanoseconds * 1e-9
        if elapsed >= self._budget_sim_s:
            self._timer.cancel()
            return
        cmd = Twist()
        cmd.linear.x = self._heading_vx
        self._cmd_pub.publish(cmd)

    def rmse(self):
        if not self._errors:
            return float('nan')
        return math.sqrt(sum(e * e for e in self._errors) / len(self._errors))


def run_once(dcs_enabled: bool, budget_sim_s: float, heading_vx: float):
    node = DcsAbEvalRunner(dcs_enabled, budget_sim_s, heading_vx)
    end_time = node.get_clock().now().nanoseconds + int(budget_sim_s * 1.5 * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end_time:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node._timer.is_canceled():
            break
    rmse = node.rmse()
    mean_current = node.mean_ground_truth_current()
    node.destroy_node()
    return rmse, mean_current


def main():
    if len(sys.argv) < 2:
        print('Usage: dcs_ab_eval.py <output_csv> [budget_sim_s] [heading_vx]')
        sys.exit(1)
    csv_path = sys.argv[1]
    budget_sim_s = float(sys.argv[2]) if len(sys.argv) > 2 else 20.0
    heading_vx = float(sys.argv[3]) if len(sys.argv) > 3 else 0.4

    rclpy.init()
    dcs_off_rmse, off_current = run_once(
        dcs_enabled=False, budget_sim_s=budget_sim_s, heading_vx=heading_vx)
    dcs_on_rmse, on_current = run_once(
        dcs_enabled=True, budget_sim_s=budget_sim_s, heading_vx=heading_vx)
    rclpy.shutdown()

    write_header = not os.path.exists(csv_path)
    with open(csv_path, 'a', newline='') as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow([
                'dcs_enabled', 'cross_track_rmse_mps', 'heading_vx', 'budget_sim_s',
                'mean_ground_truth_current_mps'])
        writer.writerow([False, dcs_off_rmse, heading_vx, budget_sim_s, off_current])
        writer.writerow([True, dcs_on_rmse, heading_vx, budget_sim_s, on_current])

    print(f'DCS off: cross-track RMSE = {dcs_off_rmse:.4f} m/s (mean current {off_current:.3f})')
    print(f'DCS on:  cross-track RMSE = {dcs_on_rmse:.4f} m/s (mean current {on_current:.3f})')


if __name__ == '__main__':
    main()
