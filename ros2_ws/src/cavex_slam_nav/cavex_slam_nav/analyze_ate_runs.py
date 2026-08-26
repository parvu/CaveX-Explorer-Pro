#!/usr/bin/env python3
"""
analyze_ate_runs.py

Reads the CSV log produced by ate_evaluator_node.py across multiple runs
and reports mean +/- standard deviation of ATE RMSE, matching the exact
methodology stated in the Funding Application's OS2 objective:
"measured against ground truth over a minimum of 10 runs, reported as
mean +/- standard deviation."

No ROS2 dependency -- this is a pure post-processing script, runnable
directly on the CSV log after a batch of simulation runs.

Usage:
    python3 analyze_ate_runs.py cavex_ate_runs.csv
"""

import csv
import sys

import numpy as np

from cavex_slam_nav.ate_metrics import summarize_multi_run


def load_runs(csv_path):
    rmse_values = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rmse_values.append(float(row['ate_rmse_m']))
    return rmse_values


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 analyze_ate_runs.py <path_to_ate_runs.csv>")
        sys.exit(1)

    csv_path = sys.argv[1]
    rmse_values = load_runs(csv_path)

    if len(rmse_values) == 0:
        print(f"No runs found in {csv_path}")
        sys.exit(1)

    summary = summarize_multi_run(rmse_values)

    print(f"Runs analyzed: {summary['n_runs']}")
    if summary['n_runs'] < 10:
        print(f"WARNING: OS2 requires a minimum of 10 runs; only {summary['n_runs']} found. "
              "Results below are preliminary, not yet meeting the stated methodology.")
    print(f"ATE RMSE: {summary['mean']:.4f} +/- {summary['std']:.4f} m "
          f"(min={summary['min']:.4f}, max={summary['max']:.4f})")
    print()
    print("NOTE: this measures whatever pose stream was configured as ate_evaluator_node's")
    print("'estimate_topic' -- as of this wiring, that is /gtsam_slam/odometry, produced by")
    print("dead_reckoning_prototype_node.py: a real but MINIMAL PROTOTYPE (cmd_vel+IMU dead")
    print("reckoning, bias-corrected against RTAB-Map's pose). It is NOT the full sonar +")
    print("Invariant-EKF + GTSAM CurrentFactor system described in the Funding Application")
    print("(Section B.2.2, ref. [9]) -- that remains a WP2-WP3 deliverable. Label results as")
    print("'gtsam_slam v0 (prototype)', not 'gtsam_slam', until the full system replaces")
    print("this node.")
    print()
    print("Reporting sentence template (update Section B.2.1/OS2):")
    print(f'  "Across {summary["n_runs"]} runs, gtsam_slam v0 (prototype) achieved an ATE of '
          f'{summary["mean"]:.3f} +/- {summary["std"]:.3f} m, '
          f'{"meeting" if summary["mean"] < 0.5 else "not yet meeting"} '
          f'the OS2 simulation target of < 0.5 m."')


if __name__ == '__main__':
    main()
