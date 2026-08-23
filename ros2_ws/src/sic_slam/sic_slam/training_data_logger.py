#!/usr/bin/env python3
"""Logs (sonar row, ground-truth position) pairs for training
AcousticUUVController.

Corrected 2026-08-23 (real bug found before any training happened): the
model (sic_slam/model.py) outputs a 3D landmark/position vector, not
thruster commands -- an earlier version of this logger paired sonar rows
with commanded thrust instead, which is not a usable label for this model
at all. The graph backend (sic_slam_graph_backend.py) treats the predicted
vector as a noisy absolute-position fix and blends it into its own
dead-reckoned pose, which starts at the origin at spawn time -- so the
correct supervision target is the vehicle's real position relative to its
own spawn point (Gazebo ground truth, not something a perception node
would have in deployment, but this is offline training data collection,
not the live pipeline).

One CSV row per incoming /ping360_sonar/scan_image frame: that frame's
beam_count intensity samples (0-255) plus (gt_x, gt_y, gt_z) -- the
bluerov2 model's ground-truth position at that instant, read directly via
gz-transport (same pattern as corridor_walk_demo.py/ate_baseline_demo.py),
relative to spawn (0, 0, -2) so it starts at (0, 0, 0) like the graph
backend's own internal frame.
"""
import csv
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

SPAWN_XYZ = (0.0, 0.0, -2.0)  # matches sim_launch.py's -x 0 -y 0 -z -2

_latest_gt = {"pos": None}


def _pose_cb(msg: Pose_V):
    for pose in msg.pose:
        if pose.name == "bluerov2":
            _latest_gt["pos"] = (pose.position.x, pose.position.y, pose.position.z)


class TrainingDataLoggerNode(Node):
    def __init__(self):
        super().__init__('training_data_logger')

        self.declare_parameter('output_csv_path', 'sic_slam_training_data.csv')
        csv_path = self.get_parameter('output_csv_path').get_parameter_value().string_value
        self.csv_path = os.path.expanduser(csv_path)

        self.gz_node = GzNode()
        self.gz_node.subscribe(Pose_V, "/world/sic_slam_tank/pose/info", _pose_cb)

        self.sonar_sub = self.create_subscription(
            Image, '/ping360_sonar/scan_image', self.on_sonar, 10)

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self._header_written = False
        self._row_count = 0
        self._skipped_no_pose = 0

        self.get_logger().info(f'training_data_logger active, writing to {self.csv_path}')

    def on_sonar(self, msg: Image):
        if _latest_gt["pos"] is None:
            self._skipped_no_pose += 1
            return

        row = np.frombuffer(msg.data, dtype=np.uint8)
        if not self._header_written:
            header = (
                ['timestamp_sec', 'timestamp_nanosec']
                + [f'sonar_{i}' for i in range(len(row))]
                + ['gt_x', 'gt_y', 'gt_z']
            )
            self.csv_writer.writerow(header)
            self._header_written = True

        gx, gy, gz = _latest_gt["pos"]
        self.csv_writer.writerow(
            [msg.header.stamp.sec, msg.header.stamp.nanosec]
            + row.tolist()
            + [gx - SPAWN_XYZ[0], gy - SPAWN_XYZ[1], gz - SPAWN_XYZ[2]]
        )
        self._row_count += 1
        if self._row_count % 50 == 0:
            self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info(
            f'training_data_logger: wrote {self._row_count} rows '
            f'({self._skipped_no_pose} sonar frames skipped, no ground-truth pose yet) '
            f'to {self.csv_path}')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrainingDataLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
