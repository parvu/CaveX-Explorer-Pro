#!/usr/bin/env python3
"""Logs (sonar row, commanded thrust) pairs for training AcousticUUVController.

One CSV row per incoming /ping360_sonar/scan_image frame: that frame's
beam_count intensity samples (0-255) plus the most recently commanded value
on each of the 6 /bluerov2/thrusterN/cmd_thrust topics at that instant. This
is imitation-learning data -- whatever is driving the vehicle (e.g.
corridor_walk_demo.py) is the "expert" being cloned; the model never sees
ground-truth pose, only what the real sonar pipeline would give it plus the
actions actually taken.
"""
import csv
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
import numpy as np


class TrainingDataLoggerNode(Node):
    def __init__(self):
        super().__init__('training_data_logger')

        self.declare_parameter('output_csv_path', 'sic_slam_training_data.csv')
        csv_path = self.get_parameter('output_csv_path').get_parameter_value().string_value
        self.csv_path = os.path.expanduser(csv_path)

        self.latest_thrust = [0.0] * 6
        for i in range(6):
            self.create_subscription(
                Float64, f'/bluerov2/thruster{i + 1}/cmd_thrust',
                self._make_thrust_cb(i), 10)

        self.sonar_sub = self.create_subscription(
            Image, '/ping360_sonar/scan_image', self.on_sonar, 10)

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self._header_written = False
        self._row_count = 0

        self.get_logger().info(f'training_data_logger active, writing to {self.csv_path}')

    def _make_thrust_cb(self, idx):
        def cb(msg):
            self.latest_thrust[idx] = msg.data
        return cb

    def on_sonar(self, msg: Image):
        row = np.frombuffer(msg.data, dtype=np.uint8)
        if not self._header_written:
            header = (
                ['timestamp_sec', 'timestamp_nanosec']
                + [f'sonar_{i}' for i in range(len(row))]
                + [f'thrust_{i + 1}' for i in range(6)]
            )
            self.csv_writer.writerow(header)
            self._header_written = True

        self.csv_writer.writerow(
            [msg.header.stamp.sec, msg.header.stamp.nanosec]
            + row.tolist()
            + list(self.latest_thrust)
        )
        self._row_count += 1
        if self._row_count % 50 == 0:
            self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.flush()
        self.csv_file.close()
        self.get_logger().info(f'training_data_logger: wrote {self._row_count} rows to {self.csv_path}')
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
