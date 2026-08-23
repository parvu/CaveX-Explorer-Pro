#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped
from sensor_msgs.msg import Imu
import csv
import os


class SicSlamFlightLoggerNode(Node):
    def __init__(self):
        super().__init__('sic_slam_flight_logger')

        self.declare_parameter('output_csv_name', 'SIC_SLAM_Flight_Log.csv')
        # ponytail: was hardcoded to ~/dev_ws/, a workspace path that doesn't
        # exist on every machine this runs on -- default to cwd (wherever
        # `ros2 launch` was invoked from) instead, still overridable.
        self.declare_parameter('output_dir', os.getcwd())
        csv_filename = self.get_parameter('output_csv_name').get_parameter_value().string_value
        output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.csv_path = os.path.join(os.path.expanduser(output_dir), csv_filename)

        self.latest_imu = [0.0, 0.0, 0.0]
        self.latest_landmark = [0.0, 0.0, 0.0]

        self.csv_file = open(self.csv_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'Timestamp_sec', 'Timestamp_nanosec',
            'IMU_Accel_X', 'IMU_Accel_Y', 'IMU_Accel_Z',
            'Landmark_X', 'Landmark_Y', 'Landmark_Z',
            'Optimized_Pose_X', 'Optimized_Pose_Y', 'Optimized_Pose_Z'
        ])

        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )
        self.landmark_sub = self.create_subscription(
            PointStamped, '/sic_slam/predicted_landmarks', self.landmark_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/sic_slam/odometry', self.pose_callback, 10
        )

        self.get_logger().info(f'SIC-SLAM Logging Engine Active. Writing to: {self.csv_path}')

    def imu_callback(self, msg):
        self.latest_imu = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]

    def landmark_callback(self, msg):
        self.latest_landmark = [msg.point.x, msg.point.y, msg.point.z]

    def pose_callback(self, msg):
        timestamp = msg.header.stamp
        self.csv_writer.writerow([
            timestamp.sec, timestamp.nanosec,
            self.latest_imu[0], self.latest_imu[1], self.latest_imu[2],
            self.latest_landmark[0], self.latest_landmark[1], self.latest_landmark[2],
            msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        ])
        self.csv_file.flush()

    def destroy_node(self):
        self.csv_file.close()
        self.get_logger().info('Flight log file closed cleanly.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SicSlamFlightLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupt caught, spinning down logging loops.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
