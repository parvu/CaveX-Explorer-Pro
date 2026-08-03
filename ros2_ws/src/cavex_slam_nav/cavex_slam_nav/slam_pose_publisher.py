#!/usr/bin/env python3
"""
slam_pose_publisher.py

RTAB-Map's SLAM-corrected pose lives in the map -> base_footprint TF chain
(map -> odom is RTAB-Map's loop-closure correction; odom -> base_footprint
comes from odom_tf_broadcaster.py). The topic-based outputs rtabmap offers
(/localization_pose, /mapPath) are event-driven (localization mode / graph
updates) and didn't produce continuous data in mapping mode during testing.

This node looks up that TF chain on a timer and republishes it as a plain
nav_msgs/Odometry stream on /cavex/slam/odom -- ate_evaluator_node.py's
default estimate topic -- so it can be scored against ground truth like
any other odometry-shaped estimate. This is RTAB-Map's real, TF-verified
corrected pose, not a placeholder.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from nav_msgs.msg import Odometry


class SlamPosePublisher(Node):
    def __init__(self):
        super().__init__('slam_pose_publisher')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('output_topic', '/cavex/slam/odom')
        self.declare_parameter('rate_hz', 10.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pub = self.create_publisher(Odometry, self.get_parameter('output_topic').value, 10)

        rate_hz = self.get_parameter('rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _tick(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time(), timeout=Duration(seconds=0.05))
        except TransformException:
            return

        msg = Odometry()
        msg.header.stamp = tf.header.stamp
        msg.header.frame_id = self.map_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = tf.transform.translation.x
        msg.pose.pose.position.y = tf.transform.translation.y
        msg.pose.pose.position.z = tf.transform.translation.z
        msg.pose.pose.orientation = tf.transform.rotation
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SlamPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
