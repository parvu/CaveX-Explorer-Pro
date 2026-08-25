#!/usr/bin/env python3
"""Publishes the two corner readouts for sic_slam_cave_water.world's GUI
(current speed bottom-left, turbidity bottom-right -- see that world
file's InfoLabel plugin entries, sic_slam_gui/src/InfoLabel.{hh,cc,qml})
and draws a current vector field (gz Marker ARROWs) spanning the full
water volume.

Both current_speed_mps and turbidity_absorption_db_per_m are plain launch
parameters here (real request: "make current speed and turbidity run
parameters" -- previously current speed was read live off /ocean_current,
which is more correct for a genuinely time-varying current profile, but
this project's own current_field_node profiles used so far are all
constant-for-the-run, so a live subscription was solving a problem this
project doesn't have yet; a static parameter is simpler and matches
turbidity's own existing convention). If a time-varying profile is used
later, this would need to go back to live-reading /ocean_current -- noted
here rather than silently dropped.

InfoLabel subscribes over gz-transport (it's a gz-gui plugin, not a ROS
node), so both readouts publish gz.msgs.StringMsg, pre-formatted, rather
than the raw ROS std_msgs/Float64 used before TopicEcho was replaced.

Marker topic ("/marker", gz.msgs.Marker) confirmed by the plugin's own
real name (MarkerManager, GUI-side, no server-side marker system exists in
this gz-sim8 install -- checked directly, not guessed) and Gazebo's
documented marker API.
"""
import math

import rclpy
from rclpy.node import Node

from gz.transport13 import Node as GzNode
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.stringmsg_pb2 import StringMsg

# Water region extent, matching sic_slam_cave_water.world's own geometry
# (x[15,35], y[-12,12], floor z=5.9, surface z=7.9). "Full water volume"
# (real request) -- a 3D grid spanning the whole navigable water box, not
# just one depth slice.
GRID_X = [18, 22, 25, 28, 32]
GRID_Y = [-8, -4, 0, 4, 8]
GRID_Z = [6.2, 6.9, 7.6]
# Scales arrow length per m/s of current -- at ~0.3 m/s (the range used
# elsewhere in this project) arrows read as a clearly visible ~1.5m, but
# don't overrun the grid spacing (~3-4m in x/y) at higher speeds.
LENGTH_PER_MPS = 5.0
MIN_VISIBLE_LEN = 0.05  # a near-zero-length arrow still renders as a dot


class SimInfoPublisher(Node):
    def __init__(self):
        super().__init__('sim_info_publisher')
        self.declare_parameter('current_speed_mps', 0.0)
        self.declare_parameter('current_heading_rad', 0.0)
        self.declare_parameter('turbidity_absorption_db_per_m', 0.0)
        self.speed = self.get_parameter(
            'current_speed_mps').get_parameter_value().double_value
        self.heading = self.get_parameter(
            'current_heading_rad').get_parameter_value().double_value
        self.turbidity = self.get_parameter(
            'turbidity_absorption_db_per_m').get_parameter_value().double_value

        self.gz_node = GzNode()
        self.speed_label_pub = self.gz_node.advertise(
            '/sic_slam/current_speed_label', StringMsg)
        self.turbidity_label_pub = self.gz_node.advertise(
            '/sic_slam/turbidity_label', StringMsg)
        self.marker_pub = self.gz_node.advertise('/marker', Marker)

        self.create_timer(1.0, self._publish_labels)
        self.create_timer(1.0, self._publish_markers)
        self._publish_labels()
        self._publish_markers()
        self.get_logger().info(
            f'sim_info_publisher ready: current_speed_mps={self.speed}, '
            f'turbidity_absorption_db_per_m={self.turbidity}, '
            f'{len(GRID_X) * len(GRID_Y) * len(GRID_Z)} current-vector markers '
            f'across the full water volume -> /marker')

    def _publish_labels(self):
        self.speed_label_pub.publish(
            StringMsg(data=f'Current speed: {self.speed:.2f} m/s'))
        self.turbidity_label_pub.publish(
            StringMsg(data=f'Turbidity: {self.turbidity:.2f} dB/m'))

    def _publish_markers(self):
        length = max(self.speed * LENGTH_PER_MPS, MIN_VISIBLE_LEN)
        qz, qw = math.sin(self.heading / 2.0), math.cos(self.heading / 2.0)

        marker_id = 1
        for gx in GRID_X:
            for gy in GRID_Y:
                for gz in GRID_Z:
                    m = Marker()
                    m.ns = 'current_field'
                    m.id = marker_id
                    m.action = Marker.ADD_MODIFY
                    m.type = Marker.ARROW
                    m.visibility = Marker.GUI
                    m.pose.position.x = gx
                    m.pose.position.y = gy
                    m.pose.position.z = gz
                    m.pose.orientation.z = qz
                    m.pose.orientation.w = qw
                    m.scale.x = length
                    m.scale.y = 0.25
                    m.scale.z = 0.25
                    m.material.ambient.r = 0.9
                    m.material.ambient.g = 0.9
                    m.material.ambient.b = 0.1
                    m.material.ambient.a = 0.9
                    m.material.diffuse.r = 0.9
                    m.material.diffuse.g = 0.9
                    m.material.diffuse.b = 0.1
                    m.material.diffuse.a = 0.9
                    self.marker_pub.publish(m)
                    marker_id += 1


def main():
    rclpy.init()
    node = SimInfoPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
