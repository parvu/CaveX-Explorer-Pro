#!/usr/bin/env python3
"""Publishes the two corner readouts for sic_slam_cave_water.world's GUI
(current speed, turbidity -- see that world file's InfoLabel plugin
entries, sic_slam_gui/src/InfoLabel.{hh,cc,qml}) and draws a current
vector field (gz Marker ARROWs) spanning the full water volume.

Both values start from launch parameters (current_vx/absorption_db_per_m)
but are live-editable from the GUI during a run (real request: "i want to
change them during simulation") -- InfoLabel's input field publishes the
edited text to a gz-transport "_set" topic here, which is used two ways:

- Current speed: this node owns /ocean_current publishing itself for
  sic_slam_cave_water.world (see own_ocean_current below and
  sim_launch.py's own comment on why current_field_node.py -- cavex_sonar,
  reused unmodified, no live-reconfigure support -- isn't launched for
  that world at all instead of fighting this node over the same topic).
- Turbidity: republished on the ROS topic
  /sic_slam/turbidity_absorption_db_per_m, which sonar_node.cpp
  (cavex_sonar) now subscribes to directly (a small, additive live-update
  hook added there, mirroring that file's own existing current_vx_/
  current_vy_ pattern) -- this works for either world, not just
  cave_water, since sonar_node itself doesn't know which world it's in.

Marker topic ("/marker", gz.msgs.Marker) confirmed by the plugin's own
real name (MarkerManager, GUI-side, no server-side marker system exists in
this gz-sim8 install -- checked directly, not guessed) and Gazebo's
documented marker API.
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64

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
# elsewhere in this project) arrows read as a clearly visible ~1.8m, but
# don't overrun the grid spacing (~3-4m in x/y) at higher speeds.
LENGTH_PER_MPS = 6.0
# Real request: "make velocity field more visible" -- 0.05m (the original
# value) rendered as a near-invisible dot at zero/low current, and 0.25
# shaft thickness read as a thin line against the cave's own rock texture
# and the water plane's tint. Raised the floor length so the grid pattern
# itself stays legible even with no current (still short relative to the
# ~3-4m grid spacing, so it doesn't read as false current), and thickened
# the shaft/head (ARROW's shaft+head radius scale with scale.y/scale.z).
MIN_VISIBLE_LEN = 0.8
ARROW_THICKNESS = 0.45


class SimInfoPublisher(Node):
    def __init__(self):
        super().__init__('sim_info_publisher')
        self.declare_parameter('current_speed_mps', 0.0)
        self.declare_parameter('current_heading_rad', 0.0)
        self.declare_parameter('turbidity_absorption_db_per_m', 0.0)
        self.declare_parameter('own_ocean_current', False)
        self.speed = self.get_parameter(
            'current_speed_mps').get_parameter_value().double_value
        self.heading = self.get_parameter(
            'current_heading_rad').get_parameter_value().double_value
        self.turbidity = self.get_parameter(
            'turbidity_absorption_db_per_m').get_parameter_value().double_value
        self.own_ocean_current = self.get_parameter(
            'own_ocean_current').get_parameter_value().bool_value

        self.gz_node = GzNode()
        self.speed_label_pub = self.gz_node.advertise(
            '/sic_slam/current_speed_label', StringMsg)
        self.turbidity_label_pub = self.gz_node.advertise(
            '/sic_slam/turbidity_label', StringMsg)
        self.marker_pub = self.gz_node.advertise('/marker', Marker)
        self.gz_node.subscribe(StringMsg, '/sic_slam/current_speed_set', self._speed_set_cb)
        self.gz_node.subscribe(StringMsg, '/sic_slam/turbidity_set', self._turbidity_set_cb)

        self.ocean_current_pub = self.create_publisher(Vector3, '/ocean_current', 10)
        self.turbidity_ros_pub = self.create_publisher(
            Float64, '/sic_slam/turbidity_absorption_db_per_m', 10)

        self.create_timer(1.0, self._publish_labels)
        self.create_timer(1.0, self._publish_markers)
        if self.own_ocean_current:
            self.create_timer(0.1, self._publish_ocean_current)
        self._publish_labels()
        self._publish_markers()
        self._publish_turbidity_to_sonar()
        self.get_logger().info(
            f'sim_info_publisher ready: current_speed_mps={self.speed} '
            f'(own_ocean_current={self.own_ocean_current}), '
            f'turbidity_absorption_db_per_m={self.turbidity}, both live-editable '
            f'via /sic_slam/current_speed_set and /sic_slam/turbidity_set, '
            f'{len(GRID_X) * len(GRID_Y) * len(GRID_Z)} current-vector markers '
            f'across the full water volume -> /marker')

    def _speed_set_cb(self, msg: StringMsg):
        try:
            self.speed = float(msg.data)
        except ValueError:
            self.get_logger().warn(f'ignoring unparseable current-speed input: {msg.data!r}')
            return
        self._publish_labels()

    def _turbidity_set_cb(self, msg: StringMsg):
        try:
            self.turbidity = float(msg.data)
        except ValueError:
            self.get_logger().warn(f'ignoring unparseable turbidity input: {msg.data!r}')
            return
        self._publish_labels()
        self._publish_turbidity_to_sonar()

    def _publish_turbidity_to_sonar(self):
        self.turbidity_ros_pub.publish(Float64(data=self.turbidity))

    def _publish_ocean_current(self):
        vx = self.speed * math.cos(self.heading)
        vy = self.speed * math.sin(self.heading)
        self.ocean_current_pub.publish(Vector3(x=vx, y=vy, z=0.0))

    def _publish_labels(self):
        self.speed_label_pub.publish(
            StringMsg(data=f'{self.speed:.2f}'))
        self.turbidity_label_pub.publish(
            StringMsg(data=f'{self.turbidity:.2f}'))

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
                    m.scale.y = ARROW_THICKNESS
                    m.scale.z = ARROW_THICKNESS
                    # Orange, not yellow -- more contrast against the
                    # water's own blue tint and the cave's grey/brown rock
                    # texture. Emissive (self-illuminating, ignores scene
                    # lighting) added on top of ambient/diffuse -- real
                    # request ("more visible"): at ambient_light=0.4 (see
                    # this world's <gui> section), a purely lit material
                    # reads dim/washed out; emissive keeps it bright
                    # regardless of the scene's own light level.
                    m.material.ambient.r = 1.0
                    m.material.ambient.g = 0.5
                    m.material.ambient.b = 0.0
                    m.material.ambient.a = 1.0
                    m.material.diffuse.r = 1.0
                    m.material.diffuse.g = 0.5
                    m.material.diffuse.b = 0.0
                    m.material.diffuse.a = 1.0
                    m.material.emissive.r = 0.6
                    m.material.emissive.g = 0.3
                    m.material.emissive.b = 0.0
                    m.material.emissive.a = 1.0
                    self.marker_pub.publish(m)
                    marker_id += 1


def main():
    rclpy.init()
    node = SimInfoPublisher()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
