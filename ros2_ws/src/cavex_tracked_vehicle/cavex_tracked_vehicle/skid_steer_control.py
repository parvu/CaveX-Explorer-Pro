#!/usr/bin/env python3
"""
skid_steer_control.py

Land locomotion for cavex_tracked_blueboat. Replaces the gz-sim
TrackedVehicle / TrackController plugin drive: those systems steer by
setting track-link velocities and relying on anisotropic surface
friction, a path that works under dartsim but produces NO motion under
bullet-featherstone (confirmed live 2026-08-28 -- track_cmd_vel stayed
silent, the vehicle sat still). The world switched to bullet-featherstone
for real mesh collision + RTF; this node is the trade-off fix.

It is a plain body-frame speed / yaw-rate controller: reads the shared
/model/cavex_tracked_blueboat/cmd_vel (gz-transport Twist, same topic
manual_gui_bridge.py / cmd_vel_gz_bridge.py publish and
boat_thruster_control.py also taps) and drives base_link with a
force + yaw torque via /world/cavex_world/wrench, only while
/cavex/locomotion_mode is 'tracks' or 'retracting' (mirrors
boat_thruster_control.py's own ('props','deploying') gate, so exactly one
of the two ever drives at a time).

Wrench mechanics match boat_buoyancy_control.py: the plain /world/.../wrench
topic applies a message for ONE physics step, so the control tick only
COMPUTES the wrench and a ~physics-rate timer re-publishes it (the
/persistent topic would ACCUMULATE per message and run away).
"""
import math
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity

ACTIVE_MODES = ('tracks', 'retracting')

KP_V = 800.0        # N per (m/s) of forward-speed error
KP_W = 130.0        # N*m per (rad/s) of yaw-rate error
MAX_FORCE_N = 950.0
MAX_TORQUE_NM = 180.0

LINK = 'cavex_tracked_blueboat::base_link'

_lock = threading.Lock()
_state = {"mode": "tracks", "cmd_v": 0.0, "cmd_w": 0.0}


def yaw_from_quat(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _cmd_vel_cb(msg: Twist):
    with _lock:
        _state["cmd_v"] = msg.linear.x
        _state["cmd_w"] = msg.angular.z


class SkidSteerControl(Node):
    def __init__(self, gz_pub):
        super().__init__('skid_steer_control')
        self.gz_pub = gz_pub
        self._wrench = None
        self._prev = None  # (t, x, y, yaw)
        self.create_subscription(String, '/cavex/locomotion_mode', self._mode_cb, 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self.create_timer(1.0 / 250.0, self._republish)
        self.get_logger().info(
            f"skid_steer_control ready: cmd_vel -> base_link force/torque "
            f"only while /cavex/locomotion_mode in {ACTIVE_MODES}.")

    def _mode_cb(self, msg: String):
        with _lock:
            _state["mode"] = msg.data

    def _republish(self):
        if self._wrench is not None:
            self.gz_pub.publish(self._wrench)

    def _odom_cb(self, msg: Odometry):
        t = self.get_clock().now().nanoseconds * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)

        with _lock:
            mode = _state["mode"]
            cmd_v, cmd_w = _state["cmd_v"], _state["cmd_w"]

        if mode not in ACTIVE_MODES:
            self._prev = None
            self._set_wrench(0.0, 0.0, 0.0)
            return

        vx = vy = yaw_rate = 0.0
        if self._prev is not None:
            dt = t - self._prev[0]
            if dt > 1e-3:
                vx = (x - self._prev[1]) / dt
                vy = (y - self._prev[2]) / dt
                yaw_rate = wrap(yaw - self._prev[3]) / dt
        self._prev = (t, x, y, yaw)

        fwd = vx * math.cos(yaw) + vy * math.sin(yaw)   # current forward speed
        f_body = max(-MAX_FORCE_N, min(MAX_FORCE_N, KP_V * (cmd_v - fwd)))
        tz = max(-MAX_TORQUE_NM, min(MAX_TORQUE_NM, KP_W * (cmd_w - yaw_rate)))

        self._set_wrench(f_body * math.cos(yaw), f_body * math.sin(yaw), tz)

    def _set_wrench(self, fx, fy, tz):
        w = EntityWrench()
        w.entity.name = LINK
        w.entity.type = Entity.LINK
        w.wrench.force.x = fx
        w.wrench.force.y = fy
        w.wrench.torque.z = tz
        self._wrench = w  # applied every step by _republish()


def main(args=None):
    rclpy.init(args=args)
    gz_node = GzNode()
    gz_node.subscribe(Twist, '/model/cavex_tracked_blueboat/cmd_vel', _cmd_vel_cb)
    gz_pub = gz_node.advertise('/world/cavex_world/wrench', EntityWrench)
    node = SkidSteerControl(gz_pub)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
