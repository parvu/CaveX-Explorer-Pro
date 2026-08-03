#!/usr/bin/env python3
"""
web_telemetry_bridge.py

Gazebo's own GUI (gzclient) and RTAB-Map's Qt viewer (rtabmap_viz) are
disabled for this stack (see gazebo_sim.launch.py / rtabmap_nav.launch.py)
-- visualization instead goes to the CaveX-Explorer-Pro web frontend at
http://localhost:3000, via a plain HTTP POST to /api/telemetry (see
server.ts). No new ROS2 or Python dependency: urllib is stdlib.

Subscribes to the real pose/eval topics and posts a JSON snapshot at a
fixed rate. If the web server isn't up, POSTs just fail silently (logged
once) -- this bridge is a display sink, not part of the control/eval path.
"""

import json
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


class WebTelemetryBridge(Node):
    def __init__(self):
        super().__init__('web_telemetry_bridge')

        self.declare_parameter('web_server_url', 'http://localhost:3000/api/telemetry')
        self.declare_parameter('post_rate_hz', 5.0)
        self.url = self.get_parameter('web_server_url').value

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self._latest = {
            'ground_truth': None,
            'rtabmap_pose': None,
            'sic_slam_pose': None,
            'ate_rmse': None,
        }

        self.create_subscription(Odometry, '/odom', self._gt_cb, best_effort)
        self.create_subscription(Odometry, '/cavex/slam/odom', self._rtabmap_cb, 10)
        self.create_subscription(Odometry, '/sic_slam/odometry', self._sic_slam_cb, 10)
        self.create_subscription(Float64, '/cavex/eval/ate_rmse', self._ate_cb, 10)

        self._warned = False
        rate_hz = self.get_parameter('post_rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self._post)

    def _pose_dict(self, msg: Odometry):
        p = msg.pose.pose.position
        return {'x': p.x, 'y': p.y, 'z': p.z, 'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9}

    def _gt_cb(self, msg):
        self._latest['ground_truth'] = self._pose_dict(msg)

    def _rtabmap_cb(self, msg):
        self._latest['rtabmap_pose'] = self._pose_dict(msg)

    def _sic_slam_cb(self, msg):
        self._latest['sic_slam_pose'] = self._pose_dict(msg)

    def _ate_cb(self, msg):
        self._latest['ate_rmse'] = msg.data

    def _post(self):
        body = json.dumps(self._latest).encode('utf-8')
        req = urllib.request.Request(
            self.url, data=body, headers={'Content-Type': 'application/json'}, method='POST')
        try:
            urllib.request.urlopen(req, timeout=0.5).close()
        except (urllib.error.URLError, OSError) as e:
            if not self._warned:
                self.get_logger().warn(f"Web telemetry server unreachable at {self.url}: {e}")
                self._warned = True


def main(args=None):
    rclpy.init(args=args)
    node = WebTelemetryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
