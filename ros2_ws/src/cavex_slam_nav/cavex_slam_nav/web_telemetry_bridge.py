#!/usr/bin/env python3
"""
web_telemetry_bridge.py

Gazebo's own GUI (gzclient) and RTAB-Map's Qt viewer (rtabmap_viz) are
disabled for this stack (see gazebo_sim.launch.py / rtabmap_nav.launch.py)
-- visualization instead goes to the CaveX-Explorer-Pro web frontend at
http://localhost:3000, via a plain HTTP POST to /api/telemetry (see
server.ts). No new ROS2 or Python dependency: urllib is stdlib.

Two directions over plain HTTP polling (no rosbridge/websocket dependency):
  ROS2 -> web : POST /api/telemetry with pose/lidar/ATE snapshots.
  web -> ROS2 : GET /api/waypoint-goal (pop semantics -- returns and clears
                the pending goal) -> republished as /cavex/nav/goal for
                waypoint_follower.py to drive to. See its docstring for the
                honest scope of what "waypoint navigation" means here (a
                straight-line P-controller, not Nav2).

If the web server isn't up, requests just fail silently (logged once) --
this bridge is a display/command sink, not part of the control/eval path.
"""

import json
import math
import urllib.request
import urllib.error

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from sensor_msgs.msg import LaserScan, JointState
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

# Task 5's real, established joint limits for the tracked vehicle's
# retract joints (see model.sdf.tracked): 0.0 rad = tracks deployed,
# 1.4 rad = tracks fully retracted. Anything meaningfully between the
# two is a track in motion.
TRACK_DEPLOYED_POS = 0.0
TRACK_RETRACTED_POS = 1.4
TRACK_STATE_TOLERANCE = 0.05


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WebTelemetryBridge(Node):
    def __init__(self):
        super().__init__('web_telemetry_bridge')

        self.declare_parameter('web_base_url', 'http://localhost:3000')
        self.declare_parameter('post_rate_hz', 5.0)
        self.base_url = self.get_parameter('web_base_url').value

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
            'lidar_ranges': None,
            'nav_goal': None,
            'tracked_vehicle_gt': None,
            'frontier_count': None,
            'track_state': None,
        }

        self.create_subscription(Odometry, '/odom', self._gt_cb, best_effort)
        self.create_subscription(Odometry, '/cavex/slam/odom', self._rtabmap_cb, 10)
        self.create_subscription(Odometry, '/sic_slam/odometry', self._sic_slam_cb, 10)
        self.create_subscription(Float64, '/cavex/eval/ate_rmse', self._ate_cb, 10)
        self.create_subscription(LaserScan, '/lidar/scan', self._scan_cb, best_effort)
        self.create_subscription(Float64, '/cavex/nav/distance_remaining', self._nav_dist_cb, 10)
        self.create_subscription(PoseStamped, '/cavex/nav/goal', self._nav_goal_cb, 10)
        # Tracked BlueBoat-like vehicle (Tasks 5/12/13): real ground-truth
        # odom, explore_lite's real frontier markers (computed from Nav2's
        # costmap, independent of whether ArduPilot ever arms -- see
        # task-14-report.md), and the two track-retraction joints.
        self.create_subscription(Odometry, '/odom_ground_truth', self._tracked_vehicle_gt_cb, best_effort)
        self.create_subscription(MarkerArray, '/explore/frontiers', self._frontiers_cb, 10)
        self.create_subscription(JointState, '/joint_states', self._track_state_cb, 10)

        self.goal_pub = self.create_publisher(PoseStamped, '/cavex/nav/goal', 10)

        self._warned_post = False
        self._warned_get = False
        rate_hz = self.get_parameter('post_rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self._tick)

    def _pose_dict(self, msg: Odometry):
        p = msg.pose.pose.position
        return {
            'x': p.x, 'y': p.y, 'z': p.z,
            'yaw': yaw_from_quaternion(msg.pose.pose.orientation),
            'stamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        }

    def _gt_cb(self, msg):
        self._latest['ground_truth'] = self._pose_dict(msg)

    def _rtabmap_cb(self, msg):
        self._latest['rtabmap_pose'] = self._pose_dict(msg)

    def _sic_slam_cb(self, msg):
        self._latest['sic_slam_pose'] = self._pose_dict(msg)

    def _ate_cb(self, msg):
        self._latest['ate_rmse'] = msg.data

    def _scan_cb(self, msg: LaserScan):
        # Real gpu_lidar scan, 360 samples (see cavex_robot.urdf.xacro) --
        # matches the frontend's SensorData.lidarRanges shape directly.
        # Out-of-range beams come back as inf (REP-117); json.dumps emits
        # literal `Infinity` for that, which is not valid JSON and Express's
        # strict parser 400s on it -- clamp to range_max instead (a real
        # "no obstacle within max range" reading, not fabricated data).
        self._latest['lidar_ranges'] = [
            r if math.isfinite(r) else msg.range_max for r in msg.ranges
        ]

    def _nav_goal_cb(self, msg: PoseStamped):
        goal = (self._latest['nav_goal'] or {})
        goal['x'] = msg.pose.position.x
        goal['y'] = msg.pose.position.y
        self._latest['nav_goal'] = goal

    def _nav_dist_cb(self, msg: Float64):
        goal = (self._latest['nav_goal'] or {})
        goal['distance_remaining'] = msg.data
        self._latest['nav_goal'] = goal

    def _tracked_vehicle_gt_cb(self, msg: Odometry):
        self._latest['tracked_vehicle_gt'] = self._pose_dict(msg)

    def _frontiers_cb(self, msg: MarkerArray):
        # explore_lite republishes this MarkerArray from its own frontier
        # search over Nav2's costmap -- real frontier count, not derived
        # from anything ArduPilot-specific. Real bug found in final review,
        # fixed here: explore_lite's own visualizeFrontiers()
        # (m-explore-ros2/explore/src/explore.cpp) publishes TWO ADD markers
        # per real frontier (a POINTS marker then a SPHERE marker, same id
        # sequence), plus trailing DELETE markers for the previous cycle's
        # now-unused marker slots -- len(msg.markers) was double-plus-stale
        # the real frontier count, not equal to it. Marker.ADD == 0.
        add_markers = sum(1 for m in msg.markers if m.action == 0)
        self._latest['frontier_count'] = add_markers // 2

    def _track_state_cb(self, msg: JointState):
        try:
            left = msg.position[msg.name.index('left_track_retract_joint')]
            right = msg.position[msg.name.index('right_track_retract_joint')]
        except ValueError:
            return  # this JointState doesn't include the track joints
        avg = (left + right) / 2.0
        if abs(avg - TRACK_DEPLOYED_POS) < TRACK_STATE_TOLERANCE:
            state = 'deployed'
        elif abs(avg - TRACK_RETRACTED_POS) < TRACK_STATE_TOLERANCE:
            state = 'retracted'
        else:
            state = 'moving'
        self._latest['track_state'] = state

    def _tick(self):
        self._post_telemetry()
        self._poll_waypoint_goal()

    def _post_telemetry(self):
        body = json.dumps(self._latest).encode('utf-8')
        req = urllib.request.Request(
            f'{self.base_url}/api/telemetry', data=body,
            headers={'Content-Type': 'application/json'}, method='POST')
        try:
            urllib.request.urlopen(req, timeout=0.5).close()
        except (urllib.error.URLError, OSError) as e:
            if not self._warned_post:
                self.get_logger().warn(f"Web telemetry server unreachable at {self.base_url}: {e}")
                self._warned_post = True

    def _poll_waypoint_goal(self):
        req = urllib.request.Request(f'{self.base_url}/api/waypoint-goal', method='GET')
        try:
            with urllib.request.urlopen(req, timeout=0.5) as resp:
                data = json.loads(resp.read())
        except (urllib.error.URLError, OSError, json.JSONDecodeError) as e:
            if not self._warned_get:
                self.get_logger().warn(f"Could not poll waypoint goal from {self.base_url}: {e}")
                self._warned_get = True
            return

        goal = data.get('goal')
        if not goal:
            return

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(goal['x'])
        msg.pose.position.y = float(goal['y'])
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)
        self.get_logger().info(f"New waypoint goal from web UI: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = WebTelemetryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
