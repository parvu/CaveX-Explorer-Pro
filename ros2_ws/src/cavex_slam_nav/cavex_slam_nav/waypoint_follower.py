#!/usr/bin/env python3
"""
waypoint_follower.py

Real, minimal waypoint navigation: a straight-line proportional (P)
controller that turns to face a goal, then drives to it, publishing
/cmd_vel. This is genuinely running and genuinely driving the robot -- but
it is NOT Nav2 (no costmap, no global/local planner, no obstacle
avoidance). Label it "waypoint follower", not "Nav2 navigation", in any
report. See package.xml's nav2_bringup dependency -- swapping this out for
real Nav2 (costmap from /lidar/scan, DWB/RPP local planner) is the natural
upgrade path if obstacle-aware navigation is needed.

Closes the control loop on /cavex/slam/odom (RTAB-Map's real corrected
pose) rather than /odom (ground truth) -- a real robot only has its own
state estimate to steer by, and using ground truth here would silently
hide SLAM errors from the navigation behavior.

Goal input: /cavex/nav/goal (geometry_msgs/PoseStamped), e.g. from the web
UI via web_telemetry_bridge.py, or `ros2 topic pub` directly.
Status output: /cavex/nav/distance_remaining (std_msgs/Float64).
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.declare_parameter('pose_topic', '/cavex/slam/odom')
        self.declare_parameter('goal_tolerance_m', 0.3)
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 0.6)
        self.declare_parameter('kp_linear', 0.6)
        self.declare_parameter('kp_angular', 1.5)
        self.declare_parameter('control_rate_hz', 10.0)

        pose_topic = self.get_parameter('pose_topic').value
        self.tolerance = self.get_parameter('goal_tolerance_m').value
        self.max_v = self.get_parameter('max_linear_speed').value
        self.max_w = self.get_parameter('max_angular_speed').value
        self.kp_v = self.get_parameter('kp_linear').value
        self.kp_w = self.get_parameter('kp_angular').value

        self.x = self.y = self.theta = None  # unknown until first pose arrives
        self.goal = None  # (x, y) or None

        self.create_subscription(Odometry, pose_topic, self._pose_cb, 10)
        self.create_subscription(PoseStamped, '/cavex/nav/goal', self._goal_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dist_pub = self.create_publisher(Float64, '/cavex/nav/distance_remaining', 10)

        rate_hz = self.get_parameter('control_rate_hz').value
        self.timer = self.create_timer(1.0 / rate_hz, self._control_tick)

        self.get_logger().info(
            f"waypoint_follower started (P-controller, no obstacle avoidance), "
            f"closing the loop on {pose_topic}. Publish geometry_msgs/PoseStamped "
            f"to /cavex/nav/goal to send it somewhere."
        )

    def _pose_cb(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.theta = yaw_from_quaternion(msg.pose.pose.orientation)

    def _goal_cb(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f"New goal: ({self.goal[0]:.2f}, {self.goal[1]:.2f})")

    def _control_tick(self):
        if self.goal is None or self.x is None:
            return

        dx = self.goal[0] - self.x
        dy = self.goal[1] - self.y
        distance = math.hypot(dx, dy)
        self.dist_pub.publish(Float64(data=distance))

        if distance < self.tolerance:
            self.cmd_pub.publish(Twist())  # stop
            self.get_logger().info("Goal reached.")
            self.goal = None
            return

        heading_error = wrap_angle(math.atan2(dy, dx) - self.theta)

        cmd = Twist()
        cmd.angular.z = max(-self.max_w, min(self.max_w, self.kp_w * heading_error))
        # Slow down the closer we are to facing the wrong way, so it turns
        # in place first instead of arcing wide -- ponytail: no path
        # planning or obstacle avoidance, straight-line only; upgrade to
        # Nav2 (already an unused dependency) if the cave layout needs it.
        alignment = max(0.0, math.cos(heading_error))
        cmd.linear.x = max(-self.max_v, min(self.max_v, self.kp_v * distance * alignment))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
