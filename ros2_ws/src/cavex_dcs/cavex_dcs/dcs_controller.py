#!/usr/bin/env python3
# Copyright 2026 CaveX Explorer Pro
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""
dcs_controller.

Drift/Current Suppression: sits between /cmd_vel_rov_desired and
/cmd_vel_rov, which cmd_vel_to_ardusub.py already consumes unmodified.
Feed-forward compensates the estimated current; PI feedback catches what
feed-forward misses. Publishes /dcs/status on saturation or low confidence.

"Track" here means the instantaneous commanded heading direction, not a
stored path -- the BlueROV2 is manually teleoperated with no waypoint/path
system (see the spec's non-goals), so cross-track error is measured against
the heading implied by the current /cmd_vel_rov_desired command rather than
a persisted line. This still nulls out lateral drift for a continuously
driven vehicle; a persistent-path version is future work if waypoint
navigation is added for the ROV.
"""
import math

from cavex_dcs.dcs_math import (
    confidence_from_covariance,
    cross_track_error,
    feed_forward,
    PiController,
    rotate_world_to_body,
    saturate,
)
from geometry_msgs.msg import Twist, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DcsController(Node):
    """Rotate/compensate a desired ROV velocity for the estimated current."""

    def __init__(self):
        super().__init__('dcs_controller')
        self._max_speed_mps = self.declare_parameter('max_speed_mps', 1.0).value
        self._cov_threshold = self.declare_parameter('cov_confidence_threshold', 0.1).value
        kp = self.declare_parameter('pi_kp', 0.6).value
        ki = self.declare_parameter('pi_ki', 0.1).value
        i_max = self.declare_parameter('pi_i_max', 0.5).value
        self._pi = PiController(kp=kp, ki=ki, i_max=i_max)

        self._current_vx = 0.0
        self._current_vy = 0.0
        self._current_cov_trace = self._cov_threshold  # start unconfident
        self._yaw = 0.0
        self._actual_body_vx = 0.0
        self._actual_body_vy = 0.0
        self._last_time = None

        self.create_subscription(Twist, '/cmd_vel_rov_desired', self._desired_cb, 10)
        self.create_subscription(
            TwistWithCovarianceStamped, '/sic_slam/current_estimate', self._current_cb, 10)
        self.create_subscription(Odometry, '/sic_slam/odometry', self._odom_cb, 10)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_rov', 10)
        self._status_pub = self.create_publisher(String, '/dcs/status', 10)

        self.get_logger().info(
            'dcs_controller ready: /cmd_vel_rov_desired -> /cmd_vel_rov, '
            'compensating for /sic_slam/current_estimate.')

    def _current_cb(self, msg: TwistWithCovarianceStamped):
        self._current_vx = msg.twist.twist.linear.x
        self._current_vy = msg.twist.twist.linear.y
        self._current_cov_trace = (
            msg.twist.covariance[0] + msg.twist.covariance[7] + msg.twist.covariance[14])

    def _odom_cb(self, msg: Odometry):
        q = msg.pose.pose.orientation
        # yaw from quaternion (z-axis rotation only, sufficient for a
        # roughly-level ROV driving through the water column)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)
        world_vx = msg.twist.twist.linear.x
        world_vy = msg.twist.twist.linear.y
        self._actual_body_vx, self._actual_body_vy = rotate_world_to_body(
            world_vx, world_vy, self._yaw)

    def _desired_cb(self, msg: Twist):
        now = self.get_clock().now()
        dt = 0.05
        if self._last_time is not None:
            dt = max(1e-3, (now - self._last_time).nanoseconds * 1e-9)
        self._last_time = now

        confidence = confidence_from_covariance(self._current_cov_trace, self._cov_threshold)
        ff_vx, ff_vy = feed_forward(
            desired_body_vx=msg.linear.x, desired_body_vy=msg.linear.y,
            current_world_vx=self._current_vx, current_world_vy=self._current_vy,
            yaw=self._yaw, confidence=confidence)

        err = cross_track_error(
            actual_body_vx=self._actual_body_vx, actual_body_vy=self._actual_body_vy,
            desired_body_vx=msg.linear.x, desired_body_vy=msg.linear.y)
        correction = self._pi.update(err, dt)
        heading_norm = math.hypot(msg.linear.x, msg.linear.y)
        if heading_norm > 1e-6:
            corr_vx = -msg.linear.y / heading_norm * correction
            corr_vy = msg.linear.x / heading_norm * correction
        else:
            corr_vx = corr_vy = 0.0

        cmd_vx = ff_vx - corr_vx
        cmd_vy = ff_vy - corr_vy
        cmd_vx, cmd_vy, shortfall = saturate(cmd_vx, cmd_vy, self._max_speed_mps)

        out = Twist()
        out.linear.x = cmd_vx
        out.linear.y = cmd_vy
        out.linear.z = msg.linear.z
        out.angular = msg.angular
        self._cmd_pub.publish(out)

        status = String()
        if shortfall > 1e-6:
            status.data = f'SATURATED shortfall={shortfall:.2f} m/s conf={confidence:.2f}'
        else:
            status.data = f'OK conf={confidence:.2f}'
        self._status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = DcsController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
