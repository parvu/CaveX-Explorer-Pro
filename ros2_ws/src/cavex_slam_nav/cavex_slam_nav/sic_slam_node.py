#!/usr/bin/env python3
"""
sic_slam_node.py

SIC-SLAM v0 -- a real, minimal prototype of the "current-bias-compensated"
pose correction described in the funding application's WP2-WP3 roadmap
(Section B.2.2). This is NOT the full sonar + Invariant-EKF + GTSAM
factor-graph system described there (the robot in this sim has no sonar,
and there is no GTSAM CurrentFactor here) -- it is a small, real, working
subset: a complementary filter that dead-reckons from /cmd_vel + IMU yaw
rate between RTAB-Map corrections, and estimates a slowly-varying 2D bias
term (the "CurrentFactor" analog: a persistent push the dead-reckoning
model doesn't account for, e.g. a real water current or drift) from the
residual against RTAB-Map's SLAM-corrected pose each time one arrives.

Inputs (all real topics, no synthetic data):
  /cmd_vel            : commanded body velocity (what VelocityControl
                         actually executes -- a legitimate dead-reckoning
                         input, same role as wheel odometry).
  /imu                : real IMU sensor (gz-sim Imu system), used for yaw
                         rate only -- accelerometer double-integration is
                         not used, it drifts too fast to be a real
                         improvement over cmd_vel-based dead reckoning.
  /cavex/slam/odom     : RTAB-Map's real map->base_footprint corrected pose
                         (see slam_pose_publisher.py), used as the periodic
                         correction/measurement.

Output:
  /sic_slam/odometry (nav_msgs/Odometry) -- the fused pose.

Do not present this as the full SIC-SLAM system from the Funding
Application; it is an honest, working prototype subset. See the same
caveat in ate_evaluator_node.py.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quaternion_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (z, w) for a pure-yaw rotation


class SicSlamNode(Node):
    def __init__(self):
        super().__init__('sic_slam_node')

        self.declare_parameter('slam_topic', '/cavex/slam/odom')
        self.declare_parameter('predict_rate_hz', 20.0)
        # Fraction of the SLAM correction trusted outright on each update
        # (vs. kept from the dead-reckoning prediction).
        self.declare_parameter('correction_alpha', 0.9)
        # Learning rate for the slowly-varying current-bias estimate.
        self.declare_parameter('bias_gain', 0.05)

        slam_topic = self.get_parameter('slam_topic').value
        self.alpha = self.get_parameter('correction_alpha').value
        self.bias_gain = self.get_parameter('bias_gain').value

        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
        )

        self.x = self.y = self.theta = 0.0
        self.bias_x = self.bias_y = 0.0  # estimated persistent current push, m/s
        self.last_cmd = Twist()
        self.imu_wz = 0.0
        self._got_slam_fix = False

        self.create_subscription(Twist, '/cmd_vel', self._cmd_cb, 10)
        self.create_subscription(Imu, '/imu', self._imu_cb, best_effort)
        self.create_subscription(Odometry, slam_topic, self._slam_cb, 10)
        self.pub = self.create_publisher(Odometry, '/sic_slam/odometry', 10)

        rate_hz = self.get_parameter('predict_rate_hz').value
        self._last_predict = self.get_clock().now()
        self.timer = self.create_timer(1.0 / rate_hz, self._predict_and_publish)

        self.get_logger().info(
            f"SIC-SLAM v0 started (prototype: cmd_vel+IMU dead reckoning, "
            f"bias-corrected against {slam_topic}). See module docstring for scope."
        )

    def _cmd_cb(self, msg):
        self.last_cmd = msg

    def _imu_cb(self, msg):
        self.imu_wz = msg.angular_velocity.z

    def _slam_cb(self, msg):
        slam_x = msg.pose.pose.position.x
        slam_y = msg.pose.pose.position.y
        slam_theta = yaw_from_quaternion(msg.pose.pose.orientation)

        if not self._got_slam_fix:
            # First fix: snap to it outright, nothing to correct yet.
            self.x, self.y, self.theta = slam_x, slam_y, slam_theta
            self._got_slam_fix = True
            return

        residual_x = slam_x - self.x
        residual_y = slam_y - self.y
        self.bias_x += self.bias_gain * residual_x
        self.bias_y += self.bias_gain * residual_y

        self.x = self.alpha * slam_x + (1.0 - self.alpha) * self.x
        self.y = self.alpha * slam_y + (1.0 - self.alpha) * self.y
        self.theta = slam_theta

    def _predict_and_publish(self):
        if not self._got_slam_fix:
            # Before the first RTAB-Map correction, (x, y, theta) are still
            # at the arbitrary (0,0,0) init value, not the robot's real
            # spawn pose -- publishing here would score as a ~robot-spawn-
            # distance outlier against ground truth for no real reason
            # (this was the actual cause of the large max-error outlier
            # seen in the first SIC-SLAM v0 run, not turn dynamics).
            self._last_predict = self.get_clock().now()
            return

        now = self.get_clock().now()
        dt = (now - self._last_predict).nanoseconds * 1e-9
        self._last_predict = now
        if dt <= 0.0 or dt > 1.0:
            dt = 0.0  # clock jump (e.g. sim reset) -- skip this step, don't corrupt state

        # Yaw rate from IMU when available, else fall back to the commanded rate.
        wz = self.imu_wz if self.imu_wz != 0.0 else self.last_cmd.angular.z
        self.theta += wz * dt

        vx_body = self.last_cmd.linear.x
        vy_body = self.last_cmd.linear.y
        vx_world = vx_body * math.cos(self.theta) - vy_body * math.sin(self.theta)
        vy_world = vx_body * math.sin(self.theta) + vy_body * math.cos(self.theta)

        self.x += (vx_world + self.bias_x) * dt
        self.y += (vy_world + self.bias_y) * dt

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_footprint'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        qz, qw = quaternion_from_yaw(self.theta)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SicSlamNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
