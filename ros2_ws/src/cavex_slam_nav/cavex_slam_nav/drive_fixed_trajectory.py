#!/usr/bin/env python3
"""
drive_fixed_trajectory.py

Runs N identical trajectories (forward, turn, forward, stop, settle,
finish_run) gated on ROS sim time rather than wall-clock sleep(), so
repeated ATE evaluation runs are directly comparable to each other
regardless of the simulation's real-time-factor. A wall-clock sleep()
loop gives each phase a fixed number of *wall* seconds, but how much sim
time (and therefore how far the robot actually drives) elapses in that
window varies with real-time-factor -- which fluctuates with system load.
Gating on get_clock().now() (backed by /clock, requires use_sim_time:=true)
fixes the sim-time length of each phase instead.

Usage:
    ros2 run cavex_slam_nav drive_fixed_trajectory.py --ros-args \
        -p use_sim_time:=true -p num_runs:=10
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty


class FixedTrajectoryDriver(Node):
    def __init__(self):
        super().__init__('drive_fixed_trajectory')

        self.declare_parameter('num_runs', 10)
        self.declare_parameter('forward_speed', 0.4)
        self.declare_parameter('turn_rate', 0.3)
        self.declare_parameter('forward1_sim_s', 2.0)
        self.declare_parameter('turn_sim_s', 1.5)
        self.declare_parameter('forward2_sim_s', 2.0)
        self.declare_parameter('settle_sim_s', 2.0)
        self.declare_parameter('between_runs_sim_s', 1.0)
        self.declare_parameter('cmd_rate_hz', 10.0)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.finish_pub = self.create_publisher(Empty, '/cavex/eval/finish_run', 10)

    def _wait_for_clock(self):
        # use_sim_time nodes report time 0 until the first /clock message
        # arrives -- spin until it does, or the sim-time gating below would
        # measure phase length from a bogus zero point.
        self.get_logger().info("Waiting for /clock (use_sim_time)...")
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _run_phase(self, twist: Twist, sim_duration_s: float):
        rate_hz = self.get_parameter('cmd_rate_hz').value
        period = 1.0 / rate_hz
        end_ns = self.get_clock().now().nanoseconds + int(sim_duration_s * 1e9)
        while rclpy.ok() and self.get_clock().now().nanoseconds < end_ns:
            self.cmd_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=period)

    def run(self):
        self._wait_for_clock()
        n = self.get_parameter('num_runs').value
        v = self.get_parameter('forward_speed').value
        w = self.get_parameter('turn_rate').value

        fwd = Twist()
        fwd.linear.x = v
        turn = Twist()
        turn.angular.z = w
        stop = Twist()

        for i in range(1, n + 1):
            self.get_logger().info(f"=== fixed trajectory run {i}/{n} (sim-time gated) ===")
            self._run_phase(fwd, self.get_parameter('forward1_sim_s').value)
            self._run_phase(turn, self.get_parameter('turn_sim_s').value)
            self._run_phase(fwd, self.get_parameter('forward2_sim_s').value)
            self._run_phase(stop, self.get_parameter('settle_sim_s').value)
            self.finish_pub.publish(Empty())
            self.get_logger().info(f"Run {i}: finish_run sent.")
            self._run_phase(stop, self.get_parameter('between_runs_sim_s').value)

        self.get_logger().info("All runs complete.")


def main(args=None):
    rclpy.init(args=args)
    node = FixedTrajectoryDriver()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
