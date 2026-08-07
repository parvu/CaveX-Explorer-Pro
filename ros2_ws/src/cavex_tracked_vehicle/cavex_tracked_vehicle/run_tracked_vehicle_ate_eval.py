#!/usr/bin/env python3
"""
run_tracked_vehicle_ate_eval.py

Fixed sim-time-budget ATE evaluation for autonomous exploration runs.
No cmd_vel sent -- explore_lite (via ArduPilot) is already driving the
vehicle. Gates finish_run on a fixed sim-time budget so repeated runs
are comparable by time-budget, even though the actual path differs run
to run (expected, exploration is autonomous).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class TrackedVehicleAteEvalRunner(Node):
    def __init__(self):
        super().__init__('run_tracked_vehicle_ate_eval')
        self.declare_parameter('num_runs', 10)
        self.declare_parameter('budget_sim_s', 60.0)
        self.finish_pub = self.create_publisher(Empty, '/cavex/eval/finish_run', 10)

    def _wait_for_clock(self):
        while rclpy.ok() and self.get_clock().now().nanoseconds == 0:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run(self):
        self._wait_for_clock()
        n = self.get_parameter('num_runs').value
        budget = self.get_parameter('budget_sim_s').value
        for i in range(1, n + 1):
            self.get_logger().info(f"=== exploration run {i}/{n}: {budget}s sim-time budget ===")
            end_ns = self.get_clock().now().nanoseconds + int(budget * 1e9)
            while rclpy.ok() and self.get_clock().now().nanoseconds < end_ns:
                rclpy.spin_once(self, timeout_sec=0.2)
            self.finish_pub.publish(Empty())
            self.get_logger().info(f"Run {i}: finish_run sent.")
        self.get_logger().info("All runs complete.")


def main(args=None):
    rclpy.init(args=args)
    node = TrackedVehicleAteEvalRunner()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
