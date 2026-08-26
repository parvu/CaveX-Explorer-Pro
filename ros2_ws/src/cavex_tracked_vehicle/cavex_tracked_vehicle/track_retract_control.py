#!/usr/bin/env python3
"""
track_retract_control.py

Commands both track_retract_joints together via track_retract_controller's
JointTrajectory topic, given a simple "deployed"/"retracted" string command.

Real request 2026-08-26: gated to only actuate while the boat is in the
water region AND actually floating (boat_buoyancy_control.py's lift has
brought it up near its target float height, not still on the cave floor
mid-transition) -- retracting/deploying makes sense as a water-vs-land
locomotion switch, not something to allow mid-drive on dry land or while
still sinking/rising through the water column right after crossing the
boundary. Reads the same live /odom_ground_truth ground truth every other
control node in this package uses (vehicle_switch_node.py,
boat_buoyancy_control.py) rather than tracking its own state.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

# Real request 2026-08-26: kept a small margin off the joints' true SDF hard
# stops (0.0/1.4 in blueboat/model.sdf.tracked) -- commanding exactly to the
# hard limit reproduced a real, live-confirmed gz-sim/DART bug where the
# joint locks up entirely once driven flush against its limit and then
# ignores every later command. See the joint's own SDF comment for the full
# story (widening the limit and adding damping were both tried and made it
# worse); staying just short of the stop avoids the lock outright.
DEPLOYED = 0.05
RETRACTED = 1.35

# Same constant vehicle_switch_node.py and boat_buoyancy_control.py use.
WATER_BOUNDARY_X = 15.0
# boat_buoyancy_control.py's own TARGET_FLOAT_Z is 8.15 (surface at 7.9,
# floor at 5.9, floating above the surface per real request 2026-08-26);
# this margin below it is "close enough to floating", not "exactly at the
# target" (the float controller has some live-verified residual bob
# around its target).
FLOAT_Z_MIN = 7.5


class TrackRetractControl(Node):
    def __init__(self):
        super().__init__('track_retract_control')
        self.pub = self.create_publisher(
            JointTrajectory, '/track_retract_controller/joint_trajectory', 10)
        self.create_subscription(String, '/cavex/tracks/command', self._cb, 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self._x = None
        self._z = None

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._z = msg.pose.pose.position.z

    def _cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'deployed':
            target = DEPLOYED
        elif cmd == 'retracted':
            target = RETRACTED
        else:
            self.get_logger().warn(f"Unknown track command: {msg.data!r} (expected 'deployed' or 'retracted')")
            return

        if self._x is None:
            self.get_logger().warn(
                "Ignoring track command: no /odom_ground_truth received yet, "
                "can't confirm the boat is in the water and floating.")
            return
        if self._x <= WATER_BOUNDARY_X or self._z < FLOAT_Z_MIN:
            self.get_logger().warn(
                f"Ignoring track command {cmd!r}: boat isn't in the water and "
                f"floating (x={self._x:.2f}, z={self._z:.2f}; need x>"
                f"{WATER_BOUNDARY_X} and z>={FLOAT_Z_MIN}).")
            return

        traj = JointTrajectory()
        traj.joint_names = ['left_track_retract_joint', 'right_track_retract_joint']
        point = JointTrajectoryPoint()
        point.positions = [target, target]
        point.time_from_start = Duration(sec=2, nanosec=0)
        traj.points = [point]
        self.pub.publish(traj)
        self.get_logger().info(f"Tracks commanded {cmd} (joint target {target} rad).")


def main(args=None):
    rclpy.init(args=args)
    node = TrackRetractControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
