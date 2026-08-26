#!/usr/bin/env python3
"""
track_retract_control.py

Commands both track_retract_joints together via track_retract_controller's
JointTrajectory topic, given a simple "deployed"/"retracted" string command.
No automatic trigger logic (e.g. water detection) -- manual/topic-commanded
only this phase, per the design spec's explicit non-goal.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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


class TrackRetractControl(Node):
    def __init__(self):
        super().__init__('track_retract_control')
        self.pub = self.create_publisher(
            JointTrajectory, '/track_retract_controller/joint_trajectory', 10)
        self.create_subscription(String, '/cavex/tracks/command', self._cb, 10)

    def _cb(self, msg: String):
        cmd = msg.data.strip().lower()
        if cmd == 'deployed':
            target = DEPLOYED
        elif cmd == 'retracted':
            target = RETRACTED
        else:
            self.get_logger().warn(f"Unknown track command: {msg.data!r} (expected 'deployed' or 'retracted')")
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
