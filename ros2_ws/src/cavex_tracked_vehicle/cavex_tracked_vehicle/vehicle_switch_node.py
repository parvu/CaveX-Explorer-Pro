#!/usr/bin/env python3
"""
vehicle_switch_node.py

Watches the tracked vehicle's real ground-truth pose (/odom_ground_truth,
Task 13's real gz-transport-sourced Odometry -- see
tracked_vehicle_ground_truth_odom.py) and, on crossing into the water
region, triggers the water-boundary handoff: retract the tracks and
release the carried BlueROV2 (model.sdf.tracked's own DetachableJoint
plugin -- see that file's comment for the real, working gz-sim mechanism
this uses, confirmed against the actual upstream reference example at
/usr/share/gz/gz-sim8/worlds/detachable_joint.sdf).

Scope, explicitly narrower than the original plan: no ArduSub SITL
launch/control is started or required here -- that's the same real,
separately-documented arm-rejection limitation already found live this
session (see README.md's "BlueROV2 / ArduSub" section), and is out of
scope for the mechanical carry/release handoff this node owns. Bringing
ArduSub up (or arming/controlling the released BlueROV2) is left as a
separate step for whoever operates the released vehicle.

Real, confirmed limitation: this is a ONE-WAY handoff. DetachableJoint
has no built-in re-attach mechanism usable here (its own /attach topic
requires the same relative pose the joint was created at, which a
released, independently-drifting BlueROV2 won't be back at) -- crossing
back out of the water does not re-dock the ROV or re-deploy the tracks
automatically. This matches the SDD ledger's own documented deferral of
bidirectional handoff.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Empty, String

# The real, live-verified x edge of the flooded chamber -- NOT the plan's
# original guess of 10.0. Derived from cave_floor_patch's own real,
# vertex-confirmed floor coverage (cavex_world.world), the same real
# geometry the water_surface region (x [15,65]) was re-anchored to. See
# that file's own comments for the full derivation.
WATER_BOUNDARY_X = 15.0


class VehicleSwitchNode(Node):
    def __init__(self):
        super().__init__('vehicle_switch_node')
        self._released = False
        self.track_cmd_pub = self.create_publisher(String, '/cavex/tracks/command', 10)
        self.detach_pub = self.create_publisher(Empty, '/cavex/rov_release/detach', 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self.get_logger().info(
            f"vehicle_switch_node ready: watching /odom_ground_truth, will retract "
            f"tracks + release the carried BlueROV2 at x >= {WATER_BOUNDARY_X}.")

    def _odom_cb(self, msg: Odometry):
        if self._released:
            return
        x = msg.pose.pose.position.x
        if x >= WATER_BOUNDARY_X:
            self.get_logger().info(
                f"Crossing into water at x={x:.2f} -- retracting tracks and "
                f"releasing the carried BlueROV2.")
            self.track_cmd_pub.publish(String(data='retracted'))
            self.detach_pub.publish(Empty())
            self._released = True


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSwitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
