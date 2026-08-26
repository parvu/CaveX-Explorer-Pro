#!/usr/bin/env python3
"""
vehicle_switch_node.py

Watches the tracked vehicle's real ground-truth pose (/odom_ground_truth,
Task 13's real gz-transport-sourced Odometry -- see
tracked_vehicle_ground_truth_odom.py) and retracts/redeploys the tracks
on the dry-section/water-boundary crossing.

Real request, 2026-08-26 ("make bluerov2 static in reference with
blueboat not the world"): the ROV lock/unlock and motorized-tether
logic this node used to also own were removed. bluerov2 is now a fixed
child link of the boat's own model (model.sdf.tracked's bluerov2_link)
instead of a separately spawned entity held on by a DetachableJoint +
tether -- there is no longer a separate ROV to lock, unlock, release,
or tether. See perception branch for the full, functional,
independently-swimming, tethered BlueROV2.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String

# Matches water_surface's own x-start in cavex_world.world (re-derived by
# real mesh-vertex inspection, not the plan's original guess of 10.0 --
# see that file's own comments for the full derivation and history). Was
# briefly moved to 0.0 when the water region got extended to x=0; reverted
# back to 15.0 along with that region after live-testing confirmed x<15 is
# a real void in the cave mesh, not just unverified.
WATER_BOUNDARY_X = 15.0


class VehicleSwitchNode(Node):
    def __init__(self):
        super().__init__('vehicle_switch_node')
        self._in_water = False

        self.track_cmd_pub = self.create_publisher(String, '/cavex/tracks/command', 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)

        self.get_logger().info(
            f"vehicle_switch_node ready: will retract tracks at "
            f"x >= {WATER_BOUNDARY_X} (and redeploy on the way back out).")

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x

        now_in_water = x >= WATER_BOUNDARY_X
        if now_in_water != self._in_water:
            self._in_water = now_in_water
            if now_in_water:
                self.get_logger().info(
                    f"Crossing into water at x={x:.2f} -- retracting tracks.")
                self.track_cmd_pub.publish(String(data='retracted'))
            else:
                self.get_logger().info(
                    f"Crossing back out of water at x={x:.2f} -- redeploying tracks.")
                self.track_cmd_pub.publish(String(data='deployed'))


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSwitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
