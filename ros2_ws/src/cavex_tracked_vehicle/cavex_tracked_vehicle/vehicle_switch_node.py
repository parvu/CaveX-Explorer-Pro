#!/usr/bin/env python3
"""
vehicle_switch_node.py

Watches the tracked vehicle's real ground-truth pose (/odom_ground_truth,
Task 13's real gz-transport-sourced Odometry -- see
tracked_vehicle_ground_truth_odom.py) and, on crossing the water boundary,
triggers the handoff: retract the tracks and pay the motorized tether out
to let the BlueROV2 operate independently in the water region (see
motorized_tether_control.py for the real force-based constraint this now
commands via /cavex/tether/payout_length_cmd).

Scope, explicitly narrower than the original plan: no ArduSub SITL
launch/control is started or required here -- that's the same real,
separately-documented arm-rejection limitation already found live this
session (see README.md's "BlueROV2 / ArduSub" section), and is out of
scope for the mechanical tether-payout handoff this node owns. Bringing
ArduSub up (or arming/controlling the BlueROV2) is left as a separate step
for whoever operates it.

Real change from the old design: this is now a BIDIRECTIONAL handoff.
The old gz-sim-detachable-joint-system carry was a one-way rigid
attach/detach (its own /attach topic required the same relative pose the
joint was created at, which a released, independently-drifting BlueROV2
wouldn't be back at) -- the tether has no such limitation, since it's a
continuous force constraint rather than a joint that has to be
re-created. Crossing back out of the water re-deploys the tracks and
reels the tether back in.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, String

# The real, live-verified x edge of the flooded chamber -- NOT the plan's
# original guess of 10.0. Derived from cave_floor_patch's own real,
# vertex-confirmed floor coverage (cavex_world.world), the same real
# geometry the water_surface region (x [15,65]) was re-anchored to. See
# that file's own comments for the full derivation.
WATER_BOUNDARY_X = 15.0

# Matches motorized_tether_control.py's own MIN/MAX_PAYOUT_LENGTH bounds --
# docked short (held still by tether_frame_link's real cradle, see that
# link's comment in model.sdf.tracked) while dry, paid out to operate once
# in the water region.
TETHER_LENGTH_DOCKED = 0.1
TETHER_LENGTH_DEPLOYED = 8.0


class VehicleSwitchNode(Node):
    def __init__(self):
        super().__init__('vehicle_switch_node')
        self._in_water = False
        self.track_cmd_pub = self.create_publisher(String, '/cavex/tracks/command', 10)
        self.tether_length_pub = self.create_publisher(Float64, '/cavex/tether/payout_length_cmd', 10)
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)
        self.get_logger().info(
            f"vehicle_switch_node ready: watching /odom_ground_truth, will retract "
            f"tracks + pay the tether out to {TETHER_LENGTH_DEPLOYED}m at "
            f"x >= {WATER_BOUNDARY_X} (and reverse on the way back out).")

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        now_in_water = x >= WATER_BOUNDARY_X
        if now_in_water == self._in_water:
            return
        self._in_water = now_in_water
        if now_in_water:
            self.get_logger().info(
                f"Crossing into water at x={x:.2f} -- retracting tracks and "
                f"paying the tether out to {TETHER_LENGTH_DEPLOYED}m.")
            self.track_cmd_pub.publish(String(data='retracted'))
            self.tether_length_pub.publish(Float64(data=TETHER_LENGTH_DEPLOYED))
        else:
            self.get_logger().info(
                f"Crossing back out of water at x={x:.2f} -- reeling the "
                f"tether in to {TETHER_LENGTH_DOCKED}m and redeploying tracks.")
            self.tether_length_pub.publish(Float64(data=TETHER_LENGTH_DOCKED))
            self.track_cmd_pub.publish(String(data='deployed'))


def main(args=None):
    rclpy.init(args=args)
    node = VehicleSwitchNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
