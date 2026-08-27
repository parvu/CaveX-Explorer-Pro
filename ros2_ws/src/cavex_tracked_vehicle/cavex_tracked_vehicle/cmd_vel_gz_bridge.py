#!/usr/bin/env python3
"""
cmd_vel_gz_bridge.py

Real problem, live-diagnosed: autonomous driving (explore_lite/Nav2's
collision_monitor, and dead_end_backtrack_node's own direct publishes) all
land on plain ROS2 /cmd_vel, same as before -- but the only existing
consumer of that topic, cmd_vel_to_ardupilot.py, relays it into ArduPilot's
Rover SITL, which never arms in this environment (stuck repeating "PreArm:
AHRS: not using configured AHRS type" in the SITL console, confirmed live).
So every autonomous /cmd_vel command was being correctly computed and
published, then silently swallowed -- the vehicle never physically moved.

manual_gui_bridge.py already has a real, confirmed-working path around this
exact problem for manual control: it publishes Twist directly via
gz-transport onto /model/cavex_tracked_blueboat/cmd_vel (the tracked
vehicle's own diff-drive plugin topic), bypassing ArduPilot entirely.

This node gives autonomous driving the same bypass. Real request
2026-08-27: tracks and props now live on separate gz-transport topics
(model.sdf.tracked's TrackedVehicle plugin got its own
/model/.../track_cmd_vel -- see that file's own comment), gated by
vehicle_switch_node.py's locomotion-mode state machine
(/cavex/locomotion_mode) -- routes the live command to whichever one is
actually active, same as manual_gui_bridge.py now does, instead of always
writing the single old shared cmd_vel topic. Body-frame passthrough (no
world-frame conversion needed -- unlike cmd_vel_to_ardupilot.py's AHRS
workaround, this never goes through ArduPilot's drifted heading estimate
at all). Yields to manual control the same way manual_gui_bridge yields to
autonomous: tracks /cavex/manual_cmd's manual_on/manual_off state itself
(duplicated few-line flag, not worth a shared service for this) and stays
silent on both gz-transport topics while manual is on, so the two never
publish to the same topic at once.
"""
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist as RosTwist
from std_msgs.msg import String
from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.msgs10.stringmsg_pb2 import StringMsg

TRACK_MODES = ('tracks', 'retracting')

_manual_on = {"value": False}


def _manual_cmd_cb(msg: StringMsg):
    if msg.data == "manual_on":
        _manual_on["value"] = True
    elif msg.data == "manual_off":
        _manual_on["value"] = False


class CmdVelGzBridge(Node):
    def __init__(self, cmd_vel_pub, track_cmd_vel_pub):
        super().__init__('cmd_vel_gz_bridge')
        self._cmd_vel_pub = cmd_vel_pub
        self._track_cmd_vel_pub = track_cmd_vel_pub
        self._mode = 'tracks'
        self.create_subscription(RosTwist, '/cmd_vel', self._cmd_vel_cb, 10)
        self.create_subscription(String, '/cavex/locomotion_mode', self._mode_cb, 10)

    def _mode_cb(self, msg: String):
        self._mode = msg.data

    def _cmd_vel_cb(self, msg: RosTwist):
        if _manual_on["value"]:
            return
        gz_msg = GzTwist()
        gz_msg.linear.x = msg.linear.x
        gz_msg.linear.y = msg.linear.y
        gz_msg.linear.z = msg.linear.z
        gz_msg.angular.x = msg.angular.x
        gz_msg.angular.y = msg.angular.y
        gz_msg.angular.z = msg.angular.z
        if self._mode in TRACK_MODES:
            self._track_cmd_vel_pub.publish(gz_msg)
        else:
            self._cmd_vel_pub.publish(gz_msg)


def main():
    rclpy.init()

    gz_node = GzNode()
    gz_node.subscribe(StringMsg, "/cavex/manual_cmd", _manual_cmd_cb)
    cmd_vel_pub = gz_node.advertise("/model/cavex_tracked_blueboat/cmd_vel", GzTwist)
    track_cmd_vel_pub = gz_node.advertise(
        "/model/cavex_tracked_blueboat/track_cmd_vel", GzTwist)

    node = CmdVelGzBridge(cmd_vel_pub, track_cmd_vel_pub)
    print("cmd_vel_gz_bridge ready: /cmd_vel -> track_cmd_vel or cmd_vel "
          "(whichever /cavex/locomotion_mode says is active) while Manual "
          "is off")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
