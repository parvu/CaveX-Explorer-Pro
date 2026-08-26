#!/usr/bin/env python3
"""Bridges cavex_tracked_vehicle_gui's Qt/gz-gui plugins (ManualControl,
ActionButtons) into real vehicle control.

Those plugins only speak gz-transport (StringMsg) -- they don't touch
ROS2 or /cmd_vel directly, same convention as perception branch's
sic_slam_gui/manual_control_node.py pair. This node is the consumer on
the other end of both control topics:

- /cavex/manual_cmd (ManualControl's D-pad + turn-left/right + Manual
  toggle): held-command semantics, same as sic_slam's
  manual_control_node.py -- a direction button sets the current command
  and it stays in effect (re-published at CONTROL_PERIOD_S) until "stop"
  or a different direction is pressed. Drives
  /model/cavex_tracked_blueboat/cmd_vel directly via gz-transport, NOT
  through the ArduPilot cmd_vel_to_ardupilot.py -> SITL -> AP_DDS chain --
  that chain is currently broken in this environment (ArduPilot's rover
  instance never clears its AHRS prearm check, confirmed live
  2026-08-26), so this uses the same direct gz-transport bypass verified
  working that day (real ~0.4 m/s+ measured displacement, vs. the
  ArduPilot path producing zero motion). While "manual_on" is false, this
  publishes NOTHING at all on cmd_vel, so an autonomous script keeps sole
  control of the vehicle.

- /cavex/track_cmd (the Track ActionButtons instance): single-shot
  commands, not held. Republished as the real ROS2 message
  track_retract_control.py already consumes (/cavex/tracks/command
  String) -- this is a manual OVERRIDE of what vehicle_switch_node.py
  normally does automatically based on odometry; running both at once
  is not guarded against.

Real request, 2026-08-26: the Rover lock/unlock ActionButtons panel
this node used to also bridge (/cavex/rov_lock_cmd ->
/cavex/rov_lock/attach|detach) was removed along with its GUI panel --
bluerov2 is now a fixed child link of the boat's own model, so there is
no longer a separate ROV entity to lock or unlock.
"""
import threading
import time

import rclpy
from rclpy.node import Node as RclpyNode
from std_msgs.msg import String

from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.stringmsg_pb2 import StringMsg

LINEAR_MPS = 0.8
ANGULAR_RAD_S = 0.5
CONTROL_PERIOD_S = 0.1

# command -> (linear_x, angular_z)
DRIVE_COMMANDS = {
    "forward": (LINEAR_MPS, 0.0),
    "backward": (-LINEAR_MPS, 0.0),
    "left": (0.0, ANGULAR_RAD_S),
    "right": (0.0, -ANGULAR_RAD_S),
    "turn_left": (0.0, ANGULAR_RAD_S),
    "turn_right": (0.0, -ANGULAR_RAD_S),
    "stop": (0.0, 0.0),
}

_drive_state = {"cmd": "stop", "manual_on": False}


def _manual_cmd_cb(msg: StringMsg):
    data = msg.data
    if data == "manual_on":
        _drive_state["manual_on"] = True
    elif data == "manual_off":
        _drive_state["manual_on"] = False
        _drive_state["cmd"] = "stop"
    elif data in DRIVE_COMMANDS:
        _drive_state["cmd"] = data


class RosBridge(RclpyNode):
    def __init__(self):
        super().__init__('manual_gui_bridge')
        self.track_pub = self.create_publisher(String, '/cavex/tracks/command', 10)

    def track_cmd_cb(self, msg: StringMsg):
        # ActionButtons sends the exact command strings track_retract_control.py
        # already expects ("deployed"/"retracted") -- see the Track plugin's
        # <button1_cmd>/<button2_cmd> config in cavex_world.world.
        if msg.data in ('deployed', 'retracted'):
            self.track_pub.publish(String(data=msg.data))


def main():
    rclpy.init()
    ros_bridge = RosBridge()

    gz_node = GzNode()
    gz_node.subscribe(StringMsg, "/cavex/manual_cmd", _manual_cmd_cb)
    # gz-transport callbacks run on gz's own thread; RosBridge's publishers
    # are plain rclpy calls (thread-safe for publish, unlike subscription
    # callbacks), so wiring these gz-transport callbacks straight to
    # ros_bridge's methods is safe without a queued-connection-style hop.
    gz_node.subscribe(StringMsg, "/cavex/track_cmd", ros_bridge.track_cmd_cb)

    cmd_vel_pub = gz_node.advertise("/model/cavex_tracked_blueboat/cmd_vel", Twist)

    spin_thread = threading.Thread(target=rclpy.spin, args=(ros_bridge,), daemon=True)
    spin_thread.start()

    print("manual_gui_bridge ready: /cavex/manual_cmd -> "
          "/model/cavex_tracked_blueboat/cmd_vel (while Manual is on); "
          "/cavex/track_cmd -> /cavex/tracks/command")

    try:
        while rclpy.ok():
            if _drive_state["manual_on"]:
                linear_x, angular_z = DRIVE_COMMANDS[_drive_state["cmd"]]
                msg = Twist()
                msg.linear.x = linear_x
                msg.angular.z = angular_z
                cmd_vel_pub.publish(msg)
            time.sleep(CONTROL_PERIOD_S)
    except KeyboardInterrupt:
        pass
    finally:
        if _drive_state["manual_on"]:
            cmd_vel_pub.publish(Twist())
        ros_bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
