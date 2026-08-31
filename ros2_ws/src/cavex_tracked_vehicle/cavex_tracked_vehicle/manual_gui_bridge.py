#!/usr/bin/env python3
"""Bridges the web viewer's manual-drive commands into real vehicle control.

The web viewer (web_viewer/control_server.py) publishes plain gz-transport
StringMsg on /cavex/manual_cmd and /cavex/track_cmd -- it doesn't touch
ROS2 or /cmd_vel directly. This node is the consumer on the other end of
both control topics. (The custom gz-gui panels that originally published
these -- cavex_tracked_vehicle_gui's ActionButtons/ManualControl -- were
removed 2026-08-31; the topic contract is unchanged.)

- /cavex/manual_cmd (D-pad + speed-up/speed-down + Manual toggle):
  held-command semantics -- a direction command sets the current command
  and it stays in effect (re-published at CONTROL_PERIOD_S) until "stop"
  or a different direction is sent. Drives
  /model/cavex_tracked_blueboat/cmd_vel directly via gz-transport, NOT
  through the ArduPilot cmd_vel_to_ardupilot.py -> SITL -> AP_DDS chain --
  that chain is currently broken in this environment (ArduPilot's rover
  instance never clears its AHRS prearm check, confirmed live
  2026-08-26), so this uses the same direct gz-transport bypass verified
  working that day (real ~0.4 m/s+ measured displacement, vs. the
  ArduPilot path producing zero motion). While "manual_on" is false, this
  publishes NOTHING at all on cmd_vel, so an autonomous script keeps sole
  control of the vehicle.

  Real request, 2026-08-26: the "Manual speed control" panel's
  turn-left/turn-right pair was replaced with speed-up/speed-down (the
  D-pad's own left/right already turn the vehicle). "speed_up"/
  "speed_down" adjust a shared multiplier applied to every direction's
  base linear/angular rate, not a direction of their own -- they don't
  set _drive_state["cmd"].

- /cavex/track_cmd (track deploy/retract): single-shot commands, not
  held. Republished as the real ROS2 message track_retract_control.py
  already consumes (/cavex/tracks/command String) -- this is a manual
  OVERRIDE of what vehicle_switch_node.py normally does automatically
  based on odometry; running both at once is not guarded against.

Real request, 2026-08-26: a /cavex/rov_lock_cmd ->
/cavex/rov_lock/attach|detach control this node used to also bridge was
removed -- bluerov2 is now a fixed child link of the boat's own model,
so there is no longer a separate ROV entity to lock or unlock.
"""
import threading
import time

import rclpy
from rclpy.node import Node as RclpyNode
from std_msgs.msg import String

from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.stringmsg_pb2 import StringMsg

BASE_LINEAR_MPS = 0.8
BASE_ANGULAR_RAD_S = 0.5
CONTROL_PERIOD_S = 0.1

SPEED_SCALE_STEP = 0.25
SPEED_SCALE_MIN = 0.25
SPEED_SCALE_MAX = 3.0

# command -> unit (linear_x, angular_z) direction, scaled by speed_scale
# at publish time (see main()'s own control loop below).
DRIVE_COMMANDS = {
    "forward": (1.0, 0.0),
    "backward": (-1.0, 0.0),
    "left": (0.0, 1.0),
    "right": (0.0, -1.0),
    "stop": (0.0, 0.0),
}

# Real bug found live 2026-08-27: tried giving the TrackedVehicle SDF
# plugin its own separate <topic> (a real attempt at splitting tracks and
# props onto independent gz-transport topics, for genuine mutual
# exclusion via vehicle_switch_node.py's locomotion-mode state machine) --
# root-caused and reverted: <topic> is not actually a functional
# parameter for this plugin's Twist SUBSCRIPTION (confirmed live: track
# velocity read back as exactly 0.0 regardless of command, on either the
# override topic or the original default, while the SAME plugin's
# odometry -- an unrelated publish-only code path -- kept working fine
# the whole time, which is what made this so misleading). Reverting the
# SDF override alone restored normal driving (track velocity matched the
# commanded value exactly). Both tracks and props are back on the single
# shared /model/.../cmd_vel topic; boat_thruster_control.py still gates
# its OWN output off /cavex/locomotion_mode (mode is read there, not
# used to pick a topic here) -- tracks just physically do nothing useful
# while retracted, same "rely on retracted tracks going idle" tradeoff
# this project accepted as the fallback when the topic-split was first
# proposed.
_drive_state = {"cmd": "stop", "manual_on": False, "speed_scale": 1.0}


def _manual_cmd_cb(msg: StringMsg):
    data = msg.data
    if data == "manual_on":
        _drive_state["manual_on"] = True
    elif data == "manual_off":
        _drive_state["manual_on"] = False
        _drive_state["cmd"] = "stop"
    elif data == "speed_up":
        _drive_state["speed_scale"] = min(
            SPEED_SCALE_MAX, _drive_state["speed_scale"] + SPEED_SCALE_STEP)
    elif data == "speed_down":
        _drive_state["speed_scale"] = max(
            SPEED_SCALE_MIN, _drive_state["speed_scale"] - SPEED_SCALE_STEP)
    elif data in DRIVE_COMMANDS:
        _drive_state["cmd"] = data


class RosBridge(RclpyNode):
    def __init__(self):
        super().__init__('manual_gui_bridge')
        self.track_pub = self.create_publisher(String, '/cavex/tracks/command', 10)

    def track_cmd_cb(self, msg: StringMsg):
        # The web viewer sends the exact command strings
        # track_retract_control.py already expects ("deployed"/"retracted").
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
                unit_x, unit_z = DRIVE_COMMANDS[_drive_state["cmd"]]
                scale = _drive_state["speed_scale"]
                msg = Twist()
                msg.linear.x = unit_x * BASE_LINEAR_MPS * scale
                msg.angular.z = unit_z * BASE_ANGULAR_RAD_S * scale
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
