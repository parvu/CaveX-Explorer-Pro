#!/usr/bin/env python3
"""
boat_thruster_control.py

Real request 2026-08-26: real water propulsion for the boat's twin motor/
prop assemblies (model.sdf.tracked's motor_port_joint/motor_stbd_joint,
driven by two gz-sim-thruster-system plugin instances). Mixes the
vehicle's real drive command into differential port/stbd thrust and
publishes it on each thruster's real cmd_thrust topic.

Taps /model/cavex_tracked_blueboat/cmd_vel directly (gz-transport, same
pattern as manual_gui_bridge.py's own publisher) rather than the ROS 2
/track_cmd_vel topic track_cmd_vel_bridge.py publishes: manual driving
(manual_gui_bridge.py) bypasses /track_cmd_vel entirely and writes this
gz-transport topic directly, so it's the one point both the autonomous
(track_cmd_vel_bridge.py) and manual paths actually converge on before
reaching the TrackedVehicle plugin -- same topic that already drives the
tracks, so the props and tracks always agree on where the vehicle is
being told to go.

Gated the same way track_retract_control.py now gates track commands:
only fires while the boat is in the water region AND actually floating
(boat_buoyancy_control.py's lift has it up near its target height, not
still on the cave floor) -- driving on dry land is the tracks' job, and
spinning the props against nothing there would be nonsensical.
"""
import threading

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.double_pb2 import Double

WATER_BOUNDARY_X = 15.0
FLOAT_Z_MIN = 7.5  # same threshold track_retract_control.py uses

# Real motor y-offset from model.sdf.tracked (motor_port_joint/motor_stbd_joint
# poses, +-0.295).
HALF_SEPARATION = 0.295

THRUST_GAIN_LINEAR = 300.0   # N per (m/s) of commanded linear.x
THRUST_GAIN_ANGULAR = 80.0   # N per (rad/s) of commanded angular.z, applied
                              # differentially (see mix below)
MAX_THRUST_N = 400.0

_state = {"x": None, "z": None, "linear_x": 0.0, "angular_z": 0.0}
_lock = threading.Lock()


def _cmd_vel_cb(msg: Twist):
    with _lock:
        _state["linear_x"] = msg.linear.x
        _state["angular_z"] = msg.angular.z


class OdomWatcher(Node):
    def __init__(self):
        super().__init__('boat_thruster_control')
        self.create_subscription(Odometry, '/odom_ground_truth', self._odom_cb, 10)

    def _odom_cb(self, msg: Odometry):
        with _lock:
            _state["x"] = msg.pose.pose.position.x
            _state["z"] = msg.pose.pose.position.z


def main(args=None):
    rclpy.init(args=args)
    odom_watcher = OdomWatcher()

    gz_node = GzNode()
    gz_node.subscribe(Twist, '/model/cavex_tracked_blueboat/cmd_vel', _cmd_vel_cb)
    port_pub = gz_node.advertise(
        '/model/cavex_tracked_blueboat/joint/motor_port_joint/cmd_thrust', Double)
    stbd_pub = gz_node.advertise(
        '/model/cavex_tracked_blueboat/joint/motor_stbd_joint/cmd_thrust', Double)

    odom_watcher.get_logger().info(
        f"boat_thruster_control ready: mixing cmd_vel into port/stbd thrust "
        f"only while x > {WATER_BOUNDARY_X} and z >= {FLOAT_Z_MIN}.")

    timer_period = 0.1

    def control_tick():
        with _lock:
            x, z = _state["x"], _state["z"]
            linear_x, angular_z = _state["linear_x"], _state["angular_z"]

        if x is None or x <= WATER_BOUNDARY_X or z < FLOAT_Z_MIN:
            port_thrust = 0.0
            stbd_thrust = 0.0
        else:
            base = THRUST_GAIN_LINEAR * linear_x
            turn = THRUST_GAIN_ANGULAR * angular_z
            port_thrust = max(-MAX_THRUST_N, min(MAX_THRUST_N, base - turn))
            stbd_thrust = max(-MAX_THRUST_N, min(MAX_THRUST_N, base + turn))

        port_pub.publish(Double(data=port_thrust))
        stbd_pub.publish(Double(data=stbd_thrust))

    timer = odom_watcher.create_timer(timer_period, control_tick)
    rclpy.spin(odom_watcher)
    timer.cancel()
    odom_watcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
