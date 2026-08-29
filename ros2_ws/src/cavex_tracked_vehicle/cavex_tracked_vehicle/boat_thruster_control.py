#!/usr/bin/env python3
"""
boat_thruster_control.py

Real request 2026-08-26: real water propulsion for the boat's twin motor/
prop assemblies (model.sdf.tracked's motor_port_joint/motor_stbd_joint,
driven by two gz-sim-thruster-system plugin instances). Mixes the
vehicle's real drive command into differential port/stbd thrust and
publishes it on each thruster's real cmd_thrust topic.

Taps /model/cavex_tracked_blueboat/cmd_vel directly (gz-transport, same
pattern as manual_gui_bridge.py's own publisher) -- the same topic the
TrackedVehicle plugin also reads for the tracks. Real request 2026-08-27:
briefly tried splitting them onto separate topics (an SDF <topic>
override on TrackedVehicle) for real mutual exclusion; reverted after
confirming live that <topic> silently breaks that plugin's Twist
SUBSCRIPTION (its odometry, an unrelated publish-only code path, kept
working fine the whole time, which is what made this so misleading) --
see manual_gui_bridge.py's own comment for the full story. Tracks and
thrusters are both driven from this one shared topic again; this file's
own ACTIVE_MODES gate below still stops thrust output independently.

Real request 2026-08-27: fires purely off /cavex/locomotion_mode
(vehicle_switch_node.py's tracks<->props state machine), not its own
copy of an x/z threshold -- this file, track_retract_control.py, and
vehicle_switch_node.py each independently re-deriving the same threshold
is exactly what caused two stale-constant bugs the same day (a
water-surface-height change updated some copies, not all). Active during
"props" AND "deploying" (thrusters keep driving through the deploy
transition so the vehicle doesn't stall right as the tracks redeploy --
see vehicle_switch_node.py's own docstring for the full state machine).
"""
import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from gz.transport13 import Node as GzNode
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.double_pb2 import Double

# Real motor y-offset from model.sdf.tracked (motor_port_joint/motor_stbd_joint
# poses, +-0.295).
HALF_SEPARATION = 0.295

# 2026-08-29: linear was 300 -> 130 -> 80 (0.8 m/s command was driving the
# boat ~2 m/s, and fwd/rev felt violent next to turning). angular 80 -> 180
# to bring turn authority up to parity -- a left/right press (0.5 rad/s
# cmd) now makes a ~2*180*0.295 = ~106 N*m yaw couple vs ~90 N of forward
# thrust, roughly matched.
THRUST_GAIN_LINEAR = 80.0    # N per (m/s) of commanded linear.x
THRUST_GAIN_ANGULAR = 140.0  # N per (rad/s) of commanded angular.z, applied
                              # differentially (see mix below)
MAX_THRUST_N = 260.0

# 2026-08-29: the props sit ~0.1 m below the CoM, so a step change in thrust
# throws a pitch couple that dips a pontoon (worse before the buoyancy
# drag/lift split was fixed, still worth softening). Ramp thrust toward its
# target instead of snapping -- a full 0->240 N (0.8 m/s cmd) now takes
# ~0.8 s instead of one tick, so the leveling PID keeps up.
THRUST_SLEW_N_PER_S = 300.0

ACTIVE_MODES = ('props', 'deploying')

_state = {"mode": "tracks", "linear_x": 0.0, "angular_z": 0.0}
_lock = threading.Lock()


def _cmd_vel_cb(msg: Twist):
    with _lock:
        _state["linear_x"] = msg.linear.x
        _state["angular_z"] = msg.angular.z


class ModeWatcher(Node):
    def __init__(self):
        super().__init__('boat_thruster_control')
        self.create_subscription(String, '/cavex/locomotion_mode', self._mode_cb, 10)

    def _mode_cb(self, msg: String):
        with _lock:
            _state["mode"] = msg.data


def main(args=None):
    rclpy.init(args=args)
    mode_watcher = ModeWatcher()

    gz_node = GzNode()
    gz_node.subscribe(Twist, '/model/cavex_tracked_blueboat/cmd_vel', _cmd_vel_cb)
    port_pub = gz_node.advertise(
        '/model/cavex_tracked_blueboat/joint/motor_port_joint/cmd_thrust', Double)
    stbd_pub = gz_node.advertise(
        '/model/cavex_tracked_blueboat/joint/motor_stbd_joint/cmd_thrust', Double)

    mode_watcher.get_logger().info(
        f"boat_thruster_control ready: mixing cmd_vel into port/stbd thrust "
        f"only while /cavex/locomotion_mode is in {ACTIVE_MODES}.")

    timer_period = 0.1
    slew_step = THRUST_SLEW_N_PER_S * timer_period
    cur = {"port": 0.0, "stbd": 0.0}

    def _toward(now, target):
        return now + max(-slew_step, min(slew_step, target - now))

    def control_tick():
        with _lock:
            mode = _state["mode"]
            linear_x, angular_z = _state["linear_x"], _state["angular_z"]

        if mode not in ACTIVE_MODES:
            port_target = stbd_target = 0.0
        else:
            base = THRUST_GAIN_LINEAR * linear_x
            turn = THRUST_GAIN_ANGULAR * angular_z
            port_target = max(-MAX_THRUST_N, min(MAX_THRUST_N, base - turn))
            stbd_target = max(-MAX_THRUST_N, min(MAX_THRUST_N, base + turn))

        cur["port"] = _toward(cur["port"], port_target)
        cur["stbd"] = _toward(cur["stbd"], stbd_target)
        port_pub.publish(Double(data=cur["port"]))
        stbd_pub.publish(Double(data=cur["stbd"]))

    timer = mode_watcher.create_timer(timer_period, control_tick)
    rclpy.spin(mode_watcher)
    timer.cancel()
    mode_watcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
