#!/usr/bin/env python3
"""
cmd_vel_to_ardupilot.py

Relays the project's standard /cmd_vel (geometry_msgs/Twist, from Nav2 /
explore_lite) into ArduPilot's real AP_DDS cmd_vel input (geometry_msgs/
TwistStamped) on /ap/cmd_vel, and arms the vehicle + sets Rover GUIDED mode
on the first /cmd_vel received.

All topic/service names and values below were verified empirically against
a real running `ardurover` SITL + micro_ros_agent DDS bridge (Task 6), not
assumed:
  - /ap/cmd_vel: real, type geometry_msgs/msg/TwistStamped, 1 subscriber
    (`ros2 topic type /ap/cmd_vel`).
  - /ap/arm_motors [ardupilot_msgs/srv/ArmMotors], /ap/mode_switch
    [ardupilot_msgs/srv/ModeSwitch]: both present in `ros2 service list`
    and `ros2 service type`, matching AP_DDS_Service_Table.h's
    "arm_motorsService"/"mode_switchService" DDS names as exposed over
    ROS 2.
  - GUIDED = 15 for Rover: confirmed both by reading
    ardupilot/Rover/mode.h ("GUIDED = 15") and live:
    `ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch
    "{mode: 15}"` returned `status=True, curr_mode=15`.
  - `ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors
    "{arm: true}"` returned `result=True`.

Real fix for a "moving sideways / wrong direction" bug, root-caused by
reading ArduPilot's own AP_DDS source
(ardupilot/libraries/AP_DDS/AP_DDS_ExternalControl.cpp,
handle_velocity_control()): when the outgoing TwistStamped's frame_id is
"base_link" (what this node used to always send), ArduPilot converts the
body-frame command to world/NED internally via `ahrs.body_to_earth()` --
using ITS OWN EKF heading estimate. This vehicle has no GPS/navsat sensor
(removed earlier as orphaned/unwired), so that EKF is pure IMU
dead-reckoning with nothing to correct accumulated drift -- live-measured
at ~91deg of heading drift from Gazebo's real ground truth after a modest
amount of driving. Every command sent as frame_id="base_link" was therefore
being rotated by ArduPilot's own wrong heading before it ever reached the
simulated vehicle, regardless of anything done downstream
(track_cmd_vel_bridge.py's own ground-truth-based fix corrects AP's
*output* twist for Gazebo, but can't undo a command that was already
misdirected on the way *in*).

Real fix: convert the incoming body-frame /cmd_vel into world-frame ENU
using Gazebo's own ground-truth heading (same gz-transport mechanism and
/world/<world>/pose/info topic as track_cmd_vel_bridge.py) before
publishing, and send it with frame_id="map" instead -- AP_DDS's own
handle_velocity_control() has a second branch for exactly this case (ENU
input, converted to NED with a plain axis swap, no ahrs-based rotation at
all), bypassing ArduPilot's drifted heading entirely for this conversion.
"""
import math
import threading

import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.duration import Duration
from geometry_msgs.msg import Twist, TwistStamped
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

ROVER_MODE_GUIDED = 15

WORLD_NAME = 'cavex_world'
VEHICLE_MODEL_NAME = 'cavex_tracked_blueboat'
GZ_POSE_TOPIC = f'/world/{WORLD_NAME}/pose/info'

# Real bug found and fixed: arm=true on an ALREADY-armed vehicle returns
# result=False (confirmed live: a manual `ros2 service call .../arm_motors
# "{arm: true}"` on an already-armed SITL instance returns False, not a
# genuine rejection) -- this node used to treat every False as "not armed,
# retry," so any external arm (a manual service call, or a re-arm after this
# node's own first success) left `self._armed` permanently False, and
# _ensure_armed_and_guided retried on EVERY /cmd_vel callback (5-20Hz)
# forever. That DDS-flood competes with /ap/cmd_vel on the same DDS/
# micro_ros_agent bridge, and was directly observed live to starve real
# velocity commands badly enough to trip ArduPilot's own 3s GUIDED-target
# timeout ("target not received last 3secs, stopping") -- a real driving
# stall, not a cave-geometry problem. Rate-limited instead of removed
# entirely: a genuine transient failure should still eventually retry.
ARM_RETRY_MIN_INTERVAL_S = 2.0


def yaw_from_quat(x, y, z, w):
    """Standard ENU-quaternion -> yaw (rad), same formula used throughout
    this project (e.g. dead_end_backtrack_node.py's _yaw_from_quat)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def body_to_world(forward, left, yaw):
    """Rotate a body-frame (Forward, Left) velocity into world-frame ENU
    (East, North) using the vehicle's current yaw (rad, ENU convention --
    0 = facing +X/East). Inverse of track_cmd_vel_bridge.py's
    world_to_body. Pure function, no ROS/gz dependency, exercised directly
    by the self-check below."""
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    vx = forward * cos_y - left * sin_y
    vy = forward * sin_y + left * cos_y
    return vx, vy


class CmdVelToArduPilot(RclpyNode):
    def __init__(self):
        super().__init__('cmd_vel_to_ardupilot')
        self.pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        self._yaw_lock = threading.Lock()
        self._yaw = 0.0
        self._got_first_gt_pose = False

        self._gz_node = GzNode()
        self._gz_node.subscribe(Pose_V, GZ_POSE_TOPIC, self._gz_pose_cb)

        # Each of these becomes True only once its service call is confirmed
        # (via the response's success field) to have actually succeeded --
        # never assumed just because the request was dispatched. The
        # corresponding "_pending" flag prevents re-sending a request while
        # one is already in flight. Until both "_set"/"_armed" are True,
        # _ensure_armed_and_guided retries on every subsequent /cmd_vel.
        self._mode_set = False
        self._mode_pending = False
        self._mode_last_attempt = None
        self._armed = False
        self._armed_pending = False
        self._armed_last_attempt = None
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.get_logger().info(
            "cmd_vel_to_ardupilot ready: relaying /cmd_vel -> /ap/cmd_vel "
            "(rotated body->world using GAZEBO GROUND-TRUTH yaw, sent as "
            "frame_id=map -- bypasses ArduPilot's own, possibly-drifted "
            "EKF heading; see module docstring); will arm + set Rover "
            "GUIDED via /ap/arm_motors and /ap/mode_switch on first "
            "/cmd_vel (retried until confirmed).")

    def _gz_pose_cb(self, msg: Pose_V):
        for pose in msg.pose:
            if pose.name == VEHICLE_MODEL_NAME:
                q = pose.orientation
                yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
                with self._yaw_lock:
                    self._yaw = yaw
                if not self._got_first_gt_pose:
                    self._got_first_gt_pose = True
                    self.get_logger().info(
                        "First real ground-truth pose received for "
                        f"{VEHICLE_MODEL_NAME}; heading source is live.")
                return

    def _ensure_armed_and_guided(self):
        if self._mode_set and self._armed:
            return
        now = self.get_clock().now()
        min_interval = Duration(seconds=ARM_RETRY_MIN_INTERVAL_S)
        if (not self._mode_set and not self._mode_pending
                and (self._mode_last_attempt is None
                     or now - self._mode_last_attempt >= min_interval)):
            self._mode_last_attempt = now
            if self.mode_client.wait_for_service(timeout_sec=1.0):
                req = ModeSwitch.Request()
                req.mode = ROVER_MODE_GUIDED
                self._mode_pending = True
                self.mode_client.call_async(req).add_done_callback(
                    self._on_mode_switch_response)
            else:
                self.get_logger().warn(
                    "/ap/mode_switch service not available; will retry.")
        if (not self._armed and not self._armed_pending
                and (self._armed_last_attempt is None
                     or now - self._armed_last_attempt >= min_interval)):
            self._armed_last_attempt = now
            if self.arm_client.wait_for_service(timeout_sec=1.0):
                req = ArmMotors.Request()
                req.arm = True
                self._armed_pending = True
                self.arm_client.call_async(req).add_done_callback(
                    self._on_arm_response)
            else:
                self.get_logger().warn(
                    "/ap/arm_motors service not available; will retry.")

    def _on_mode_switch_response(self, future):
        self._mode_pending = False
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f"ModeSwitch service call failed: {exc}")
            return
        if resp.status:
            self._mode_set = True
            self.get_logger().info(
                f"ArduPilot mode set to GUIDED (curr_mode={resp.curr_mode}).")
        else:
            self.get_logger().warn(
                f"ArduPilot mode switch to GUIDED failed (status=False, "
                f"curr_mode={resp.curr_mode}); will retry.")

    def _on_arm_response(self, future):
        self._armed_pending = False
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f"ArmMotors service call failed: {exc}")
            return
        if resp.result:
            self._armed = True
            self.get_logger().info("ArduPilot armed successfully.")
        else:
            self.get_logger().warn(
                "ArduPilot arm request failed (result=False); will retry "
                "on next /cmd_vel.")

    def _cb(self, msg: Twist):
        self._ensure_armed_and_guided()
        with self._yaw_lock:
            yaw = self._yaw
        vx, vy = body_to_world(msg.linear.x, msg.linear.y, yaw)
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.twist.linear.x = vx
        out.twist.linear.y = vy
        out.twist.linear.z = msg.linear.z
        out.twist.angular = msg.angular
        self.pub.publish(out)


def _self_check():
    """Ponytail: smallest runnable check for body_to_world's non-trivial
    trig (the exact inverse of track_cmd_vel_bridge.py's world_to_body).
    Run directly: `python3 cmd_vel_to_ardupilot.py --self-check`."""
    # Facing East (yaw=0): pure forward should read as pure world East.
    vx, vy = body_to_world(1.0, 0.0, 0.0)
    assert abs(vx - 1.0) < 1e-9 and abs(vy - 0.0) < 1e-9, (vx, vy)

    # Facing North (yaw=+90deg): pure forward should read as pure world
    # North.
    vx, vy = body_to_world(1.0, 0.0, math.pi / 2.0)
    assert abs(vx - 0.0) < 1e-6 and abs(vy - 1.0) < 1e-6, (vx, vy)

    # Round-trip with track_cmd_vel_bridge.py's world_to_body at an
    # arbitrary yaw: body_to_world then world_to_body should recover the
    # original body-frame vector.
    import importlib.util
    import os
    spec = importlib.util.spec_from_file_location(
        "track_cmd_vel_bridge",
        os.path.join(os.path.dirname(__file__), "track_cmd_vel_bridge.py"))
    tcvb = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tcvb)
    yaw = math.radians(37.0)
    vx, vy = body_to_world(0.4, -0.1, yaw)
    f, l = tcvb.world_to_body(vx, vy, yaw)
    assert abs(f - 0.4) < 1e-9 and abs(l - (-0.1)) < 1e-9, (f, l)

    print("cmd_vel_to_ardupilot self-check: OK")


def main(args=None):
    import sys
    if args is None:
        args = sys.argv[1:]
    if '--self-check' in args:
        _self_check()
        return
    rclpy.init(args=args)
    node = CmdVelToArduPilot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
