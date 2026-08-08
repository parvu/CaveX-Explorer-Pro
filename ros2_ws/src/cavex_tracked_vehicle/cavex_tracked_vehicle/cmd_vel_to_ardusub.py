#!/usr/bin/env python3
"""
cmd_vel_to_ardusub.py

Relays /cmd_vel_rov (manual teleop, geometry_msgs/Twist -- no Nav2/
explore_lite for the BlueROV2, per this project's non-goal) into the
second ArduSub SITL instance's real AP_DDS cmd_vel input (geometry_msgs/
TwistStamped), and arms + sets GUIDED mode on the first /cmd_vel_rov
received.

All topic/service names and values below were verified empirically
against a real running second `ardusub` SITL + micro_ros_agent DDS bridge
(instance 1, port 2029), not assumed:
  - Real namespace collision found and fixed first: AP_DDS's own topic/
    service names are hardcoded ("ap/cmd_vel" etc, baked into the
    firmware's generated DDS bindings) regardless of which ROS 2
    micro_ros_agent instance bridges them -- running a second instance on
    a different UDP port still produced identical `/ap/...` topic names,
    colliding with the first (Rover) instance's own `/ap/...` topics in
    the same ROS 2 graph (`ros2 node list` showed two nodes both named
    `/ap`). Fixed via ArduPilot's own real `DDS_USE_NS` parameter
    (AP_DDS_Client.cpp: "When enabled, ROS 2 topic and service names
    include a v<MAV_SYSID> segment") -- set in
    ../config/dds_udp_instance1.parm. Confirmed live: topics/services
    appear under /ap/v1/... (the v<N> segment reflects this SITL
    instance's real MAV_SYSID at DDS-client-init time, not necessarily
    the SYSID_THISMAV value in that same defaults file -- confirmed to
    land on v1 here, not v2, but critically distinct from Rover's
    unnamespaced /ap/... on the same ROS 2 graph, which was the actual
    requirement).
  - /ap/v1/cmd_vel: real, type geometry_msgs/msg/TwistStamped (`ros2
    topic list` + `ros2 topic type /ap/v1/cmd_vel`).
  - /ap/v1/arm_motors [ardupilot_msgs/srv/ArmMotors], /ap/v1/mode_switch
    [ardupilot_msgs/srv/ModeSwitch]: both present in `ros2 service list`.
  - GUIDED = 4 for ArduSub: confirmed by reading ardupilot/ArduSub/mode.h
    ("GUIDED = 4, // fully automatic fly to coordinate or fly at
    velocity/direction using GCS immediate commands") -- NOT Rover's
    GUIDED=15, a distinct per-vehicle mode enum as the brief warned.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch

SUB_MODE_GUIDED = 4


class CmdVelToArduSub(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ardusub')
        self.pub = self.create_publisher(TwistStamped, '/ap/v1/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel_rov', self._cb, 10)
        # Same confirm-before-trusting pattern as cmd_vel_to_ardupilot.py:
        # each flag only becomes True once the service response's own
        # success field confirms it, never assumed from request dispatch
        # alone, and retried on every subsequent /cmd_vel_rov until confirmed.
        self._mode_set = False
        self._mode_pending = False
        self._armed = False
        self._armed_pending = False
        self.arm_client = self.create_client(ArmMotors, '/ap/v1/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/v1/mode_switch')
        self.get_logger().info(
            "cmd_vel_to_ardusub ready: relaying /cmd_vel_rov -> /ap/v1/cmd_vel; "
            "will arm + set ArduSub GUIDED via /ap/v1/arm_motors and "
            "/ap/v1/mode_switch on first /cmd_vel_rov (retried until confirmed).")

    def _ensure_armed_and_guided(self):
        if self._mode_set and self._armed:
            return
        if not self._mode_set and not self._mode_pending:
            if self.mode_client.wait_for_service(timeout_sec=1.0):
                req = ModeSwitch.Request()
                req.mode = SUB_MODE_GUIDED
                self._mode_pending = True
                self.mode_client.call_async(req).add_done_callback(
                    self._on_mode_switch_response)
            else:
                self.get_logger().warn(
                    "/ap/v1/mode_switch service not available; will retry on "
                    "next /cmd_vel_rov.")
        if not self._armed and not self._armed_pending:
            if self.arm_client.wait_for_service(timeout_sec=1.0):
                req = ArmMotors.Request()
                req.arm = True
                self._armed_pending = True
                self.arm_client.call_async(req).add_done_callback(
                    self._on_arm_response)
            else:
                self.get_logger().warn(
                    "/ap/v1/arm_motors service not available; will retry on "
                    "next /cmd_vel_rov.")

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
                f"ArduSub mode set to GUIDED (curr_mode={resp.curr_mode}).")
        else:
            self.get_logger().warn(
                f"ArduSub mode switch to GUIDED failed (status=False, "
                f"curr_mode={resp.curr_mode}); will retry on next /cmd_vel_rov.")

    def _on_arm_response(self, future):
        self._armed_pending = False
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f"ArmMotors service call failed: {exc}")
            return
        if resp.result:
            self._armed = True
            self.get_logger().info("ArduSub armed successfully.")
        else:
            self.get_logger().warn(
                "ArduSub arm request failed (result=False); will retry "
                "on next /cmd_vel_rov.")

    def _cb(self, msg: Twist):
        self._ensure_armed_and_guided()
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToArduSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
