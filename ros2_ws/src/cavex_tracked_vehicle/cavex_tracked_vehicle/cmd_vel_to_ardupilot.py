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
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
from ardupilot_msgs.srv import ArmMotors
from ardupilot_msgs.srv import ModeSwitch

ROVER_MODE_GUIDED = 15


class CmdVelToArduPilot(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ardupilot')
        self.pub = self.create_publisher(TwistStamped, '/ap/cmd_vel', 10)
        self.create_subscription(Twist, '/cmd_vel', self._cb, 10)
        # Each of these becomes True only once its service call is confirmed
        # (via the response's success field) to have actually succeeded --
        # never assumed just because the request was dispatched. The
        # corresponding "_pending" flag prevents re-sending a request while
        # one is already in flight. Until both "_set"/"_armed" are True,
        # _ensure_armed_and_guided retries on every subsequent /cmd_vel.
        self._mode_set = False
        self._mode_pending = False
        self._armed = False
        self._armed_pending = False
        self.arm_client = self.create_client(ArmMotors, '/ap/arm_motors')
        self.mode_client = self.create_client(ModeSwitch, '/ap/mode_switch')
        self.get_logger().info(
            "cmd_vel_to_ardupilot ready: relaying /cmd_vel -> /ap/cmd_vel; "
            "will arm + set Rover GUIDED via /ap/arm_motors and "
            "/ap/mode_switch on first /cmd_vel (retried until confirmed).")

    def _ensure_armed_and_guided(self):
        if self._mode_set and self._armed:
            return
        if not self._mode_set and not self._mode_pending:
            if self.mode_client.wait_for_service(timeout_sec=1.0):
                req = ModeSwitch.Request()
                req.mode = ROVER_MODE_GUIDED
                self._mode_pending = True
                self.mode_client.call_async(req).add_done_callback(
                    self._on_mode_switch_response)
            else:
                self.get_logger().warn(
                    "/ap/mode_switch service not available; will retry on "
                    "next /cmd_vel.")
        if not self._armed and not self._armed_pending:
            if self.arm_client.wait_for_service(timeout_sec=1.0):
                req = ArmMotors.Request()
                req.arm = True
                self._armed_pending = True
                self.arm_client.call_async(req).add_done_callback(
                    self._on_arm_response)
            else:
                self.get_logger().warn(
                    "/ap/arm_motors service not available; will retry on "
                    "next /cmd_vel.")

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
                f"curr_mode={resp.curr_mode}); will retry on next /cmd_vel.")

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
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'base_link'
        out.twist = msg
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToArduPilot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
