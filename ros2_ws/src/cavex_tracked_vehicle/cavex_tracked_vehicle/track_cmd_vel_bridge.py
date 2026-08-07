#!/usr/bin/env python3
"""
track_cmd_vel_bridge.py

Relays ArduPilot's real DDS control-law output (/ap/twist/filtered,
geometry_msgs/msg/TwistStamped -- confirmed live in Task 6, matches Task 3's
finding) into a plain ROS 2 /track_cmd_vel (geometry_msgs/msg/Twist) topic.

Why /track_cmd_vel and not /cmd_vel:
The design spec's original sketch used a separate /track_cmd_vel name, and
this file's stub considered reusing bare /cmd_vel instead (on the theory,
from Task 3's `strings` scan, that TrackedVehicle might subscribe there
directly). Task 3's *live* verification (not just `strings`) found the real
TrackedVehicle input is always model-scoped gz-transport --
/model/<model_name>/cmd_vel (confirmed here as
/model/cavex_tracked_blueboat/cmd_vel per Task 5's real spawned model name)
-- never bare /cmd_vel, and it is gz-transport, not a ROS 2 topic at all. So
neither of the stub's two premises for reusing /cmd_vel held up. Reusing
/cmd_vel would also create a real feedback loop: cmd_vel_to_ardupilot.py
subscribes /cmd_vel (Nav2's command) to relay to ArduPilot; if this node also
published ArduPilot's own filtered output back onto /cmd_vel, that output
would loop back into cmd_vel_to_ardupilot and be re-sent to ArduPilot as a
new external command. Publishing to /track_cmd_vel instead keeps the two
adapter nodes' topics disjoint (Nav2 -> /cmd_vel -> ArduPilot -> /ap/twist/
filtered -> /track_cmd_vel -> Gazebo), which is a closed loop through the
autopilot exactly once, not twice.

/track_cmd_vel is a ROS 2 topic; the real TrackedVehicle input is
gz-transport, not ROS 2. This node cannot publish to it directly -- getting
onto /model/cavex_tracked_blueboat/cmd_vel requires a `ros_gz_bridge`
parameter_bridge (see ../config/track_cmd_vel_bridge.yaml) chained after
this node's ROS 2 publish. That bridge is committed alongside this node
(not folded into this node itself, since ros_gz_bridge's parameter_bridge
is the standard, already-a-dependency mechanism for ROS2<->gz-transport
topic bridging -- reimplementing that in Python would just be a worse
version of an existing tool).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped, Twist


class TrackCmdVelBridge(Node):
    def __init__(self):
        super().__init__('track_cmd_vel_bridge')
        self.pub = self.create_publisher(Twist, '/track_cmd_vel', 10)
        # AP_DDS publishes /ap/twist/filtered as BEST_EFFORT (confirmed live via
        # `ros2 topic info -v /ap/twist/filtered`); rclpy's default subscription
        # QoS is RELIABLE, which is incompatible and silently drops all messages
        # (only a QoS-mismatch warning, no error). qos_profile_sensor_data is
        # BEST_EFFORT and matches.
        self.create_subscription(
            TwistStamped, '/ap/twist/filtered', self._cb, qos_profile_sensor_data)
        self._got_first_msg = False
        self.get_logger().info(
            "track_cmd_vel_bridge ready: relaying /ap/twist/filtered -> "
            "/track_cmd_vel (ros_gz_bridge then forwards this to "
            "/model/cavex_tracked_blueboat/cmd_vel).")

    def _cb(self, msg: TwistStamped):
        if not self._got_first_msg:
            self._got_first_msg = True
            self.get_logger().info(
                "First /ap/twist/filtered message received; bridge is live.")
        self.pub.publish(msg.twist)


def main(args=None):
    rclpy.init(args=args)
    node = TrackCmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
