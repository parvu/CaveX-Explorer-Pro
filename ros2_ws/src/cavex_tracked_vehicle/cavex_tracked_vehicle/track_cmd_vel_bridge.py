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
parameter_bridge (see ../config/gazebo_tracked_vehicle_bridge.yaml --
merged with the sensor/pose/clock bridge entries as of a later fix; see
that file's own header comment for why two separate parameter_bridge
processes structurally broke icp_odometry via a duplicate /clock relay)
chained after this node's ROS 2 publish. That bridge is committed
alongside this node (not folded into this node itself, since
ros_gz_bridge's parameter_bridge
is the standard, already-a-dependency mechanism for ROS2<->gz-transport
topic bridging -- reimplementing that in Python would just be a worse
version of an existing tool).

Task 7's real go/no-go drive test found /ap/twist/filtered's linear x/y
components do NOT line up with Gazebo's model body frame that
TrackedVehicle expects on /model/cavex_tracked_blueboat/cmd_vel (which only
acts on linear.x=forward and angular.z=turn, same convention confirmed by
commanding pure /cmd_vel linear.y and observing zero output -- Rover's
AP_DDS twist input itself already ignores non-x/angular.z components, so
that is not the mismatch). Empirically, commanding /cmd_vel linear.x=0.5
produced a steady-state /ap/twist/filtered of linear=(x=-0.04, y=0.498) --
consistently close to a -90 degree yaw rotation of the intended forward
vector, not a straight pass-through. Confirmed live: relaying that raw
(x=-0.04, y=0.498) straight onto /model/cavex_tracked_blueboat/cmd_vel
(TrackedVehicle ignores the y component and only reads the near-zero x)
produced a near-zero real position delta in Gazebo over 15s -- the exact
failure mode this task's brief warned about.

Root cause (traced, not guessed): model.sdf.tracked's ArduPilotPlugin still
carries the vendored BlueBoat's `<gazeboXYZToNED>` yaw=90 term unchanged
from the original boat model (Task 4 vendored it as-is; nothing in Tasks
4-6 exercised a real closed-loop drive-via-ArduPilot test to catch this --
Task 5 tested TrackedVehicle directly over raw gz-transport, bypassing
ArduPilot's frame conversion entirely, and Task 6's report explicitly notes
it never ran a live SITL session for its own adapter nodes). That yaw=90
term rotates the frame AP_DDS reports body-frame velocity in relative to
Gazebo's model frame, 90 degrees off. Compensating for it here (rather than
touching model.sdf.tracked's ArduPilotPlugin block, which Task 7 is not
scoped to modify, and which also feeds real IMU/GPS state estimation that
already works -- changing it risks breaking EKF convergence that Task 3/6
already verified) is the minimal, correct fix: rotate the incoming twist by
+90 degrees (gazebo_x = ap_y, gazebo_y = -ap_x) before publishing.
angular.z is left unrotated -- a yaw-only frame rotation leaves the
z-angular-velocity component unchanged (confirmed empirically: no evidence
of an angular-axis swap, only the linear x/y swap+sign observed above).
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
            "/track_cmd_vel (rotated +90deg yaw to correct for "
            "model.sdf.tracked's inherited gazeboXYZToNED offset -- see "
            "module docstring) -- ros_gz_bridge then forwards this to "
            "/model/cavex_tracked_blueboat/cmd_vel.")

    def _cb(self, msg: TwistStamped):
        if not self._got_first_msg:
            self._got_first_msg = True
            self.get_logger().info(
                "First /ap/twist/filtered message received; bridge is live.")
        out = Twist()
        # +90deg yaw rotation compensating for model.sdf.tracked's inherited
        # ArduPilotPlugin gazeboXYZToNED yaw=90 term -- see module docstring
        # for the live evidence this was derived from.
        out.linear.x = msg.twist.linear.y
        out.linear.y = -msg.twist.linear.x
        out.linear.z = msg.twist.linear.z
        out.angular.x = msg.twist.angular.x
        out.angular.y = msg.twist.angular.y
        out.angular.z = msg.twist.angular.z
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TrackCmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
