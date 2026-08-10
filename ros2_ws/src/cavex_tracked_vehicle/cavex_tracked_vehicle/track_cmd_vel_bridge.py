#!/usr/bin/env python3
"""
track_cmd_vel_bridge.py

Relays ArduPilot's real DDS control-law output (/ap/twist/filtered,
geometry_msgs/msg/TwistStamped) into a plain ROS 2 /track_cmd_vel
(geometry_msgs/msg/Twist) topic.

Why /track_cmd_vel and not /cmd_vel:
The real TrackedVehicle input is always model-scoped gz-transport --
/model/<model_name>/cmd_vel (confirmed as
/model/cavex_tracked_blueboat/cmd_vel per this vehicle's real spawned model
name) -- never bare /cmd_vel, and it is gz-transport, not a ROS 2 topic at
all. Reusing /cmd_vel would also create a real feedback loop:
cmd_vel_to_ardupilot.py subscribes /cmd_vel (Nav2's command) to relay to
ArduPilot; if this node also published ArduPilot's own filtered output back
onto /cmd_vel, that output would loop back into cmd_vel_to_ardupilot and be
re-sent to ArduPilot as a new external command. Publishing to /track_cmd_vel
instead keeps the two adapter nodes' topics disjoint (Nav2 -> /cmd_vel ->
ArduPilot -> /ap/twist/filtered -> /track_cmd_vel -> Gazebo), which is a
closed loop through the autopilot exactly once, not twice.

/track_cmd_vel is a ROS 2 topic; the real TrackedVehicle input is
gz-transport, not ROS 2. This node cannot publish to it directly -- getting
onto /model/cavex_tracked_blueboat/cmd_vel requires a `ros_gz_bridge`
parameter_bridge (see ../config/gazebo_tracked_vehicle_bridge.yaml) chained
after this node's ROS 2 publish.

Real root cause found (after two failed Gazebo-SDF-side attempts -- see
model.sdf.tracked's ArduPilotPlugin comment for that history), by reading
ArduPilot's own AP_DDS source directly
(ardupilot/libraries/AP_DDS/AP_DDS_Client.cpp, update_topic(TwistStamped&)):
    Vector3f velocity;
    if (ahrs.get_velocity_NED(velocity)) {
        msg.twist.linear.x = velocity[1];   // East
        msg.twist.linear.y = velocity[0];   // North
        msg.twist.linear.z = -velocity[2];  // Up
    }
/ap/twist/filtered's linear.x/y is ahrs.get_velocity_NED(), converted to
ROS's ENU convention (REP 103) -- real WORLD-FRAME ground velocity
(East/North), not body-frame forward/lateral velocity, despite the
message's own frame_id being labeled "base_link".

First fix attempt rotated that world-frame velocity into body-frame using
the vehicle's yaw from ArduPilot's own /ap/pose/filtered (same AHRS source
as the velocity). That was mathematically correct but empirically still
wrong: live-verified (a raw gz-transport command bypassing ArduPilot
entirely produced motion within ~1deg of the commanded heading, proving
Gazebo/TrackedVehicle itself has no bug) that ArduPilot's own EKF heading
had drifted ~91deg from Gazebo's real ground truth by that point in a run --
this vehicle has no GPS/navsat sensor (removed earlier as orphaned/unwired),
so ArduPilot's EKF is pure IMU dead-reckoning with nothing to correct
accumulated drift. The rotation math was right; the heading it was fed was
wrong.

Real fix: use Gazebo's own ground-truth orientation for the rotation
instead of ArduPilot's (possibly drifted) EKF estimate -- read directly via
gz-transport's world pose stream (gz.msgs10.Pose_V on
/world/<world>/pose/info), the same mechanism and topic this project
already uses elsewhere (e.g. spawn_bluerov2_retry.py, motorized_tether_control.py),
filtered for this vehicle's own model name. This does intentionally use
simulator-only ground truth rather than something a real vehicle would have
-- see this project's own standing "honesty caveats" convention (README) --
but the alternative (trusting a demonstrably-drifting dead-reckoning EKF for
basic forward/backward driving) is strictly worse for this project's actual
goal of exercising SLAM/Nav2 navigation, not exercising ArduPilot's own
GPS-denied EKF performance.
"""
import math
import threading

import rclpy
from rclpy.node import Node as RclpyNode
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped, Twist

from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V

WORLD_NAME = 'cavex_world'
VEHICLE_MODEL_NAME = 'cavex_tracked_blueboat'
GZ_POSE_TOPIC = f'/world/{WORLD_NAME}/pose/info'


def yaw_from_quat(x, y, z, w):
    """Standard ENU-quaternion -> yaw (rad), same formula used throughout
    this project (e.g. dead_end_backtrack_node.py's _yaw_from_quat)."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def world_to_body(vx, vy, yaw):
    """Rotate a world-frame ENU (East, North) velocity into body-frame
    (Forward, Left) using the vehicle's current yaw (rad, ENU convention --
    0 = facing +X/East). Pure function, no ROS/gz dependency, so it can be
    exercised directly by the self-check below."""
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)
    forward = vx * cos_y + vy * sin_y
    left = -vx * sin_y + vy * cos_y
    return forward, left


class TrackCmdVelBridge(RclpyNode):
    def __init__(self):
        super().__init__('track_cmd_vel_bridge')
        self.pub = self.create_publisher(Twist, '/track_cmd_vel', 10)
        self._yaw_lock = threading.Lock()
        self._yaw = 0.0
        self._got_first_gt_pose = False
        self._got_first_twist = False

        self._gz_node = GzNode()
        self._gz_node.subscribe(Pose_V, GZ_POSE_TOPIC, self._gz_pose_cb)

        # AP_DDS publishes /ap/twist/filtered as BEST_EFFORT (confirmed live via
        # `ros2 topic info -v /ap/twist/filtered`); rclpy's default subscription
        # QoS is RELIABLE, which is incompatible and silently drops all messages
        # (only a QoS-mismatch warning, no error). qos_profile_sensor_data is
        # BEST_EFFORT and matches.
        self.create_subscription(
            TwistStamped, '/ap/twist/filtered', self._twist_cb, qos_profile_sensor_data)
        self.get_logger().info(
            "track_cmd_vel_bridge ready: relaying /ap/twist/filtered -> "
            "/track_cmd_vel, rotated from world-frame ENU into body-frame "
            "using GAZEBO GROUND-TRUTH yaw (gz-transport "
            f"{GZ_POSE_TOPIC}, not ArduPilot's own EKF estimate -- that "
            "was tried first and found to drift ~90deg with no GPS/navsat "
            "aiding; see module docstring) -- ros_gz_bridge then forwards "
            "this to /model/cavex_tracked_blueboat/cmd_vel.")

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

    def _twist_cb(self, msg: TwistStamped):
        if not self._got_first_twist:
            self._got_first_twist = True
            self.get_logger().info(
                "First /ap/twist/filtered message received; bridge is live.")
        with self._yaw_lock:
            yaw = self._yaw
        forward, left = world_to_body(
            msg.twist.linear.x, msg.twist.linear.y, yaw)
        out = Twist()
        out.linear.x = forward
        out.linear.y = left
        out.linear.z = msg.twist.linear.z
        out.angular.x = msg.twist.angular.x
        out.angular.y = msg.twist.angular.y
        out.angular.z = msg.twist.angular.z
        self.pub.publish(out)


def _self_check():
    """Ponytail: smallest runnable check for world_to_body's non-trivial
    trig. Run directly: `python3 track_cmd_vel_bridge.py --self-check`."""
    # Facing East (yaw=0): world East velocity should read as pure forward.
    f, l = world_to_body(1.0, 0.0, 0.0)
    assert abs(f - 1.0) < 1e-9 and abs(l - 0.0) < 1e-9, (f, l)

    # Facing North (yaw=+90deg): world North velocity should read as pure
    # forward (the vehicle is pointed that way).
    f, l = world_to_body(0.0, 1.0, math.pi / 2.0)
    assert abs(f - 1.0) < 1e-6 and abs(l - 0.0) < 1e-6, (f, l)

    # Facing East (yaw=0), moving North (world +Y): purely lateral (left)
    # relative to a vehicle facing East.
    f, l = world_to_body(0.0, 1.0, 0.0)
    assert abs(f - 0.0) < 1e-9 and abs(l - 1.0) < 1e-9, (f, l)

    print("track_cmd_vel_bridge self-check: OK")


def main(args=None):
    import sys
    if args is None:
        args = sys.argv[1:]
    if '--self-check' in args:
        _self_check()
        return
    rclpy.init(args=args)
    node = TrackCmdVelBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
