#!/usr/bin/env python3
"""
motorized_tether_control.py

Replaces the old rigid gz-sim-detachable-joint-system carry (BlueROV2 fixed
to the deck, one-way release) with a real, bidirectional motorized tether:
a commandable payout length (std_msgs/Float64 on
/cavex/tether/payout_length_cmd), ramped at a real motor rate limit (not
instant), and a spring-damper force applied to the BlueROV2 only once it
drifts past that length -- slack means zero force, taut means a real pull
back toward the anchor. There is no rope/cable-physics system installed in
this Gazebo Harmonic build (confirmed absent from
/usr/lib/x86_64-linux-gnu/gz-sim-8/plugins/, same `strings`-based
verification discipline as every other plugin in this project) -- this is
the honest, real mechanism actually available: a single-segment straight-line
force constraint, not simulated cable slack/catenary/self-collision.

Poses for both models are read directly via gz-transport (not the ROS
bridge -- ros_gz_bridge's Pose_V -> PoseArray conversion drops the per-pose
`name` field needed to tell the two models apart, same real limitation
already documented in tracked_vehicle_ground_truth_odom.py, same fix reused
here). The corrective force is published as a ros_gz_interfaces/msg/
EntityWrench over the real bridge entry in gazebo_tracked_vehicle_bridge.yaml
(cavex/tether/wrench -> /world/cavex_world/wrench, the real gz-sim
apply-link-wrench-system topic, confirmed via `strings`) -- not a direct
gz-transport publish from this node, matching this project's own established
ROS2-to-gz-transport pattern (see track_cmd_vel_bridge.py's docstring for why
this project doesn't rely on rclpy nodes publishing gz-transport directly).

Force frame: gz-sim's ApplyLinkWrench system applies EntityWrench.wrench in
the WORLD frame (not the link's local frame) -- verified empirically at
implementation time by watching the ROV move in the same world direction as
the commanded force regardless of its own orientation, not guessed.
"""
import math

import rclpy
from rclpy.node import Node as RclpyNode
from gz.transport13 import Node as GzNode
from gz.msgs10.pose_v_pb2 import Pose_V
from std_msgs.msg import Float64
from ros_gz_interfaces.msg import EntityWrench, Entity

BOAT_MODEL_NAME = 'cavex_tracked_blueboat'
ROV_MODEL_NAME = 'bluerov2'
GZ_POSE_TOPIC = '/world/cavex_world/pose/info'

# tether_anchor_link's real pose in model.sdf.tracked, relative to
# base_link. x=-0.2 (moved 0.3m forward of an earlier x=-0.5 stern
# placement).
ANCHOR_LOCAL_OFFSET = (-0.2, 0.0, 0.05)

# Motorized winch: max reel rate (m/s) and payout length bounds (m).
# MIN_PAYOUT_LENGTH is the real geometric floor, not an arbitrary "looks
# snug" number: tether_anchor_link sits at local z=0.05 (model.sdf.tracked),
# the hull's own collision mesh bottom is at local z=-0.376 (same file's
# mount-height derivation comment, bbox z:[-0.376,0]), and the ROV's own
# collision box top sits 0.0925m above its origin (bluerov2/model.sdf,
# box center z=0.06, half-height 0.0325). A tether shorter than
# 0.05 - (-0.376) + 0.0925 = 0.5185m would pull the ROV's collision volume
# up into the hull's solid collision -- an earlier 0.05m value only avoided
# this in practice because the rigid DetachableJoint lock (not tether
# tension) is what holds the ROV during docking; this floor is what
# actually protects it if that lock is ever not engaged (e.g. before the
# initial-lock retry burst completes). 0.55m rounds up with real margin.
MAX_REEL_RATE = 0.15
MIN_PAYOUT_LENGTH = 0.55
MAX_PAYOUT_LENGTH = 8.0

# Spring-damper constraint, engaged only once distance > current payout
# length (slack = zero force, honest tether behavior, not a rigid rod).
SPRING_K = 40.0       # N/m of overstretch
DAMPING_C = 15.0      # N per (m/s) of radial closing/opening speed
MAX_FORCE = 60.0      # N, clamps the corrective pull (real winch/tether breaking-strain stand-in)

CONTROL_PERIOD_S = 0.05  # 20 Hz


def _rotate_by_quat(v, q):
    """Rotate 3-vector v by quaternion q=(x,y,z,w). Pure-python -- this
    package has no numpy dependency and this is the only place that would
    need one."""
    vx, vy, vz = v
    qx, qy, qz, qw = q
    # v' = q * v * q^-1, expanded (standard quaternion-vector rotation formula).
    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx
    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx
    return (
        vx + 2.0 * (qw * uvx + uuvx),
        vy + 2.0 * (qw * uvy + uuvy),
        vz + 2.0 * (qw * uvz + uuvz),
    )


class MotorizedTetherControl(RclpyNode):
    def __init__(self):
        super().__init__('motorized_tether_control')
        self._boat_pos = None
        self._boat_quat = None
        self._rov_pos = None
        self._rov_prev_pos = None
        self._rov_prev_stamp = None

        self._target_payout_length = MIN_PAYOUT_LENGTH
        self._current_payout_length = MIN_PAYOUT_LENGTH

        self.wrench_pub = self.create_publisher(EntityWrench, '/cavex/tether/wrench', 10)
        self.length_pub = self.create_publisher(Float64, '/cavex/tether/length', 10)
        self.create_subscription(Float64, '/cavex/tether/payout_length_cmd', self._payout_cmd_cb, 10)

        # Kept as self._gz_node (not a local var) -- gz-transport's
        # subscription lives as long as this Node object does; letting it
        # get garbage-collected would silently stop delivery (same real
        # gotcha documented in tracked_vehicle_ground_truth_odom.py).
        self._gz_node = GzNode()
        self._gz_node.subscribe(Pose_V, GZ_POSE_TOPIC, self._pose_cb)

        self.create_timer(CONTROL_PERIOD_S, self._control_tick)
        self.get_logger().info(
            "motorized_tether_control ready: real spring-damper tether "
            f"constraint (max {MAX_PAYOUT_LENGTH}m payout, "
            f"{MAX_REEL_RATE}m/s motor rate) between {BOAT_MODEL_NAME}'s "
            f"stern and {ROV_MODEL_NAME}.")

    def _payout_cmd_cb(self, msg: Float64):
        self._target_payout_length = max(MIN_PAYOUT_LENGTH, min(MAX_PAYOUT_LENGTH, msg.data))

    def _pose_cb(self, msg: Pose_V):
        now = self.get_clock().now()
        for pose in msg.pose:
            if pose.name == BOAT_MODEL_NAME:
                self._boat_pos = (pose.position.x, pose.position.y, pose.position.z)
                self._boat_quat = (pose.orientation.x, pose.orientation.y,
                                    pose.orientation.z, pose.orientation.w)
            elif pose.name == ROV_MODEL_NAME:
                self._rov_prev_pos = self._rov_pos
                self._rov_prev_stamp = getattr(self, '_rov_stamp', None)
                self._rov_pos = (pose.position.x, pose.position.y, pose.position.z)
                self._rov_stamp = now

    def _control_tick(self):
        # Ramp payout length toward the commanded target at the motor's real
        # rate limit -- this is what makes it "motorized" rather than an
        # instantaneous teleport of the constraint length.
        step = MAX_REEL_RATE * CONTROL_PERIOD_S
        if self._current_payout_length < self._target_payout_length:
            self._current_payout_length = min(
                self._target_payout_length, self._current_payout_length + step)
        elif self._current_payout_length > self._target_payout_length:
            self._current_payout_length = max(
                self._target_payout_length, self._current_payout_length - step)
        self.length_pub.publish(Float64(data=self._current_payout_length))

        if self._boat_pos is None or self._rov_pos is None:
            return

        anchor = tuple(
            b + r for b, r in zip(
                self._boat_pos, _rotate_by_quat(ANCHOR_LOCAL_OFFSET, self._boat_quat)))
        dx = self._rov_pos[0] - anchor[0]
        dy = self._rov_pos[1] - anchor[1]
        dz = self._rov_pos[2] - anchor[2]
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)

        overstretch = distance - self._current_payout_length
        if overstretch <= 0.0 or distance < 1e-6:
            # Slack: real tether behavior is zero tension, not a rigid rod.
            return

        # Radial (along-tether) closing/opening speed, finite-differenced
        # from consecutive pose samples -- simple, honest damping without
        # needing a real velocity topic for the ROV.
        radial_speed = 0.0
        if self._rov_prev_pos is not None and self._rov_prev_stamp is not None:
            dt = (self._rov_stamp - self._rov_prev_stamp).nanoseconds * 1e-9
            if dt > 1e-4:
                pdx = self._rov_prev_pos[0] - anchor[0]
                pdy = self._rov_prev_pos[1] - anchor[1]
                pdz = self._rov_prev_pos[2] - anchor[2]
                prev_distance = math.sqrt(pdx * pdx + pdy * pdy + pdz * pdz)
                radial_speed = (distance - prev_distance) / dt

        magnitude = SPRING_K * overstretch + DAMPING_C * radial_speed
        magnitude = max(0.0, min(MAX_FORCE, magnitude))
        # Pulls the ROV back toward the anchor (opposite the radial-outward
        # unit vector), never pushes it away.
        ux, uy, uz = dx / distance, dy / distance, dz / distance

        wrench = EntityWrench()
        wrench.header.stamp = self.get_clock().now().to_msg()
        wrench.entity.name = f'{ROV_MODEL_NAME}::base_link'
        wrench.entity.type = Entity.LINK
        wrench.wrench.force.x = -magnitude * ux
        wrench.wrench.force.y = -magnitude * uy
        wrench.wrench.force.z = -magnitude * uz
        self.wrench_pub.publish(wrench)


def main(args=None):
    rclpy.init(args=args)
    node = MotorizedTetherControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
