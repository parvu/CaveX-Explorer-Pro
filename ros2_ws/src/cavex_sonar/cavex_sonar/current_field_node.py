#!/usr/bin/env python3
"""
current_field_node.py.

Drives the water current the BlueROV2 experiences. The stock Hydrodynamics
plugin already implements ocean current and listens on the Gazebo topic
/ocean_current; no new physics is implemented here -- the force on the
vehicle is computed by upstream Gazebo, not by this project.

This node publishes a plain ROS geometry_msgs/msg/Vector3 on the ROS topic
/ocean_current. It does not talk to Gazebo directly: a ros_gz_bridge entry
in gazebo_tracked_vehicle_bridge.yaml (direction ROS_TO_GZ) relays that ROS
topic onto the real gz-transport /ocean_current topic (gz.msgs.Vector3d)
that the plugin subscribes to. Publishing straight to Gazebo per-tick via
the `gz topic` CLI was considered and rejected: at 10 Hz that is 10 process
spawns per second, which would perturb the real-time-factor measurements
this project cares about.

Also republishes the commanded current on /cavex/current_ground_truth for
EVALUATION ONLY. No perception or control node may ever subscribe to that
topic; it exists so a scorer can compare an estimate against the truth, the
same discipline this project already applies to ground-truth pose.
"""
import math

from geometry_msgs.msg import Vector3, Vector3Stamped
import rclpy
from rclpy.node import Node


def current_at(profile, t, params):
    """
    Return the (x, y, z) water current in m/s at time t seconds.

    Pure function, no ROS, so it is directly unit-testable.
    """
    vx = params.get('vx', 0.0)
    vy = params.get('vy', 0.0)
    vz = params.get('vz', 0.0)

    if profile == 'constant':
        return (vx, vy, vz)
    if profile == 'step':
        return (vx, vy, vz) if t >= params.get('step_time', 10.0) else (0.0, 0.0, 0.0)
    if profile == 'sinusoidal':
        period = params.get('period_s', 20.0)
        s = math.sin(2.0 * math.pi * t / period)
        return (vx * s, vy * s, vz * s)

    # Fail loudly. A typo in a launch argument must not silently disable the
    # disturbance and quietly invalidate a whole evaluation run.
    raise ValueError(
        f'unknown current profile {profile!r}; '
        "expected 'constant', 'step' or 'sinusoidal'")


class CurrentFieldNode(Node):

    def __init__(self):
        super().__init__('current_field_node')
        self.declare_parameter('profile', 'constant')
        self.declare_parameter('vx', 0.3)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('vz', 0.0)
        self.declare_parameter('step_time', 10.0)
        self.declare_parameter('period_s', 20.0)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.profile = self.get_parameter('profile').value
        self.params = {
            'vx': self.get_parameter('vx').value,
            'vy': self.get_parameter('vy').value,
            'vz': self.get_parameter('vz').value,
            'step_time': self.get_parameter('step_time').value,
            'period_s': self.get_parameter('period_s').value,
        }

        # Validate the profile once at startup rather than failing per-tick.
        current_at(self.profile, 0.0, self.params)

        self.truth_pub = self.create_publisher(
            Vector3Stamped, '/cavex/current_ground_truth', 10)
        self.gz_pub = self.create_publisher(Vector3, '/ocean_current', 10)
        self.t0 = self.get_clock().now()
        rate = self.get_parameter('publish_rate_hz').value
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'current_field_node: profile={self.profile} params={self.params}. '
            'Publishing to Gazebo /ocean_current (via ros_gz_bridge) and, '
            'for scoring only, /cavex/current_ground_truth.')

    def _tick(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        vx, vy, vz = current_at(self.profile, t, self.params)

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.vector.x, msg.vector.y, msg.vector.z = vx, vy, vz
        self.truth_pub.publish(msg)

        gz_msg = Vector3()
        gz_msg.x, gz_msg.y, gz_msg.z = vx, vy, vz
        self.gz_pub.publish(gz_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CurrentFieldNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
