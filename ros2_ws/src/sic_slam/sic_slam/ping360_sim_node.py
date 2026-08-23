#!/usr/bin/env python3
"""Adapts cavex_sonar's real acoustic-physics sonar (transmission loss,
Lambertian backscatter, Rayleigh speckle, clutter/turbidity, real current
coupling -- see cavex_sonar/include/cavex_sonar/sonar_acoustics.hpp) into
the CNN's expected input shape.

Earlier version of this node faked an acoustic intensity row directly from
raw gpu_lidar ranges (exponential falloff + noise) -- exercised the wiring
only, no turbidity/current physics. Replaced (2026-08-23, real request) to
reuse cavex_sonar's sonar_node unmodified rather than duplicate its acoustic
model: sim_launch.py now runs sonar_node against sic_slam_tank's own
/bluerov2/sonar_rays, publishing a real per-beam echo-level LaserScan on
/bluerov2/sonar (beam_count beams, intensities in dB, real absorption/
clutter/current-drift baked in). This node normalizes those per-beam dB
levels into [0,1] and republishes as the same sensor_msgs/Image contract
sic_slam_perception_bridge.py already expects on /ping360_sonar/scan_image
-- one row per message. Width is now beam_count (a full simultaneous
multi-beam scan) rather than a single beam's range profile: Gazebo has no
true mechanically-scanning sonar sensor either way (same gap cavex_sonar's
own model.sdf comment documents), so "one row per full scan, rolled over
time" is the same kind of stand-in the earlier fake row already was, just
now backed by real acoustic physics instead of a fabricated bump.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
import numpy as np


class Ping360SimNode(Node):
    def __init__(self):
        super().__init__('ping360_sim_node')

        # Must match sonar_node's own source_level_db/detection_threshold_db
        # (cavex_sonar/src/sonar_node.cpp) for this normalization to line up
        # with what it actually emits -- not independent tuning knobs.
        self.declare_parameter('source_level_db', 200.0)
        self.declare_parameter('detection_threshold_db', 100.0)
        self.source_level_db = self.get_parameter('source_level_db').get_parameter_value().double_value
        self.threshold_db = self.get_parameter('detection_threshold_db').get_parameter_value().double_value
        self.db_span = max(self.source_level_db - self.threshold_db, 1e-6)

        self.sub = self.create_subscription(
            LaserScan, '/bluerov2/sonar', self.on_scan, 10)
        self.pub = self.create_publisher(Image, '/ping360_sonar/scan_image', 10)

        self.get_logger().info('ping360_sim_node ready: /bluerov2/sonar (real acoustic physics) -> /ping360_sonar/scan_image')

    def on_scan(self, msg: LaserScan):
        intensities_db = np.array(msg.intensities, dtype=np.float32)
        norm = np.clip((intensities_db - self.threshold_db) / self.db_span, 0.0, 1.0)
        row = (norm * 255.0).astype(np.uint8)

        img = Image()
        img.header = msg.header
        img.height = 1
        img.width = len(row)
        img.encoding = 'mono8'
        img.is_bigendian = 0
        img.step = len(row)
        img.data = row.tobytes()
        self.pub.publish(img)


def main(args=None):
    rclpy.init(args=args)
    node = Ping360SimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
