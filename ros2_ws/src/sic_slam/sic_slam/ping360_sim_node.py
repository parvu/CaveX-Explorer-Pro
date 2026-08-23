#!/usr/bin/env python3
"""Stands in for the real Ping360 hardware driver in simulation.

Gazebo Harmonic has no native sonar sensor type (same gap CaveX-Explorer-Pro
hit -- see its cavex_sonar package), so the vehicle SDF carries a gpu_lidar
sensor for geometry only. This node subscribes to that raw LaserScan
(bridged from gz-transport), converts each incoming scan's closest return
into a fake acoustic intensity row (exponential falloff toward the
detected range, speckle noise), and republishes it as a sensor_msgs/Image
on /ping360_sonar/scan_image -- the same topic and one-row-per-message
shape sic_slam_perception_bridge.py already expects from the real driver.

Not acoustically calibrated -- exercises the pipeline, not a physics model.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image
import numpy as np


class Ping360SimNode(Node):
    def __init__(self):
        super().__init__('ping360_sim_node')

        self.declare_parameter('num_samples', 500)
        self.declare_parameter('max_range_m', 10.0)
        self.declare_parameter('noise_std', 0.03)
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        self.max_range_m = self.get_parameter('max_range_m').get_parameter_value().double_value
        self.noise_std = self.get_parameter('noise_std').get_parameter_value().double_value

        self.rng = np.random.default_rng(42)

        self.sub = self.create_subscription(
            LaserScan, '/bluerov2/sonar_rays', self.on_scan, 10)
        self.pub = self.create_publisher(Image, '/ping360_sonar/scan_image', 10)

        self.get_logger().info('ping360_sim_node ready: /bluerov2/sonar_rays -> /ping360_sonar/scan_image')

    def on_scan(self, msg: LaserScan):
        ranges = np.array(msg.ranges, dtype=np.float32)
        finite = ranges[np.isfinite(ranges)]
        closest = float(finite.min()) if finite.size else self.max_range_m

        # Fake acoustic return profile: intensity rises as an exponential
        # bump around the closest obstacle's range bin, flat noise floor
        # elsewhere -- mimics "wall reflection" shape, not real backscatter.
        bins = np.linspace(0.0, self.max_range_m, self.num_samples, dtype=np.float32)
        hit_bin = np.clip(closest, 0.0, self.max_range_m)
        profile = np.exp(-((bins - hit_bin) ** 2) / (2 * 0.15 ** 2))
        noise = self.rng.normal(0.0, self.noise_std, size=self.num_samples).astype(np.float32)
        intensity = np.clip(profile + noise, 0.0, 1.0)
        row = (intensity * 255.0).astype(np.uint8)

        img = Image()
        img.header = msg.header
        img.height = 1
        img.width = self.num_samples
        img.encoding = 'mono8'
        img.is_bigendian = 0
        img.step = self.num_samples
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
