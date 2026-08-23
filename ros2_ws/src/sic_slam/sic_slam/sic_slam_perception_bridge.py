#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
import numpy as np
import torch

from sic_slam.model import AcousticUUVController


class SicSlamPerceptionBridgeNode(Node):
    def __init__(self):
        super().__init__('sic_slam_perception_bridge')

        # 1. Declare Parameters matching hardware and model configurations
        self.declare_parameter('checkpoint_path', 'uuv_controller.pth')
        self.declare_parameter('time_steps', 50)
        self.declare_parameter('num_samples', 500)
        self.declare_parameter('device', 'cpu')

        checkpoint_path = self.get_parameter('checkpoint_path').get_parameter_value().string_value
        self.time_steps = self.get_parameter('time_steps').get_parameter_value().integer_value
        self.num_samples = self.get_parameter('num_samples').get_parameter_value().integer_value
        device_name = self.get_parameter('device').get_parameter_value().string_value

        # 2. Local Rolling Buffer Allocation (Simulating a continuous queue of past pings)
        self.rolling_sonar_buffer = np.zeros((self.time_steps, self.num_samples), dtype=np.float32)

        # 3. Load the PyTorch model directly -- no TensorRT/pycuda. Those are
        # Jetson-only; running locally (CPU, or CUDA if available) skips the
        # ONNX export + trtexec compile step entirely and just runs eager
        # PyTorch inference, which is plenty fast for this pipeline's target
        # rate off-Jetson.
        self.device = torch.device(device_name if torch.cuda.is_available() or device_name == 'cpu' else 'cpu')
        self.model = AcousticUUVController(time_steps=self.time_steps, num_samples=self.num_samples)
        try:
            state_dict = torch.load(checkpoint_path, map_location=self.device)
            self.model.load_state_dict(state_dict)
            self.get_logger().info(f'Loaded checkpoint: {checkpoint_path}')
        except FileNotFoundError:
            self.get_logger().warn(
                f'Checkpoint {checkpoint_path} not found -- running with '
                'randomly-initialized (untrained) weights.'
            )
        self.model.to(self.device).eval()

        # 4. Initialize Core Pub/Sub interfaces
        self.sonar_subscription = self.create_subscription(
            Image,
            '/ping360_sonar/scan_image',
            self.sonar_beam_callback,
            10
        )

        self.landmark_publisher = self.create_publisher(
            PointStamped,
            '/sic_slam/predicted_landmarks',
            10
        )

        self.get_logger().info('SIC-SLAM PyTorch Perception Bridge Node Online.')

    def sonar_beam_callback(self, msg):
        """Processes raw incoming message updates sequentially to preserve temporal history."""
        incoming_intensity_row = np.frombuffer(msg.data, dtype=np.uint8).astype(np.float32) / 255.0

        if len(incoming_intensity_row) >= self.num_samples:
            incoming_intensity_row = incoming_intensity_row[:self.num_samples]
        else:
            incoming_intensity_row = np.pad(incoming_intensity_row, (0, self.num_samples - len(incoming_intensity_row)))

        self.rolling_sonar_buffer = np.roll(self.rolling_sonar_buffer, -1, axis=0)
        self.rolling_sonar_buffer[-1, :] = incoming_intensity_row

        start_time = self.get_clock().now()

        with torch.no_grad():
            input_tensor = torch.from_numpy(self.rolling_sonar_buffer).unsqueeze(0).to(self.device)
            output = self.model(input_tensor).squeeze(0).cpu().numpy()

        inference_latency_ms = (self.get_clock().now() - start_time).nanoseconds / 1e6

        landmark_msg = PointStamped()
        landmark_msg.header = msg.header
        landmark_msg.point.x = float(output[0])
        landmark_msg.point.y = float(output[1])
        landmark_msg.point.z = float(output[2])

        self.landmark_publisher.publish(landmark_msg)

        self.get_logger().debug(f'Inference executed in {inference_latency_ms:.2f} ms.')


def main(args=None):
    rclpy.init(args=args)
    node = SicSlamPerceptionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down perception bridge pipeline safely.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
