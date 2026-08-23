import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Hardware Driver Node Configuration
        # NOTE: 'ping360_sonar' is a third-party driver package
        # (https://github.com/CentraleNantesRobotics/ping360_sonar) and is
        # NOT built by this workspace. It must be cloned into src/ separately
        # and built with colcon, or this node will fail to launch.
        Node(package='ping360_sonar', executable='ping360.py', name='p360_node',
             parameters=[{'device': '/dev/ttyUSB0'}]),

        # TensorRT Neural Engine Extraction Node Configuration
        Node(package='sic_slam', executable='sic_slam_perception_bridge.py', name='trt_bridge'),

        # GTSAM iSAM2 Tracking Backend Graph Configuration
        Node(package='sic_slam', executable='sic_slam_graph_backend.py', name='graph_backend'),

        # Automated Evaluation Logging Engine
        Node(package='sic_slam', executable='sic_slam_flight_logger.py', name='flight_logger', output='screen')
    ])
