"""
gtsam_slam.launch.py.

Brings up the GTSAM-SLAM factor graph node. Run alongside the existing
tracked-vehicle simulation and cavex_sonar's sonar_and_current.launch.py,
once the BlueROV2 has been released into the water section.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Start gtsam_slam_node with its defaults."""
    return LaunchDescription([
        Node(
            package='cavex_gtsam_slam',
            executable='gtsam_slam_node',
            name='gtsam_slam_node',
            output='screen',
        ),
    ])
