"""
gtsam_slam.launch.py.

Brings up the GTSAM-SLAM factor graph node (real request, 2026-08-26:
cavex_sonar and the CurrentFactor subsystem removed from this branch --
see perception branch for the sonar/current version). The node's own
scan-registration machinery is unchanged, but with cavex_sonar gone
nothing publishes /bluerov2/sonar any more, so it degrades to IMU-only
dead reckoning in practice, same graceful "no scan points this
keyframe" path it already had for a scan-starved streak. Run alongside
the existing tracked-vehicle simulation once the BlueROV2 has been
released into the water section.
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
