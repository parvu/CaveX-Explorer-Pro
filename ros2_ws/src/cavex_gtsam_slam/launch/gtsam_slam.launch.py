"""
gtsam_slam.launch.py.

Brings up the GTSAM-SLAM factor graph node (real request, 2026-08-26:
cavex_sonar and the CurrentFactor subsystem removed from this branch,
and bluerov2 itself is now a forced-static decorative prop with no
sensors at all -- see perception branch for the full, functional
version of all of this). The node's own IMU-preintegration and
scan-registration machinery is unchanged in the code, but with no
/bluerov2/imu or /bluerov2/sonar publisher left on this branch it has
nothing to consume in practice -- it will start and idle rather than
producing real odometry. Kept structurally intact rather than deleted
in case this branch's BlueROV2 gains sensors again later.
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
