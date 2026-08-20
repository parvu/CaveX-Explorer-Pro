"""
sonar_and_current.launch.py.

Brings up the simulated acoustic sonar and the water current driver. Run
alongside the existing tracked-vehicle simulation launch, once the BlueROV2
has been released into the water section.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Declare launch arguments and start the sonar and current nodes."""
    profile = LaunchConfiguration('profile')
    vx = LaunchConfiguration('vx')
    seed = LaunchConfiguration('seed')

    return LaunchDescription([
        DeclareLaunchArgument(
            'profile', default_value='constant',
            description='constant, step or sinusoidal'),
        DeclareLaunchArgument(
            'vx', default_value='0.3',
            description='current along world X, m/s'),
        DeclareLaunchArgument(
            'seed', default_value='42',
            description='sonar speckle seed; fix for reproducible runs'),
        Node(
            package='cavex_sonar',
            executable='sonar_node',
            name='sonar_node',
            output='screen',
            parameters=[{'seed': seed}],
        ),
        Node(
            package='cavex_sonar',
            executable='current_field_node.py',
            name='current_field_node',
            output='screen',
            parameters=[{'profile': profile, 'vx': vx}],
        ),
    ])
