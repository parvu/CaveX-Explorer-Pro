import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    pkg_cavex = get_package_share_directory('cavex_slam_nav')
    
    world_file = os.path.join(pkg_cavex, 'worlds', 'cavex_world.world')
    urdf_file = os.path.join(pkg_cavex, 'urdf', 'cavex_robot.urdf.xacro')
    
    start_gazebo_server = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world_file],
        output='screen'
    )
    
    start_gazebo_client = ExecuteProcess(
        cmd=['gzclient'],
        output='screen'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
    )
    
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'cavex_robot', '-topic', 'robot_description', '-x', '-35', '-y', '0', '-z', '1.5'],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo_server,
        start_gazebo_client,
        robot_state_publisher,
        spawn_entity
    ])
