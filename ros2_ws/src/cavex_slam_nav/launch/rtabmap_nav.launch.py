import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rtabmap_args = '--delete_db_on_start'

    return LaunchDescription([
        # Base Station Controller
        Node(
            package='cavex_slam_nav',
            executable='base_station_controller.py',
            name='base_station_controller',
            output='screen'
        ),
        
        # Drone Commander
        Node(
            package='cavex_slam_nav',
            executable='drone_commander.py',
            name='drone_commander',
            output='screen'
        ),

        # RTAB-Map SLAM Node
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'subscribe_depth': True,
                'subscribe_rgb': True,
                'subscribe_scan': True,
                'frame_id': 'base_link',
                'Grid/FromDepth': 'true',
                'Grid/RayTracing': 'true',
                'Grid/3D': 'true', # For 3D volumetric mapping of the shaft
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('depth/image', '/camera/depth/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('scan', '/lidar/scan')
            ],
            arguments=[rtabmap_args]
        ),

        # RTAB-Map Visualization (rviz / rtabmap_viz)
        Node(
            package='rtabmap_viz',
            executable='rtabmap_viz',
            name='rtabmap_viz',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]
        )
    ])
