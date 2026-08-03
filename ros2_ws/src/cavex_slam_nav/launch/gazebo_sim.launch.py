import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_cavex = get_package_share_directory('cavex_slam_nav')

    world_file = os.path.join(pkg_cavex, 'worlds', 'cavex_world.world')
    urdf_file = os.path.join(pkg_cavex, 'urdf', 'cavex_robot.urdf.xacro')

    # Ported from Gazebo Classic (gzserver/gazebo_ros) to Gazebo Harmonic
    # (gz-sim + ros_gz_sim/ros_gz_bridge), since gazebo_ros isn't packaged
    # for ROS2 Jazzy.
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        # -s = server only, no gzclient GUI. Visualization now goes to the
        # web frontend (web_telemetry_bridge.py) instead of a desktop window.
        launch_arguments={'gz_args': f'-s -r {world_file}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        # Without use_sim_time, TF is stamped with wall-clock time while all
        # sensor data is stamped with sim time -- rtabmap's TF lookups for
        # the sensor timestamps then always fail ("TF ... is not set!").
        parameters=[{'robot_description': Command(['xacro ', urdf_file]), 'use_sim_time': True}]
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-name', 'cavex_robot',
                   '-topic', 'robot_description', '-x', '-35', '-y', '0', '-z', '1.5'],
        output='screen'
    )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        # Real gz-transport topic names verified empirically with `gz topic
        # -l` / `gz topic -i` after spawn -- the sensor <topic> overrides in
        # the URDF replace the full path rather than aliasing it, so these
        # don't follow the usual /world/.../sensor/... scoped convention.
        arguments=[
            '/lidar/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/color@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/cavex_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # Without this, every use_sim_time=true node (rtabmap included)
            # has no time source at all and effectively never progresses.
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            ('/camera/color', '/camera/color/image_raw'),
            ('/camera/camera_info', '/camera/color/camera_info'),
            ('/model/cavex_robot/odometry', '/odom'),
        ],
        output='screen'
    )

    odom_tf_broadcaster = Node(
        package='cavex_slam_nav',
        executable='odom_tf_broadcaster.py',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge,
        odom_tf_broadcaster,
    ])
