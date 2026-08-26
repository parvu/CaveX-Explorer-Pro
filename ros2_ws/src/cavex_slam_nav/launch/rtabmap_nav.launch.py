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
                # No depth camera exists on cavex_robot (RGB camera + 2D
                # lidar only) -- subscribe_depth=True left this node
                # waiting forever on a /camera/depth/image_raw that no one
                # publishes, so it never processed a single frame. RGB +
                # 2D lidar is a real, supported RTAB-Map mode.
                'subscribe_depth': False,
                'subscribe_rgb': True,
                'subscribe_scan': True,
                'frame_id': 'base_link',
                # ros_gz_bridge publishes Image/Odometry/CameraInfo/LaserScan
                # as best-effort; rtabmap's default subscriber QoS is
                # reliable, which silently never matches (confirmed via
                # `ros2 topic echo --qos-reliability best_effort`, which
                # receives data instantly where the default profile got
                # nothing). 2 = best effort in rtabmap_ros's qos_* convention.
                'qos_image': 2,
                'qos_camera_info': 2,
                'qos_scan': 2,
                'qos_odom': 2,
                'Grid/FromDepth': 'false',
                'Grid/RayTracing': 'true',
                'Grid/3D': 'false', # no depth source -> 2D grid from the lidar scan
            }],
            remappings=[
                ('rgb/image', '/camera/color/image_raw'),
                ('rgb/camera_info', '/camera/color/camera_info'),
                ('scan', '/lidar/scan')
            ],
            arguments=[rtabmap_args]
        ),

        # rtabmap_viz (Qt desktop GUI) is intentionally not launched here --
        # visualization goes to the web frontend instead (web_telemetry_bridge
        # below), not a desktop window.

        # Republishes RTAB-Map's map -> base_footprint TF chain (its real
        # SLAM-corrected pose) as a plain Odometry stream, so it can be
        # scored like any other odometry-shaped estimate.
        Node(
            package='cavex_slam_nav',
            executable='slam_pose_publisher.py',
            name='slam_pose_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Dead-reckoning prototype (real: cmd_vel+IMU dead reckoning, bias-
        # corrected against RTAB-Map's pose -- see
        # dead_reckoning_prototype_node.py for the honest scope of what
        # this is and isn't; it publishes /gtsam_slam/odometry as a
        # placeholder for the real cavex_gtsam_slam factor-graph node's
        # own output).
        Node(
            package='cavex_slam_nav',
            executable='dead_reckoning_prototype_node.py',
            name='dead_reckoning_prototype_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ATE evaluation harness (real Umeyama-aligned trajectory error,
        # see ate_metrics.py). Ground truth here is /odom -- gz-sim's
        # OdometryPublisher computes it directly from true simulator state
        # with no noise model, so it IS ground truth in this simulation
        # (not a claim about basin/real-hardware ground truth). Estimate is
        # the dead-reckoning prototype's fused pose, a real (if
        # prototype-scope) system, not a placeholder. Publish an Empty
        # message to /cavex/eval/finish_run to score a run.
        Node(
            package='cavex_slam_nav',
            executable='ate_evaluator_node.py',
            name='ate_evaluator_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'ground_truth_topic': '/odom',
                'estimate_topic': '/gtsam_slam/odometry',
            }],
        ),

        # Real (if minimal -- straight-line P-controller, no obstacle
        # avoidance) waypoint navigation. See waypoint_follower.py for scope.
        Node(
            package='cavex_slam_nav',
            executable='waypoint_follower.py',
            name='waypoint_follower',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Visualization sink: pushes pose/ATE/lidar telemetry to the web
        # frontend at http://localhost:3000 instead of a desktop GUI, and
        # relays waypoint goals from the web UI back to /cavex/nav/goal.
        Node(
            package='cavex_slam_nav',
            executable='web_telemetry_bridge.py',
            name='web_telemetry_bridge',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
