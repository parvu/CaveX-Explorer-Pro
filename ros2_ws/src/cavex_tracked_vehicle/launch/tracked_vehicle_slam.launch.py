import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Ported from the abandoned cavex-legged-walker-phase1 branch's
    # walker_slam.launch.py (real, working config for a different vehicle in
    # this project). That branch used frame_id='base_footprint' because its
    # URDF's TF root is base_footprint -> base_link (base_link's only parent
    # is that static joint) -- using 'base_link' there would have given
    # base_link a second, conflicting parent.
    #
    # This vehicle is different: Task 7's stub URDF (cavex_tracked_vehicle.urdf,
    # read directly for this task) declares only <link name="base_link"/> plus
    # the two track_retract_joint children -- no base_footprint link/joint at
    # all. Confirmed live: launched gazebo_tracked_vehicle.launch.py and ran
    # `ros2 run tf2_tools view_frames`; the resulting tree's root is base_link
    # (published by robot_state_publisher from that same stub URDF), with
    # lidar_link/camera_link/imu_link etc. hanging off it directly -- no
    # base_footprint frame exists anywhere in the tree. So base_link is this
    # vehicle's real TF root, and that's what icp_odometry/rtabmap must
    # publish odom -> base_link to, not 'base_footprint'.
    frame_id = 'base_link'

    # Real, live-discovered gap (Step 3 of this task): lidar_link/camera_link
    # only exist in model.sdf.tracked (the native SDF spawned into Gazebo,
    # Task 7's post-review addition) via <joint type="fixed"> -- they are NOT
    # in cavex_tracked_vehicle.urdf (the launch-time stub robot_state_publisher
    # actually publishes TF from). view_frames confirmed this live: the real
    # TF tree only has left_track_retract_mount/right_track_retract_mount
    # under base_link, no lidar_link/camera_link anywhere. Without that TF,
    # icp_odometry can't look up base_link -> lidar_link and aborts every
    # scan ("TF of received scan cloud ... is not set"), confirmed live too.
    # Fixed offsets copied from model.sdf.tracked's own <pose relative_to=
    # "base_link"> tags for each link (0 0 0.55 for lidar_link -- raised from
    # 0.4 by this task's lidar-self-occlusion fix, see model.sdf.tracked's
    # lidar_link comment -- 0.55 0 0.15 for camera_link) --
    # static_transform_publisher, not a full URDF/xacro rewrite, since both
    # mounts are rigid and unactuated.
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0.55',
                   '--frame-id', 'base_link', '--child-frame-id', 'lidar_link'],
    )
    camera_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_static_tf',
        output='screen',
        arguments=['--x', '0.55', '--y', '0', '--z', '0.15',
                   '--frame-id', 'base_link', '--child-frame-id', 'camera_link'],
    )

    # RTAB-Map's rtabmap_slam node doesn't compute its own odometry from raw
    # lidar data -- it only does SLAM (map graph + loop closure) given an
    # existing odometry estimate on 'odom'. This vehicle has no wheel/track
    # odometry source feeding RTAB-Map directly either (model.sdf.tracked's
    # TrackedVehicle plugin drives the tracks as velocity-controlled links,
    # not rotating joints -- gazebo_tracked_vehicle.launch.py's gz_bridge
    # comment already documents its OdometryPublisher output isn't reliable
    # ground truth), so icp_odometry (frame-to-frame ICP odometry from the
    # same lidar cloud) is needed here too, same real fix as the reference.
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'odom_frame_id': 'odom',
            # ros_gz_bridge publishes sensor data best-effort; rtabmap_ros's
            # default subscriber QoS is reliable -- same mismatch fix as the
            # reference file.
            'qos': 2,
            'Icp/PointToPlane': 'true',
            'Icp/VoxelSize': '0.1',
        }],
        remappings=[
            ('scan_cloud', '/lidar/points'),
        ],
    )

    # RTAB-Map SLAM node, 3D lidar mode, remapped to this vehicle's real
    # sensor topics (Task 7).
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_depth': False,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'frame_id': frame_id,
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_scan_cloud': 2,
            'Grid/FromDepth': 'false',
            'Grid/3D': 'true',
            'Icp/PointToPlane': 'true',
            'Icp/VoxelSize': '0.1',
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('scan_cloud', '/lidar/points'),
        ],
        arguments=['--delete_db_on_start'],
    )

    # Republishes RTAB-Map's map -> base_link TF chain as a plain Odometry
    # stream on /cavex/slam/odom. Existing node from cavex_slam_nav,
    # unmodified -- cross-package reuse, base_frame overridden to this
    # vehicle's real root frame found above (its default is 'base_footprint',
    # which doesn't exist in this vehicle's TF tree).
    slam_pose_publisher = Node(
        package='cavex_slam_nav',
        executable='slam_pose_publisher.py',
        name='slam_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'base_frame': frame_id}],
    )

    # Nav2 bringup (costmap-only: no amcl/map_server, RTAB-Map above already
    # owns SLAM and publishes /map). Included here rather than as a
    # separate launch file since Nav2's static_layer needs RTAB-Map's /map
    # already flowing -- see tracked_vehicle_nav2_params.yaml for the full
    # rationale (ported from the abandoned cavex-legged-walker-phase1
    # branch's real, working config for a different vehicle in this
    # project, same no-/scan situation).
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(get_package_share_directory('cavex_tracked_vehicle'), 'config', 'tracked_vehicle_nav2_params.yaml'),
        }.items(),
    )

    return LaunchDescription([
        lidar_static_tf,
        camera_static_tf,
        icp_odometry,
        rtabmap,
        slam_pose_publisher,
        nav2_bringup_launch,
    ])
