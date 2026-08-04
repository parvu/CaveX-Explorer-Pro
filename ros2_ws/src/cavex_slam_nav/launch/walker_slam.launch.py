import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # frame_id is 'base_footprint', not 'base_link' (unlike the wheeled
    # robot's 2D rtabmap_nav.launch.py, which this file's rtabmap block is
    # copied from). cavex_walker.urdf.xacro's TF tree is
    # base_footprint -> base_link -> {lidar_link, camera_link, leg links}
    # (base_link's ONLY parent is the static base_footprint joint). If
    # icp_odometry/rtabmap published odom -> base_link directly (frame_id=
    # 'base_link', matching the copied 2D snippet literally), base_link
    # would get a second, conflicting parent (odom) alongside the existing
    # static one (base_footprint) -- a broken TF tree. Using
    # frame_id='base_footprint' keeps the chain as map -> odom ->
    # base_footprint -> base_link, exactly what this task's own Interfaces
    # section (and slam_pose_publisher.py's default base_frame) expects.
    frame_id = 'base_footprint'

    # RTAB-Map's rtabmap_slam node does not compute its own odometry from
    # raw lidar data -- it only does SLAM (map graph + loop closure) given
    # an existing odometry estimate on 'odom' (topic or TF). The wheeled
    # robot's 2D setup gets that for free from gz's OdometryPublisher (see
    # gazebo_sim.launch.py's /odom bridge + odom_tf_broadcaster.py).
    # gazebo_walker.launch.py (Task 5) has no such /odom source for the
    # walker (champ_base's state_estimation_node isn't wired up -- it needs
    # champ_msgs-typed IMU/foot-contact relays this sim doesn't provide).
    # Confirmed empirically: `ros2 topic list` shows no /odom while the
    # walker stack is running, and rtabmap defaults to subscribing to a
    # topic named 'odom' -- with none published, the sync callback never
    # fires and rtabmap never processes a frame.
    #
    # The standard, real fix for lidar-only SLAM (no wheel/leg odometry) is
    # rtabmap_odom's icp_odometry node -- frame-to-frame ICP odometry from
    # the same lidar cloud, feeding rtabmap's SLAM node. This is exactly
    # the pattern in rtabmap_examples/launch/lidar3d.launch.py (the
    # upstream reference example for this exact scenario, installed at
    # /opt/ros/jazzy/share/rtabmap_examples/launch/lidar3d.launch.py).
    icp_odometry = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'frame_id': frame_id,
            'odom_frame_id': 'odom',
            # Same best-effort-vs-reliable QoS mismatch this project has
            # hit before (ros_gz_bridge publishes sensor data best-effort;
            # rtabmap_ros's default subscriber QoS is reliable).
            'qos': 2,
            'Icp/PointToPlane': 'true',
            'Icp/VoxelSize': '0.1',
        }],
        remappings=[
            ('scan_cloud', '/lidar/points'),
        ],
    )

    # RTAB-Map SLAM node, 3D lidar mode. Copied from rtabmap_nav.launch.py's
    # rtabmap Node block (2D scan mode for the wheeled robot), reconfigured
    # per this task's brief: subscribe_scan_cloud instead of subscribe_scan,
    # Grid/3D true (real 3D occupancy now that we have a real 3D lidar
    # instead of the wheeled robot's 2D one), and ICP registration params
    # for the point cloud.
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

    # Republishes RTAB-Map's map -> base_footprint TF chain as a plain
    # Odometry stream on /cavex/slam/odom. Existing node, unmodified --
    # its default base_frame ('base_footprint') matches this robot's link
    # naming (see frame_id comment above), so it's reused as-is.
    slam_pose_publisher = Node(
        package='cavex_slam_nav',
        executable='slam_pose_publisher.py',
        name='slam_pose_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        icp_odometry,
        rtabmap,
        slam_pose_publisher,
    ])
