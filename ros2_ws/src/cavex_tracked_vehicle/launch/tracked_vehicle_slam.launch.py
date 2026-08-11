import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Ported from a prior project vehicle's working SLAM launch config. That
    # vehicle used frame_id='base_footprint' because its URDF's TF root is
    # base_footprint -> base_link (base_link's only parent is that static
    # joint) -- using 'base_link' there would have given base_link a second,
    # conflicting parent.
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
    # Real, live-diagnosed bug (final whole-branch review, Critical 1):
    # camera_link above is body-convention (x-forward/y-left/z-up, same as
    # base_link -- zero rotation), but model.sdf.tracked's camera sensor now
    # reports its image header's frame_id as camera_link_optical (REP
    # 103/145 optical convention: z-forward, x-right, y-down), which is what
    # image_geometry::PinholeCameraModel::project3dToPixel (sic_slam_node.cpp)
    # actually requires. Zero-translation, pure-rotation static TF -- the
    # standard body-to-optical rotation, same convention every ROS camera
    # driver publishes.
    camera_optical_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_static_tf',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--roll', '-1.5707963', '--pitch', '0', '--yaw', '-1.5707963',
                   '--frame-id', 'camera_link', '--child-frame-id', 'camera_link_optical'],
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
            # Switched 3D -> 2D lidar (real request, see
            # docs/superpowers/specs/2026-08-11-2d-lidar-switch-design.md):
            # point-to-point ICP, same reason as the rtabmap node below --
            # a flat single-ring scan can't provide reliable surface
            # normals for point-to-plane.
            'Icp/PointToPlane': 'false',
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
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'frame_id': frame_id,
            'qos_image': 2,
            'qos_camera_info': 2,
            'qos_scan_cloud': 2,
            'Grid/FromDepth': 'false',
            # Switched 3D -> 2D lidar (real request, see
            # docs/superpowers/specs/2026-08-11-2d-lidar-switch-design.md):
            # lidar_sensor's vertical scan is now a single flat ring
            # (model.sdf.tracked), so the occupancy grid is built directly
            # by ray-casting that flat scan, not from 3D-derived normal
            # segmentation -- Grid/3D false matches.
            'Grid/3D': 'false',
            # Explicit, not relying on RTAB-Map's own default (0 = uncapped,
            # tracks the sensor's own reported max range) -- matches
            # lidar_sensor's own max range in model.sdf.tracked (doubled
            # 30.0 -> 60.0 alongside cave_world's 2x mesh scale), spelled
            # out here so this doesn't silently drift out of sync if either
            # one changes again.
            'Grid/RangeMax': '60.0',
            # NOTE: Grid/MaxGroundAngle (was set to 65deg here to fix a real
            # 3D normal-based ground-misclassification bug -- see commit
            # history for that investigation) is REMOVED as part of the 3D
            # -> 2D lidar switch: with Grid/3D false, occupancy comes from
            # ray-casting the flat scan directly, not normal segmentation,
            # so that failure mode no longer exists -- nothing to configure
            # here for it.
            #
            # Switched 3D -> 2D lidar (real request): point-to-point ICP
            # instead of point-to-plane -- a flat single-ring scan can't
            # provide the reliable surface normals point-to-plane needs;
            # point-to-point is RTAB-Map's standard mode for 2D-lidar SLAM.
            'Icp/PointToPlane': 'false',
            'Icp/VoxelSize': '0.1',
            # Task 11 fix round 1: RTAB-Map's WM stayed at 1 forever despite real,
            # confirmed vehicle travel (15+ m) with healthy icp_odometry ratios
            # (~0.27). Root cause confirmed live via --udebug: every candidate
            # frame was rejected with "Ignoring location N because the
            # displacement is too small! (d=0.100000 a=0.100000)" -- RTAB-Map's
            # own RGBD/LinearUpdate and RGBD/AngularUpdate defaults (both 0.1),
            # which skip processing frames below that per-frame linear/angular
            # displacement. Not Mem/RehearsalSimilarity (rehearsal's own debug
            # line showed merged=0 -- it was NOT rejecting frames as too similar
            # -- the displacement gate that runs after rehearsal was the real
            # blocker). Per RTAB-Map's own documented semantics, 0 explicitly
            # disables this skip (always process/consider each frame).
            'RGBD/LinearUpdate': '0.0',
            'RGBD/AngularUpdate': '0.0',
            # Default Rtabmap/DetectionRate is 1Hz (matches the "Rate=1.00s"
            # seen live in rtabmap's own log line) -- that's also how often
            # it publishes the map->odom TF. Lidar/odom messages land between
            # those publishes, so rviz/costmap TF lookups at those in-between
            # stamps intermittently hit "frame does not exist" or
            # extrapolation errors even once SLAM is healthy. Raised to
            # 10Hz (further raised from an earlier 5Hz fix) -- matches
            # lidar_sensor's own real update_rate (model.sdf.tracked, also
            # 10Hz), so RTAB-Map now processes every incoming scan rather
            # than every other one; 0 (uncapped) was considered but there's
            # no new data to process faster than the sensor itself produces.
            'Rtabmap/DetectionRate': '10.0',
        }],
        remappings=[
            ('rgb/image', '/camera/color/image_raw'),
            ('rgb/camera_info', '/camera/color/camera_info'),
            ('depth/image', '/camera/depth/image_raw'),
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
    # rationale (ported from a prior project vehicle's real, working config,
    # same no-/scan situation).
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'params_file': os.path.join(get_package_share_directory('cavex_tracked_vehicle'), 'config', 'tracked_vehicle_nav2_params.yaml'),
        }.items(),
    )

    # Frontier exploration: drives autonomous /cmd_vel via Nav2's costmap.
    # Param names verified live against the vendored source (Task 12):
    # explore/src/costmap_client.cpp declares costmap_topic,
    # costmap_updates_topic, robot_base_frame; explore/src/explore.cpp
    # declares planner_frequency, progress_timeout, visualize -- all match
    # the brief's example verbatim, no renames needed.
    #
    # Real, live-diagnosed premature-stop bug (found after the /clock and
    # bootstrap_nudge-duration fixes above, once ArduPilot could finally
    # stay armed long enough to expose it): explore_node used to start
    # immediately and connect to Nav2's move_base action server as soon as
    # it comes up -- which only requires `odom` (Task 11's
    # initial_transform_timeout fix), NOT the `map` frame. explore_lite's
    # own costmap_client_ needs the full `map` -> base_link TF chain
    # (published by RTAB-Map only once it has processed real map data) to
    # compute the robot's pose for frontier search. Confirmed live: at the
    # moment explore_node connected and called its first makePlan(),
    # `map` didn't exist yet ("map" passed to lookupTransform argument
    # target_frame does not exist), frontier_search found zero frontiers
    # from a garbage/failed pose, and explore.cpp's stop(true) is
    # PERMANENT -- no automatic retry once a "No frontiers found" stop
    # fires (confirmed by reading explore.cpp: stop(finished_exploring=true)
    # sets a state flag with no self-resume path). This produced a
    # convincing-looking-but-false "exploration complete" after only ~13
    # real seconds of connection, with the costmap still ~98% unknown.
    # Fix: delay explore_node's own start until after bootstrap_nudge's
    # real driving window has completed and RTAB-Map has had time to
    # publish a stable `map` frame from that real motion.
    #
    # This fix genuinely works -- re-verified live in a later session:
    # explore_node now connects AFTER a real `map` frame exists, no more
    # lookupTransform failure. But "No frontiers found, stopping" still
    # reproduced anyway, from a SECOND, independent real bug: the
    # inflation_layer's own inflation_radius (see
    # tracked_vehicle_nav2_params.yaml's own comment on that parameter for
    # the full data) was consuming nearly all of the map's real free
    # space, leaving no free cells bordering unknown ones for
    # frontier_search to find -- fixed there, not here. Both bugs produced
    # the identical-looking symptom ("No frontiers found, stopping" after
    # a normal-looking connection), which is why the first fix looked
    # complete at the time it landed but didn't actually resolve
    # autonomous exploration on its own.
    explore_node = TimerAction(
        period=320.0,
        actions=[Node(
            package='explore_lite',
            executable='explore',
            name='explore_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'costmap_topic': 'global_costmap/costmap',
                'costmap_updates_topic': 'global_costmap/costmap_updates',
                'visualize': True,
                'planner_frequency': 0.5,
                'progress_timeout': 30.0,
                'robot_base_frame': frame_id,
            }],
        )],
    )

    # Real request: "implement dead end algorithm: backtrack until another
    # opening or corridor is found." Nav2's own stock BackUp recovery (see
    # tracked_vehicle_nav_to_pose_bt.xml) only backs up a fixed 0.60m per
    # attempt -- real, but not enough for an actual dead-end tunnel many
    # meters deep. This node handles that: detects genuine no-progress stalls
    # (not just idle), cancels the active Nav2 goal, and reverses along its
    # own recorded trail until the global costmap shows a real lateral
    # opening. Safe to start immediately alongside everything else -- its own
    # stuck-detection requires a recent nonzero /cmd_vel command, so it can't
    # fire before real driving (bootstrap_nudge or explore_lite) begins. See
    # dead_end_backtrack_node.py's own module docstring for the full design.
    dead_end_backtrack_node = Node(
        package='cavex_tracked_vehicle',
        executable='dead_end_backtrack_node.py',
        name='dead_end_backtrack_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Task 12: real, structural cold-start deadlock discovered live -- not a
    # code bug, a known characteristic of pure ICP scan-matching odometry.
    # From a stationary spawn, icp_odometry cannot compute its first
    # keyframe at all: confirmed live via its own console output,
    # "OdometryF2M.cpp: Scan complexity too low (0.000301) to init first
    # keyframe" -- this cave section's geometry, seen from one fixed
    # viewpoint, doesn't have enough 3D structure for ICP to bootstrap from.
    # Consequently odom->base_link never appears, Nav2's costmap never
    # activates, and explore_lite (which waits on the costmap) never starts
    # -- a genuine chicken-and-egg deadlock with no manual driving at all.
    # ICP odometry only starts succeeding once the vehicle has moved and
    # observed the same geometry from more than one viewpoint (confirmed:
    # this is exactly why every earlier task's own verification always
    # published a real /cmd_vel before checking icp_odometry's output).
    # Fix: a short, one-time bootstrap nudge, timed to fire after the stack
    # is up but well before it, publishes a burst of /cmd_vel messages then
    # stops -- just enough real motion for icp_odometry to observe parallax
    # and initialize. This is NOT ongoing manual control: it exits on its
    # own after a fixed real-time duration, and explore_lite (already
    # running, just waiting on the costmap) takes over all driving once
    # Nav2 comes alive as a direct result of this nudge -- same role a
    # human giving a stalled real robot a manual push-start would play, not
    # a substitute for autonomous exploration.
    #
    # Duration real, live-verified, and RTF-aware: this environment's
    # real_time_factor under the full stack is consistently very slow and
    # variable (confirmed repeatedly across Tasks 7/10/11/12, ranging
    # ~0.017-0.03 typical, occasionally higher) -- `ros2 topic pub -r 5`'s
    # rate is wall-clock, not sim-time, so what actually matters for
    # icp_odometry's bootstrap is real (wall-clock) nudge duration divided
    # by RTF, not message count. Two earlier, shorter versions of this
    # nudge (first 10 messages/2s, then 150 messages/30s) were both
    # live-tested and found genuinely insufficient at this environment's
    # real RTF: icp_odometry's registration ratio stayed at 0.000000 the
    # entire time in both cases despite the nudge completing and ArduPilot
    # driving correctly (confirmed via real pose deltas) -- confirming the
    # earlier "150 messages should be comfortably more than enough" comment
    # was wrong: it reasoned from sim-time-equivalent distance, not
    # accounting for how little sim-time 30 *real* seconds actually
    # advances at RTF~0.03 (roughly 1 sim-second). 1500 messages at 5Hz
    # (300 real seconds / 5 minutes) is calibrated against this
    # environment's real, observed worst-case RTF (~0.017) to reliably
    # cover the ~15 sim-seconds of continuous driving this project's own
    # manual verification runs needed to get icp_odometry's ratio above
    # its registration threshold in this same cave section.
    bootstrap_nudge = TimerAction(
        period=5.0,
        actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', '5', '--times', '1500',
                 '/cmd_vel', 'geometry_msgs/msg/Twist',
                 '{linear: {x: 0.3}}'],
            output='screen',
        )],
    )

    # Task 13: ATE measurement harness. tracked_vehicle_ground_truth_odom.py
    # republishes this vehicle's real ground-truth pose (see that file's own
    # header comment for why it subscribes via gz-transport directly rather
    # than the launch file's already-bridged, unlabeled world PoseArray) as
    # /odom_ground_truth; ate_evaluator_node.py (existing, unmodified, from
    # cavex_slam_nav -- cross-package reuse) compares it against
    # /cavex/slam/odom (slam_pose_publisher above) on each
    # /cavex/eval/finish_run trigger.
    ate_evaluator = Node(
        package='cavex_slam_nav',
        executable='ate_evaluator_node.py',
        name='ate_evaluator_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'ground_truth_topic': '/odom_ground_truth',
            'estimate_topic': '/cavex/slam/odom',
        }],
    )
    tracked_vehicle_ground_truth_odom = Node(
        package='cavex_tracked_vehicle',
        executable='tracked_vehicle_ground_truth_odom.py',
        name='tracked_vehicle_ground_truth_odom',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Task 4 (SIC-SLAM RGB-D + lidar fusion): colorizes the lidar cloud via
    # the live camera projection, clusters it into instances, tracks ids
    # frame-to-frame, and publishes /sic_slam/instances + /sic_slam/colored_points
    # for Task 5. Needs map -> lidar_link TF (rtabmap above) and camera_link TF
    # (camera_static_tf above), so it's safe to start alongside everything else --
    # it just skips frames until those are live, same pattern as icp_odometry.
    #
    # DISABLED (real request): rtabmap's own subscribe_depth/subscribe_rgb/
    # subscribe_scan_cloud (all True above) already fuse the RGB-D + lidar
    # data directly for real SLAM mapping -- sic_slam_node was always a
    # separate, additional consumer of those same raw topics for instance
    # clustering (dead_end_backtrack_node's survey-penalty feature), not
    # part of the core SLAM path, so disabling it doesn't change what
    # rtabmap sees or does. dead_end_backtrack_node degrades gracefully with
    # no /sic_slam/instances publisher (its own _instance_centroids stays
    # [], instance_penalty always returns 0 -- by design, see its own
    # module docstring). cavex_perception package/build left intact; only
    # the launch entry is removed, easy to re-enable.
    # sic_slam_node = Node(
    #     package='cavex_perception',
    #     executable='sic_slam_node',
    #     name='sic_slam_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    return LaunchDescription([
        lidar_static_tf,
        camera_static_tf,
        camera_optical_static_tf,
        icp_odometry,
        rtabmap,
        slam_pose_publisher,
        nav2_bringup_launch,
        explore_node,
        dead_end_backtrack_node,
        bootstrap_nudge,
        ate_evaluator,
        tracked_vehicle_ground_truth_odom,
    ])
