import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessIO
from launch.events.process import ShutdownProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # nav2:=false runs SLAM only (rtabmap + icp_odometry + foxglove) without
    # the Nav2 bringup, explore_lite, or dead_end_backtrack -- the ~12
    # lifecycle nodes + 2 costmaps are the bulk of the CPU load and aren't
    # needed just to build a map.
    nav2 = LaunchConfiguration('nav2', default='true')

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
    # image_geometry::PinholeCameraModel::project3dToPixel (instance_clustering_node.cpp)
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
    # Same lidar_link/camera_link gap (imu_link only exists in
    # model.sdf.tracked, not in cavex_tracked_vehicle.urdf's stub
    # robot_state_publisher actually publishes TF from) -- missed when the
    # lidar/camera static TFs above were added, causing rtabmap's
    # "getTransform() ... imu_link ... does not exist" lookup failures.
    # imu_link's own <joint>/<link> in the SDF carry no <pose> override, so
    # the real offset is identity, same as camera_optical_static_tf above.
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',
        output='screen',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
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
            # 2026-08-30: 2D single-ring lidar. Point-to-plane ICP needs 3D
            # surface normals from a non-collinear neighbourhood; a single
            # horizontal ring is collinear, so RTAB-Map reports "Scan
            # complexity 0.0" and never inits a keyframe (no odom -> no map
            # frame -> RViz/Nav2 see nothing). Point-to-point + Force3DoF
            # (a 2D scan can't observe z/roll/pitch) is the 2D-lidar config.
            'Icp/PointToPlane': 'false',
            'Reg/Force3DoF': 'true',
            'Icp/VoxelSize': '0.1',
            # Real, live-diagnosed problem (2026-08-27, see bootstrap_nudge_node.py's
            # own module docstring for the full story): once icp_odometry loses
            # tracking it gets permanently stuck ("RegistrationIcp cannot do
            # registration with a null guess", ratio pinned at 0.0 forever) --
            # it has no prior guess transform left to seed a fresh registration
            # attempt, and default behavior never resets that state on its own.
            # This is RTAB-Map's own documented parameter for exactly this
            # situation: reset odometry's internal state (drop the stale guess
            # requirement) after this many CONSECUTIVE frames it couldn't
            # compute odometry for, so a later attempt can re-bootstrap from
            # scratch instead of staying wedged forever. Small value (a few
            # frames at this node's ~2-5Hz update rate, not many seconds) --
            # this is a genuine dead end being cleared, not a normal transient
            # to tolerate.
            'Odom/ResetCountdown': '3',
        }],
        remappings=[
            ('scan_cloud', '/lidar/points'),
            # icp_odometry auto-subscribes 'scan' too and refuses to run both;
            # the real /scan (LaserScan) is for reactive_controller_node only.
            ('scan', '_icp_scan_unused'),
        ],
    )

    # RTAB-Map SLAM node, lidar-cloud-only mode, remapped to this vehicle's
    # real sensor topics (Task 7).
    # 2026-08-30: RGBD camera removed from model.sdf.tracked and the lidar is
    # 2D now. subscribe_depth/subscribe_rgb dropped to False -- with them True,
    # RTAB-Map's sync callback waits forever for RGB+depth frames that never
    # publish and NO map is ever built. scan_cloud (the 360-pt 2D ring on
    # /lidar/points) is the only input now.
    rtabmap = Node(
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'subscribe_depth': False,
            'subscribe_rgb': False,
            'subscribe_scan': False,
            'subscribe_scan_cloud': True,
            'frame_id': frame_id,
            'qos_scan_cloud': 2,
            'Grid/FromDepth': 'false',
            'Grid/3D': 'false',   # 2026-08-30: 2D lidar -> 2D occupancy grid
            # Explicit, not relying on RTAB-Map's own default (0 = uncapped,
            # tracks the sensor's own reported max range) -- matches
            # lidar_sensor's own max range in model.sdf.tracked (doubled
            # 30.0 -> 60.0 alongside cave_world's 2x mesh scale), spelled
            # out here so this doesn't silently drift out of sync if either
            # one changes again.
            'Grid/RangeMax': '60.0',
            # REAL BUG FOUND AND FIXED: ground/obstacle classification in
            # RTAB-Map's occupancy grid relies entirely on surface-normal
            # angle (Grid/NormalsSegmentation, on by default) since
            # Grid/MaxGroundHeight/MinGroundHeight are both 0.0 (disabled,
            # RTAB-Map's own default -- confirmed live via `ros2 param get`,
            # not assumed) -- no height-based fallback at all. Default
            # Grid/MaxGroundAngle=45deg, live-confirmed via the same
            # command, was tight enough that this project's real, genuinely
            # undulating vendored cave floor mesh (already documented
            # elsewhere in this file/README as real height variance of
            # ~0.6m across sampled slices, not a flat lab floor) locally
            # exceeded it -- misclassifying real floor points near the
            # vehicle as obstacles instead of ground. Confirmed live by
            # comparing /map, local_costmap, and raw /lidar/points at the
            # exact moment behavior_server's Spin/BackUp recovery aborted
            # with "Collision Ahead": /map and local_costmap both showed
            # lethal cost at/near the robot's own position while the live
            # lidar showed nothing within 3m in any direction -- this
            # deadlocked Nav2's recovery loop indefinitely (bt_navigator
            # kept re-issuing goals that got preempted, every recovery
            # attempt aborted instantly, no escape). Raised to 65deg --
            # generous margin above what a bumpy-but-navigable floor should
            # locally produce, while staying well clear of a real wall's
            # ~90deg normal, so genuine obstacles are still classified
            # correctly. Needs live re-verification that the false-lethal
            # pattern is actually gone, not just that this value looks
            # reasonable on paper.
            'Grid/MaxGroundAngle': '65',
            # 2026-08-31: lidar-only (no camera). RTAB-Map's default
            # Reg/Strategy=0 is VISUAL feature registration -- every
            # loop-closure / proximity attempt then logs "Missing visual
            # features ... Transform cannot be estimated" and finds nothing.
            # Strategy 1 = ICP (uses the scan cloud). Also drop odom visual
            # features and keypoint extraction -- there are no images.
            'Reg/Strategy': '1',
            'Mem/UseOdomFeatures': 'false',
            'Kp/MaxFeatures': '-1',
            'Icp/PointToPlane': 'false',   # 2026-08-30: 2D single-ring lidar
            'Reg/Force3DoF': 'true',
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
            # RTAB-Map processes a frame (and publishes map->odom) at this
            # rate. Was 10Hz to match a 10Hz lidar; 2026-08-30 both dropped
            # (lidar update_rate 10->5, this 10->3) to claw back RTF -- the
            # map graph doesn't need 10Hz keyframes and the RTAB-Map= per-
            # frame cost was ~50% of a core. icp_odometry still publishes
            # odom->base_link at the full scan rate, so the odom TF stays
            # smooth; only map->odom drops to 3Hz, which "latest" TF lookups
            # (reactive_controller_node) tolerate fine. Costmap-based lookups
            # in the full Nav2 path are more sensitive -- bump back toward 5
            # if they get noisy.
            'Rtabmap/DetectionRate': '3.0',
        }],
        remappings=[
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
        condition=IfCondition(nav2),
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
    # Real, live-diagnosed problem (2026-08-27), found once bootstrap_nudge_node
    # (below) became adaptive: this fixed 320s delay was originally sized to
    # match the OLD fixed-300s nudge (wait for that nudge's driving window to
    # finish, then a little more). Now that the nudge stops in seconds on a
    # normal run, this same fixed 320s delay instead leaves the vehicle sitting
    # completely still for most of that window -- and standing still that long
    # turned out to be enough for icp_odometry to LOSE the very lock the nudge
    # just gave it (confirmed live: icp_inliers_ratio held a stable 0.966 for
    # ~380 consecutive readings while idle, then dropped to lost=true around
    # the 700s mark), breaking odom->base_link TF and producing exactly the
    # "map and base_link are not part of the same tree" error explore_node
    # itself then logs when it tries to start. Not a race, not a costmap
    # timing issue -- the delay itself was the bug once the nudge got fast.
    # Fix: start explore_node the moment bootstrap_nudge_node actually exits
    # (see RegisterEventHandler below) instead of guessing a fixed delay that
    # no longer matches how long the nudge really runs.
    # Real, live-diagnosed THIRD variant of this same problem: even with
    # EXPLORE_START_GRACE_S above, explore_node's first search can still land
    # on a costmap snapshot too sparse to have any frontier YET (the
    # bootstrap nudge now drives so briefly that the initially-mapped bubble
    # is tiny) -- and explore.cpp's own "No frontiers found, stopping." is a
    # PERMANENT stop with no self-retry (this file's older comment on
    # explore_node above already found this reading the vendored source
    # directly). Confirmed live, twice: costmap genuinely had 900+ free cells
    # bordering unknown space within seconds of explore_node giving up, and
    # simply restarting the process picked up a real goal immediately.
    # Bounded auto-retry below reproduces that exact manual fix automatically
    # instead of needing a human to notice and restart it: watches
    # explore_node's own stdout for that exact message and respawns it after
    # another grace period, capped at MAX_EXPLORE_RETRIES so a genuine
    # "actually done exploring" stop (no more real frontiers anywhere) still
    # ends up stopped for good, same as before this existed -- it just no
    # longer gets stuck on a false stop from an immature costmap.
    MAX_EXPLORE_RETRIES = 2
    EXPLORE_RETRY_DELAY_S = 20.0

    def _make_explore_node():
        return Node(
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
                # Real, live-diagnosed problem (2026-08-27): explore_lite's own
                # frontier centroid can land almost on the robot when a
                # frontier blob wraps around it (lidar self-occlusion merged
                # with real nearby unexplored space) -- confirmed live,
                # goals sent ~0.16m from the robot, well inside Nav2's
                # xy_goal_tolerance (0.25m), so every goal "completed"
                # instantly with zero real driving. Patched explore_lite
                # itself (frontier_search.cpp, vendored in this workspace)
                # to exclude cells within this radius from a frontier's
                # centroid/size calculation while still traversing them for
                # connectivity -- see that file's own comment for the full
                # mechanism. 0.6 = 2x this vehicle's own robot_radius (0.3,
                # tracked_vehicle_nav2_params.yaml), covering its real
                # footprint plus margin without swallowing genuinely close
                # real frontiers.
                'robot_exclusion_radius': 0.6,
            }],
        )

    def _spawn_explore_node(retry_count=0):
        node = _make_explore_node()
        entities = [node]
        if retry_count < MAX_EXPLORE_RETRIES:
            # Real bug caught testing this live: RCLCPP_WARN (what explore.cpp
            # uses for "No frontiers found, stopping.") writes to stderr by
            # rclcpp's own default, not stdout -- an on_stdout-only handler here
            # never saw it and the retry silently never fired. Watch both.
            #
            # Second real bug caught testing this live, right after fixing the
            # first: a retry here just STARTS a new explore_node Node action --
            # ros2 launch does not stop the old one for you, so each retry left
            # the previous, already-given-up instance running forever. Confirmed
            # live: three separate explore_node processes ended up alive at
            # once, all still subscribed to the same costmap/move_base topics,
            # extra CPU load on top of the actual problem. Explicitly shut the
            # stale instance down before spawning its replacement.
            def _on_io(event, retry_count=retry_count):
                text = bytes(event.text).decode(errors='replace')
                if 'No frontiers found, stopping.' not in text:
                    return None
                return [
                    EmitEvent(event=ShutdownProcess(
                        process_matcher=lambda action, n=node: action is n)),
                    TimerAction(
                        period=EXPLORE_RETRY_DELAY_S,
                        actions=_spawn_explore_node(retry_count + 1),
                    ),
                ]
            entities.append(RegisterEventHandler(OnProcessIO(
                target_action=node, on_stdout=_on_io, on_stderr=_on_io)))
        return entities

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
        condition=IfCondition(nav2),
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
    # advances at RTF~0.03 (roughly 1 sim-second). A fixed 1500-message/300s
    # burst (calibrated against this environment's real, observed
    # worst-case RTF ~0.017) reliably bootstrapped icp_odometry, but pinned
    # EVERY launch to that worst case even on a good-RTF run, and kept
    # driving blind after bootstrap had already succeeded, competing with
    # any other /cmd_vel publisher for however much of the 300s remained.
    #
    # Smaller nudge: bootstrap_nudge_node.py drives at the same linear.x=0.3
    # but watches icp_odometry's own /odom_info (rtabmap_msgs/OdomInfo)
    # live and stops the instant icp_inliers_ratio reports real bootstrap
    # success, instead of always waiting out the worst case. 300s is kept
    # as MAX_DURATION_S inside that node -- a safety ceiling, not a target
    # -- so a genuinely bad-RTF run is no worse off than the old fixed
    # nudge; a normal run now stops far sooner.
    # Real, live-diagnosed problem: autonomous /cmd_vel (Nav2's collision_monitor
    # output, and dead_end_backtrack_node's own direct publishes) was reaching
    # cmd_vel_to_ardupilot.py correctly, but ArduPilot's Rover SITL never arms in
    # this environment (stuck repeating "PreArm: AHRS: not using configured AHRS
    # type" -- confirmed live in the SITL console) -- so every autonomous command
    # was silently swallowed and the vehicle never physically moved, while
    # manual_gui_bridge's own gz-transport bypass (below) worked fine. Gives
    # autonomous driving the same working bypass; see cmd_vel_gz_bridge.py's own
    # module docstring for the full mechanism, including how it yields to manual
    # control instead of fighting it for the same gz-transport topic.
    cmd_vel_gz_bridge = Node(
        package='cavex_tracked_vehicle',
        executable='cmd_vel_gz_bridge.py',
        name='cmd_vel_gz_bridge',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Real request 2026-08-27: an RViz-equivalent 3D view (TF, /map, costmap,
    # sensors) in the browser, via Foxglove -- exposes this node's real ROS2
    # graph over a websocket (default port 8765) that Foxglove's web client
    # connects to directly (web_viewer/index.html links to it, opened in its
    # own tab -- app.foxglove.dev requires an account sign-in now, so that
    # link points at a self-hosted client instead). No topic allow/deny-list
    # here: exposes everything on the graph, same as what RViz itself can
    # already subscribe to locally.
    #
    # Real, live-diagnosed version archaeology: this apt package (3.4.1,
    # Foxglove's newer Rust-SDK rewrite, protocol "foxglove.sdk.v1") is NOT
    # compatible with the last fully open-source Foxglove Studio release
    # (protocol "foxglove.websocket.v1", confirmed by reading both sides'
    # source directly) -- a self-hosted build of that old client fails to
    # even connect to this bridge. Building an old (0.8.5) foxglove_bridge
    # from source to match it DID connect, but then failed to decode this
    # old client's messages ("Unsupported encoding cdr" -- that release
    # predates CDR-decode support in the frontend). Real fix: Flora
    # (github.com/flora-suite/flora), an ACTIVELY MAINTAINED open-source fork
    # continued after Foxglove closed-sourced Studio -- confirmed live, this
    # exact apt package connects to it cleanly, protocol and encoding both
    # match. Self-host it (needs a real sudo password this launch file can't
    # supply non-interactively, so it's a one-time manual step, not an
    # automated launch action -- persists across launches, --restart
    # unless-stopped):
    #   sudo dockerd > /tmp/dockerd.log 2>&1 &
    #   git clone --depth 1 https://github.com/flora-suite/flora /tmp/flora
    #   cd /tmp/flora && sudo docker build -t flora .
    #   sudo docker run -d --name flora --restart unless-stopped \
    #     -p 8766:8080 flora
    # Then open http://localhost:8766 (or click "open Foxglove" in
    # web_viewer/index.html) and add a 3D panel (same idiom as RViz's own
    # "Add Display") -- an empty dashboard on first connect is normal, no
    # panel is added by default.
    # 2026-08-30: foxglove_bridge is NOT autostarted any more. With no client
    # connected it still serializes every topic on the graph (~60% of a core,
    # measured) -- pure waste on a launch that mostly runs headless. Start it
    # by hand only when actually opening Foxglove:
    #   ros2 run foxglove_bridge foxglove_bridge --ros-args -p use_sim_time:=true -p port:=8765 &
    # (also in the README). Can't "kill on idle" cleanly -- once it's dead new
    # clients can't connect without an inetd-style listener.

    bootstrap_nudge_node_action = Node(
        package='cavex_tracked_vehicle',
        executable='bootstrap_nudge_node.py',
        name='bootstrap_nudge_node',
        output='screen',
        # watchdog only with Nav2. Under nav2:=false the reactive_controller
        # owns recovery; the watchdog's "resume driving on icp loss" fights it
        # (drives forward into a sealed dead end while it tries to back out).
        parameters=[{'use_sim_time': use_sim_time,
                     'watchdog': ParameterValue(nav2, value_type=bool)}],
    )
    bootstrap_nudge = TimerAction(
        period=5.0,
        actions=[bootstrap_nudge_node_action],
    )

    # explore_node starts shortly after bootstrap_nudge_node's INITIAL
    # bootstrap completes (whether that's a few seconds in on a normal run or
    # the full 300s ceiling on a bad-RTF one) instead of a fixed delay sized
    # for the old, always-300s nudge -- see explore_node's own comment above
    # for that half of the bug.
    #
    # Real, live-diagnosed SECOND half of the same bug (found when the naive
    # "start explore_node the instant the nudge exits" version was tried
    # first): the old 320s delay wasn't only there to outlast the nudge, it
    # was ALSO covering a separate, already-documented race -- explore_node
    # connects to Nav2's move_base server and can run its first frontier
    # search before RTAB-Map has published a stable `map` frame from the
    # nudge's own real motion (see this file's much older comment on
    # explore_node above, "Real, live-diagnosed premature-stop bug", for the
    # original discovery of that race). With the nudge now bootstrapping in
    # ~1-2s, starting explore_node with zero delay reproduces that exact race
    # again -- confirmed live: "No frontiers found, stopping" fired within
    # ~40s of the nudge exiting, while the costmap genuinely had free cells
    # bordering unknown space moments later. EXPLORE_START_GRACE_S is a
    # small, fixed cushion for RTAB-Map to publish that stable map frame --
    # unlike the old 320s, this is on top of an adaptive nudge exit, not
    # instead of one, so it stays small regardless of this environment's RTF.
    #
    # Real, live-diagnosed THIRD half of this same story: bootstrap_nudge_node
    # no longer exits after its initial bootstrap -- it stays alive as a
    # watchdog for the whole launch, re-driving whenever icp_odometry loses
    # lock again later (see bootstrap_nudge_node.py's own module docstring for
    # why that's necessary). Since there's no more process exit to key off,
    # watch its stdout for the distinct "initial bootstrap complete" line
    # instead (same OnProcessIO technique already used for explore_node's own
    # auto-retry below).
    EXPLORE_START_GRACE_S = 20.0

    def _on_bootstrap_stdout(event):
        text = bytes(event.text).decode(errors='replace')
        if 'initial bootstrap complete' not in text:
            return None
        return [TimerAction(period=EXPLORE_START_GRACE_S, actions=_spawn_explore_node(0))]

    start_explore_after_nudge = RegisterEventHandler(
        event_handler=OnProcessIO(
            target_action=bootstrap_nudge_node_action,
            on_stdout=_on_bootstrap_stdout,
            on_stderr=_on_bootstrap_stdout,
        ),
        condition=IfCondition(nav2),
    )

    # Task 13: ATE measurement harness. /odom_ground_truth is published by
    # tracked_vehicle_ground_truth_odom.py, now started in
    # gazebo_tracked_vehicle.launch.py (every control node there depends on
    # it); ate_evaluator_node.py (existing, unmodified, from cavex_slam_nav
    # -- cross-package reuse) compares it against /cavex/slam/odom
    # (slam_pose_publisher above) on each /cavex/eval/finish_run trigger.
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

    # Task 4 (Instance-clustering RGB-D + lidar fusion): colorizes the lidar cloud via
    # the live camera projection, clusters it into instances, tracks ids
    # frame-to-frame, and publishes /instance_clustering/instances + /instance_clustering/colored_points
    # for Task 5. Needs map -> lidar_link TF (rtabmap above) and camera_link TF
    # (camera_static_tf above), so it's safe to start alongside everything else --
    # it just skips frames until those are live, same pattern as icp_odometry.
    #
    # DISABLED (real request): rtabmap's own subscribe_depth/subscribe_rgb/
    # subscribe_scan_cloud (all True above) already fuse the RGB-D + lidar
    # data directly for real SLAM mapping -- instance_clustering_node was always a
    # separate, additional consumer of those same raw topics for instance
    # clustering (dead_end_backtrack_node's survey-penalty feature), not
    # part of the core SLAM path, so disabling it doesn't change what
    # rtabmap sees or does. dead_end_backtrack_node degrades gracefully with
    # no /instance_clustering/instances publisher (its own _instance_centroids stays
    # [], instance_penalty always returns 0 -- by design, see its own
    # module docstring). cavex_perception package/build left intact; only
    # the launch entry is removed, easy to re-enable.
    # instance_clustering_node = Node(
    #     package='cavex_perception',
    #     executable='instance_clustering_node',
    #     name='instance_clustering_node',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    # )

    # nav2:=false alternative -- Tier 2 reactive explorer, no Nav2/costmaps.
    # frontier_explorer_node picks goals off /map; reactive_controller_node
    # follow-the-gaps on /scan -> /cmd_vel. Started after bootstrap_nudge has
    # had a chance to give icp_odometry its first lock (nudge starts at 5 s).
    reactive_explorer = TimerAction(
        period=25.0,
        actions=[
            Node(package='cavex_tracked_vehicle', executable='frontier_explorer_node.py',
                 name='frontier_explorer_node', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}],
                 condition=UnlessCondition(nav2)),
            Node(package='cavex_tracked_vehicle', executable='reactive_controller_node.py',
                 name='reactive_controller_node', output='screen',
                 parameters=[{'use_sim_time': use_sim_time}],
                 condition=UnlessCondition(nav2)),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav2', default_value='true',
            description='true: full Nav2 + explore_lite. false: Tier 2 reactive '
                        'explorer (frontier_explorer + follow-the-gap), no costmaps.'),
        lidar_static_tf,
        camera_static_tf,
        camera_optical_static_tf,
        imu_static_tf,
        icp_odometry,
        rtabmap,
        slam_pose_publisher,
        nav2_bringup_launch,
        start_explore_after_nudge,
        dead_end_backtrack_node,
        reactive_explorer,
        cmd_vel_gz_bridge,
        bootstrap_nudge,
        ate_evaluator,
    ])
