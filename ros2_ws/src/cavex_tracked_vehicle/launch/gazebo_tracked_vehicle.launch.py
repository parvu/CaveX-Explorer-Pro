import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# This project's existing convention (see ardupilot_gazebo_env.sh, and
# model.sdf.tracked's own gz_ros2_control <parameters> comment pre-Task-7): a
# worktree-absolute path, not portable across machines/CI. Kept as one
# constant here rather than baked into multiple strings.
WORKTREE_ROOT = '/home/parvu/CaveX-Explorer-Pro/.worktrees/cavex-tracked-blueboat-ardupilot'

# Task 4/5/6's real, confirmed spawned model name -- NOT cavex_tracked_vehicle
# (the brief's Interfaces section names the package, not the spawned model;
# using the package name for model-scoped topics was the brief's own mistake,
# corrected here and in the verification commands below).
VEHICLE_MODEL_NAME = 'cavex_tracked_blueboat'


def generate_launch_description():
    pkg_cavex_tracked = get_package_share_directory('cavex_tracked_vehicle')
    pkg_cavex_slam = get_package_share_directory('cavex_slam_nav')

    world_file = os.path.join(pkg_cavex_slam, 'worlds', 'cavex_world.world')
    urdf_stub_file = os.path.join(pkg_cavex_tracked, 'urdf', 'cavex_tracked_vehicle.urdf')
    ros2_control_yaml = os.path.join(pkg_cavex_tracked, 'config', 'cavex_tracked_vehicle_ros2_control.yaml')
    track_cmd_vel_bridge_yaml = os.path.join(pkg_cavex_tracked, 'config', 'track_cmd_vel_bridge.yaml')
    sdf_template_file = os.path.join(pkg_cavex_tracked, 'models', 'blueboat', 'model.sdf.tracked')

    # model.sdf.tracked's gz_ros2_control <parameters> tag can't use xacro's
    # $(find pkg) (this is native SDF, no xacro preprocessing step -- Task 5
    # confirmed the literal, unexpanded string aborts gz sim at model load),
    # so the checked-in file ships a __ROS2_CONTROL_PARAMS_PATH__ placeholder
    # token instead of a worktree-absolute literal. Substitute a real
    # FindPackageShare-resolved path here at launch time and spawn the
    # generated copy -- not the checked-in template -- so the template stays
    # portable across machines/worktrees/CI. A plain string .replace() is the
    # right amount of code for one token; no templating library needed.
    with open(sdf_template_file) as f:
        sdf_content = f.read().replace('__ROS2_CONTROL_PARAMS_PATH__', ros2_control_yaml)
    generated_sdf_file = '/tmp/cavex_tracked_blueboat.generated.sdf'
    with open(generated_sdf_file, 'w') as f:
        f.write(sdf_content)

    # ardupilot_gazebo_env.sh's two env vars, set here as SetEnvironmentVariable
    # actions so they reach the `gz sim` subprocess the IncludeLaunchDescription
    # below launches (sourcing the .sh file before `ros2 launch` runs would
    # only affect this launch-file process's own env, not necessarily every
    # subprocess it spawns via `ros2 launch`'s process-execution machinery --
    # setting them as launch actions is the reliable path). Also prepends
    # cavex_tracked_vehicle's installed `models` dir to GZ_SIM_RESOURCE_PATH --
    # Task 5's report found this exact directory (not its parent) is required
    # for model.sdf.tracked's `models://blueboat/...` mesh URIs to resolve.
    #
    # Task 8 addition: pkg_cavex_slam's installed `models` dir, real, live
    # necessary fix found running this exact launch file after vendoring the
    # cave mesh -- cavex_world.world's new `<include><uri>model://cave_world`
    # failed with "Unable to find uri[model://cave_world]" even though
    # ardupilot_gazebo_env.sh (sourced manually) already covers this path,
    # because this launch file builds its own GZ_SIM_RESOURCE_PATH from
    # scratch (see comment above) rather than inheriting the sourced shell's
    # env for the `gz sim` subprocess -- this line was simply missing before
    # cave_world existed for it to need to resolve.
    set_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(WORKTREE_ROOT, 'ardupilot_gazebo', 'build') + ':' +
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_cavex_tracked, 'models') + ':' +
        os.path.join(pkg_cavex_slam, 'models') + ':' +
        os.path.join(WORKTREE_ROOT, 'ardupilot_gazebo', 'models') + ':' +
        os.path.join(WORKTREE_ROOT, 'ardupilot_gazebo', 'worlds') + ':' +
        os.environ.get('GZ_SIM_RESOURCE_PATH', ''))

    # Structure copied from cavex_slam_nav/launch/gazebo_walker.launch.py
    # (real, proven pattern): gz_sim server-only, same world (dry-cave section
    # has room at this spawn point; Task 8 replaces the placeholder geometry).
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-s -r {world_file}'}.items(),
    )

    # gz_ros2_control's real, reproduced behavior (Task 5 finding, contradicts
    # gz_ros2_control's own upstream docs): GazeboSimROS2ControlPlugin.Configure()
    # still calls controller_manager::ControllerManager::robot_description_callback,
    # which blocks forever waiting for an external actor to publish a URDF (not
    # the SDF) with matching <ros2_control> joint/interface names on
    # /robot_description, transient-local -- even though model.sdf.tracked
    # already has its own SDF-embedded <ros2_control> block. Without this,
    # joint_state_broadcaster/track_retract_controller never initialize and the
    # retraction joints are silently never actuatable. This stub URDF (declares
    # only base_link + the two retraction joints + the matching <ros2_control>
    # block -- not a physical replica of the real hull) exists solely to
    # satisfy that wait; it is never spawned into the simulation.
    with open(urdf_stub_file) as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    # Spawns the real vehicle: native SDF (the generated/templated copy of
    # model.sdf.tracked, per the path fix above), not the robot_state_publisher
    # URDF stub above (unrelated -- that stub is launch-time-only).
    #
    # Task 8 update: cavex_world.world's placeholder "dry_cave" box (the
    # z=2.5-on-top-of-a-solid-box spawn point Task 7 recorded, see
    # task-7-report.md) is gone, replaced by the real vendored
    # LTU-RAI/gazebo_cave_world mesh. Re-measured live with Task 8's
    # probe-drop test (a 0.5m box dropped from z=12 at x=-60, y=0): it
    # settled at z~=0.25 (box half-height, i.e. the real cave floor is at
    # z~=0 here) and did NOT fall through or spawn embedded in rock -- the
    # first (x, y) tried already worked, no grid-walk fallback needed. x/y
    # moved from the placeholder's -30/0 to this real floor point's -60/0;
    # z = 0.25 (measured floor/box-rest height) + 0.5 clearance, per this
    # task's own convention.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-file', generated_sdf_file,
                   '-name', VEHICLE_MODEL_NAME,
                   '-x', '-60', '-y', '0', '-z', '0.75'],
        output='screen',
    )

    # gz_bridge: post-Task-7-review Finding 1 fix -- model.sdf.tracked now has
    # real lidar_link/camera_link <sensor> blocks (added alongside this launch
    # file change), so their gz-transport topics are bridged here too.
    #
    # Topic names below re-verified empirically (Step 2, and again live for
    # this fix) rather than assumed, per this project's established gz-sim
    # topic-naming gotcha (<topic> overrides / sensor scoping don't reliably
    # follow the naive convention). Specifically, gz-sim's gpu_lidar sensor
    # publishes a gz.msgs.LaserScan on the bare <topic> name (here,
    # /lidar/points) and auto-appends "/points" for the actual
    # gz.msgs.PointCloudPacked point cloud -- same real gotcha the abandoned
    # cavex-legged-walker-phase1 branch's gazebo_walker.launch.py already
    # documented and worked around; reproduced here rather than assumed.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # imu_sensor has no <topic> override in model.sdf.tracked, so it
            # uses gz-sim's default scoped sensor-topic convention.
            f'/world/cavex_world/model/{VEHICLE_MODEL_NAME}/link/imu_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu[gz.msgs.IMU',
            # lidar_sensor/camera_sensor both set an explicit <topic> override
            # in model.sdf.tracked, so these are the bare override names (plus
            # the lidar's real auto-appended "/points" suffix), not the
            # /world/.../sensor/... scoped convention imu_sensor uses above.
            '/lidar/points/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/camera/color@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # World-broadcast pose topic (always present in gz-sim8) -- real
            # ground truth, world-frame, all models. Pose_V -> PoseArray, same
            # pattern as the abandoned branch's ground-truth pose bridge;
            # ros2 topic echo/gz topic -e callers filter by this model's name
            # within the array. Deliberately NOT bridged/remapped to
            # /model/<name>/pose: model.sdf.tracked's own OdometryPublisher
            # plugin already owns that exact gz-transport topic name, and
            # (real, live-discovered during this task's Step 3) its output is
            # NOT reliable ground truth for this vehicle -- OdometryPublisher
            # dead-reckons from wheel/joint velocities, but left_track/
            # right_track have no rotating joint at all (TrackedVehicle drives
            # them as velocity-controlled links directly, joined via *_fixed
            # joints), so it has no real signal to integrate and reports
            # near-static output regardless of true motion. Bridging our
            # world-pose topic under that same ROS name would have collided
            # with and masked that broken topic's name. Task 3's finding that
            # "there is no [reliable] /model/<name>/pose topic" holds here too
            # -- use this bridged topic (kept at its real gz-transport name)
            # instead.
            '/world/cavex_world/pose/info@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        remappings=[
            (f'/world/cavex_world/model/{VEHICLE_MODEL_NAME}/link/imu_link/sensor/imu_sensor/imu', '/imu'),
            ('/lidar/points/points', '/lidar/points'),
            ('/camera/color', '/camera/color/image_raw'),
            ('/camera/camera_info', '/camera/color/camera_info'),
        ],
        output='screen',
    )

    # track_cmd_vel's ROS2<->gz-transport leg (Task 6): carries
    # track_cmd_vel_bridge.py's /track_cmd_vel (ROS2 Twist) onto the real
    # gz-transport topic /model/cavex_tracked_blueboat/cmd_vel (gz.msgs.Twist)
    # that TrackedVehicle actually subscribes to.
    track_cmd_vel_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='track_cmd_vel_gz_bridge',
        arguments=['--ros-args', '-p', f'config_file:={track_cmd_vel_bridge_yaml}'],
        output='screen',
    )

    # Real launch arg (confirmed via `ros2 launch ardupilot_sitl
    # sitl_dds_udp.launch.py --show-args`): 'command' selects the SITL binary,
    # 'ardurover' is a valid choice; 'model:=rover' + comma-joined
    # 'defaults:=rover.parm,dds_udp.parm' is ArduPilot's own multi-file
    # defaults syntax and is what actually turns DDS on (dds_udp.parm sets
    # DDS_ENABLE 1 / DDS_UDP_PORT 2019) -- same combination Task 3 verified
    # live end-to-end.
    pkg_ardupilot_sitl = get_package_share_directory('ardupilot_sitl')
    rover_defaults = os.path.join(pkg_ardupilot_sitl, 'config', 'default_params', 'rover.parm')
    dds_udp_defaults = os.path.join(pkg_ardupilot_sitl, 'config', 'default_params', 'dds_udp.parm')
    ardupilot_sitl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ardupilot_sitl, 'launch', 'sitl_dds_udp.launch.py')
        ),
        launch_arguments={
            'command': 'ardurover',
            'model': 'rover',
            'defaults': f'{rover_defaults},{dds_udp_defaults}',
            'synthetic_clock': 'False',
            'use_sim_time': 'False',
        }.items(),
    )

    cmd_vel_to_ardupilot = Node(
        package='cavex_tracked_vehicle',
        executable='cmd_vel_to_ardupilot.py',
        name='cmd_vel_to_ardupilot',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    track_cmd_vel_bridge_node = Node(
        package='cavex_tracked_vehicle',
        executable='track_cmd_vel_bridge.py',
        name='track_cmd_vel_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    track_retract_control = Node(
        package='cavex_tracked_vehicle',
        executable='track_retract_control.py',
        name='track_retract_control',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # ros2_control's controllers are declared to the controller_manager the
    # gz_ros2_control plugin starts on model spawn, but nothing loads/activates
    # them by itself (same real, empirically-confirmed requirement as the
    # abandoned branch's gazebo_walker.launch.py -- controller_manager just
    # sits there waiting otherwise). Real explicit spawners, sequenced via
    # OnProcessExit (spawn_entity's "create" process exits once the entity is
    # spawned, which is roughly when the controller_manager service becomes
    # available) -- the standard ros2_control launch idiom, copied directly.
    load_joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    load_track_retract_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['track_retract_controller'],
        output='screen',
    )

    return LaunchDescription([
        set_plugin_path,
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_entity,
        gz_bridge,
        track_cmd_vel_gz_bridge,
        ardupilot_sitl_launch,
        cmd_vel_to_ardupilot,
        track_cmd_vel_bridge_node,
        track_retract_control,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_broadcaster,
                on_exit=[load_track_retract_controller],
            )
        ),
    ])
