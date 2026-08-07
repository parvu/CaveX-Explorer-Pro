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
    set_plugin_path = SetEnvironmentVariable(
        'GZ_SIM_SYSTEM_PLUGIN_PATH',
        os.path.join(WORKTREE_ROOT, 'ardupilot_gazebo', 'build') + ':' +
        os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''))
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(pkg_cavex_tracked, 'models') + ':' +
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
    # URDF stub above (unrelated -- that stub is launch-time-only). Placeholder
    # -era spawn point: cavex_world.world still has the flat placeholder
    # dry_cave box here; Task 8 replaces it with the real cave mesh and updates
    # this spawn point to the mesh-derived one it records.
    #
    # z=2.5, not the brief's literal z=0.3 -- real, live evidence found during
    # this task's own go/no-go drive test: cavex_world.world's "dry_cave"
    # placeholder (cavex_slam_nav/worlds/cavex_world.world) is a fully SOLID
    # box, <pose>-22 0 1 0 0 0</pose> + <size>34 12 2</size>, i.e. solid
    # collision filling z=[0, 2] across x=[-39, -5] -- not a hollow interior.
    # Spawning at z=0.3 (inside that solid range) embeds the whole hull in it;
    # confirmed live via `gz service -s /world/cavex_world/set_pose ...`
    # teleports and `/world/cavex_world/pose/info` ground-truth pose readings:
    # at z=0.3 real position was static (delta ~0.001m over 15s of driving);
    # at z=2.5 (resting on the box's top surface, settles ~z=2.425) the exact
    # same drive command produced a real, unambiguous position delta (~7.9m
    # over 15s -- see task-7-report.md). x=-30/y=0 (the dry-cave section) kept
    # exactly as specified; only z changed, to rest on top of the placeholder
    # box instead of inside it. Task 8 replacing the placeholder geometry with
    # the real cave mesh is expected to also fix this and update the spawn
    # point again, same as the brief already anticipated for other reasons.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-world', 'cavex_world', '-file', generated_sdf_file,
                   '-name', VEHICLE_MODEL_NAME,
                   '-x', '-30', '-y', '0', '-z', '2.5'],
        output='screen',
    )

    # gz_bridge: only bridges sensors that actually exist on model.sdf.tracked.
    # NOTE / real gap found during this task: unlike the abandoned legged-walker
    # branch's URDF, model.sdf.tracked (Task 4) never got camera or lidar
    # sensors added -- it only carries over the real BlueBoat's imu_sensor and
    # navsat_sensor. The brief's Interfaces section (and its Step 2 example
    # `ros2 topic list | grep -iE "lidar|camera|..."`) assumes those sensors
    # exist; they don't, and adding them is an SDF-authoring change out of
    # Task 7's scope (its own instructions restrict committed model.sdf.tracked
    # changes to the URDF/robot_description and absolute-path fixes only).
    # `ponytail:` deferred -- add camera/lidar <sensor> blocks to
    # model.sdf.tracked in a follow-up task if/when downstream SLAM/perception
    # work actually needs them; /lidar/points and /camera/... are NOT bridged
    # here because there is nothing gz-transport-side to bridge.
    #
    # Topic names below re-verified empirically (Step 2) rather than assumed,
    # per this project's established gz-sim topic-naming gotcha (<topic>
    # overrides / sensor scoping don't reliably follow the naive convention).
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # imu_sensor has no <topic> override in model.sdf.tracked, so it
            # uses gz-sim's default scoped sensor-topic convention.
            f'/world/cavex_world/model/{VEHICLE_MODEL_NAME}/link/imu_link/sensor/imu_sensor/imu'
            '@sensor_msgs/msg/Imu[gz.msgs.IMU',
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
